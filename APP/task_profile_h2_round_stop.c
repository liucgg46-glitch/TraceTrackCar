#include "task_profile_h2_round_stop.h"

#include "bsp_key.h"
#include "bsp_systick.h"
#include "chassis.h"
#include "drv_encoder.h"
#include "drv_gray_sensor.h"
#include "lcd_ui.h"
#include "line_follow_app.h"
#include "motion_action.h"
#include "route_profile_h_oval.h"

#include <stdio.h>

static H2Task_Info_t s_task;
static uint8_t s_key_release_samples;
static uint8_t s_stop_settle_active;
static uint32_t s_start_tick;
static uint32_t s_state_enter_ms;
static uint32_t s_stop_settle_start_ms;
static uint32_t s_last_lcd_text_ms;
static int32_t s_start_left_mm;
static int32_t s_start_right_mm;
static int32_t s_last_encoder_distance_mm;

static int32_t H2Task_Abs32(int32_t value)
{
    return (value >= 0) ? value : -value;
}

static int32_t H2Task_RelativeMm(int32_t current, int32_t origin)
{
    return (int32_t)((uint32_t)current - (uint32_t)origin);
}

static const char *H2Task_PhaseName(uint8_t route_state)
{
    switch ((HOvalRoute_State_t)route_state) {
    case H_OVAL_ROUTE_LEAVING_A:       return "LEAVE A";
    case H_OVAL_ROUTE_FIRST_STRAIGHT:  return "STRAIGHT1";
    case H_OVAL_ROUTE_FIRST_CURVE:     return "CURVE1";
    case H_OVAL_ROUTE_SECOND_STRAIGHT: return "STRAIGHT2";
    case H_OVAL_ROUTE_SECOND_CURVE:    return "CURVE2";
    case H_OVAL_ROUTE_FINISH_ARMED:    return "ARMED";
    case H_OVAL_ROUTE_FINISH_CONFIRM:  return "FINISH";
    case H_OVAL_ROUTE_FINISHED:        return "MARK";
    case H_OVAL_ROUTE_IDLE:            return "IDLE";
    case H_OVAL_ROUTE_FAULT:
    default:                           return "FAULT";
    }
}

static void H2Task_EnterState(H2Task_State_t state, uint32_t now_ms)
{
    if (s_task.state == state) {
        return;
    }

    s_task.state = state;
    s_task.state_elapsed_ms = 0U;
    s_state_enter_ms = now_ms;
    s_task.transition_count++;
}

static void H2Task_StopVehicle(uint8_t preserve_route)
{
    Motion_Stop();
    if (preserve_route != 0U) {
        LineFollow_StopPreserveRoute();
    } else {
        LineFollow_Stop();
    }
    Chassis_EmergencyStop();
}

static void H2Task_EnterFault(H2Task_Fault_t fault, uint32_t now_ms)
{
    s_task.fault = fault;
    s_task.timer_running = 0U;
    s_task.final_time_ms =
        (s_start_tick == 0U) ? 0U :
        (uint32_t)(now_ms - s_start_tick);
    H2Task_StopVehicle(1U);
    H2Task_EnterState(H2_FAULT, now_ms);
}

static void H2Task_ResetKeyArm(void)
{
    s_task.keys_armed = 0U;
    s_key_release_samples = 0U;
}

static uint8_t H2Task_UpdateKeyArm(void)
{
    if (s_task.keys_armed != 0U) {
        return 1U;
    }

#if BSP_KEY1_ENABLE
    (void)BSP_Key_WasPressed(BSP_KEY1);
    (void)BSP_Key_WasReleased(BSP_KEY1);
#endif
#if BSP_KEY4_ENABLE
    (void)BSP_Key_WasPressed(BSP_KEY4);
    (void)BSP_Key_WasReleased(BSP_KEY4);
#endif

#if BSP_KEY1_ENABLE && BSP_KEY4_ENABLE
    if ((BSP_Key_IsPressed(BSP_KEY1) == 0U) &&
        (BSP_Key_IsPressed(BSP_KEY4) == 0U)) {
#elif BSP_KEY1_ENABLE
    if (BSP_Key_IsPressed(BSP_KEY1) == 0U) {
#elif BSP_KEY4_ENABLE
    if (BSP_Key_IsPressed(BSP_KEY4) == 0U) {
#else
    if (1) {
#endif
        if (s_key_release_samples <
            H2_KEY_RELEASE_CONFIRM_SAMPLES) {
            s_key_release_samples++;
        }
        if (s_key_release_samples >=
            H2_KEY_RELEASE_CONFIRM_SAMPLES) {
            s_task.keys_armed = 1U;
            s_key_release_samples = 0U;
        }
    } else {
        s_key_release_samples = 0U;
    }

    return s_task.keys_armed;
}

static void H2Task_UpdateEncoderSnapshot(void)
{
    int32_t left_relative;
    int32_t right_relative;
    int32_t encoder_step;

    left_relative =
        H2Task_RelativeMm(Drv_Encoder_GetLeftTotalMm(),
                          s_start_left_mm);
    right_relative =
        H2Task_RelativeMm(Drv_Encoder_GetRightTotalMm(),
                          s_start_right_mm);
    s_task.encoder_distance_mm =
        (left_relative + right_relative) / 2;
    encoder_step =
        s_task.encoder_distance_mm -
        s_last_encoder_distance_mm;
    s_last_encoder_distance_mm =
        s_task.encoder_distance_mm;

    s_task.left_speed_cps = Drv_Encoder_GetLeftSpeedCps();
    s_task.right_speed_cps = Drv_Encoder_GetRightSpeedCps();
    if ((H2Task_Abs32(s_task.left_speed_cps) >
         H2_ENCODER_MAX_REASONABLE_CPS) ||
        (H2Task_Abs32(s_task.right_speed_cps) >
         H2_ENCODER_MAX_REASONABLE_CPS) ||
        (H2Task_Abs32(encoder_step) >
         H2_ENCODER_MAX_STEP_MM)) {
        /*
         * 编码器只作为诊断辅助；异常时降低可信度，不据此提前判断终点或停车。
         */
        s_task.encoder_reliable = 0U;
    }
}

static void H2Task_UpdateRouteSnapshot(HOvalRoute_Info_t *route)
{
    if (HRoute_GetH2Info(route) != PROJECT_OK) {
        route->state = H_OVAL_ROUTE_FAULT;
        route->fault = H_OVAL_ROUTE_FAULT_PARAM;
        route->finish_armed = 0U;
        route->finish_candidate = 0U;
        route->arrived = 0U;
    }

    s_task.route_state = (uint8_t)route->state;
    s_task.finish_armed = route->finish_armed;
    s_task.finish_candidate = route->finish_candidate;
    s_task.line_follow_running =
        (LineFollow_GetState() == LINE_FOLLOW_RUN) ? 1U : 0U;
    s_task.chassis_fault = (uint8_t)Chassis_GetFault();
}

static void H2Task_UpdateDisplay(uint32_t now_ms, uint8_t force)
{
    char line1[24];
    char line2[24];
    char line3[24];
    uint32_t display_ms;

    if ((force == 0U) &&
        ((uint32_t)(now_ms - s_last_lcd_text_ms) <
         H2_LCD_TEXT_UPDATE_MS)) {
        return;
    }
    s_last_lcd_text_ms = now_ms;

    display_ms = (s_task.state == H2_STOPPED) ?
                 s_task.final_time_ms : s_task.elapsed_ms;
    (void)snprintf(line2,
                   sizeof(line2),
                   "TIME %02lu.%03lus",
                   (unsigned long)(display_ms / 1000U),
                   (unsigned long)(display_ms % 1000U));

    switch (s_task.state) {
    case H2_IDLE:
    case H2_WAIT_START:
        (void)snprintf(line1, sizeof(line1), "H2 READY");
        (void)snprintf(line3, sizeof(line3), "KEY1 START");
        break;
    case H2_LEAVE_A:
    case H2_RUNNING:
    case H2_FINISH_ARMED:
        (void)snprintf(line1, sizeof(line1), "H2 RUN");
        (void)snprintf(line3,
                       sizeof(line3),
                       "PHASE %s",
                       H2Task_PhaseName(s_task.route_state));
        break;
    case H2_FINAL_APPROACH:
        (void)snprintf(line1, sizeof(line1), "H2 APPROACH");
        (void)snprintf(line3,
                       sizeof(line3),
                       "DELAY %lums",
                       (unsigned long)H2_STOP_DELAY_MS);
        break;
    case H2_BRAKING:
        (void)snprintf(line1, sizeof(line1), "H2 BRAKING");
        (void)snprintf(line3, sizeof(line3), "WAIT STABLE");
        break;
    case H2_STOPPED:
        (void)snprintf(line1, sizeof(line1), "H2 DONE");
        (void)snprintf(line3,
                       sizeof(line3),
                       "OFFSET %ld/%dmm",
                       (long)s_task.stop_offset_mm,
                       H2_MAX_STOP_OFFSET_MM);
        break;
    case H2_FAULT:
    default:
        (void)snprintf(line1, sizeof(line1), "H2 FAULT");
        (void)snprintf(line3,
                       sizeof(line3),
                       "CODE %u",
                       (unsigned int)s_task.fault);
        break;
    }

    LcdUi_ShowStatus(line1, line2, line3);
}

static void H2Task_Start(uint32_t now_ms)
{
    BSP_Status_t status;

    if (Drv_GraySensor_IsOnline() == 0U) {
        H2Task_EnterFault(H2_FAULT_GRAY_OFFLINE, now_ms);
        return;
    }
    if (Chassis_GetFault() != CHASSIS_FAULT_NONE) {
        H2Task_EnterFault(H2_FAULT_CHASSIS, now_ms);
        return;
    }
    if (LineFollow_SetSpeedProfile(H2_RUN_SPEED_CPS,
                                   H2_CURVE_SPEED_CPS,
                                   H2_CURVE_SPEED_CPS) != BSP_OK) {
        H2Task_EnterFault(H2_FAULT_INTERNAL, now_ms);
        return;
    }

    s_task.fault = H2_FAULT_NONE;
    s_task.timer_running = 1U;
    s_task.elapsed_ms = 0U;
    s_task.final_time_ms = 0U;
    s_task.finish_armed = 0U;
    s_task.finish_candidate = 0U;
    s_task.encoder_reliable = 1U;
    s_task.marker_distance_mm = 0;
    s_task.stop_offset_mm = 0;
    s_start_tick = now_ms;
    s_start_left_mm = Drv_Encoder_GetLeftTotalMm();
    s_start_right_mm = Drv_Encoder_GetRightTotalMm();
    s_last_encoder_distance_mm = 0;

    status = LineFollow_Start();
    if (status == BSP_OK) {
        H2Task_EnterState(H2_LEAVE_A, now_ms);
    } else if (status == BSP_ERROR) {
        H2Task_EnterFault(H2_FAULT_GRAY_OFFLINE, now_ms);
    } else {
        H2Task_EnterFault(H2_FAULT_CONTROL_BUSY, now_ms);
    }
}

static H2Task_Fault_t H2Task_MapRouteFault(HOvalRoute_Fault_t fault)
{
    switch (fault) {
    case H_OVAL_ROUTE_FAULT_LINE_LOST:
        return H2_FAULT_ROUTE_LINE_LOST;
    case H_OVAL_ROUTE_FAULT_PHASE_TIMEOUT:
    case H_OVAL_ROUTE_FAULT_RUN_TIMEOUT:
        return H2_FAULT_ROUTE_TIMEOUT;
    case H_OVAL_ROUTE_FAULT_PARAM:
    default:
        return H2_FAULT_INTERNAL;
    }
}

void H2Task_Init(void)
{
    uint32_t now_ms = BSP_GetTickMs();

    s_task.state = H2_WAIT_START;
    s_task.fault = H2_FAULT_NONE;
    s_task.keys_armed = 0U;
    s_task.timer_running = 0U;
    s_task.route_state = (uint8_t)H_OVAL_ROUTE_IDLE;
    s_task.finish_armed = 0U;
    s_task.finish_candidate = 0U;
    s_task.encoder_reliable = 1U;
    s_task.line_follow_running = 0U;
    s_task.chassis_fault = (uint8_t)CHASSIS_FAULT_NONE;
    s_task.elapsed_ms = 0U;
    s_task.final_time_ms = 0U;
    s_task.state_elapsed_ms = 0U;
    s_task.transition_count = 0U;
    s_task.encoder_distance_mm = 0;
    s_task.marker_distance_mm = 0;
    s_task.stop_offset_mm = 0;
    s_task.left_speed_cps = 0;
    s_task.right_speed_cps = 0;

    s_key_release_samples = 0U;
    s_stop_settle_active = 0U;
    s_start_tick = 0U;
    s_state_enter_ms = now_ms;
    s_stop_settle_start_ms = now_ms;
    s_last_lcd_text_ms = now_ms - H2_LCD_TEXT_UPDATE_MS;
    s_start_left_mm = Drv_Encoder_GetLeftTotalMm();
    s_start_right_mm = Drv_Encoder_GetRightTotalMm();
    s_last_encoder_distance_mm = 0;

    H2Task_StopVehicle(0U);
    (void)LineFollow_SetSpeedProfile(H2_RUN_SPEED_CPS,
                                     H2_CURVE_SPEED_CPS,
                                     H2_CURVE_SPEED_CPS);
    H2Task_UpdateDisplay(now_ms, 1U);
}

void H2Task_Reset(void)
{
    if (Chassis_GetFault() != CHASSIS_FAULT_NONE) {
        (void)Chassis_ClearFault();
    }
    H2Task_Init();
}

void H2Task_Update(void)
{
    uint32_t now_ms = BSP_GetTickMs();
    uint8_t key1_pressed = 0U;
    uint8_t key4_pressed = 0U;
    HOvalRoute_Info_t route;

    s_task.state_elapsed_ms =
        (uint32_t)(now_ms - s_state_enter_ms);
    if (s_task.timer_running != 0U) {
        s_task.elapsed_ms = (uint32_t)(now_ms - s_start_tick);
    }

    H2Task_UpdateEncoderSnapshot();
    H2Task_UpdateRouteSnapshot(&route);

    if (H2Task_UpdateKeyArm() != 0U) {
#if BSP_KEY1_ENABLE
        key1_pressed = BSP_Key_WasPressed(BSP_KEY1);
#endif
#if BSP_KEY4_ENABLE
        key4_pressed = BSP_Key_WasPressed(BSP_KEY4);
#endif
    }

    if (key4_pressed != 0U) {
        H2Task_EnterFault(H2_FAULT_MANUAL_STOP, now_ms);
        H2Task_ResetKeyArm();
        H2Task_UpdateDisplay(now_ms, 1U);
        return;
    }

    if ((s_task.timer_running != 0U) &&
        (s_task.elapsed_ms >= H2_MAX_RUN_TIME_MS)) {
        H2Task_EnterFault(H2_FAULT_RUN_TIMEOUT, now_ms);
        H2Task_UpdateDisplay(now_ms, 1U);
        return;
    }

    switch (s_task.state) {
    case H2_IDLE:
        H2Task_EnterState(H2_WAIT_START, now_ms);
        break;

    case H2_WAIT_START:
        if (key1_pressed != 0U) {
            H2Task_Start(now_ms);
        }
        break;

    case H2_LEAVE_A:
        if (Drv_GraySensor_IsOnline() == 0U) {
            H2Task_EnterFault(H2_FAULT_GRAY_OFFLINE, now_ms);
        } else if (route.fault != H_OVAL_ROUTE_FAULT_NONE) {
            H2Task_EnterFault(H2Task_MapRouteFault(route.fault),
                              now_ms);
        } else if (s_task.chassis_fault !=
                   (uint8_t)CHASSIS_FAULT_NONE) {
            H2Task_EnterFault(H2_FAULT_CHASSIS, now_ms);
        } else if (route.state != H_OVAL_ROUTE_LEAVING_A) {
            H2Task_EnterState(H2_RUNNING, now_ms);
        } else if (s_task.line_follow_running == 0U) {
            H2Task_EnterFault(H2_FAULT_INTERNAL, now_ms);
        }
        break;

    case H2_RUNNING:
        if (Drv_GraySensor_IsOnline() == 0U) {
            H2Task_EnterFault(H2_FAULT_GRAY_OFFLINE, now_ms);
        } else if (route.fault != H_OVAL_ROUTE_FAULT_NONE) {
            H2Task_EnterFault(H2Task_MapRouteFault(route.fault),
                              now_ms);
        } else if (s_task.chassis_fault !=
                   (uint8_t)CHASSIS_FAULT_NONE) {
            H2Task_EnterFault(H2_FAULT_CHASSIS, now_ms);
        } else if (route.finish_armed != 0U) {
            /*
             * 第二段右弯确认后先降到终点搜索速度，降低横线识别到制动之间
             * 的机械惯性；识别横线后还会再切到更低的补偿速度。
             */
            if (LineFollow_SetSpeedProfile(
                    H2_FINISH_SEARCH_SPEED_CPS,
                    H2_FINISH_SEARCH_SPEED_CPS,
                    H2_FINISH_SEARCH_SPEED_CPS) != BSP_OK) {
                H2Task_EnterFault(H2_FAULT_INTERNAL, now_ms);
            } else {
                H2Task_EnterState(H2_FINISH_ARMED, now_ms);
            }
        } else if (s_task.line_follow_running == 0U) {
            H2Task_EnterFault(H2_FAULT_INTERNAL, now_ms);
        }
        break;

    case H2_FINISH_ARMED:
        if (Drv_GraySensor_IsOnline() == 0U) {
            H2Task_EnterFault(H2_FAULT_GRAY_OFFLINE, now_ms);
        } else if (s_task.chassis_fault !=
                   (uint8_t)CHASSIS_FAULT_NONE) {
            H2Task_EnterFault(H2_FAULT_CHASSIS, now_ms);
        } else if (route.fault != H_OVAL_ROUTE_FAULT_NONE) {
            H2Task_EnterFault(H2Task_MapRouteFault(route.fault),
                              now_ms);
        } else if (route.arrived != 0U) {
            s_task.marker_distance_mm =
                s_task.encoder_distance_mm;
            if (LineFollow_SetSpeedProfile(
                    H2_APPROACH_SPEED_CPS,
                    H2_APPROACH_SPEED_CPS,
                    H2_APPROACH_SPEED_CPS) != BSP_OK) {
                H2Task_EnterFault(H2_FAULT_INTERNAL, now_ms);
            } else {
                H2Task_EnterState(H2_FINAL_APPROACH, now_ms);
            }
        } else if (s_task.line_follow_running == 0U) {
            H2Task_EnterFault(H2_FAULT_INTERNAL, now_ms);
        }
        break;

    case H2_FINAL_APPROACH:
        s_task.stop_offset_mm =
            s_task.encoder_distance_mm -
            s_task.marker_distance_mm;
        if (Drv_GraySensor_IsOnline() == 0U) {
            H2Task_EnterFault(H2_FAULT_GRAY_OFFLINE, now_ms);
        } else if (s_task.chassis_fault !=
                   (uint8_t)CHASSIS_FAULT_NONE) {
            H2Task_EnterFault(H2_FAULT_CHASSIS, now_ms);
        } else if (s_task.state_elapsed_ms >= H2_STOP_DELAY_MS) {
            H2Task_StopVehicle(1U);
            s_stop_settle_active = 0U;
            H2Task_EnterState(H2_BRAKING, now_ms);
        } else if (s_task.line_follow_running == 0U) {
            H2Task_EnterFault(H2_FAULT_INTERNAL, now_ms);
        }
        break;

    case H2_BRAKING:
        Chassis_EmergencyStop();
        if ((H2Task_Abs32(s_task.left_speed_cps) <=
             H2_STOP_SPEED_THRESHOLD_CPS) &&
            (H2Task_Abs32(s_task.right_speed_cps) <=
             H2_STOP_SPEED_THRESHOLD_CPS)) {
            if (s_stop_settle_active == 0U) {
                s_stop_settle_active = 1U;
                s_stop_settle_start_ms = now_ms;
            } else if ((uint32_t)(now_ms -
                                  s_stop_settle_start_ms) >=
                       H2_STOP_SETTLE_MS) {
                s_task.final_time_ms =
                    (uint32_t)(now_ms - s_start_tick);
                s_task.elapsed_ms = s_task.final_time_ms;
                s_task.timer_running = 0U;
                H2Task_EnterState(H2_STOPPED, now_ms);
            }
        } else {
            s_stop_settle_active = 0U;
        }

        if ((s_task.state == H2_BRAKING) &&
            (s_task.state_elapsed_ms >= H2_BRAKE_TIME_MS)) {
            H2Task_EnterFault(H2_FAULT_BRAKE_TIMEOUT, now_ms);
        }
        break;

    case H2_STOPPED:
        Chassis_EmergencyStop();
        if (key1_pressed != 0U) {
            H2Task_Start(now_ms);
        }
        break;

    case H2_FAULT:
        Chassis_EmergencyStop();
        if (key1_pressed != 0U) {
            H2Task_Reset();
            H2Task_ResetKeyArm();
        }
        break;

    default:
        H2Task_EnterFault(H2_FAULT_INTERNAL, now_ms);
        break;
    }

    H2Task_UpdateDisplay(now_ms, 0U);
}

BSP_Status_t H2Task_GetInfo(H2Task_Info_t *info)
{
    if (info == 0) {
        return BSP_PARAM;
    }

    *info = s_task;
    return BSP_OK;
}
