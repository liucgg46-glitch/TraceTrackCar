#include "task_fsm.h"

#include "ball_balance_app.h"
#include "bsp_key.h"
#include "bsp_systick.h"
#include "chassis.h"
#include "drv_gray_sensor.h"
#include "k210_comm.h"
#include "line_follow_app.h"
#include "motion_action.h"
#include "route_manager.h"
#include "task_profile_h2_round_stop.h"

#define MISSION_CUSTOM_STEP_MM_X10             50
#define MISSION_CUSTOM_MIN_MM_X10              (-1000)
#define MISSION_CUSTOM_MAX_MM_X10              1000
#define MISSION_TARGET_O_MM_X10                0
#define MISSION_TARGET_PLUS_50_MM_X10          500
#define MISSION_TARGET_MINUS_50_MM_X10         (-500)
#define MISSION_ARM_TIMEOUT_MS                 5000U
#define MISSION_BALL_ERROR_LIMIT_MM_X10        100
#define MISSION_ROUTE_START_ROOM               0U
#define MISSION_M3_PLUS_REACH_POSITION_MM      45.0f
#define MISSION_M3_PLUS_MIN_VELOCITY_MM_S      (-5.0f)
#define MISSION_M3_PLUS_CONFIRM_FRAME_COUNT    2U

static Mission_Info_t s_mission;
static uint32_t s_state_enter_ms;
static uint32_t s_substate_enter_ms;
static uint8_t s_last_key_ready;
static uint8_t s_m3_plus_confirm_count;
static uint32_t s_m3_last_processed_sample_ms;
static uint8_t s_m3_sample_tracking_valid;

static int16_t Mission_Abs16(int16_t value)
{
    return (value < 0) ? (int16_t)(-value) : value;
}

static int16_t Mission_LimitTarget(int16_t target_mm_x10)
{
    if (target_mm_x10 < MISSION_CUSTOM_MIN_MM_X10) {
        return MISSION_CUSTOM_MIN_MM_X10;
    }
    if (target_mm_x10 > MISSION_CUSTOM_MAX_MM_X10) {
        return MISSION_CUSTOM_MAX_MM_X10;
    }
    return target_mm_x10;
}

static void Mission_StopVehicleSafely(void)
{
    LineFollow_StopPreserveRoute();
    Motion_Stop();
    Chassis_EmergencyStop();
}

static void Mission_SetSubstate(Mission_SubState_t substate)
{
    if (s_mission.substate == (uint8_t)substate) {
        return;
    }
    s_mission.substate = (uint8_t)substate;
    s_substate_enter_ms = BSP_GetTickMs();
}

static void Mission_ClearMode3PlusConfirm(void)
{
    s_m3_plus_confirm_count = 0U;
    s_m3_last_processed_sample_ms = 0U;
    s_m3_sample_tracking_valid = 0U;
    s_mission.m3_plus_confirm_count = 0U;
    s_mission.m3_last_processed_sample_ms = 0U;
}

static void Mission_EnterState(Mission_State_t state,
                               Mission_Result_t result,
                               uint16_t fault_code)
{
    uint32_t now_ms;

    now_ms = BSP_GetTickMs();
    if (s_mission.state != state) {
        s_state_enter_ms = now_ms;
    }
    s_mission.state = state;
    s_mission.result = result;
    s_mission.fault_code = fault_code;
    s_mission.running =
        (state == MISSION_STATE_RUNNING) ? 1U : 0U;
    s_mission.finished =
        ((state == MISSION_STATE_FINISH) ||
         (state == MISSION_STATE_ABORT) ||
         (state == MISSION_STATE_FAULT)) ? 1U : 0U;
}

static void Mission_UpdateLimits(void)
{
    switch (s_mission.mode) {
    case MISSION_MODE_2_LINE_LAP:
        s_mission.score_limit_ms = 20000U;
        s_mission.safety_timeout_ms = 30000U;
        break;
    case MISSION_MODE_3_BALL_TRANSFER:
        s_mission.score_limit_ms = 5000U;
        s_mission.safety_timeout_ms = 15000U;
        break;
    case MISSION_MODE_4_LINE_TO_B_WITH_BALL_O:
        s_mission.score_limit_ms = 8000U;
        s_mission.safety_timeout_ms = 15000U;
        break;
    case MISSION_MODE_5_LINE_LAP_WITH_BALL_O:
    case MISSION_MODE_6_LINE_LAP_WITH_CUSTOM_BALL:
    default:
        s_mission.score_limit_ms = 30000U;
        s_mission.safety_timeout_ms = 45000U;
        break;
    }
}

static void Mission_SetBallTarget(int16_t target_mm_x10,
                                  uint8_t enable)
{
    s_mission.active_ball_target_mm_x10 = target_mm_x10;
    BallBalance_App_SetTargetMmX10(target_mm_x10);
    BallBalance_App_SetVehicleFeedforwardEnabled(
        MISSION_BALL_VEHICLE_FEEDFORWARD_DEFAULT_ENABLE);
    s_mission.vehicle_feedforward_enabled =
        MISSION_BALL_VEHICLE_FEEDFORWARD_DEFAULT_ENABLE;
    if (enable != 0U) {
        BallBalance_App_Enable();
    } else {
        BallBalance_App_Disable();
    }
}

static void Mission_ResetRuntime(void)
{
    s_mission.armed_ready = 0U;
    s_mission.route_ready = 0U;
    s_mission.ball_ready = 0U;
    s_mission.running = 0U;
    s_mission.finished = 0U;
    s_mission.current_ball_position_mm_x10 = 0;
    s_mission.current_ball_error_mm_x10 = 0;
    s_mission.max_ball_error_mm_x10 = 0;
    s_mission.ball_filtered_velocity_mm_s_x10 = 0;
    s_mission.ball_settled = 0U;
    s_mission.start_ms = 0U;
    s_mission.elapsed_ms = 0U;
    s_mission.route_events = 0U;
    s_mission.fault_code = MISSION_FAULT_NONE;
    Mission_ClearMode3PlusConfirm();
    Mission_UpdateLimits();
}

static void Mission_ResetRouteEvents(void)
{
    RouteManager_ClearEvents(0xFFFFFFFFUL);
    s_mission.route_events = 0U;
}

static void Mission_EnterFault(Mission_Result_t result,
                               Mission_FaultCode_t fault)
{
    Mission_StopVehicleSafely();
    Mission_ClearMode3PlusConfirm();
    if ((fault == MISSION_FAULT_BALL) ||
        (fault == MISSION_FAULT_K210_TIMEOUT) ||
        (fault == MISSION_FAULT_SERVO)) {
        BallBalance_App_Disable();
    }
    Mission_EnterState(MISSION_STATE_FAULT, result, (uint16_t)fault);
}

static void Mission_Finish(Mission_Result_t result)
{
    Mission_StopVehicleSafely();
    Mission_EnterState(MISSION_STATE_FINISH, result, MISSION_FAULT_NONE);

    if (s_mission.mode == MISSION_MODE_2_LINE_LAP) {
        Mission_SetBallTarget(MISSION_TARGET_O_MM_X10, 0U);
    } else if (s_mission.mode == MISSION_MODE_3_BALL_TRANSFER) {
        Mission_SetBallTarget(MISSION_TARGET_MINUS_50_MM_X10, 1U);
    } else if ((s_mission.mode == MISSION_MODE_4_LINE_TO_B_WITH_BALL_O) ||
               (s_mission.mode == MISSION_MODE_5_LINE_LAP_WITH_BALL_O)) {
        Mission_SetBallTarget(MISSION_TARGET_O_MM_X10, 1U);
    } else {
        Mission_SetBallTarget(s_mission.custom_ball_target_mm_x10, 1U);
    }
}

static uint8_t Mission_IsBallUsable(const BallBalance_AppInfo_t *ball)
{
    if (ball == 0) {
        return 0U;
    }
    if ((ball->enabled == 0U) ||
        (ball->servo_fault != 0U) ||
        (ball->state == BALL_BALANCE_APP_FAULT) ||
        (ball->data_timeout != 0U)) {
        return 0U;
    }
    return ((ball->state == BALL_BALANCE_APP_ACTIVE) ||
            (ball->tracking_ready != 0U)) ? 1U : 0U;
}

static uint8_t Mission_IsBallReady(const BallBalance_AppInfo_t *ball)
{
    if (Mission_IsBallUsable(ball) == 0U) {
        return 0U;
    }
    return (ball->settled != 0U) ? 1U : 0U;
}

static void Mission_UpdateBallSnapshot(void)
{
    BallBalance_AppInfo_t ball;
    int16_t error_x10;
    int16_t abs_error_x10;

    if (BallBalance_App_GetInfo(&ball) != BSP_OK) {
        s_mission.ball_ready = 0U;
        return;
    }

    s_mission.current_ball_position_mm_x10 =
        (int16_t)(ball.estimator.position_mm * 10.0f);
    error_x10 =
        (int16_t)(s_mission.current_ball_position_mm_x10 -
                  s_mission.active_ball_target_mm_x10);
    s_mission.current_ball_error_mm_x10 = error_x10;

    if (s_mission.running != 0U) {
        abs_error_x10 = Mission_Abs16(error_x10);
        if (abs_error_x10 > s_mission.max_ball_error_mm_x10) {
            s_mission.max_ball_error_mm_x10 = abs_error_x10;
        }
    }

    s_mission.vehicle_feedforward_enabled =
        ball.vehicle_feedforward_enabled;
    s_mission.ball_filtered_velocity_mm_s_x10 =
        (int16_t)(ball.control.output.filtered_velocity_mm_s * 10.0f);
    s_mission.ball_settled = ball.settled;
    s_mission.ball_ready = Mission_IsBallReady(&ball);
}

static uint8_t Mission_CheckMode3PlusReach(void)
{
    BallBalance_AppInfo_t ball;
    uint8_t reached;

    if (BallBalance_App_GetInfo(&ball) != BSP_OK) {
        Mission_ClearMode3PlusConfirm();
        return 0U;
    }

    s_mission.current_ball_position_mm_x10 =
        (int16_t)(ball.estimator.position_mm * 10.0f);
    s_mission.ball_filtered_velocity_mm_s_x10 =
        (int16_t)(ball.control.output.filtered_velocity_mm_s * 10.0f);
    s_mission.ball_settled = ball.settled;

    if ((Mission_IsBallUsable(&ball) == 0U) ||
        (ball.last_sample_valid == 0U) ||
        (ball.last_valid_sample_ms == 0U)) {
        Mission_ClearMode3PlusConfirm();
        return 0U;
    }

    if ((s_m3_sample_tracking_valid != 0U) &&
        (ball.last_valid_sample_ms ==
         s_m3_last_processed_sample_ms)) {
        return 0U;
    }

    s_m3_sample_tracking_valid = 1U;
    s_m3_last_processed_sample_ms =
        ball.last_valid_sample_ms;
    s_mission.m3_last_processed_sample_ms =
        s_m3_last_processed_sample_ms;

    reached =
        ((ball.estimator.position_mm >=
          MISSION_M3_PLUS_REACH_POSITION_MM) &&
         (ball.control.output.filtered_velocity_mm_s >=
          MISSION_M3_PLUS_MIN_VELOCITY_MM_S)) ? 1U : 0U;

    if (reached != 0U) {
        if (s_m3_plus_confirm_count <
            MISSION_M3_PLUS_CONFIRM_FRAME_COUNT) {
            s_m3_plus_confirm_count++;
        }
    } else {
        s_m3_plus_confirm_count = 0U;
    }

    s_mission.m3_plus_confirm_count =
        s_m3_plus_confirm_count;

    return (s_m3_plus_confirm_count >=
            MISSION_M3_PLUS_CONFIRM_FRAME_COUNT) ? 1U : 0U;
}

static uint8_t Mission_IsRouteReady(void)
{
    if ((Drv_GraySensor_IsOnline() == 0U) ||
        (Chassis_GetFault() != CHASSIS_FAULT_NONE)) {
        return 0U;
    }
    return 1U;
}

static uint8_t Mission_StartLineFollow(void)
{
    BSP_Status_t status;

    if (LineFollow_SetSpeedProfile(H2_RUN_SPEED_CPS,
                                   H2_CURVE_SPEED_CPS,
                                   H2_CURVE_SPEED_CPS) != BSP_OK) {
        return 0U;
    }
    status = LineFollow_Start();
    return (status == BSP_OK) ? 1U : 0U;
}

static void Mission_StartRunning(Mission_SubState_t substate)
{
    uint32_t now_ms;

    now_ms = BSP_GetTickMs();
    Mission_ResetRouteEvents();
    s_mission.start_ms = now_ms;
    s_mission.elapsed_ms = 0U;
    s_mission.max_ball_error_mm_x10 = 0;
    Mission_SetSubstate(substate);
    Mission_EnterState(MISSION_STATE_RUNNING,
                       MISSION_RESULT_NONE,
                       MISSION_FAULT_NONE);
}

static void Mission_CheckRunningFaults(uint8_t need_route,
                                       uint8_t need_ball)
{
    BallBalance_AppInfo_t ball;

    if (s_mission.elapsed_ms > s_mission.safety_timeout_ms) {
        Mission_EnterFault(MISSION_RESULT_TIMEOUT,
                           MISSION_FAULT_TIMEOUT);
        return;
    }
    if (need_route != 0U) {
        if ((s_mission.route_events &
             (ROUTE_EVENT_FAILED |
              ROUTE_EVENT_LINE_LOST |
              ROUTE_EVENT_ACTION_ERROR)) != 0U) {
            Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                               MISSION_FAULT_ROUTE);
            return;
        }
        if ((LineFollow_GetState() != LINE_FOLLOW_RUN) &&
            ((s_mission.substate == MISSION_SUB_M2_WAIT_LEAVE_A) ||
             (s_mission.substate == MISSION_SUB_M2_RUNNING_LAP) ||
             (s_mission.substate == MISSION_SUB_M4_RUNNING_TO_B) ||
             (s_mission.substate == MISSION_SUB_M5_RUNNING_LAP) ||
             (s_mission.substate == MISSION_SUB_M6_RUNNING_LAP))) {
            Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                               MISSION_FAULT_ROUTE);
            return;
        }
        if (Chassis_GetFault() != CHASSIS_FAULT_NONE) {
            Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                               MISSION_FAULT_CHASSIS);
            return;
        }
    }
    if (need_ball != 0U) {
        if (BallBalance_App_GetInfo(&ball) != BSP_OK) {
            Mission_EnterFault(MISSION_RESULT_BALL_FAULT,
                               MISSION_FAULT_BALL);
        } else if (ball.servo_fault != 0U) {
            Mission_EnterFault(MISSION_RESULT_BALL_FAULT,
                               MISSION_FAULT_SERVO);
        } else if (ball.data_timeout != 0U) {
            Mission_EnterFault(MISSION_RESULT_BALL_FAULT,
                               MISSION_FAULT_K210_TIMEOUT);
        } else if (Mission_IsBallUsable(&ball) == 0U) {
            Mission_EnterFault(MISSION_RESULT_BALL_FAULT,
                               MISSION_FAULT_BALL);
        }
    }
}

static Mission_Result_t Mission_ResultByScore(void)
{
    if (s_mission.elapsed_ms > s_mission.score_limit_ms) {
        return MISSION_RESULT_TIME_FAIL;
    }
    if (((s_mission.mode == MISSION_MODE_4_LINE_TO_B_WITH_BALL_O) ||
         (s_mission.mode == MISSION_MODE_5_LINE_LAP_WITH_BALL_O) ||
         (s_mission.mode == MISSION_MODE_6_LINE_LAP_WITH_CUSTOM_BALL)) &&
        (s_mission.max_ball_error_mm_x10 >
         MISSION_BALL_ERROR_LIMIT_MM_X10)) {
        return MISSION_RESULT_BALL_ERROR_FAIL;
    }
    return MISSION_RESULT_PASS;
}

static void Mission_HandleIdleKeys(void)
{
#if BSP_KEY1_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY1) != 0U) {
        if (s_mission.mode <= MISSION_MODE_2_LINE_LAP) {
            s_mission.mode = MISSION_MODE_6_LINE_LAP_WITH_CUSTOM_BALL;
        } else {
            s_mission.mode = (Mission_Mode_t)((uint8_t)s_mission.mode - 1U);
        }
        Mission_UpdateLimits();
    }
#endif
#if BSP_KEY2_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY2) != 0U) {
        if (s_mission.mode >= MISSION_MODE_6_LINE_LAP_WITH_CUSTOM_BALL) {
            s_mission.mode = MISSION_MODE_2_LINE_LAP;
        } else {
            s_mission.mode = (Mission_Mode_t)((uint8_t)s_mission.mode + 1U);
        }
        Mission_UpdateLimits();
    }
#endif
#if BSP_KEY3_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY3) != 0U) {
        s_mission.custom_ball_target_mm_x10 =
            Mission_LimitTarget(
                (int16_t)(s_mission.custom_ball_target_mm_x10 -
                          MISSION_CUSTOM_STEP_MM_X10));
    }
#endif
#if BSP_KEY4_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY4) != 0U) {
        s_mission.custom_ball_target_mm_x10 =
            Mission_LimitTarget(
                (int16_t)(s_mission.custom_ball_target_mm_x10 +
                          MISSION_CUSTOM_STEP_MM_X10));
    }
#endif
#if BSP_KEY5_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY5) != 0U) {
        Mission_ResetRuntime();
        Mission_SetSubstate(MISSION_SUB_ARM_PREPARE);
        Mission_EnterState(MISSION_STATE_ARMED,
                           MISSION_RESULT_NONE,
                           MISSION_FAULT_NONE);
    }
#endif
}

static void Mission_HandleArmed(void)
{
    BallBalance_AppInfo_t ball;
    uint32_t now_ms;
    uint8_t key5_pressed;

    now_ms = BSP_GetTickMs();
    key5_pressed = 0U;
#if BSP_KEY5_ENABLE
    key5_pressed = BSP_Key_WasPressed(BSP_KEY5);
#endif

    Mission_StopVehicleSafely();
    s_mission.route_ready = Mission_IsRouteReady();

    if (s_mission.substate == MISSION_SUB_ARM_PREPARE) {
        Mission_ResetRouteEvents();
        if (s_mission.mode == MISSION_MODE_2_LINE_LAP) {
            Mission_SetBallTarget(MISSION_TARGET_O_MM_X10, 0U);
        } else if (s_mission.mode == MISSION_MODE_6_LINE_LAP_WITH_CUSTOM_BALL) {
            Mission_SetBallTarget(s_mission.custom_ball_target_mm_x10, 1U);
        } else {
            Mission_SetBallTarget(MISSION_TARGET_O_MM_X10, 1U);
        }
        Mission_SetSubstate(MISSION_SUB_ARM_WAIT_READY);
        return;
    }

    if (s_mission.mode == MISSION_MODE_2_LINE_LAP) {
        s_mission.ball_ready = 1U;
        s_mission.armed_ready = s_mission.route_ready;
    } else {
        if (BallBalance_App_GetInfo(&ball) == BSP_OK) {
            s_mission.ball_ready = Mission_IsBallReady(&ball);
        } else {
            s_mission.ball_ready = 0U;
        }
        if (s_mission.mode == MISSION_MODE_3_BALL_TRANSFER) {
            s_mission.armed_ready = s_mission.ball_ready;
        } else {
            s_mission.armed_ready =
                ((s_mission.ball_ready != 0U) &&
                 (s_mission.route_ready != 0U)) ? 1U : 0U;
        }
    }

    if ((s_mission.armed_ready == 0U) &&
        ((uint32_t)(now_ms - s_state_enter_ms) > MISSION_ARM_TIMEOUT_MS)) {
        s_mission.result = MISSION_RESULT_NOT_READY;
        s_mission.fault_code = MISSION_FAULT_NOT_READY;
    }

    if (key5_pressed == 0U) {
        return;
    }
    if (s_mission.armed_ready == 0U) {
        s_mission.result = MISSION_RESULT_NOT_READY;
        s_mission.fault_code = MISSION_FAULT_NOT_READY;
        return;
    }

    if (s_mission.mode == MISSION_MODE_2_LINE_LAP) {
        Mission_StartRunning(MISSION_SUB_M2_START);
    } else if (s_mission.mode == MISSION_MODE_3_BALL_TRANSFER) {
        Mission_StartRunning(MISSION_SUB_M3_SET_PLUS_50);
    } else if (s_mission.mode == MISSION_MODE_4_LINE_TO_B_WITH_BALL_O) {
        Mission_StartRunning(MISSION_SUB_M4_START);
    } else if (s_mission.mode == MISSION_MODE_5_LINE_LAP_WITH_BALL_O) {
        Mission_StartRunning(MISSION_SUB_M5_START);
    } else {
        Mission_StartRunning(MISSION_SUB_M6_START);
    }
}

static void Mission_HandleMode2(void)
{
    switch ((Mission_SubState_t)s_mission.substate) {
    case MISSION_SUB_M2_START:
        Mission_SetBallTarget(MISSION_TARGET_O_MM_X10, 0U);
        if (Mission_StartLineFollow() != 0U) {
            Mission_SetSubstate(MISSION_SUB_M2_WAIT_LEAVE_A);
        } else {
            Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                               MISSION_FAULT_ROUTE);
        }
        break;
    case MISSION_SUB_M2_WAIT_LEAVE_A:
        if ((s_mission.route_events & ROUTE_EVENT_LEFT_A) != 0U) {
            Mission_SetSubstate(MISSION_SUB_M2_RUNNING_LAP);
        }
        break;
    case MISSION_SUB_M2_RUNNING_LAP:
        if ((s_mission.route_events & ROUTE_EVENT_LAP_COMPLETE) != 0U) {
            Mission_SetSubstate(MISSION_SUB_M2_BRAKE);
        }
        break;
    case MISSION_SUB_M2_BRAKE:
        Mission_SetSubstate(MISSION_SUB_M2_DONE);
        break;
    case MISSION_SUB_M2_DONE:
        Mission_Finish(Mission_ResultByScore());
        break;
    default:
        Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                           MISSION_FAULT_INTERNAL);
        break;
    }
}

static void Mission_HandleMode3(void)
{
    switch ((Mission_SubState_t)s_mission.substate) {
    case MISSION_SUB_M3_SET_PLUS_50:
        Mission_StopVehicleSafely();
        Mission_ClearMode3PlusConfirm();
        Mission_SetBallTarget(MISSION_TARGET_PLUS_50_MM_X10, 1U);
        Mission_SetSubstate(MISSION_SUB_M3_WAIT_PLUS_50);
        break;
    case MISSION_SUB_M3_WAIT_PLUS_50:
        if ((s_mission.active_ball_target_mm_x10 ==
             MISSION_TARGET_PLUS_50_MM_X10) &&
            (Mission_CheckMode3PlusReach() != 0U)) {
            Mission_SetSubstate(MISSION_SUB_M3_SET_MINUS_50);
        }
        break;
    case MISSION_SUB_M3_SET_MINUS_50:
        Mission_SetBallTarget(MISSION_TARGET_MINUS_50_MM_X10, 1U);
        Mission_SetSubstate(MISSION_SUB_M3_WAIT_MINUS_50);
        break;
    case MISSION_SUB_M3_WAIT_MINUS_50:
        if ((s_mission.active_ball_target_mm_x10 ==
             MISSION_TARGET_MINUS_50_MM_X10) &&
            (BallBalance_App_IsSettled() != 0U)) {
            Mission_SetSubstate(MISSION_SUB_M3_DONE);
        }
        break;
    case MISSION_SUB_M3_DONE:
        Mission_Finish(Mission_ResultByScore());
        break;
    default:
        Mission_EnterFault(MISSION_RESULT_BALL_FAULT,
                           MISSION_FAULT_INTERNAL);
        break;
    }
}

static void Mission_HandleMode4(void)
{
    switch ((Mission_SubState_t)s_mission.substate) {
    case MISSION_SUB_M4_START:
        Mission_SetBallTarget(MISSION_TARGET_O_MM_X10, 1U);
        if (Mission_StartLineFollow() != 0U) {
            Mission_SetSubstate(MISSION_SUB_M4_RUNNING_TO_B);
        } else {
            Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                               MISSION_FAULT_ROUTE);
        }
        break;
    case MISSION_SUB_M4_RUNNING_TO_B:
        if ((s_mission.route_events & ROUTE_EVENT_PASSED_B) != 0U) {
            Mission_SetSubstate(MISSION_SUB_M4_BRAKE);
        }
        break;
    case MISSION_SUB_M4_BRAKE:
        Mission_SetSubstate(MISSION_SUB_M4_DONE);
        break;
    case MISSION_SUB_M4_DONE:
        Mission_Finish(Mission_ResultByScore());
        break;
    default:
        Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                           MISSION_FAULT_INTERNAL);
        break;
    }
}

static void Mission_HandleMode5(uint8_t custom)
{
    Mission_SubState_t substate;
    int16_t target;

    substate = (Mission_SubState_t)s_mission.substate;
    target = (custom != 0U) ?
        s_mission.custom_ball_target_mm_x10 :
        MISSION_TARGET_O_MM_X10;

    if ((substate == MISSION_SUB_M5_START) ||
        (substate == MISSION_SUB_M6_START)) {
        Mission_SetBallTarget(target, 1U);
        if (Mission_StartLineFollow() != 0U) {
            Mission_SetSubstate((custom != 0U) ?
                MISSION_SUB_M6_RUNNING_LAP :
                MISSION_SUB_M5_RUNNING_LAP);
        } else {
            Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                               MISSION_FAULT_ROUTE);
        }
        return;
    }

    if ((substate == MISSION_SUB_M5_RUNNING_LAP) ||
        (substate == MISSION_SUB_M6_RUNNING_LAP)) {
        if ((s_mission.route_events & ROUTE_EVENT_LAP_COMPLETE) != 0U) {
            Mission_SetSubstate((custom != 0U) ?
                MISSION_SUB_M6_BRAKE :
                MISSION_SUB_M5_BRAKE);
        }
        return;
    }

    if ((substate == MISSION_SUB_M5_BRAKE) ||
        (substate == MISSION_SUB_M6_BRAKE)) {
        Mission_SetSubstate((custom != 0U) ?
            MISSION_SUB_M6_DONE :
            MISSION_SUB_M5_DONE);
        return;
    }

    if ((substate == MISSION_SUB_M5_DONE) ||
        (substate == MISSION_SUB_M6_DONE)) {
        Mission_Finish(Mission_ResultByScore());
        return;
    }

    Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                       MISSION_FAULT_INTERNAL);
}

static void Mission_HandleRunning(void)
{
    uint8_t need_route;
    uint8_t need_ball;

#if BSP_KEY5_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY5) != 0U) {
        Mission_StopVehicleSafely();
        Mission_ClearMode3PlusConfirm();
        Mission_EnterState(MISSION_STATE_ABORT,
                           MISSION_RESULT_USER_ABORT,
                           MISSION_FAULT_NONE);
        return;
    }
#endif

    if (s_mission.start_ms != 0U) {
        s_mission.elapsed_ms =
            (uint32_t)(BSP_GetTickMs() - s_mission.start_ms);
    }
    s_mission.route_events = RouteManager_GetEvents();
    need_route = (s_mission.mode == MISSION_MODE_3_BALL_TRANSFER) ?
        0U : 1U;
    need_ball = (s_mission.mode == MISSION_MODE_2_LINE_LAP) ?
        0U : 1U;
    Mission_CheckRunningFaults(need_route, need_ball);
    if (s_mission.state != MISSION_STATE_RUNNING) {
        return;
    }

    switch (s_mission.mode) {
    case MISSION_MODE_2_LINE_LAP:
        Mission_HandleMode2();
        break;
    case MISSION_MODE_3_BALL_TRANSFER:
        Mission_HandleMode3();
        break;
    case MISSION_MODE_4_LINE_TO_B_WITH_BALL_O:
        Mission_HandleMode4();
        break;
    case MISSION_MODE_5_LINE_LAP_WITH_BALL_O:
        Mission_HandleMode5(0U);
        break;
    case MISSION_MODE_6_LINE_LAP_WITH_CUSTOM_BALL:
        Mission_HandleMode5(1U);
        break;
    default:
        Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                           MISSION_FAULT_INTERNAL);
        break;
    }
}

static void Mission_HandleTerminalKeys(void)
{
#if BSP_KEY5_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY5) != 0U) {
        Mission_StopVehicleSafely();
        Mission_ResetRuntime();
        Mission_SetSubstate(MISSION_SUB_IDLE);
        Mission_EnterState(MISSION_STATE_IDLE,
                           MISSION_RESULT_NONE,
                           MISSION_FAULT_NONE);
    }
#endif
}

void TaskFSM_Init(void)
{
    s_mission.initialized = 1U;
    s_mission.mode = MISSION_MODE_2_LINE_LAP;
    s_mission.state = MISSION_STATE_BOOT;
    s_mission.substate = (uint8_t)MISSION_SUB_IDLE;
    s_mission.result = MISSION_RESULT_NONE;
    s_mission.custom_ball_target_mm_x10 = 0;
    s_mission.active_ball_target_mm_x10 = 0;
    s_state_enter_ms = BSP_GetTickMs();
    s_substate_enter_ms = s_state_enter_ms;
    s_last_key_ready = 0U;
    Mission_ResetRuntime();
}

void TaskFSM_Reset(void)
{
    Mission_StopVehicleSafely();
    BallBalance_App_Disable();
    TaskFSM_Init();
}

void TaskFSM_Update(void)
{
    if (s_mission.initialized == 0U) {
        TaskFSM_Init();
    }

    Mission_UpdateBallSnapshot();
    s_mission.route_events = RouteManager_GetEvents();

    switch (s_mission.state) {
    case MISSION_STATE_BOOT:
        Mission_StopVehicleSafely();
        BallBalance_App_SetVehicleFeedforwardEnabled(0U);
        BallBalance_App_Disable();
        Mission_ResetRouteEvents();
        Mission_SetSubstate(MISSION_SUB_IDLE);
        Mission_EnterState(MISSION_STATE_IDLE,
                           MISSION_RESULT_NONE,
                           MISSION_FAULT_NONE);
        break;
    case MISSION_STATE_IDLE:
        if (s_last_key_ready == 0U) {
            if ((BSP_Key_IsPressed(BSP_KEY1) == 0U) &&
                (BSP_Key_IsPressed(BSP_KEY2) == 0U) &&
                (BSP_Key_IsPressed(BSP_KEY3) == 0U) &&
                (BSP_Key_IsPressed(BSP_KEY4) == 0U) &&
                (BSP_Key_IsPressed(BSP_KEY5) == 0U)) {
                s_last_key_ready = 1U;
            }
        } else {
            Mission_HandleIdleKeys();
        }
        break;
    case MISSION_STATE_ARMED:
        Mission_HandleArmed();
        break;
    case MISSION_STATE_RUNNING:
        Mission_HandleRunning();
        break;
    case MISSION_STATE_FINISH:
    case MISSION_STATE_ABORT:
    case MISSION_STATE_FAULT:
        Mission_HandleTerminalKeys();
        break;
    default:
        Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                           MISSION_FAULT_INTERNAL);
        break;
    }
}

BSP_Status_t TaskFSM_GetInfo(Mission_Info_t *info)
{
    if (info == 0) {
        return BSP_PARAM;
    }
    *info = s_mission;
    return BSP_OK;
}
