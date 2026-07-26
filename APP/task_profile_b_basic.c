#include "task_profile_b_basic.h"

#include "bsp_key.h"
#include "bsp_systick.h"
#include "chassis.h"
#include "drv_buzzer.h"
#include "line_follow_app.h"
#include "motion_action.h"
#include "route_manager.h"

static BTask_Info_t s_task;
static uint8_t s_key_release_samples;
static uint32_t s_state_enter_ms;

static void BTask_StopVehicle(uint8_t reset_route)
{
    Motion_Stop();

    if (reset_route != 0U) {
        LineFollow_Stop();
    }

    Chassis_EmergencyStop();
}

static void BTask_EnterState(BTask_State_t state)
{
    if (s_task.state == state) {
        return;
    }

    s_task.state = state;
    s_task.state_elapsed_ms = 0U;
    s_state_enter_ms = BSP_GetTickMs();
    s_task.transition_count++;
}

static void BTask_EnterFault(BTask_Fault_t fault)
{
    s_task.fault = fault;
    Drv_Buzzer_Off();
    BTask_StopVehicle(1U);
    BTask_EnterState(B_TASK_STATE_FAULT);
}

static void BTask_ResetKeyArm(void)
{
    s_task.keys_armed = 0U;
    s_key_release_samples = 0U;
}

/*
 * 上电或复位后必须先确认KEY1、KEY4都稳定松开约100 ms。
 * 同时清除启动阶段残留边沿，防止一上电就自动启动。
 */
static uint8_t BTask_UpdateKeyArm(void)
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
            B_TASK_KEY_RELEASE_CONFIRM_SAMPLES) {
            s_key_release_samples++;
        }

        if (s_key_release_samples >=
            B_TASK_KEY_RELEASE_CONFIRM_SAMPLES) {
            s_task.keys_armed = 1U;
            s_key_release_samples = 0U;
        }
    } else {
        s_key_release_samples = 0U;
    }

    return s_task.keys_armed;
}

static void BTask_UpdateStatusSnapshot(void)
{
    RouteManager_Info_t route_info;

    s_task.line_follow_running =
        (LineFollow_GetState() == LINE_FOLLOW_RUN) ? 1U : 0U;
    s_task.chassis_fault = (uint8_t)Chassis_GetFault();

    if (RouteManager_GetInfo(&route_info) != PROJECT_OK) {
        s_task.route_state = 0U;
        s_task.route_arrived = 0U;
        s_task.route_error = 1U;
        return;
    }

    s_task.route_state = route_info.profile_state;
    s_task.route_arrived = route_info.arrived;
    s_task.route_error = route_info.error;
}

static void BTask_StartRoute(void)
{
    BSP_Status_t status;

    Drv_Buzzer_Off();
    s_task.fault = B_TASK_FAULT_NONE;

    if (Chassis_GetFault() != CHASSIS_FAULT_NONE) {
        BTask_EnterFault(B_TASK_FAULT_CHASSIS);
        return;
    }

    status = LineFollow_Start();
    s_task.start_status = (uint8_t)status;

    if (status == BSP_OK) {
        BTask_EnterState(B_TASK_STATE_RUNNING);
    } else if (status == BSP_ERROR) {
        BTask_EnterFault(B_TASK_FAULT_GRAY_OFFLINE);
    } else {
        BTask_EnterFault(B_TASK_FAULT_CONTROL_BUSY);
    }
}

void BTask_Init(void)
{
    s_task.state = B_TASK_STATE_WAIT_START;
    s_task.fault = B_TASK_FAULT_NONE;
    s_task.keys_armed = 0U;
    s_task.route_state = 0U;
    s_task.route_arrived = 0U;
    s_task.route_error = 0U;
    s_task.line_follow_running = 0U;
    s_task.chassis_fault = (uint8_t)CHASSIS_FAULT_NONE;
    s_task.start_status = (uint8_t)BSP_OK;
    s_task.state_elapsed_ms = 0U;
    s_task.transition_count = 0U;

    s_key_release_samples = 0U;
    s_state_enter_ms = BSP_GetTickMs();

    Drv_Buzzer_Off();
    BTask_StopVehicle(1U);
}

void BTask_Reset(void)
{
    Drv_Buzzer_Off();
    BTask_StopVehicle(1U);

    if (Chassis_GetFault() != CHASSIS_FAULT_NONE) {
        (void)Chassis_ClearFault();
    }

    BTask_Init();
}

void BTask_Update(void)
{
    uint8_t key1_pressed = 0U;
    uint8_t key4_pressed = 0U;

    s_task.state_elapsed_ms =
        (uint32_t)(BSP_GetTickMs() - s_state_enter_ms);
    BTask_UpdateStatusSnapshot();

    if (BTask_UpdateKeyArm() == 0U) {
        return;
    }

#if BSP_KEY1_ENABLE
    key1_pressed = BSP_Key_WasPressed(BSP_KEY1);
#endif
#if BSP_KEY4_ENABLE
    key4_pressed = BSP_Key_WasPressed(BSP_KEY4);
#endif

    /* KEY4在所有状态下优先执行安全停止并返回等待启动。 */
    if (key4_pressed != 0U) {
        Drv_Buzzer_Off();
        BTask_StopVehicle(1U);
        if (Chassis_GetFault() != CHASSIS_FAULT_NONE) {
            (void)Chassis_ClearFault();
        }
        s_task.fault = B_TASK_FAULT_NONE;
        s_task.start_status = (uint8_t)BSP_OK;
        BTask_EnterState(B_TASK_STATE_WAIT_START);
        BTask_ResetKeyArm();
        return;
    }

    switch (s_task.state) {
    case B_TASK_STATE_WAIT_START:
        if (key1_pressed != 0U) {
            BTask_StartRoute();
        }
        break;

    case B_TASK_STATE_RUNNING:
        if (s_task.route_arrived != 0U) {
            /*
             * Route层只上报到达；整车任务层负责最终停车和提示。
             * 不调用LineFollow_Stop()，避免清掉刚产生的到达诊断状态。
             */
            Motion_Stop();
            Chassis_EmergencyStop();
            Drv_Buzzer_On();
            BTask_EnterState(B_TASK_STATE_ARRIVAL_BUZZER);
        } else if (s_task.chassis_fault !=
                   (uint8_t)CHASSIS_FAULT_NONE) {
            BTask_EnterFault(B_TASK_FAULT_CHASSIS);
        } else if (s_task.route_error != 0U) {
            BTask_EnterFault(B_TASK_FAULT_ROUTE);
        } else if (s_task.line_follow_running == 0U) {
            BTask_EnterFault(B_TASK_FAULT_ROUTE);
        }
        break;

    case B_TASK_STATE_ARRIVAL_BUZZER:
        Chassis_EmergencyStop();
        if (s_task.state_elapsed_ms >=
            B_TASK_ARRIVAL_BUZZER_MS) {
            Drv_Buzzer_Off();
            BTask_EnterState(B_TASK_STATE_COMPLETE);
        }
        break;

    case B_TASK_STATE_COMPLETE:
        Chassis_EmergencyStop();
        if (key1_pressed != 0U) {
            BTask_StartRoute();
        }
        break;

    case B_TASK_STATE_FAULT:
        Drv_Buzzer_Off();
        Chassis_EmergencyStop();
        break;

    default:
        BTask_EnterFault(B_TASK_FAULT_ROUTE);
        break;
    }
}

BSP_Status_t BTask_GetInfo(BTask_Info_t *info)
{
    if (info == 0) {
        return BSP_PARAM;
    }

    *info = s_task;
    return BSP_OK;
}