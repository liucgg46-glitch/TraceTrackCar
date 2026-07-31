#include "app_diagnostics.h"

#include "bsp_gpio.h"
#include "bsp_systick.h"
#include "bsp_uart.h"
#include "task_fsm.h"

#include <stdint.h>
#include <stdio.h>

void AppDiagnostics_HeartbeatUpdate(void)
{
    static uint32_t last_toggle_ms = 0U;

    if (BSP_TimeElapsed(&last_toggle_ms, 500U) != 0U) {
        BSP_GPIO_Toggle(BSP_GPIO_CH1);
    }
}

void AppDiagnostics_TaskFSMLogUpdate(void)
{
    Mission_Info_t info;
    char line[360];
    int length;

    if (TaskFSM_GetInfo(&info) != BSP_OK) {
        return;
    }

    length = snprintf(
        line,
        sizeof(line),
        "MISSION mode=%u state=%u sub=%u result=%u ready=%u/%u/%u "
        "run=%u fin=%u tgt=%d custom=%d pos=%d err=%d max=%d "
        "time=%lu score=%lu safe=%lu events=%02lX fault=%u ff=%u\r\n",
        (unsigned int)info.mode,
        (unsigned int)info.state,
        (unsigned int)info.substate,
        (unsigned int)info.result,
        (unsigned int)info.armed_ready,
        (unsigned int)info.route_ready,
        (unsigned int)info.ball_ready,
        (unsigned int)info.running,
        (unsigned int)info.finished,
        (int)info.active_ball_target_mm_x10,
        (int)info.custom_ball_target_mm_x10,
        (int)info.current_ball_position_mm_x10,
        (int)info.current_ball_error_mm_x10,
        (int)info.max_ball_error_mm_x10,
        (unsigned long)info.elapsed_ms,
        (unsigned long)info.score_limit_ms,
        (unsigned long)info.safety_timeout_ms,
        (unsigned long)info.route_events,
        (unsigned int)info.fault_code,
        (unsigned int)info.vehicle_feedforward_enabled);

    if ((length > 0) && (length < (int)sizeof(line))) {
        (void)BSP_UART_WriteFrame(DEBUG_UART_PORT,
                                  (const uint8_t *)line,
                                  (uint16_t)length);
    }
}
