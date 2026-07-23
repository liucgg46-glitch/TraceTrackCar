#include "app_diagnostics.h"

#include "bsp_gpio.h"
#include "bsp_systick.h"
#include "bsp_uart.h"
#include "chassis.h"
#include "drv_gray_sensor.h"
#include "line_follow_app.h"
#include "task_fsm.h"

#include <stdint.h>
#include <stdio.h>

static void AppDiagnostics_FormatWeight(char *out,
                                        uint16_t out_size,
                                        float weight_g)
{
    long scaled;
    unsigned long absolute_value;
    unsigned long integer_part;
    unsigned long fraction_part;
    char sign;

    if ((out == 0) || (out_size == 0U)) {
        return;
    }

    scaled = (long)((weight_g * 10.0f) +
                    ((weight_g >= 0.0f) ? 0.5f : -0.5f));
    sign = (scaled < 0L) ? '-' : '+';
    absolute_value = (scaled < 0L) ?
                     (unsigned long)(-(scaled + 1L)) + 1UL :
                     (unsigned long)scaled;
    integer_part = absolute_value / 10UL;
    fraction_part = absolute_value % 10UL;

    (void)snprintf(out,
                   out_size,
                   "%c%lu.%01lu",
                   sign,
                   integer_part,
                   fraction_part);
}

void AppDiagnostics_HeartbeatUpdate(void)
{
    static uint32_t last_toggle_ms = 0U;

    if (BSP_TimeElapsed(&last_toggle_ms, 500U) != 0U) {
        BSP_GPIO_Toggle(BSP_GPIO_CH1);
    }
}

void AppDiagnostics_TaskFSMLogUpdate(void)
{
    TaskFSM_Info_t info;
    char weight_text[24];
    const char *display_weight;
    char line[360];
    int length;

    if (TaskFSM_GetInfo(&info) != BSP_OK) {
        return;
    }

    AppDiagnostics_FormatWeight(weight_text, sizeof(weight_text), info.weight_g);
    display_weight = (weight_text[0] == '+') ? &weight_text[1] : weight_text;

    length = snprintf(
        line,
        sizeof(line),
        "TASK st=%u fault=%u target=%u lock=%u obs=%u cf=%u side=%u/x%u vf=%u "
        "k210=%u weight=%s valid=%u load=%u empty=%u "
        "start=%u lf=%u gray=%u owner=%u cmode=%u cfault=%u "
        "route=%u approach=%u vision=%u/%u wait=%u cross=%u decisions=%u "
        "arrived=%u stop=%u light=%u "
        "elapsed=%lu trans=%lu\r\n",
        (unsigned int)info.state,
        (unsigned int)info.fault,
        (unsigned int)info.target_room,
        (unsigned int)info.target_locked,
        (unsigned int)info.observed_digit,
        (unsigned int)info.target_confirm_frames,
        (unsigned int)info.vision_observed_side,
        (unsigned int)info.vision_center_x,
        (unsigned int)info.vision_confirm_frames,
        (unsigned int)info.k210_online,
        display_weight,
        (unsigned int)info.weight_valid,
        (unsigned int)info.load_state,
        (unsigned int)info.empty_seen,
        (unsigned int)info.route_start_status,
        (unsigned int)LineFollow_GetState(),
        (unsigned int)Drv_GraySensor_IsOnline(),
        (unsigned int)Chassis_GetOwner(),
        (unsigned int)Chassis_GetMode(),
        (unsigned int)Chassis_GetFault(),
        (unsigned int)info.route_state,
        (unsigned int)info.route_approach_ready,
        (unsigned int)info.route_visual_stage,
        (unsigned int)info.route_visual_ready,
        (unsigned int)info.route_waiting_visual,
        (unsigned int)info.route_intersections,
        (unsigned int)info.route_decisions,
        (unsigned int)info.route_arrived,
        (unsigned int)info.stop_confirmed,
        (unsigned int)info.status_light,
        (unsigned long)info.state_elapsed_ms,
        (unsigned long)info.transition_count);

    if ((length > 0) && (length < (int)sizeof(line))) {
        (void)BSP_UART_WriteFrame(UART_PORT1,
                                  (const uint8_t *)line,
                                  (uint16_t)length);
    }
}
