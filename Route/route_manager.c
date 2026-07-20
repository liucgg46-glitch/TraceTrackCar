#include "route_manager.h"
#include "route_config.h"
#include "route_profile_basic.h"
#include "route_profile_hjduino.h"
#include "motion_action.h"

static Route_ControlMode_t s_control_mode = ROUTE_CONTROL_STOP;

static void RouteManager_ProfileInit(void)
{
#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    BasicRoute_Init();
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_HJDUINO)
    HJduinoRoute_Init();
#else
#error "Invalid ROUTE_PROFILE_SELECT"
#endif
}

static void RouteManager_ProfileReset(void)
{
#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    BasicRoute_Reset();
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_HJDUINO)
    HJduinoRoute_Reset();
#else
#error "Invalid ROUTE_PROFILE_SELECT"
#endif
}

void RouteManager_Init(void)
{
    LineTrack_Init();
    RouteManager_ProfileInit();
    s_control_mode = ROUTE_CONTROL_STOP;
}

void RouteManager_Reset(void)
{
    /* Cancel every previous command owner before starting a new route run. */
    Motion_Stop();
    LineTrack_Reset();
    RouteManager_ProfileReset();
    s_control_mode = ROUTE_CONTROL_STOP;
}

Route_ControlMode_t RouteManager_Update(const LineDetect_Result_t *line,
                                        LineTrack_Output_t *out)
{
    if (out != 0) {
        out->linear_cps = 0;
        out->turn_cps = 0;
        out->valid = 0U;
    }

    if ((line == 0) || (out == 0)) {
        s_control_mode = ROUTE_CONTROL_ERROR;
        return s_control_mode;
    }

#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    s_control_mode = BasicRoute_Update(line, out);
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_HJDUINO)
    s_control_mode = HJduinoRoute_Update(line, out);
#else
#error "Invalid ROUTE_PROFILE_SELECT"
#endif

    return s_control_mode;
}

BSP_Status_t RouteManager_GetInfo(RouteManager_Info_t *info)
{
    LineTrack_Info_t line_info;

    if (info == 0) return BSP_PARAM;
    if (LineTrack_GetInfo(&line_info) != BSP_OK) return BSP_ERROR;

    info->profile = (uint8_t)ROUTE_PROFILE_SELECT;
    info->profile_state = 0U;
    info->control_mode = s_control_mode;
    info->motion_state = (uint8_t)Motion_GetState();
    info->event_confirm_samples = 0U;
    info->running_ms = 0U;
    info->transition_count = 0U;
    info->line_track_mode = line_info.mode;
    info->line_filtered_error = line_info.filtered_error;
    info->line_derivative_error = line_info.derivative_error;
    info->line_adaptive_linear_cps = line_info.adaptive_linear_cps;
    info->line_turn_unclamped_cps = line_info.turn_unclamped_cps;
    info->line_turn_saturated = line_info.turn_saturated;
    info->line_lost_samples = line_info.lost_samples;
    info->line_search_phase = line_info.search_phase;
    info->line_search_direction = line_info.search_direction;
    info->line_lost_ms = line_info.lost_ms;

#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_HJDUINO)
    {
        HJduinoRoute_Info_t route_info;
        if (HJduinoRoute_GetInfo(&route_info) != BSP_OK) return BSP_ERROR;
        info->profile_state = (uint8_t)route_info.state;
        info->event_confirm_samples = route_info.entry_confirm_samples;
        info->running_ms = route_info.running_ms;
        info->transition_count = route_info.transition_count;
    }
#endif

    return BSP_OK;
}
