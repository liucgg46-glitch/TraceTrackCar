#include "route_manager.h"
#include "route_config.h"
#include "route_profile_select.h"

static Route_ControlMode_t s_control_mode = ROUTE_CONTROL_STOP;
static Route_ActionState_t s_action_state = ROUTE_ACTION_STATE_IDLE;

void RouteManager_Init(void)
{
    LineTrack_Init();
    RouteProfile_Init();
    s_control_mode = ROUTE_CONTROL_STOP;
    s_action_state = ROUTE_ACTION_STATE_IDLE;
}

void RouteManager_Reset(void)
{
    LineTrack_Reset();
    RouteProfile_Reset();
    s_control_mode = ROUTE_CONTROL_STOP;
    s_action_state = ROUTE_ACTION_STATE_IDLE;
}

Route_ControlMode_t RouteManager_Update(const LineDetect_Result_t *line,
                                        const Route_ActionFeedback_t *feedback,
                                        LineTrack_Output_t *out,
                                        Route_ActionRequest_t *request)
{
    if (out != 0) {
        out->linear_cps = 0;
        out->turn_cps = 0;
        out->valid = 0U;
    }
    if (request != 0) {
        request->type = ROUTE_ACTION_NONE;
        request->distance_mm = 0;
        request->angle_deg = 0;
        request->speed_cps = 0;
    }

    if ((line == 0) || (feedback == 0) ||
        (out == 0) || (request == 0)) {
        s_control_mode = ROUTE_CONTROL_ERROR;
        return s_control_mode;
    }

    s_action_state = feedback->state;

    s_control_mode = RouteProfile_Update(line, feedback, out, request);

    return s_control_mode;
}

BSP_Status_t RouteManager_GetInfo(RouteManager_Info_t *info)
{
    LineTrack_Info_t line_info;
    RouteProfile_Info_t profile_info;

    if (info == 0) return BSP_PARAM;
    if (LineTrack_GetInfo(&line_info) != BSP_OK) return BSP_ERROR;
    if (RouteProfile_GetInfo(&profile_info) != BSP_OK) return BSP_ERROR;

    info->profile = (uint8_t)ROUTE_PROFILE_SELECT;
    info->profile_state = profile_info.state;
    info->control_mode = s_control_mode;
    info->action_state = s_action_state;
    info->event_confirm_samples = profile_info.event_confirm_samples;
    info->running_ms = profile_info.running_ms;
    info->transition_count = profile_info.transition_count;
    info->line_track_mode = line_info.mode;
    info->line_filtered_error = line_info.filtered_error;
    info->line_lost_samples = line_info.lost_samples;
    info->line_search_phase = line_info.search_phase;
    info->line_search_direction = line_info.search_direction;
    info->line_lost_ms = line_info.lost_ms;

    return BSP_OK;
}
