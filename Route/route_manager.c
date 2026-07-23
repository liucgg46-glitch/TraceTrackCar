#include "route_manager.h"
#include "route_config.h"
#include "route_profile_select.h"

static Route_ControlMode_t s_control_mode = ROUTE_CONTROL_STOP;
static Route_ActionState_t s_action_state = ROUTE_ACTION_STATE_IDLE;

void RouteManager_Init(uint32_t now_ms)
{
    LineTrack_Init();
    RouteProfile_Init(now_ms);
    s_control_mode = ROUTE_CONTROL_STOP;
    s_action_state = ROUTE_ACTION_STATE_IDLE;
}

void RouteManager_Reset(uint32_t now_ms)
{
    LineTrack_Reset();
    RouteProfile_Reset(now_ms);
    s_control_mode = ROUTE_CONTROL_STOP;
    s_action_state = ROUTE_ACTION_STATE_IDLE;
}

Project_Status_t RouteManager_ConfigureMission(
    uint8_t target_room,
    Route_MissionDirection_t direction,
    uint32_t now_ms)
{
    if (s_control_mode != ROUTE_CONTROL_STOP) {
        return PROJECT_BUSY;
    }

    return RouteProfile_ConfigureMission(target_room, direction, now_ms);
}

Project_Status_t RouteManager_SubmitVisualDecision(
    Route_VisualDirection_t direction)
{
    return RouteProfile_SubmitVisualDecision(direction);
}

Route_ControlMode_t RouteManager_Update(const LineDetect_Result_t *line,
                                        const Route_ActionFeedback_t *feedback,
                                        LineTrack_Output_t *out,
                                        Route_ActionRequest_t *request,
                                        uint32_t now_ms)
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

    s_control_mode = RouteProfile_Update(line,
                                         feedback,
                                         out,
                                         request,
                                         now_ms);

    return s_control_mode;
}

Project_Status_t RouteManager_GetInfo(RouteManager_Info_t *info)
{
    LineTrack_Info_t line_info;
    RouteProfile_Info_t profile_info;

    if (info == 0) return PROJECT_PARAM;
    if (LineTrack_GetInfo(&line_info) != PROJECT_OK) return PROJECT_ERROR;
    if (RouteProfile_GetInfo(&profile_info) != PROJECT_OK) return PROJECT_ERROR;

    info->profile = (uint8_t)ROUTE_PROFILE_SELECT;
    info->profile_state = profile_info.state;
    info->configured = profile_info.configured;
    info->target_room = profile_info.target_room;
    info->direction = profile_info.direction;
    info->room_approach_ready = profile_info.room_approach_ready;
    info->visual_stage = profile_info.visual_stage;
    info->visual_decision_ready = profile_info.visual_decision_ready;
    info->waiting_visual = profile_info.waiting_visual;
    info->intersection_count = profile_info.intersection_count;
    info->decisions_completed = profile_info.decisions_completed;
    info->arrived = profile_info.arrived;
    info->error = profile_info.error;
    info->control_mode = s_control_mode;
    info->action_state = s_action_state;
    info->event_confirm_samples = profile_info.event_confirm_samples;
    info->running_ms = profile_info.running_ms;
    info->transition_count = profile_info.transition_count;
    info->line_track_mode = line_info.mode;
    info->line_filtered_error = line_info.filtered_error;
    info->line_lost_samples = line_info.lost_samples;
    info->line_reacquire_samples = line_info.reacquire_samples;
    info->line_search_phase = line_info.search_phase;
    info->line_search_direction = line_info.search_direction;
    info->line_lost_ms = line_info.lost_ms;

    return PROJECT_OK;
}
