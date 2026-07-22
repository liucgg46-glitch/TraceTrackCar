#include "route_profile_basic.h"

void BasicRoute_Init(uint32_t now_ms)
{
    BasicRoute_Reset(now_ms);
}

void BasicRoute_Reset(uint32_t now_ms)
{
    /* Basic 没有赛道事件状态，LineTrack 由 RouteManager 统一复位。 */
    (void)now_ms;
}

Project_Status_t BasicRoute_ConfigureMission(
    uint8_t target_room,
    Route_MissionDirection_t direction,
    uint32_t now_ms)
{
    (void)target_room;
    (void)direction;
    (void)now_ms;
    return PROJECT_ERROR;
}

Project_Status_t BasicRoute_SubmitVisualDecision(
    Route_VisualDirection_t direction)
{
    (void)direction;
    return PROJECT_ERROR;
}

Route_ControlMode_t BasicRoute_Update(const LineDetect_Result_t *line,
                                      const Route_ActionFeedback_t *feedback,
                                      LineTrack_Output_t *out,
                                      Route_ActionRequest_t *request,
                                      uint32_t now_ms)
{
    if ((line == 0) || (feedback == 0) ||
        (out == 0) || (request == 0)) {
        return ROUTE_CONTROL_ERROR;
    }

    (void)feedback;
    LineTrack_Compute(line, out, now_ms);
    return (out->valid != 0U) ? ROUTE_CONTROL_LINE_TRACK
                              : ROUTE_CONTROL_STOP;
}

Project_Status_t BasicRoute_GetInfo(RouteProfile_Info_t *info)
{
    if (info == 0) {
        return PROJECT_PARAM;
    }

    info->state = 0U;
    info->configured = 0U;
    info->target_room = 0U;
    info->direction = (uint8_t)ROUTE_MISSION_OUTBOUND;
    info->room_approach_ready = 0U;
    info->visual_stage = 0U;
    info->visual_decision_ready = 0U;
    info->waiting_visual = 0U;
    info->intersection_count = 0U;
    info->decisions_completed = 0U;
    info->arrived = 0U;
    info->error = 0U;
    info->event_confirm_samples = 0U;
    info->running_ms = 0U;
    info->transition_count = 0U;
    return PROJECT_OK;
}
