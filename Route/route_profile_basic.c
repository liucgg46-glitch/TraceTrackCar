#include "route_profile_basic.h"

void BasicRoute_Init(void)
{
    BasicRoute_Reset();
}

void BasicRoute_Reset(void)
{
    /* Basic 没有赛道事件状态，LineTrack 由 RouteManager 统一复位。 */
}

Route_ControlMode_t BasicRoute_Update(const LineDetect_Result_t *line,
                                      const Route_ActionFeedback_t *feedback,
                                      LineTrack_Output_t *out,
                                      Route_ActionRequest_t *request)
{
    if ((line == 0) || (feedback == 0) ||
        (out == 0) || (request == 0)) {
        return ROUTE_CONTROL_ERROR;
    }

    (void)feedback;

    LineTrack_Compute(line, out);
    return (out->valid != 0U) ? ROUTE_CONTROL_LINE_TRACK
                              : ROUTE_CONTROL_STOP;
}

BSP_Status_t BasicRoute_GetInfo(RouteProfile_Info_t *info)
{
    if (info == 0) {
        return BSP_PARAM;
    }

    info->state = 0U;
    info->event_confirm_samples = 0U;
    info->running_ms = 0U;
    info->transition_count = 0U;
    return BSP_OK;
}
