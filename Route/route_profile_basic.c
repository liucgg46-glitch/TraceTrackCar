#include "route_profile_basic.h"

void BasicRoute_Init(void)
{
    BasicRoute_Reset();
}

void BasicRoute_Reset(void)
{
    /* Basic has no track-event state. LineTrack is reset by RouteManager. */
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
