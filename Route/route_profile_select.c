#include "route_profile_select.h"
#include "route_config.h"

#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
#include "route_profile_basic.h"
#else
#error "Invalid ROUTE_PROFILE_SELECT: add the selected profile adapter in route_profile_select.c"
#endif

void RouteProfile_Init(void)
{
#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    BasicRoute_Init();
#endif
}

void RouteProfile_Reset(void)
{
#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    BasicRoute_Reset();
#endif
}

Route_ControlMode_t RouteProfile_Update(
    const LineDetect_Result_t *line,
    const Route_ActionFeedback_t *feedback,
    LineTrack_Output_t *out,
    Route_ActionRequest_t *request)
{
#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    return BasicRoute_Update(line, feedback, out, request);
#endif
}

BSP_Status_t RouteProfile_GetInfo(RouteProfile_Info_t *info)
{
#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    return BasicRoute_GetInfo(info);
#endif
}
