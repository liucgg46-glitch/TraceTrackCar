#include "route_manager.h"
#include "route_config.h"
#include "route_profile_basic.h"
#include "route_profile_hjduino.h"

void RouteManager_Init(void)
{
#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    BasicRoute_Init();
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_HJDUINO)
    HJduinoRoute_Init();
#else
#error "Invalid ROUTE_PROFILE_SELECT"
#endif
}

void RouteManager_Reset(void)
{
#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    BasicRoute_Reset();
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_HJDUINO)
    HJduinoRoute_Reset();
#else
#error "Invalid ROUTE_PROFILE_SELECT"
#endif
}

void RouteManager_Update(const LineDetect_Result_t *line, LineTrack_Output_t *out)
{
#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    BasicRoute_Update(line, out);
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_HJDUINO)
    HJduinoRoute_Update(line, out);
#else
#error "Invalid ROUTE_PROFILE_SELECT"
#endif
}