#include "route_profile_select.h"
#include "route_config.h"

#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
#include "route_profile_basic.h"
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_MEDICINE)
#include "route_profile_medicine.h"
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_B_BASIC)
#include "route_profile_b_basic.h"
#else
#error "Invalid ROUTE_PROFILE_SELECT: add the selected profile adapter in route_profile_select.c"
#endif

void RouteProfile_Init(uint32_t now_ms)
{
#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    BasicRoute_Init(now_ms);
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_MEDICINE)
    MedicineRoute_Init(now_ms);
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_B_BASIC)
    BRoute_Init(now_ms);
#endif
}

void RouteProfile_Reset(uint32_t now_ms)
{
#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    BasicRoute_Reset(now_ms);
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_MEDICINE)
    MedicineRoute_Reset(now_ms);
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_B_BASIC)
    BRoute_Reset(now_ms);
#endif
}

Project_Status_t RouteProfile_ConfigureMission(
    uint8_t target_room,
    Route_MissionDirection_t direction,
    uint32_t now_ms)
{
#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    return BasicRoute_ConfigureMission(target_room, direction, now_ms);
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_MEDICINE)
    return MedicineRoute_ConfigureMission(target_room, direction, now_ms);
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_B_BASIC)
    return BRoute_ConfigureMission(target_room, direction, now_ms);
#endif
}

Project_Status_t RouteProfile_SubmitVisualDecision(
    Route_VisualDirection_t direction)
{
#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    return BasicRoute_SubmitVisualDecision(direction);
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_MEDICINE)
    return MedicineRoute_SubmitVisualDecision(direction);
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_B_BASIC)
    return BRoute_SubmitVisualDecision(direction);
#endif
}

Route_ControlMode_t RouteProfile_Update(
    const LineDetect_Result_t *line,
    const Route_ActionFeedback_t *feedback,
    LineTrack_Output_t *out,
    Route_ActionRequest_t *request,
    uint32_t now_ms)
{
#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    return BasicRoute_Update(line, feedback, out, request, now_ms);
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_MEDICINE)
    return MedicineRoute_Update(line, feedback, out, request, now_ms);
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_B_BASIC)
    return BRoute_Update(line, feedback, out, request, now_ms);
#endif
}

Project_Status_t RouteProfile_GetInfo(RouteProfile_Info_t *info)
{
#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    return BasicRoute_GetInfo(info);
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_MEDICINE)
    return MedicineRoute_GetInfo(info);
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_B_BASIC)
    return BRoute_GetInfo(info);
#endif
}
