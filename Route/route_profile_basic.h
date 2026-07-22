#ifndef __ROUTE_PROFILE_BASIC_H
#define __ROUTE_PROFILE_BASIC_H

#include "route_common.h"

#ifdef __cplusplus
extern "C" {
#endif

void BasicRoute_Init(uint32_t now_ms);
void BasicRoute_Reset(uint32_t now_ms);
Project_Status_t BasicRoute_ConfigureMission(
    uint8_t target_room,
    Route_MissionDirection_t direction,
    uint32_t now_ms);
Project_Status_t BasicRoute_SubmitVisualDecision(
    Route_VisualDirection_t direction);
Route_ControlMode_t BasicRoute_Update(const LineDetect_Result_t *line,
                                      const Route_ActionFeedback_t *feedback,
                                      LineTrack_Output_t *out,
                                      Route_ActionRequest_t *request,
                                      uint32_t now_ms);
Project_Status_t BasicRoute_GetInfo(RouteProfile_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __ROUTE_PROFILE_BASIC_H */
