#ifndef __ROUTE_MANAGER_H
#define __ROUTE_MANAGER_H

#include "route_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t profile;
    uint8_t profile_state;
    Route_ControlMode_t control_mode;
    Route_ActionState_t action_state;
    uint16_t event_confirm_samples;
    uint32_t running_ms;
    uint32_t transition_count;
    LineTrack_Mode_t line_track_mode;
    int16_t line_filtered_error;
    uint16_t line_lost_samples;
    uint16_t line_search_phase;
    int8_t line_search_direction;
    uint32_t line_lost_ms;
} RouteManager_Info_t;

void RouteManager_Init(void);
void RouteManager_Reset(void);
Route_ControlMode_t RouteManager_Update(const LineDetect_Result_t *line,
                                        const Route_ActionFeedback_t *feedback,
                                        LineTrack_Output_t *out,
                                        Route_ActionRequest_t *request);
BSP_Status_t RouteManager_GetInfo(RouteManager_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __ROUTE_MANAGER_H */
