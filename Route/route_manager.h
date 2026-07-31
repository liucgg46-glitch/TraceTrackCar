#ifndef __ROUTE_MANAGER_H
#define __ROUTE_MANAGER_H

#include "route_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t profile;
    uint8_t profile_state;
    uint8_t configured;
    uint8_t target_room;
    uint8_t direction;
    uint8_t room_approach_ready;
    uint8_t visual_stage;
    uint8_t visual_decision_ready;
    uint8_t waiting_visual;
    uint8_t intersection_count;
    uint8_t decisions_completed;
    uint8_t arrived;
    uint8_t error;
    Route_ControlMode_t control_mode;
    Route_ActionState_t action_state;
    uint16_t event_confirm_samples;
    uint32_t running_ms;
    uint32_t transition_count;
    LineTrack_Mode_t line_track_mode;
    int16_t line_filtered_error;
    uint16_t line_lost_samples;
    uint16_t line_reacquire_samples;
    uint16_t line_search_phase;
    int8_t line_search_direction;
    uint32_t line_lost_ms;
} RouteManager_Info_t;

typedef enum {
    ROUTE_EVENT_NONE         = 0x00U,
    ROUTE_EVENT_LEFT_A       = 0x01U,
    ROUTE_EVENT_PASSED_B     = 0x02U,
    ROUTE_EVENT_LAP_COMPLETE = 0x04U,
    ROUTE_EVENT_LINE_LOST    = 0x08U,
    ROUTE_EVENT_ACTION_ERROR = 0x10U,
    ROUTE_EVENT_FAILED       = 0x20U
} Route_Event_t;

void RouteManager_Init(uint32_t now_ms);
void RouteManager_Reset(uint32_t now_ms);
Project_Status_t RouteManager_ConfigureMission(
    uint8_t target_room,
    Route_MissionDirection_t direction,
    uint32_t now_ms);
Project_Status_t RouteManager_SubmitVisualDecision(
    Route_VisualDirection_t direction);
Route_ControlMode_t RouteManager_Update(const LineDetect_Result_t *line,
                                        const Route_ActionFeedback_t *feedback,
                                        LineTrack_Output_t *out,
                                        Route_ActionRequest_t *request,
                                        uint32_t now_ms);
Project_Status_t RouteManager_GetInfo(RouteManager_Info_t *info);
uint32_t RouteManager_GetEvents(void);
void RouteManager_ClearEvents(uint32_t events);

#ifdef __cplusplus
}
#endif

#endif /* __ROUTE_MANAGER_H */
