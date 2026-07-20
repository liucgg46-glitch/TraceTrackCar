#ifndef __ROUTE_COMMON_H
#define __ROUTE_COMMON_H

#include "bsp_common.h"
#include "line_detect.h"
#include "line_track.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Route 只描述控制意图，底盘控制权由 APP 层统一仲裁。 */
typedef enum {
    ROUTE_CONTROL_STOP = 0,
    ROUTE_CONTROL_LINE_TRACK,
    ROUTE_CONTROL_MOTION,
    ROUTE_CONTROL_ERROR
} Route_ControlMode_t;

typedef enum {
    ROUTE_ACTION_NONE = 0,
    ROUTE_ACTION_GO_DISTANCE,
    ROUTE_ACTION_TURN_ANGLE
} Route_ActionType_t;

typedef enum {
    ROUTE_ACTION_STATE_IDLE = 0,
    ROUTE_ACTION_STATE_RUNNING,
    ROUTE_ACTION_STATE_DONE,
    ROUTE_ACTION_STATE_ERROR
} Route_ActionState_t;

typedef struct {
    Route_ActionState_t state;
} Route_ActionFeedback_t;

typedef struct {
    Route_ActionType_t type;
    int32_t distance_mm;
    int16_t angle_deg;
    int16_t speed_cps;
} Route_ActionRequest_t;

/* 各赛道方案向 RouteManager 提供的统一状态快照。 */
typedef struct {
    uint8_t state;
    uint16_t event_confirm_samples;
    uint32_t running_ms;
    uint32_t transition_count;
} RouteProfile_Info_t;

#ifdef __cplusplus
}
#endif

#endif /* __ROUTE_COMMON_H */
