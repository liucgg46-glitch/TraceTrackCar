#ifndef __ROUTE_COMMON_H
#define __ROUTE_COMMON_H

#include "project_status.h"
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

/* 去程和返程共用同一条赛道描述，由任务层在启动循迹前配置。 */
typedef enum {
    ROUTE_MISSION_OUTBOUND = 0,
    ROUTE_MISSION_RETURN
} Route_MissionDirection_t;

/* 视觉只向路线层提交相对车头的左右方向，不把K210协议耦合进路线模块。 */
typedef enum {
    ROUTE_VISUAL_NONE = 0,
    ROUTE_VISUAL_LEFT,
    ROUTE_VISUAL_RIGHT,
    ROUTE_VISUAL_STRAIGHT
} Route_VisualDirection_t;

typedef struct {
    Route_ActionState_t state;
    int32_t distance_mm;
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
    uint16_t event_confirm_samples;
    uint32_t running_ms;
    uint32_t transition_count;
} RouteProfile_Info_t;

#ifdef __cplusplus
}
#endif

#endif /* __ROUTE_COMMON_H */
