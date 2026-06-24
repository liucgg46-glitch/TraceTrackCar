#ifndef __MOTION_ACTION_H
#define __MOTION_ACTION_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 非阻塞底盘动作库：motion_action
 * ============================================================================
 * 定位：把“直走指定距离、转指定角度、停止”等常用动作封装成状态机。
 *
 * 重要规则：
 *   - Motion_GoDistance() / Motion_TurnAngle() 只启动动作，不等待完成；
 *   - Motion_Update() 必须 10ms 周期调用；
 *   - 任务状态机只查询 Motion_IsDone()，不要直接盯编码器细节；
 *   - 当前 TurnAngle 默认用编码器差速估算 yaw，后续可切换 IMU yaw。
 */

typedef enum {
    MOTION_IDLE = 0,
    MOTION_RUNNING,
    MOTION_DONE,
    MOTION_ERROR
} MotionState_t;

typedef enum {
    MOTION_ACTION_NONE = 0,
    MOTION_ACTION_GO_DISTANCE,
    MOTION_ACTION_TURN_ANGLE
} MotionAction_t;

typedef struct {
    MotionState_t state;
    MotionAction_t action;
    int32_t target_distance_mm;
    int16_t target_angle_deg;
    int16_t speed_cps;
    int32_t current_distance_mm;
    float current_yaw_deg;
    uint32_t start_time_ms;
    uint32_t timeout_ms;
} Motion_Info_t;

#define MOTION_DISTANCE_TOLERANCE_MM      8
#define MOTION_ANGLE_TOLERANCE_DEG        3.0f
#define MOTION_DEFAULT_TIMEOUT_MS         8000U
#define MOTION_MIN_ABS_SPEED_CPS          150
#define MOTION_MAX_ABS_SPEED_CPS          2500

void Motion_Init(void);
void Motion_Update(void);

BSP_Status_t Motion_GoDistance(int32_t distance_mm, int16_t speed_cps);
BSP_Status_t Motion_TurnAngle(int16_t angle_deg, int16_t speed_cps);
void Motion_Stop(void);

uint8_t Motion_IsBusy(void);
uint8_t Motion_IsDone(void);
MotionState_t Motion_GetState(void);
BSP_Status_t Motion_GetInfo(Motion_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_ACTION_H */
