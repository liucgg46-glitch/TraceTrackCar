#ifndef __BALL_BALANCE_APP_H
#define __BALL_BALANCE_APP_H

#include "ball_balance_control.h"
#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * K210完成一帧钢球识别后：
 *
 * BallBalance_VisionSample_t sample;
 *
 * sample.position_mm_x10 = ball_position_mm_x10;
 * sample.valid = ball_detected;
 * sample.confidence = ball_confidence;
 * sample.sequence = ball_frame_sequence;
 * sample.timestamp_ms = BSP_GetTickMs();
 *
 * BallBalance_App_PushVisionSample(&sample);
 *
 * 注意：
 * 1. 像素坐标必须先转换成相对O点的毫米位置；
 * 2. 左侧为负、右侧为正；
 * 3. 每一帧只调用一次；
 * 4. 未检测到钢球时也应提交valid=0；
 * 5. PID模块不直接读取K210通信数据。
 */
typedef struct {
    int16_t position_mm_x10;
    uint8_t valid;
    uint8_t confidence;
    uint8_t sequence;
    uint32_t timestamp_ms;
} BallBalance_VisionSample_t;

typedef struct {
    uint8_t initialized;
    uint8_t enabled;
    uint8_t pending_sample;
    uint8_t last_sample_valid;
    uint8_t low_confidence;
    uint8_t position_out_of_range;
    uint8_t duplicate_sequence;
    uint8_t servo_fault;
    uint32_t pushed_sample_count;
    uint32_t consumed_sample_count;
    uint32_t rejected_sample_count;
    BallBalance_VisionSample_t last_sample;
    BallBalance_ControlInfo_t control;
    BSP_Status_t last_servo_status;
} BallBalance_AppInfo_t;

void BallBalance_App_Init(void);
void BallBalance_App_Update(void);

void BallBalance_App_Enable(void);
void BallBalance_App_Disable(void);

uint8_t BallBalance_App_IsEnabled(void);

void BallBalance_App_SetTargetMmX10(int16_t target_mm_x10);
void BallBalance_App_SetGains(float kp, float ki, float kd);

void BallBalance_App_PushVisionSample(
    const BallBalance_VisionSample_t *sample
);

BSP_Status_t BallBalance_App_GetInfo(BallBalance_AppInfo_t *info);

/*
 * 后续启用步骤：
 * 1. 系统初始化完成后调用BallBalance_App_Init();
 * 2. 调度器中以10ms周期调用BallBalance_App_Update();
 * 3. K210每完成一帧钢球位置计算后调用BallBalance_App_PushVisionSample();
 * 4. 测试开始时调用BallBalance_App_Enable();
 * 5. 停止时调用BallBalance_App_Disable();
 *
 * 本次框架不自动初始化、不注册10ms任务、不生成模拟位置。
 */

#ifdef __cplusplus
}
#endif

#endif /* __BALL_BALANCE_APP_H */
