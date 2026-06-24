#ifndef __PID_H
#define __PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 通用 PID 算法模块
 * ============================================================================
 * 定位：只负责 PID 数学计算，不直接访问电机、编码器、GPIO、PWM。
 *
 * 使用原则：
 *   1. 每个被控对象一份 PID_t，例如四个电机速度环各一份；
 *   2. PID_Update() 必须在固定周期调用，例如 10ms；
 *   3. 输出限幅和积分限幅必须配置，避免电机突变和积分饱和；
 *   4. 如果目标速度突变、停车、切换模式，建议 PID_Reset()。
 */

typedef struct {
    float kp;
    float ki;
    float kd;

    float out_min;       /* 输出下限 */
    float out_max;       /* 输出上限 */
    float integral_min;  /* 积分项下限 */
    float integral_max;  /* 积分项上限 */
} PID_Config_t;

typedef struct {
    PID_Config_t cfg;

    float target;
    float feedback;
    float error;
    float last_error;
    float integral;
    float derivative;
    float output;

    uint8_t first_update;
} PID_t;

void  PID_Init(PID_t *pid, const PID_Config_t *cfg);
void  PID_Reset(PID_t *pid);
void  PID_SetConfig(PID_t *pid, const PID_Config_t *cfg);
void  PID_SetTarget(PID_t *pid, float target);
float PID_Update(PID_t *pid, float feedback);
float PID_GetOutput(const PID_t *pid);
float PID_GetError(const PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
