#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 差速底盘开环 PWM 控制：chassis
 * ============================================================================
 * 定位：根据 linear_speed + turn_speed 生成左右目标值，再按固定比例直接
 * 换算为四路电机 PWM，不使用编码器速度反馈修正。
 *
 * 本模块不直接读 TIM、不直接操作 GPIO 引脚，只调用：
 *   - drv_encoder：保留四轮速度监测和里程累计；
 *   - drv_motor：输出四轮 PWM。
 *
 * 使用约定：
 *   - Chassis_Update() 固定 10ms 调用；
 *   - Chassis_SetSpeed(linear, turn) 只设置目标，不阻塞等待；
 *   - 所有速度单位默认用 count/s，先便于调车；后续可切换到 mm/s。
 */

typedef enum {
    CHASSIS_MODE_STOP = 0,
    CHASSIS_MODE_SPEED
} Chassis_Mode_t;

typedef struct {
    Chassis_Mode_t mode;
    int16_t linear_target_cps;
    int16_t turn_target_cps;
    int16_t left_target_cps;
    int16_t right_target_cps;
    int32_t fl_feedback_cps;
    int32_t fr_feedback_cps;
    int32_t rl_feedback_cps;
    int32_t rr_feedback_cps;
    int16_t fl_output;
    int16_t fr_output;
    int16_t rl_output;
    int16_t rr_output;
} Chassis_Info_t;

void         Chassis_Init(void);
void         Chassis_SetSpeed(int16_t linear_speed_cps, int16_t turn_speed_cps);
void         Chassis_Stop(void);
Chassis_Mode_t Chassis_GetMode(void);
BSP_Status_t Chassis_GetInfo(Chassis_Info_t *info);

/* 覆盖 app_task_port.c 里的弱函数，任务表仍然使用 Chassis_Update。 */
void Chassis_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_H */
