#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "bsp_common.h"
#include "pid.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 差速底盘速度闭环：chassis
 * ============================================================================
 * 定位：根据 linear_speed + turn_speed 生成左右目标速度，再用四个轮子的
 * 速度 PID 输出四路电机 PWM。
 *
 * 本模块不直接读 TIM、不直接操作 GPIO 引脚，只调用：
 *   - drv_encoder：获取四轮反馈速度；
 *   - drv_motor：输出四轮 PWM；
 *   - pid：计算速度闭环。
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

/* 默认速度环 PID。刚开始建议只用 Kp，Ki/Kd 先为 0。 */
#define CHASSIS_SPEED_PID_KP          0.8f
#define CHASSIS_SPEED_PID_KI          0.0f
#define CHASSIS_SPEED_PID_KD          0.0f
#define CHASSIS_SPEED_PID_OUT_MAX     800.0f
#define CHASSIS_SPEED_PID_I_MAX       300.0f

/* 目标速度限幅，单位 count/s。 */
#define CHASSIS_TARGET_CPS_MAX        3000

void         Chassis_Init(void);
void         Chassis_SetSpeed(int16_t linear_speed_cps, int16_t turn_speed_cps);
void         Chassis_Stop(void);
void         Chassis_ResetPID(void);
void         Chassis_SetPIDConfig(const PID_Config_t *cfg);
Chassis_Mode_t Chassis_GetMode(void);
BSP_Status_t Chassis_GetInfo(Chassis_Info_t *info);

/* 覆盖 app_task_port.c 里的弱函数，任务表仍然使用 Chassis_Update。 */
void Chassis_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_H */
