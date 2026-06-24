#ifndef __DRV_MOTOR_H
#define __DRV_MOTOR_H

#include "bsp_common.h"
#include "bsp_pwm.h"
#include "bsp_gpio.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 四电机驱动层：drv_motor
 * ============================================================================
 * 定位：负责“某个电机如何根据正负 PWM 转动”。
 *
 * 分层边界：
 *   - BSP_PWM：只输出 PWM；
 *   - BSP_GPIO：只控制方向脚；
 *   - drv_motor：把 PWM + 方向脚组合成四个电机；
 *   - chassis：根据速度 PID 决定每个电机输出多少。
 *
 * 移植方法：
 *   只改本文件配置区：电机和 PWM/GPIO 的映射、正反方向、最大输出。
 *   drv_motor.c 不需要改。
 */

typedef enum {
    MOTOR_FL = 0,     /* Front Left  左前 */
    MOTOR_FR,         /* Front Right 右前 */
    MOTOR_RL,         /* Rear Left   左后 */
    MOTOR_RR,         /* Rear Right  右后 */
    MOTOR_COUNT
} Motor_Id_t;

/* 当前引脚规划 v1.4：TIM8 PC6~PC9 PWM；PD0~PD9 中部分脚作方向。 */
#define MOTOR_FL_PWM_ID          BSP_PWM_CH1
#define MOTOR_FR_PWM_ID          BSP_PWM_CH2
#define MOTOR_RL_PWM_ID          BSP_PWM_CH3
#define MOTOR_RR_PWM_ID          BSP_PWM_CH4

#define MOTOR_FL_IN1_GPIO        BSP_GPIO_CH3   /* PD0 */
#define MOTOR_FL_IN2_GPIO        BSP_GPIO_CH4   /* PD1 */
#define MOTOR_FR_IN1_GPIO        BSP_GPIO_CH5   /* PD2 */
#define MOTOR_FR_IN2_GPIO        BSP_GPIO_CH6   /* PD3 */
#define MOTOR_RL_IN1_GPIO        BSP_GPIO_CH7   /* PD4 */
#define MOTOR_RL_IN2_GPIO        BSP_GPIO_CH8   /* PD7 */
#define MOTOR_RR_IN1_GPIO        BSP_GPIO_CH9   /* PD8 */
#define MOTOR_RR_IN2_GPIO        BSP_GPIO_CH10  /* PD9 */

/* 某个电机方向反了，优先改这里，不要在 chassis 里取反。 */
#define MOTOR_FL_REVERSE         0
#define MOTOR_FR_REVERSE         0
#define MOTOR_RL_REVERSE         0
#define MOTOR_RR_REVERSE         0

/* 输出单位是 permille：-1000~1000，对应 -100%~100% 占空比。 */
#define MOTOR_MAX_ABS_PERMILLE   1000

void         Motor_Init(void);
void         Motor_SetPermille(Motor_Id_t id, int16_t pwm_permille);
void         Motor_SetAllPermille(int16_t fl, int16_t fr, int16_t rl, int16_t rr);
void         Motor_StopOne(Motor_Id_t id);
void         Motor_StopAll(void);
int16_t      Motor_GetLastPermille(Motor_Id_t id);
BSP_Status_t Motor_GetAllLastPermille(int16_t out_pwm[MOTOR_COUNT]);

/* 兼容你流程里的简化接口：左右同侧输出相同 PWM。 */
void Motor_SetPWM(int16_t left_pwm, int16_t right_pwm);
void Motor_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_MOTOR_H */
