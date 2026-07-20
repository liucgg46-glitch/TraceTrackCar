#ifndef __DRV_SERVO_H
#define __DRV_SERVO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 水平舵机：PF8 / TIM13_CH1
 *
 * 脉宽减小：向右
 * 脉宽增大：向左
 */
#define SERVO_HORIZONTAL_RIGHT_LIMIT_US    1000U
#define SERVO_HORIZONTAL_CENTER_US         1248U
#define SERVO_HORIZONTAL_LEFT_LIMIT_US     1500U

/*
 * 俯仰舵机：PF9 / TIM14_CH1
 *
 * 脉宽减小：抬头
 * 脉宽增大：低头
 */
#define SERVO_PITCH_UP_LIMIT_US            1200U
#define SERVO_PITCH_CENTER_US              1450U
#define SERVO_PITCH_DOWN_LIMIT_US          1830U

/*
 * 每次更新最多改变1us。
 * 后续每20ms调用一次Drv_Servo_Update()。
 */
#define SERVO_MOVE_STEP_US                  1U

void Drv_Servo_Init(void);
void Drv_Servo_Center(void);

/* 立即输出指定脉宽，主要用于测试和标定。 */
void Drv_Servo_SetHorizontalPulse(uint16_t pulse_us);
void Drv_Servo_SetPitchPulse(uint16_t pulse_us);

/* 设置非阻塞运动目标。 */
void Drv_Servo_SetTarget(uint16_t horizontal_target_us,
                         uint16_t pitch_target_us);
void Drv_Servo_Update(void);

uint16_t Drv_Servo_GetHorizontalPulse(void);
uint16_t Drv_Servo_GetPitchPulse(void);
uint16_t Drv_Servo_GetHorizontalTarget(void);
uint16_t Drv_Servo_GetPitchTarget(void);
uint8_t Drv_Servo_IsAtTarget(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_SERVO_H */