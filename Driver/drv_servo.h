#ifndef __DRV_SERVO_H
#define __DRV_SERVO_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 二维舵机驱动层。
 *
 * 对上层只暴露归一化机械方向：
 *   水平 -1000 表示最右，0 表示中位，1000 表示最左；
 *   俯仰 -1000 表示最上，0 表示中位，1000 表示最下。
 *
 * 脉宽、机械限位和中位标定均由 drv_servo.c 私有保存，APP 不依赖 PWM 单位。
 */
#define DRV_SERVO_POSITION_MIN_PERMILLE    (-1000)
#define DRV_SERVO_POSITION_MAX_PERMILLE    1000

typedef struct {
    int16_t horizontal_permille;
    int16_t pitch_permille;
} Drv_Servo_Position_t;

typedef struct {
    Drv_Servo_Position_t current;
    Drv_Servo_Position_t target;
    uint16_t horizontal_pulse_us;
    uint16_t pitch_pulse_us;
    uint16_t horizontal_angle_x10;
    uint16_t horizontal_target_angle_x10;
    /* 仅表示软件输出命令已到达目标，不代表舵机具有位置反馈。 */
    uint8_t command_reached;
} Drv_Servo_Info_t;

void Drv_Servo_Init(void);

/* 由 Driver_Task() 高频调用，内部按固定周期推进非阻塞缓动。 */
void Drv_Servo_Task(void);

/* 设置非阻塞目标，由 Drv_Servo_Task() 逐步移动。 */
BSP_Status_t Drv_Servo_SetTargetPosition(
    const Drv_Servo_Position_t *position
);

/*
 * 立即输出归一化位置，供已经自行生成轨迹的控制模块使用。
 * 本接口仍执行归一化限位与脉宽限位，上层不能绕过 Driver 直接访问 BSP。
 */
BSP_Status_t Drv_Servo_SetImmediatePosition(
    const Drv_Servo_Position_t *position
);

/*
 * PF8水平舵机直接输出接口。
 * 微秒接口用于底层标定，角度接口用于现场测试和上层业务表达。
 */
BSP_Status_t Drv_Servo_SetHorizontalPulseUs(uint16_t pulse_us);
uint16_t Drv_Servo_GetHorizontalPulseUs(void);
BSP_Status_t Drv_Servo_SetHorizontalAngleDeg(uint16_t angle_deg);
uint16_t Drv_Servo_GetHorizontalAngleDeg(void);
BSP_Status_t Drv_Servo_SetHorizontalAngleX10(uint16_t angle_x10);
BSP_Status_t Drv_Servo_SetHorizontalTargetAngleX10(uint16_t angle_x10);
uint16_t Drv_Servo_GetHorizontalAngleX10(void);

void Drv_Servo_Center(void);
BSP_Status_t Drv_Servo_GetInfo(Drv_Servo_Info_t *info);
uint8_t Drv_Servo_IsCommandReached(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_SERVO_H */
