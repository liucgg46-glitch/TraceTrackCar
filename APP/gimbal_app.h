#ifndef __GIMBAL_APP_H
#define __GIMBAL_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 云台业务状态机只负责轨迹和激光时序，不直接访问 PWM/GPIO。
 * Gimbal_Update() 需要由统一任务表每 20 ms 调用一次。
 */
typedef enum {
    GIMBAL_APP_IDLE = 0,
    GIMBAL_APP_SQUARE_TEST,
    GIMBAL_APP_STOP
} GimbalApp_State_t;

void GimbalApp_Init(void);
void GimbalApp_StartSquareTest(void);
void GimbalApp_Stop(void);
void GimbalApp_Update(void);

/* 统一任务端口名称，由 app_task_config.h 注册。 */
void Gimbal_Update(void);

GimbalApp_State_t GimbalApp_GetState(void);
uint8_t GimbalApp_GetSquareSegment(void);

#ifdef __cplusplus
}
#endif

#endif /* __GIMBAL_APP_H */
