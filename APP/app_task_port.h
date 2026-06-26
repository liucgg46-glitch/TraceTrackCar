#ifndef __APP_TASK_PORT_H
#define __APP_TASK_PORT_H

#include <stdint.h>
#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * A/B/C 三人对接任务接口
 * ============================================================================
 * 这个文件只定义“谁需要提供什么 Update 函数”。
 *
 * 规则：
 *   1. 函数名一旦被 task_list[] 使用，就不要随便改名；
 *   2. 所有 Update 都必须非阻塞；
 *   3. Part1 里提供了弱函数默认实现，保证 BSP 阶段能先独立编译；
 *   4. 后续 A/B/C 在自己的模块 .c 文件里写同名强函数，即可自动覆盖弱实现。
 *
 * 推荐责任分工：
 *   B：BSP、Encoder_Update、电机底层；
 *   A：Chassis_Update、LineTrack_Update、TaskFSM_Update、Motion_Update；
 *   C：Sensor_Update、DebugMenu_Update、OLED_Update、Log_Update。
 */

void AppTask_BSP_Background(void);  /* B：UART/I2C/SPI 后台维护，1ms */
void Encoder_Update(void);          /* B：编码器增量/速度更新，10ms */
void Sensor_Update(void);           /* C：灰度/IMU/测距/视觉统一更新，建议1ms */
void Chassis_Update(void);          /* A：底盘速度闭环，10ms */
void Motion_Update(void);           /* A：底盘动作库，10ms */
void LineTrack_Update(void);        /* A：循迹控制，10ms */
void TaskFSM_Update(void);          /* A：总任务状态机，10ms */
void DebugMenu_Update(void);        /* C：按键/OLED 调参菜单，20ms */
void OLED_Update(void);             /* C：OLED 页面刷新，100ms */
void LCD_Update(void);              /* C：TFT LCD 页面刷新，建议100~200ms */
void Log_Update(void);              /* C：串口日志输出，200ms */

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_PORT_H */
