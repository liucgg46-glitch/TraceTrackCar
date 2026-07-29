#ifndef __APP_TASK_CONFIG_H
#define __APP_TASK_CONFIG_H

#include "scheduler.h"
#include "app_task_port.h"
#include "app_diagnostics.h"
#include "task_profile_select.h"
#include "line_calibration.h"
#include "test.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 2026通信系电赛模拟赛B题基础巡线正式任务表
 * ============================================================================
 *
 * 这里只注册基础要求第（1）项实际需要的周期任务：
 *   - KEY1启动、KEY4停止；
 *   - 灰度采样、编码器反馈；
 *   - APP整车任务状态机；
 *   - Route巡线推进和底盘闭环。
 *
 * 不注册Test日志、K210、称重业务和LCD/OLED刷新；
 * Motion_Update用于转弯中心补偿、定角转弯和虚线航向保持，属于必要任务。
 */
#define APP_SCHEDULER_TASK_LIST_DEFINE()                                            \
Task_t task_list[] = {                                                              \
    { AppDiagnostics_HeartbeatUpdate, 10U, 0U }, /* 运行心跳 */         \
    { AppTask_BSP_Background, 1U, 0U }, /* UART、总线和异步驱动后台 */        \
    { Key_Update, 10U, 0U }, /* 按键扫描 */                             \
    { Sensor_Update, 1U, 0U }, /* 灰度、IMU和测距 */                      \
    { Encoder_Update, 10U, 0U }, /* 速度反馈 */                         \
    { Test_RouteCmd_Update, 10U, 0U }, /* 路线启停与复位 */                \
    { LineTrack_Update, 10U, 0U }, /* 循迹和路线推进 */                    \
    { Motion_Update, 10U, 0U }, /* 路线动作 */                          \
    { Chassis_Update, 10U, 0U }, /* 底盘闭环 */                         \
    { Test_RouteLog, 200U, 0U }, /* 激活路线页面 */                       \
    { LCD_Update, 20U, 0U }, /* 刷新 LCD */                           \
    { OLED_Update, 20U, 0U }, /* 刷新 OLED */                         \
};                                                                                  \
const uint8_t TASK_NUM =                                                             \
    (uint8_t)(sizeof(task_list) / sizeof(task_list[0]))

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_CONFIG_H */
