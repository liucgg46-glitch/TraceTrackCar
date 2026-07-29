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
 * 2026年电赛H题第2项正式任务表
 * ============================================================================
 *
 * KEY1启动、KEY4急停；灰度和编码器后台先更新，APP任务状态机随后处理
 * 计时与停车，Route/LineTrack和底盘闭环均保持10ms非阻塞周期。
 * LCD异步状态机以20ms推进，显示文本由H2任务每100ms更新。
 */
#define APP_SCHEDULER_TASK_LIST_DEFINE()                                            \
Task_t task_list[] = {                                                              \
    { AppDiagnostics_HeartbeatUpdate, 10U, 0U }, /* 运行心跳 */                    \
    { AppTask_BSP_Background, 1U, 0U }, /* UART、总线和异步驱动后台 */             \
    { Key_Update, 10U, 0U }, /* 按键扫描 */                                        \
    { Sensor_Update, 1U, 0U }, /* 灰度、IMU和测距 */                                 \
    { Encoder_Update, 10U, 0U }, /* 速度反馈 */                                    \
    { TaskProfile_Update, 10U, 0U }, /* H2整车状态机、计时和停车 */                 \
    { LineTrack_Update, 10U, 0U }, /* 灰度识别、路线推进和循迹 */                   \
    { Motion_Update, 10U, 0U }, /* 保留动作库后台的安全状态推进 */                  \
    { Chassis_Update, 10U, 0U }, /* 底盘速度闭环 */                                 \
    { LCD_Update, 20U, 0U }, /* 1.54寸LCD异步刷新 */                                \
};                                                                                  \
const uint8_t TASK_NUM =                                                             \
    (uint8_t)(sizeof(task_list) / sizeof(task_list[0]))

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_CONFIG_H */
