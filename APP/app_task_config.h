#ifndef __APP_TASK_CONFIG_H
#define __APP_TASK_CONFIG_H

#include "scheduler.h"
#include "app_task_port.h"
#include "app_diagnostics.h"
#include "test_config.h"

#if (PROJECT_TEST_TASKS_ENABLE != 0U)
#include "test.h"
#else
#include "task_profile_select.h"
#include "line_calibration.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if (PROJECT_TEST_TASKS_ENABLE != 0U)

/*
 * PF8摆杆舵机安全标定任务表。
 * 只推进异步后台、按键消抖和标定任务，不启动底盘、传感器、循迹或显示任务。
 */
#define APP_SCHEDULER_TASK_LIST_DEFINE()                                            \
Task_t task_list[] = {                                                              \
    { AppDiagnostics_HeartbeatUpdate, 10U, 0U }, /* 运行心跳 */                    \
    { AppTask_BSP_Background, 1U, 0U }, /* UART和舵机Driver后台 */                 \
    { Key_Update, 10U, 0U }, /* 按键扫描和消抖 */                                  \
    { Test_ServoBeamCalibration_Update, 10U, 0U }, /* PF8舵机安全标定 */          \
};                                                                                  \
const uint8_t TASK_NUM =                                                             \
    (uint8_t)(sizeof(task_list) / sizeof(task_list[0]))

#else

/*
 * 2026年电赛H题第2项正式任务表。
 * 将PROJECT_TEST_TASKS_ENABLE恢复为0U后，保持原周期、顺序和功能。
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

#endif

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_CONFIG_H */
