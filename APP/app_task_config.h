#ifndef __APP_TASK_CONFIG_H
#define __APP_TASK_CONFIG_H

#include "scheduler.h"
#include "app_task_port.h"
#include "app_diagnostics.h"
#include "test_config.h"

#if (PROJECT_TEST_TASKS_ENABLE != 0U)
#include "test.h"
#include "k210_comm.h"
#include "ball_balance_app.h"
#include "ball_balance_k210_adapter.h"
#else
#include "task_profile_select.h"
#include "line_calibration.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if (PROJECT_TEST_TASKS_ENABLE != 0U)

#if (PROJECT_TEST_PROFILE == PROJECT_TEST_PROFILE_SERVO_CAL)

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

#elif (PROJECT_TEST_PROFILE == PROJECT_TEST_PROFILE_K210_BALL_COMM)

/*
 * K210钢球0x32纯通信测试任务表。
 * 本档位允许Test_K210_BallCommUpdate消费GetNewBallPosition，不注册滚球PID适配层。
 */
#define APP_SCHEDULER_TASK_LIST_DEFINE()                                            \
Task_t task_list[] = {                                                              \
    { AppDiagnostics_HeartbeatUpdate, 10U, 0U }, /* 运行心跳 */                    \
    { AppTask_BSP_Background, 1U, 0U }, /* UART后台 */                             \
    { K210_Comm_Update, 5U, 0U }, /* 解析K210 0x32钢球位置 */                      \
    { Test_K210_BallCommUpdate, 10U, 0U }, /* 纯通信日志 */                       \
};                                                                                  \
const uint8_t TASK_NUM =                                                             \
    (uint8_t)(sizeof(task_list) / sizeof(task_list[0]))

#else

/*
 * K210钢球位置 + 滚球PID联调任务表。
 * BallBalance_K210Adapter_Update是GetNewBallPosition唯一消费者。
 */
#define APP_SCHEDULER_TASK_LIST_DEFINE()                                            \
Task_t task_list[] = {                                                              \
    { AppDiagnostics_HeartbeatUpdate, 10U, 0U }, /* 运行心跳 */                    \
    { AppTask_BSP_Background, 1U, 0U }, /* UART和舵机Driver后台 */                 \
    { K210_Comm_Update, 5U, 0U }, /* 解析K210钢球帧 */                             \
    { BallBalance_K210Adapter_Update, 5U, 0U }, /* 推送钢球位置到APP */           \
    { Key_Update, 10U, 0U }, /* 按键启停和目标选择 */                              \
    { Test_BallBalanceControl_Update, 10U, 0U }, /* O点/±5cm联调控制 */           \
    { BallBalance_App_Update, 10U, 0U }, /* 滚球PID和PF8舵机输出 */               \
};                                                                                  \
const uint8_t TASK_NUM =                                                             \
    (uint8_t)(sizeof(task_list) / sizeof(task_list[0]))

#endif

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
    { Sensor_Update, 1U, 0U }, /* 灰度、IMU和测距 */                               \
    { Encoder_Update, 10U, 0U }, /* 速度反馈 */                                    \
    { TaskProfile_Update, 10U, 0U }, /* H2整车状态机、计时和停车 */                \
    { LineTrack_Update, 10U, 0U }, /* 灰度识别、路线推进和循迹 */                  \
    { Motion_Update, 10U, 0U }, /* 保留动作库后台的安全状态推进 */                 \
    { Chassis_Update, 10U, 0U }, /* 底盘速度闭环 */                                \
    { LCD_Update, 20U, 0U }, /* 1.54寸LCD异步刷新 */                               \
};                                                                                  \
const uint8_t TASK_NUM =                                                             \
    (uint8_t)(sizeof(task_list) / sizeof(task_list[0]))

#endif

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_CONFIG_H */
