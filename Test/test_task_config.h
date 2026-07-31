#ifndef __TEST_TASK_CONFIG_H
#define __TEST_TASK_CONFIG_H

#include "scheduler.h"
#include "app_task_port.h"
#include "app_diagnostics.h"
#include "test_config.h"
#include "test.h"
#include "k210_comm.h"
#include "ball_balance_app.h"
#include "ball_balance_k210_adapter.h"
#include "ball_balance_vehicle_imu_adapter.h"

#ifdef __cplusplus
extern "C" {
#endif

#if (PROJECT_TEST_PROFILE == PROJECT_TEST_PROFILE_SERVO_CAL)

/* PF8摆杆舵机安全标定，不启动底盘、传感器或正式任务状态机。 */
#define TEST_SCHEDULER_TASK_LIST_DEFINE()                                           \
Task_t task_list[] = {                                                              \
    { AppDiagnostics_HeartbeatUpdate, 10U, 0U }, /* 运行心跳 */                    \
    { AppTask_BSP_Background, 1U, 0U }, /* UART和舵机Driver后台 */                 \
    { Key_Update, 10U, 0U }, /* 按键扫描和消抖 */                                  \
    { Test_ServoBeamCalibration_Update, 10U, 0U }, /* PF8舵机安全标定 */           \
};                                                                                  \
const uint8_t TASK_NUM =                                                             \
    (uint8_t)(sizeof(task_list) / sizeof(task_list[0]))

#elif (PROJECT_TEST_PROFILE == PROJECT_TEST_PROFILE_K210_BALL_COMM)

/* 纯通信档允许测试入口直接消费0x32钢球帧，不注册滚球适配层。 */
#define TEST_SCHEDULER_TASK_LIST_DEFINE()                                           \
Task_t task_list[] = {                                                              \
    { AppDiagnostics_HeartbeatUpdate, 10U, 0U }, /* 运行心跳 */                    \
    { AppTask_BSP_Background, 1U, 0U }, /* UART后台 */                             \
    { K210_Comm_Update, 5U, 0U }, /* 解析K210 0x32钢球位置 */                      \
    { Test_K210_BallCommUpdate, 10U, 0U }, /* 纯通信日志 */                        \
};                                                                                  \
const uint8_t TASK_NUM =                                                             \
    (uint8_t)(sizeof(task_list) / sizeof(task_list[0]))

#elif (PROJECT_TEST_PROFILE == PROJECT_TEST_PROFILE_BALL_MODEL_ID)

/* B0和机械方向辨识：只输出固定角度，不启动闭环和底盘。 */
#define TEST_SCHEDULER_TASK_LIST_DEFINE()                                           \
Task_t task_list[] = {                                                              \
    { AppDiagnostics_HeartbeatUpdate, 10U, 0U }, /* 运行心跳 */                    \
    { AppTask_BSP_Background, 1U, 0U }, /* UART和舵机Driver后台 */                 \
    { K210_Comm_Update, 5U, 0U }, /* 解析K210钢球帧 */                             \
    { BallBalance_K210Adapter_Update, 5U, 0U }, /* 新帧唯一控制消费者 */           \
    { Key_Update, 10U, 0U }, /* 固定角度选择 */                                    \
    { Test_BallModelIdentify_Update, 10U, 0U }, /* B0方向辨识 */                   \
};                                                                                  \
const uint8_t TASK_NUM =                                                             \
    (uint8_t)(sizeof(task_list) / sizeof(task_list[0]))

#else

/* 新状态反馈静止滚球测试，不注册底盘、循迹、传感器或正式任务状态机。 */
#define TEST_SCHEDULER_TASK_LIST_DEFINE()                                           \
Task_t task_list[] = {                                                              \
    { AppDiagnostics_HeartbeatUpdate, 10U, 0U }, /* 运行心跳 */                    \
    { AppTask_BSP_Background, 1U, 0U }, /* UART和舵机Driver后台 */                 \
    { K210_Comm_Update, 5U, 0U }, /* 解析K210钢球帧 */                             \
    { BallBalance_K210Adapter_Update, 5U, 0U }, /* 新帧唯一控制消费者 */           \
    { BallBalance_VehicleImuAdapter_Update, 10U, 0U }, /* 默认无效前馈 */          \
    { Key_Update, 10U, 0U }, /* O点和正负50mm选择 */                               \
    { Test_BallBalanceControl_Update, 10U, 0U }, /* 按键和诊断 */                  \
    { BallBalance_App_Update, 10U, 0U }, /* 状态估计、反馈和舵机输出 */            \
};                                                                                  \
const uint8_t TASK_NUM =                                                             \
    (uint8_t)(sizeof(task_list) / sizeof(task_list[0]))

#endif

#ifdef __cplusplus
}
#endif

#endif /* __TEST_TASK_CONFIG_H */
