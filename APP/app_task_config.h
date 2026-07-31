#ifndef __APP_TASK_CONFIG_H
#define __APP_TASK_CONFIG_H

#include "scheduler.h"
#include "app_task_port.h"
#include "app_diagnostics.h"
#include "test.h"
#include "k210_comm.h"
#include "ball_balance_app.h"
#include "ball_balance_k210_adapter.h"
#include "ball_balance_vehicle_imu_adapter.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 当前唯一任务表：钢球状态反馈手动联调。
 * Test_BallBalanceControl_Update负责KEY1至KEY5和周期串口诊断；
 * 不得同时注册Test_K210_BallCommUpdate，否则会提前消费0x32新帧。
 */
Task_t task_list[] = {
    { AppDiagnostics_HeartbeatUpdate, 10U, 0U }, /* 运行心跳 */
    { AppTask_BSP_Background, 1U, 0U }, /* UART和舵机Driver后台 */
    { K210_Comm_Update, 5U, 0U }, /* 解析K210钢球帧 */
    { BallBalance_K210Adapter_Update, 5U, 0U }, /* 新帧唯一消费者 */
    { BallBalance_VehicleImuAdapter_Update, 10U, 0U }, /* 前馈保持关闭 */
    { Key_Update, 10U, 0U }, /* KEY1至KEY5扫描 */
    { Test_BallBalanceControl_Update, 10U, 0U }, /* 按键和串口诊断 */
    { BallBalance_App_Update, 10U, 0U }, /* 估计、控制和舵机输出 */
};

const uint8_t TASK_NUM =
    (uint8_t)(sizeof(task_list) / sizeof(task_list[0]));

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_CONFIG_H */
