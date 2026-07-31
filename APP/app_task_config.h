#ifndef __APP_TASK_CONFIG_H
#define __APP_TASK_CONFIG_H

#include "scheduler.h"
#include "app_task_port.h"
#include "app_diagnostics.h"
#include "k210_comm.h"
#include "ball_balance_app.h"
#include "ball_balance_k210_adapter.h"
#include "ball_balance_vehicle_imu_adapter.h"
#include "task_fsm.h"
#include "line_follow_app.h"
#include "motion_action.h"
#include "chassis.h"
#include "lcd_ui.h"
#include "test.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 当前唯一任务表：H题MODE2～MODE6正式总任务。 */
Task_t task_list[] = {
    { AppDiagnostics_HeartbeatUpdate, 10U, 0U }, /* 运行心跳 */  \
    { AppTask_BSP_Background, 1U, 0U }, /* UART和舵机Driver后台 */\
    { K210_Comm_Update, 5U, 0U }, /* 解析K210钢球帧 */\
    { BallBalance_K210Adapter_Update, 5U, 0U }, /* 新帧唯一消费者 */\
    { BallBalance_VehicleImuAdapter_Update, 10U, 0U }, /* 默认无效前馈 */\
    { Key_Update, 10U, 0U }, /* O点和正负50mm选择 */\
    { Test_BallBalanceControl_Update, 10U, 0U }, /* 按键和诊断 */\
    { BallBalance_App_Update, 10U, 0U }, /* 状态估计、反馈和舵机输出 */\
};

const uint8_t TASK_NUM =
    (uint8_t)(sizeof(task_list) / sizeof(task_list[0]));

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_CONFIG_H */
