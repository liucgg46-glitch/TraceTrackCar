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
    { AppDiagnostics_HeartbeatUpdate, 10U, 0U }, /* 系统心跳 */
    { AppTask_BSP_Background, 1U, 0U }, /* BSP/UART/Driver后台 */
    { Sensor_Update, 1U, 0U }, /* 灰度、IMU、ToF、称重 */
    { Key_Update, 10U, 0U }, /* KEY1至KEY9扫描 */
    { Encoder_Update, 10U, 0U }, /* 编码器速度和里程 */
    { K210_Comm_Update, 5U, 0U }, /* K210协议解析 */
    { BallBalance_K210Adapter_Update, 5U, 0U }, /* 钢球帧唯一消费者 */
    { BallBalance_VehicleImuAdapter_Update, 10U, 0U }, /* 移动加速度适配 */
    { TaskFSM_Update, 10U, 0U }, /* 正式总任务状态机 */
    { AppDiagnostics_TaskFSMLogUpdate, 200U, 0U }, /* 串口任务状态日志 */
    { LineTrack_Update, 10U, 0U }, /* 循迹和Route */
    { Motion_Update, 10U, 0U }, /* 非阻塞动作 */
    { BallBalance_App_Update, 10U, 0U }, /* 滚球控制 */
    { Chassis_Update, 10U, 0U }, /* 底盘输出 */
    { LCD_Update, 100U, 0U }, /* 模式和结果显示 */
	{ Test_K210_GrayTuneUpdate, 20U, 0U }, /* 传统灰度参数调节 */ \
};

const uint8_t TASK_NUM =
    (uint8_t)(sizeof(task_list) / sizeof(task_list[0]));

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_CONFIG_H */
