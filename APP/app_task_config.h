#ifndef __APP_TASK_CONFIG_H
#define __APP_TASK_CONFIG_H

#include "scheduler.h"
#include "app_task_port.h"
#include "app_diagnostics.h"
#include "task_profile_config.h"
#include "task_profile_select.h"

#if (H_COMPETITION_ITEM_SELECT == H_COMPETITION_ITEM_BALL_SEQUENCE)
#include "k210_comm.h"
#include "ball_balance_app.h"
#include "ball_balance_k210_adapter.h"
#include "ball_balance_vehicle_imu_adapter.h"
#else
#include "line_calibration.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if (H_COMPETITION_ITEM_SELECT == H_COMPETITION_ITEM_BALL_SEQUENCE)

/* H题第3项正式流程：唯一顶层状态机推进O→+50mm→-50mm，底盘始终停车。 */
#define APP_SCHEDULER_TASK_LIST_DEFINE()                                            \
Task_t task_list[] = {                                                              \
    { AppDiagnostics_HeartbeatUpdate, 10U, 0U }, /* 运行心跳 */                    \
    { AppTask_BSP_Background, 1U, 0U }, /* UART和舵机Driver后台 */                 \
    { K210_Comm_Update, 5U, 0U }, /* 解析K210钢球帧 */                             \
    { BallBalance_K210Adapter_Update, 5U, 0U }, /* 新帧唯一正式消费者 */           \
    { BallBalance_VehicleImuAdapter_Update, 10U, 0U }, /* 前馈默认关闭 */           \
    { Key_Update, 10U, 0U }, /* 启动和急停按键 */                                  \
    { BallBalance_App_Update, 10U, 0U }, /* 控制安全状态机 */                       \
    { TaskProfile_Update, 10U, 0U }, /* 唯一比赛顶层状态机 */                      \
    { LCD_Update, 20U, 0U }, /* 1.54寸LCD异步刷新 */                               \
};                                                                                  \
const uint8_t TASK_NUM =                                                             \
    (uint8_t)(sizeof(task_list) / sizeof(task_list[0]))

#else

/* H题第2项整圈停车原任务表。 */
#define APP_SCHEDULER_TASK_LIST_DEFINE()                                            \
Task_t task_list[] = {                                                              \
    { AppDiagnostics_HeartbeatUpdate, 10U, 0U }, /* 运行心跳 */                    \
    { AppTask_BSP_Background, 1U, 0U }, /* UART、总线和异步驱动后台 */             \
    { Key_Update, 10U, 0U }, /* 按键扫描 */                                        \
    { Sensor_Update, 1U, 0U }, /* 灰度、IMU和测距 */                               \
    { Encoder_Update, 10U, 0U }, /* 速度反馈 */                                    \
    { TaskProfile_Update, 10U, 0U }, /* H2整车状态机、计时和停车 */                \
    { LineTrack_Update, 10U, 0U }, /* 灰度识别、路线推进和循迹 */                  \
    { Motion_Update, 10U, 0U }, /* 动作库安全状态推进 */                           \
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
