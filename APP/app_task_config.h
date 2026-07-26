#ifndef __APP_TASK_CONFIG_H
#define __APP_TASK_CONFIG_H

#include "scheduler.h"
#include "app_task_port.h"
#include "app_diagnostics.h"
#include "task_profile_select.h"

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
 * Motion_Update仅用于直角的角度粗转，属于本赛道必要任务。
 */
#define APP_SCHEDULER_TASK_LIST_DEFINE()                                            \
Task_t task_list[] = {                                                              \
    { AppDiagnostics_HeartbeatUpdate, 10U, 0U }, /* 运行心跳 */                    \
    { AppTask_BSP_Background, 1U, 0U }, /* UART、I2C、SPI后台维护 */               \
    { Key_Update, 10U, 0U }, /* KEY1启动、KEY4停止所需按键扫描 */                  \
    { Sensor_Update, 1U, 0U }, /* 灰度传感器更新 */                               \
    { Encoder_Update, 10U, 0U }, /* 轮速和路线里程反馈 */                          \
    { TaskProfile_Update, 10U, 0U }, /* B题整车正式任务状态机 */                   \
    { LineTrack_Update, 10U, 0U }, /* B题赛道巡线与路段推进 */                     \
    { Motion_Update, 10U, 0U }, /* 直角按角度粗转动作 */                            \
    { Chassis_Update, 10U, 0U }, /* 底盘速度闭环 */                                \
};                                                                                  \
const uint8_t TASK_NUM =                                                             \
    (uint8_t)(sizeof(task_list) / sizeof(task_list[0]))

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_CONFIG_H */
