#ifndef __APP_TASK_CONFIG_H
#define __APP_TASK_CONFIG_H

#include "scheduler.h"
#include "app_task_port.h"
#include "app_diagnostics.h"
#include "line_calibration.h"
#include "motion_action.h"
#include "k210_comm.h"
#include "task_profile_select.h"
#include "test.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 电赛小车静态任务表配置区
 * ============================================================================
 * 重点：以后主要改这个 .h，不改 scheduler.c。
 *
 * 修改方法：
 *   1. 改周期：直接改 period_ms；
 *   2. 暂时不用某任务：把 task_func 改成 0，或把该行注释掉；
 *   3. 加新任务：先在对应模块公共头文件声明函数，再在这里加一行；
 *   4. A/B1	/C 对接：只实现这里对应的 Update 函数，不要改函数名。
 *   5. 正式任务只使用 AppDiagnostics_* 诊断入口，不依赖 Test 目录；
 *   6. 分项测试按文档启用 Test/test_config.h，并临时包含 Test/test.h。
 *
 * 周期建议：
 *   - 1ms：只放很轻的后台维护任务；
 *   - 10ms：按键、编码器、底盘速度环、循迹、任务状态机；
 *   - 20ms：调试菜单；
 *   - 100ms：OLED；
 *   - 200ms：串口日志。
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
};                                                       \
const uint8_t TASK_NUM =                                 \
    (uint8_t)(sizeof(task_list) / sizeof(task_list[0]))

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_CONFIG_H */
