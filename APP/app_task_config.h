#ifndef __APP_TASK_CONFIG_H
#define __APP_TASK_CONFIG_H

#include "scheduler.h"
#include "app_task_port.h"
#include "line_calibration.h"
#include "motion_action.h"
#include "test.h"
#include "k210_comm.h"
#include "task_profile_select.h"

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
 *   3. 加新任务：先在 app_task_port.h 声明函数，再在这里加一行；
 *   4. A/B1	/C 对接：只实现这里对应的 Update 函数，不要改函数名。
 *   5. 所有测试任务表都必须保留 { Test_GPIO_Toggle, 10U, 0U }，
 *      即使注册“最简任务列表”也不能省略。
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
    { Test_GPIO_Toggle,        10U, 0U }, /* 运行指示灯 */ \
    { AppTask_BSP_Background,   1U, 0U }, /* BSP和Driver后台维护 */ \
    { Sensor_Update,            1U, 0U }, /* HX711非阻塞采样 */ \
    { K210_Comm_Update,         5U, 0U }, /* K210新快照解析 */ \
    { Encoder_Update,          10U, 0U }, /* 轮速反馈与停车确认 */ \
    { TaskProfile_Update,      10U, 0U }, /* 当前选择的总任务状态机 */ \
    { LineTrack_Update,        10U, 0U }, /* 去返程循迹与路线推进 */ \
    { Motion_Update,           10U, 0U }, /* 路口定角转弯 */ \
    { Chassis_Update,          10U, 0U }, /* 底盘速度闭环 */ \
    { Test_TaskFSM_Log,       200U, 0U }, /* USART1联调日志 */ \
};                                                       \
const uint8_t TASK_NUM =                                 \
    (uint8_t)(sizeof(task_list) / sizeof(task_list[0]))

//    { Test_GPIO_Toggle,        10U, 0U },                   \
//    { AppTask_BSP_Background,   1U, 0U },                   \
//    { Key_Update,              10U, 0U },                   \
//    { Sensor_Update,            1U, 0U },                   \
//    { LineCalibration_Update,     10U, 0U },                \
//    { LCD_Update,              20U, 0U },                   \
//    { OLED_Update,             20U, 0U },                   \

//	{ Test_GPIO_Toggle,        10U, 0U }, /* 运行指示灯 */   \
//	{ AppTask_BSP_Background,   1U, 0U },                   \
//	{ Key_Update,              10U, 0U },                   \
//	{ Sensor_Update,            1U, 0U },                   \
//	{ Encoder_Update,          10U, 0U },                   \
//	{ Test_RouteCmd_Update,    10U, 0U },                   \
//	{ LineTrack_Update,        10U, 0U },                   \
//	{ Motion_Update,           10U, 0U },                   \
//	{ Chassis_Update,          10U, 0U },                   \
//	{ Test_RouteLog,          200U, 0U },                   \
//	{ LCD_Update,              20U, 0U },                   \
//	{ OLED_Update,             20U, 0U },                   \

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_CONFIG_H */
