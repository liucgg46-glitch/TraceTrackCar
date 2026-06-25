#ifndef __APP_TASK_CONFIG_H
#define __APP_TASK_CONFIG_H

#include "scheduler.h"
#include "app_task_port.h"
#include "motion_action.h"
#include "test.h"
#include "lcd_ui.h"
#include "oled_ui.h"

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
 *   4. A/B/C 对接：只实现这里对应的 Update 函数，不要改函数名。
 *
 * 周期建议：
 *   - 1ms：只放很轻的后台维护任务；
 *   - 10ms：编码器、底盘速度环、循迹、任务状态机；
 *   - 20ms：调试菜单；
 *   - 100ms：OLED；
 *   - 200ms：串口日志。
 */
#define APP_SCHEDULER_TASK_LIST_DEFINE()                                            \
Task_t task_list[] = {                                                              \
	{ Test_GPIO_Toggle,       10U,   0U },                                          \
    { AppTask_BSP_Background,  1U,   0U },  /* B：BSP 后台维护 */                   \
    { Sensor_Update,          1U,   0U },  \
    { Encoder_Update,        10U,   0U },  \
    { LineTrack_Update,      10U,   0U },  \
    { Chassis_Update,        10U,   0U },  \
    { Test_LineCmd_Update,   10U,   0U },  \
    { OLED_Update,          500U,   0U },  \
    { LcdUi_Update,         200U,   0U },  \
    { Test_LineCmd_Log,     200U,   0U },  \
};                                                                                  \
const uint8_t TASK_NUM = (uint8_t)(sizeof(task_list) / sizeof(task_list[0]))


//    { Encoder_Update,         10U,   0U },  /* B：编码器速度更新 */                 \
//    { Sensor_Update,          10U,   0U },  /* C：传感器统一更新 */                 \
//    { Chassis_Update,         10U,   0U },  /* A：底盘速度闭环 */                   \
//    { LineTrack_Update,       10U,   0U },  /* A：循迹控制 */                       \
//    { TaskFSM_Update,         10U,   0U },  /* A：任务状态机 */                     \
//    { DebugMenu_Update,       20U,   0U },  /* C：按键菜单/调参 */                  \
//    { OLED_Update,           100U,   0U },  /* C：OLED 刷新 */                      \
//    { Log_Update,            200U,   0U },  /* C：串口日志 */                       \

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_CONFIG_H */
