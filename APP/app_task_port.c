#include "app_task_port.h"
#include "bsp_all.h"
#include "bsp_encoder.h"
#include "bsp_key.h"

/*
 * Part1 默认弱实现。
 * 后续真正写 drv_encoder/chassis/line_track/task_fsm 等模块时，只要实现同名函数，
 * 这些弱函数会被覆盖，不需要修改 scheduler.c 或 app_task_config.h。
 */

BSP_WEAK void AppTask_BSP_Background(void)
{
    /* UART DMA 搬运、I2C/SPI 异步超时恢复等后台维护。 */
    BSP_TaskAll();
}

BSP_WEAK void Encoder_Update(void)
{
    /* Part1 阶段先直接更新 BSP 编码器；Part2 可由 drv_encoder.c 覆盖。 */
    BSP_Encoder_UpdateAll();
}

BSP_WEAK void Sensor_Update(void)
{
    /* Part1 阶段先放按键扫描；Part4 由 sensor_manager.c 覆盖。 */
    BSP_Key_UpdateAll();
}

BSP_WEAK void Chassis_Update(void)
{
    /* Part2 由 chassis.c 覆盖：速度 PID、差速分配、电机输出。 */
}

BSP_WEAK void Motion_Update(void)
{
    /* Part3 由 motion_action.c 覆盖：非阻塞直走/转角动作。 */
}

BSP_WEAK void LineTrack_Update(void)
{
    /* Part4 由 line_follow_app.c 覆盖：灰度识别 + 循迹输出到底盘。 */
}

BSP_WEAK void TaskFSM_Update(void)
{
    /* Part7 由 task_fsm.c 覆盖：比赛任务状态切换。 */
}

BSP_WEAK void DebugMenu_Update(void)
{
    /* Part6 由 debug_menu.c 覆盖：按键调参、参数保存触发。 */
}

BSP_WEAK void OLED_Update(void)
{
    /* Part6 由 drv_oled/debug_menu 覆盖：显示状态、参数、传感器数据。 */
}

BSP_WEAK void Log_Update(void)
{
    /* Part6 由 log.c 覆盖：串口周期日志。 */
}
