#include "app_task_port.h"
#include "bsp_all.h"
#include "bsp_encoder.h"
#include "bsp_key.h"

/*
 * Part1 默认弱实现。
 * 后续真正写 drv_encoder/chassis/line_track/task_fsm 等模块时，只要实现同名函数，
 * 这些弱函数会被覆盖，不需要修改 scheduler.c 或 app_task_config.h。
 */

/**
 * @brief 后台总维护任务（周期性后台硬件收尾处理）
 * @weak 弱函数，后续可重写替换
 * 现阶段：调用所有BSP后台轮询任务（DMA收尾、通信异常恢复等）
 */
BSP_WEAK void AppTask_BSP_Background(void)
{
    /* UART DMA 搬运、I2C/SPI 异步超时恢复等后台维护。 */
    BSP_TaskAll();
}

/**
 * @brief 编码器数据刷新任务
 * 现阶段：直接调用底层BSP读取左右编码器脉冲
 * 后续：在 drv_encoder.c 写带转速滤波、里程计算的同名函数，自动覆盖本弱函数
 */
BSP_WEAK void Encoder_Update(void)
{
    /* Part1 阶段先直接更新 BSP 编码器；Part2 可由 drv_encoder.c 覆盖。 */
    BSP_Encoder_UpdateAll();
}

/**
 * @brief 传感器总刷新任务
 * 现阶段：只做按键扫描
 * 后续：在 sensor_manager.c 统一管理灰度、激光测距、IMU等所有传感器采集，覆盖本函数
 */
BSP_WEAK void Sensor_Update(void)
{
    /* Part1 阶段先放按键扫描；后续 C 可在 sensor_manager.c 中覆盖。 */
    BSP_Key_UpdateAll();
}

/**
 * @brief 底盘控制周期任务
 * 现阶段空实现占位
 * 后续：chassis.c 实现电机PID闭环、差速控制、速度限制、正反走逻辑，覆盖本弱函数
 */
BSP_WEAK void Chassis_Update(void)
{
    /* Part2 由 chassis.c 覆盖：速度 PID、差速分配、电机输出。 */
}

/**
 * @brief 循迹算法运算任务
 * 现阶段空占位
 * 后续：line_track.c 根据灰度传感器偏差计算转向修正量、过弯逻辑，覆盖本函数
 */
BSP_WEAK void LineTrack_Update(void)
{
    /* Part4 由 line_track.c 覆盖：根据 LineInfo 计算转向量。 */
}

/**
 * @brief 整车状态机调度任务（比赛流程总状态）
 * 现阶段空占位
 * 后续：task_fsm.c 实现：待机→按键启动→循迹运行→终点停止、异常保护等状态跳转逻辑
 */
BSP_WEAK void TaskFSM_Update(void)
{
    /* Part7 由 task_fsm.c 覆盖：比赛任务状态切换。 */
}

/**
 * @brief 调参菜单任务
 * 现阶段空占位
 * 后续：debug_menu.c 实现按键修改PID参数、切换模式、参数存储等功能
 */
BSP_WEAK void DebugMenu_Update(void)
{
    /* Part6 由 debug_menu.c 覆盖：按键调参、参数保存触发。 */
}

/**
 * @brief OLED屏幕刷新任务
 * 现阶段空占位
 * 后续：OLED驱动文件刷新速度、位置、传感器数值、运行状态
 */
BSP_WEAK void OLED_Update(void)
{
    /* Part6 由 drv_oled/debug_menu 覆盖：显示状态、参数、传感器数据。 */
}

/**
 * @brief 串口日志打印任务
 * 现阶段空占位
 * 后续：log.c 周期打印转速、偏差、PID输出、调试信息，方便上位机调试
 */
BSP_WEAK void Log_Update(void)
{
    /* Part6 由 log.c 覆盖：串口周期日志。 */
}
