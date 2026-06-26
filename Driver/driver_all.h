#ifndef __DRIVER_ALL_H
#define __DRIVER_ALL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * Driver 层统一初始化入口
 * ============================================================================
 * 定位：集中初始化所有 Driver 层模块，方便后续加灰度、IMU、ToF、视觉等
 * 驱动时只改本文件，不需要在 main.c 里到处加 Init。
 *
 * 当前包含：
 *   - Motor_Init()
 *   - Drv_Encoder_Init()
 *   - Drv_GraySensor_Init()
 *   - Drv_LcdTft_Init()
 *   - Drv_OledI2c_Init()
 *
 * Driver_Task() 必须被 1ms 左右后台任务周期调用，用于推进非阻塞 LCD/OLED DMA 状态机。
 */
void Driver_Init(void);
void Driver_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVER_ALL_H */
