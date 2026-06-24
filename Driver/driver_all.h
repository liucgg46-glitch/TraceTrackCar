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
 *
 * 后续 Part4/Part5 可继续加入：
 *   - Drv_Gray4051_Init()
 *   - Drv_IMU_Init()
 *   - Drv_ToF_Init()
 *   - VisionProtocol_Init()
 */
void Driver_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVER_ALL_H */
