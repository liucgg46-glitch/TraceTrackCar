#ifndef __SENSOR_MANAGER_H
#define __SENSOR_MANAGER_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 传感器统一管理层。
 *
 * 主要职责：
 *   1. 统一推进各传感器驱动的非阻塞状态机；
 *   2. 向 APP、Route 等上层模块提供稳定的传感器读取接口；
 *   3. 隔离上层业务代码与具体传感器驱动，便于后续替换硬件。
 *
 * 当前接入：
 *   - 感为八路灰度传感器；
 *   - VL53L1X 激光测距传感器。
 */

/**
 * @brief 初始化传感器管理层。
 *
 * 具体传感器驱动已经由 Driver_Init() 完成初始化，因此本函数当前
 * 不重复初始化硬件，仅作为传感器管理层的统一初始化入口保留。
 */
void SensorManager_Init(void);

/**
 * @brief 周期推进所有传感器驱动。
 *
 * 必须由任务调度器周期调用，建议周期为 1 ms：
 *
 *     { Sensor_Update, 1U, 0U },
 *
 * 本函数负责推进 VL53L1X 和灰度传感器的非阻塞状态机。
 * 上层模块只读取缓存结果，不要再次直接调用各驱动的 Update()。
 */
void Sensor_Update(void);

/**
 * @brief 获取车头 VL53L1X 的最新有效距离。
 *
 * 该函数不会发起新的 I2C 通信，只读取 VL53L1X 驱动内部已经缓存的
 * 最新有效测距结果，因此可以安全地在 APP、Route 等上层模块中调用。
 *
 * @param distance_mm  距离输出地址，单位为 mm。
 *
 * @retval BSP_OK      成功，*distance_mm 为最新有效滤波距离。
 * @retval BSP_PARAM   distance_mm 为空指针。
 * @retval BSP_ERROR   传感器离线，或当前没有有效测距结果。
 *
 * @note
 *   - 返回 BSP_OK 时才可以使用 distance_mm；
 *   - 无效测量状态不会作为有效距离返回；
 *   - 本接口返回的是驱动筛选后的有效距离，不是未经校验的原始距离。
 */
BSP_Status_t Sensor_GetFrontDistanceMm(uint16_t *distance_mm);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_MANAGER_H */
