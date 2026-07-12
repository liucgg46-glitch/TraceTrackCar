#ifndef __VL53L1_PLATFORM_CONFIG_H
#define __VL53L1_PLATFORM_CONFIG_H

#include "drv_vl53l1x.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ST ULD 使用 8-bit I2C 地址；TraceTrackCar BSP 使用 7-bit 地址。
 * vl53l1_platform.c 在每次事务前自动执行 dev >> 1。
 */
#define VL53L1X_PLATFORM_I2C_BUS              DRV_VL53L1X_I2C_BUS
#define VL53L1X_PLATFORM_DEFAULT_ADDR_8BIT    DRV_VL53L1X_I2C_ADDR_8BIT
#define VL53L1X_PLATFORM_MAX_TRANSFER_SIZE    DRV_VL53L1X_PLATFORM_MAX_TRANSFER

BSP_Status_t VL53L1X_Platform_GetLastBspStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __VL53L1_PLATFORM_CONFIG_H */
