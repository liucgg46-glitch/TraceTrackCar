#ifndef __DRV_GRAY_SENSOR_H
#define __DRV_GRAY_SENSOR_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 上层统一灰度传感器封装。
 *
 * 当前默认使用 Yahboom 8路串口灰度模块。需要切换 74HC4051、
 * MCU-I2C 或 Yahboom-UART 时，只修改 GRAY_SENSOR_SOURCE。
 */
#define GRAY_SENSOR_SOURCE_4051       0U
#define GRAY_SENSOR_SOURCE_MCU_I2C    1U
#define GRAY_SENSOR_SOURCE_YAHBOOM_UART 2U

#ifndef GRAY_SENSOR_SOURCE
#define GRAY_SENSOR_SOURCE            GRAY_SENSOR_SOURCE_YAHBOOM_UART
#endif

#if ((GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_YAHBOOM_UART) && \
     (VEHICLE_YAHBOOM_UART_ENABLE == 0U))
#error "Yahboom UART gray sensor requires VEHICLE_DRIVE_MODE_2WD"
#endif

#define GRAY_SENSOR_CHANNEL_NUM       8U

typedef struct {
    uint16_t raw[GRAY_SENSOR_CHANNEL_NUM];
    uint16_t filt[GRAY_SENSOR_CHANNEL_NUM];
    uint8_t  online;
    uint8_t  valid;
    uint8_t  source;
    uint32_t last_update_ms;
    uint32_t error_count;
} Drv_GraySensor_Info_t;

void Drv_GraySensor_Init(void);
BSP_Status_t Drv_GraySensor_Update(void);

uint16_t Drv_GraySensor_GetRaw(uint8_t index);
uint16_t Drv_GraySensor_GetFilt(uint8_t index);
BSP_Status_t Drv_GraySensor_GetRawArray(uint16_t *out_buf, uint8_t max_count);
BSP_Status_t Drv_GraySensor_GetFiltArray(uint16_t *out_buf, uint8_t max_count);
BSP_Status_t Drv_GraySensor_GetInfo(Drv_GraySensor_Info_t *info);
uint8_t Drv_GraySensor_IsOnline(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_GRAY_SENSOR_H */
