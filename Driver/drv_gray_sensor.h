#ifndef __DRV_GRAY_SENSOR_H
#define __DRV_GRAY_SENSOR_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Unified gray sensor facade for upper layers.
 *
 * Current default uses the GW MCU I2C gray sensor.
 * To switch between the 74HC4051 and MCU I2C versions, only change
 * GRAY_SENSOR_SOURCE here.
 */
#define GRAY_SENSOR_SOURCE_4051       0U
#define GRAY_SENSOR_SOURCE_MCU_I2C    1U

#define GRAY_SENSOR_SOURCE            GRAY_SENSOR_SOURCE_MCU_I2C 

#define GRAY_SENSOR_CHANNEL_NUM       8U

typedef struct {
    uint16_t raw[GRAY_SENSOR_CHANNEL_NUM];
    uint16_t filt[GRAY_SENSOR_CHANNEL_NUM];
    uint8_t  online;
    uint8_t  source;
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
