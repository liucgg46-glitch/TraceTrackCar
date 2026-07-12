#include "drv_gray_sensor.h"
#include "drv_gray_4051.h"
#include "drv_gray_mcu_i2c.h"

void Drv_GraySensor_Init(void)
{
#if (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_4051)
    Drv_Gray4051_Init();
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_MCU_I2C)
    Drv_GrayMcu_Init();
#else
#error "Unsupported GRAY_SENSOR_SOURCE"
#endif
}

BSP_Status_t Drv_GraySensor_Update(void)
{
#if (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_4051)
    Drv_Gray4051_Update();
    return BSP_OK;
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_MCU_I2C)
    return Drv_GrayMcu_Update();
#else
    return BSP_PARAM;
#endif
}

uint16_t Drv_GraySensor_GetRaw(uint8_t index)
{
#if (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_4051)
    return Drv_Gray4051_GetRaw(index);
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_MCU_I2C)
    return Drv_GrayMcu_GetRaw(index);
#else
    (void)index;
    return 0U;
#endif
}

uint16_t Drv_GraySensor_GetFilt(uint8_t index)
{
#if (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_4051)
    return Drv_Gray4051_GetFilt(index);
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_MCU_I2C)
    return Drv_GrayMcu_GetFilt(index);
#else
    (void)index;
    return 0U;
#endif
}

BSP_Status_t Drv_GraySensor_GetRawArray(uint16_t *out_buf, uint8_t max_count)
{
#if (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_4051)
    return Drv_Gray4051_GetRawArray(out_buf, max_count);
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_MCU_I2C)
    return Drv_GrayMcu_GetRawArray(out_buf, max_count);
#else
    (void)out_buf;
    (void)max_count;
    return BSP_PARAM;
#endif
}

BSP_Status_t Drv_GraySensor_GetFiltArray(uint16_t *out_buf, uint8_t max_count)
{
#if (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_4051)
    return Drv_Gray4051_GetFiltArray(out_buf, max_count);
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_MCU_I2C)
    return Drv_GrayMcu_GetFiltArray(out_buf, max_count);
#else
    (void)out_buf;
    (void)max_count;
    return BSP_PARAM;
#endif
}

BSP_Status_t Drv_GraySensor_GetInfo(Drv_GraySensor_Info_t *info)
{
    if (info == 0) return BSP_PARAM;

    (void)Drv_GraySensor_GetRawArray(info->raw, GRAY_SENSOR_CHANNEL_NUM);
    (void)Drv_GraySensor_GetFiltArray(info->filt, GRAY_SENSOR_CHANNEL_NUM);
    info->online = Drv_GraySensor_IsOnline();
    info->source = GRAY_SENSOR_SOURCE;
    return BSP_OK;
}

uint8_t Drv_GraySensor_IsOnline(void)
{
#if (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_4051)
    return Drv_Gray4051_IsValid();
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_MCU_I2C)
    return Drv_GrayMcu_IsOnline();
#else
    return 0U;
#endif
}
