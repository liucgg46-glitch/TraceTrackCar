#include "drv_gray_sensor.h"
#include "drv_gray_4051.h"
#include "drv_gray_mcu_i2c.h"
#include "drv_gray_yahboom_uart.h"

void Drv_GraySensor_Init(void)
{
#if (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_4051)
    Drv_Gray4051_Init();
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_MCU_I2C)
    Drv_GrayMcu_Init();
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_YAHBOOM_UART)
    Drv_GrayYahboom_Init();
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
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_YAHBOOM_UART)
    return Drv_GrayYahboom_Update();
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
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_YAHBOOM_UART)
    return Drv_GrayYahboom_GetRaw(index);
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
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_YAHBOOM_UART)
    return Drv_GrayYahboom_GetFilt(index);
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
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_YAHBOOM_UART)
    return Drv_GrayYahboom_GetRawArray(out_buf, max_count);
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
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_YAHBOOM_UART)
    return Drv_GrayYahboom_GetFiltArray(out_buf, max_count);
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
#if (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_4051)
    info->valid = Drv_Gray4051_IsValid();
    info->last_update_ms = 0U;
    info->error_count = 0U;
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_MCU_I2C)
    {
        Drv_GrayMcu_Info_t source_info;
        (void)Drv_GrayMcu_GetInfo(&source_info);
        info->valid = source_info.valid;
        info->last_update_ms = source_info.last_update_ms;
        info->error_count = source_info.error_count;
    }
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_YAHBOOM_UART)
    {
        Drv_GrayYahboom_Info_t source_info;
        (void)Drv_GrayYahboom_GetInfo(&source_info);
        info->valid = source_info.valid;
        info->last_update_ms = source_info.last_frame_tick;
        info->error_count = source_info.invalid_frame_count +
                            source_info.rx_overflow_count +
                            source_info.uart_error_count;
    }
#endif
    return BSP_OK;
}

uint8_t Drv_GraySensor_IsOnline(void)
{
#if (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_4051)
    return Drv_Gray4051_IsValid();
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_MCU_I2C)
    return Drv_GrayMcu_IsOnline();
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_YAHBOOM_UART)
    return Drv_GrayYahboom_IsOnline();
#else
    return 0U;
#endif
}
