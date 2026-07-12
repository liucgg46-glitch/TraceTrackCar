/**
  ******************************************************************************
  * @file    vl53l1_platform.c
  * @brief   ST VL53L1X ULD platform port for TraceTrackCar STM32F407 BSP.
  ******************************************************************************
  */

#include "vl53l1_platform.h"
#include "vl53l1_platform_config.h"
#include "bsp_i2c.h"
#include "bsp_systick.h"
#include <string.h>

#define VL53L1X_PLATFORM_OK       ((int8_t)0)
#define VL53L1X_PLATFORM_ERROR    ((int8_t)-1)

static uint8_t s_vl53l1_tx_buf[VL53L1X_PLATFORM_MAX_TRANSFER_SIZE + 2U];
static volatile BSP_Status_t s_last_bsp_status = BSP_OK;

static uint8_t VL53L1X_PlatformAddr7(uint16_t dev)
{
    return (uint8_t)((dev >> 1U) & 0x7FU);
}

static int8_t VL53L1X_PlatformStatus(BSP_Status_t status)
{
    s_last_bsp_status = status;
    return (status == BSP_OK) ? VL53L1X_PLATFORM_OK : VL53L1X_PLATFORM_ERROR;
}

BSP_Status_t VL53L1X_Platform_GetLastBspStatus(void)
{
    return s_last_bsp_status;
}

int8_t VL53L1_WriteMulti(uint16_t dev,
                         uint16_t index,
                         uint8_t *pdata,
                         uint32_t count)
{
    BSP_Status_t status;

    if ((pdata == 0) || (count == 0U) ||
        (count > VL53L1X_PLATFORM_MAX_TRANSFER_SIZE)) {
        s_last_bsp_status = BSP_PARAM;
        return VL53L1X_PLATFORM_ERROR;
    }

    s_vl53l1_tx_buf[0] = (uint8_t)(index >> 8U);
    s_vl53l1_tx_buf[1] = (uint8_t)(index & 0xFFU);
    memcpy(&s_vl53l1_tx_buf[2], pdata, (size_t)count);

    status = BSP_I2C_MasterWrite(VL53L1X_PLATFORM_I2C_BUS,
                                 VL53L1X_PlatformAddr7(dev),
                                 s_vl53l1_tx_buf,
                                 (uint16_t)(count + 2U));
    return VL53L1X_PlatformStatus(status);
}

int8_t VL53L1_ReadMulti(uint16_t dev,
                        uint16_t index,
                        uint8_t *pdata,
                        uint32_t count)
{
    uint8_t reg[2];
    BSP_Status_t status;

    if ((pdata == 0) || (count == 0U) ||
        (count > VL53L1X_PLATFORM_MAX_TRANSFER_SIZE)) {
        s_last_bsp_status = BSP_PARAM;
        return VL53L1X_PLATFORM_ERROR;
    }

    reg[0] = (uint8_t)(index >> 8U);
    reg[1] = (uint8_t)(index & 0xFFU);

    status = BSP_I2C_MasterWriteRead(VL53L1X_PLATFORM_I2C_BUS,
                                     VL53L1X_PlatformAddr7(dev),
                                     reg,
                                     2U,
                                     pdata,
                                     (uint16_t)count);
    return VL53L1X_PlatformStatus(status);
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
    return VL53L1_WriteMulti(dev, index, &data, 1U);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(data >> 8U);
    buf[1] = (uint8_t)(data & 0xFFU);
    return VL53L1_WriteMulti(dev, index, buf, 2U);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
    uint8_t buf[4];

    buf[0] = (uint8_t)(data >> 24U);
    buf[1] = (uint8_t)(data >> 16U);
    buf[2] = (uint8_t)(data >> 8U);
    buf[3] = (uint8_t)(data & 0xFFU);
    return VL53L1_WriteMulti(dev, index, buf, 4U);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata)
{
    if (pdata == 0) {
        s_last_bsp_status = BSP_PARAM;
        return VL53L1X_PLATFORM_ERROR;
    }
    return VL53L1_ReadMulti(dev, index, pdata, 1U);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata)
{
    uint8_t buf[2];
    int8_t status;

    if (pdata == 0) {
        s_last_bsp_status = BSP_PARAM;
        return VL53L1X_PLATFORM_ERROR;
    }

    status = VL53L1_ReadMulti(dev, index, buf, 2U);
    if (status == VL53L1X_PLATFORM_OK) {
        *pdata = (uint16_t)(((uint16_t)buf[0] << 8U) | (uint16_t)buf[1]);
    }
    return status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata)
{
    uint8_t buf[4];
    int8_t status;

    if (pdata == 0) {
        s_last_bsp_status = BSP_PARAM;
        return VL53L1X_PLATFORM_ERROR;
    }

    status = VL53L1_ReadMulti(dev, index, buf, 4U);
    if (status == VL53L1X_PLATFORM_OK) {
        *pdata = ((uint32_t)buf[0] << 24U) |
                 ((uint32_t)buf[1] << 16U) |
                 ((uint32_t)buf[2] << 8U)  |
                 (uint32_t)buf[3];
    }
    return status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
    uint32_t start_ms;
    uint32_t delay_ms;

    (void)dev;

    if (wait_ms <= 0) {
        s_last_bsp_status = BSP_OK;
        return VL53L1X_PLATFORM_OK;
    }

    delay_ms = (uint32_t)wait_ms;
    start_ms = BSP_GetTickMs();
    while ((uint32_t)(BSP_GetTickMs() - start_ms) < delay_ms) {
        /* 仅供官方 ULD SensorInit() 启动阶段使用。SysTick 中断继续运行。 */
    }

    s_last_bsp_status = BSP_OK;
    return VL53L1X_PLATFORM_OK;
}
