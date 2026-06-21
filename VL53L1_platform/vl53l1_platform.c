#include "vl53l1_platform.h"
#include "stm32f4xx.h"
#include "bsp_timer.h"
#include "bsp_i2c.h"

/*
 * STM32F407 StdPeriph port for ST VL53L1 full API.
 * 适配你这版 vl53l1_platform_user_data.h：
 *   VL53L1_Dev_t 里面的地址字段叫 i2c_slave_address，不是 I2cDevAddr。
 *
 * ST API 常用 8-bit 地址 0x52；STM32 I2C_Send7bitAddress 需要 7-bit 地址 0x29。
 * 所以下面统一做兼容：
 *   0x52 -> 0x29
 *   0x29 -> 0x29
 */

#define VL53L1_DEFAULT_ADDR_8BIT      0x52U
#define VL53L1_DEFAULT_ADDR_7BIT      0x29U
#define VL53L1_I2C_TIMEOUT            30000UL

static uint8_t VL53L1_Get7BitAddr(VL53L1_Dev_t *pdev)
{
    uint8_t addr;

    if (pdev == 0) {
        return VL53L1_DEFAULT_ADDR_7BIT;
    }

    addr = pdev->i2c_slave_address;

    if (addr == 0) {
        addr = VL53L1_DEFAULT_ADDR_8BIT;
    }

    /*
     * ST API 常保存 8-bit 地址 0x52/0x53；STM32 标准库 I2C_Send7bitAddress
     * 需要 7-bit 地址 0x29。注意 0x52 本身小于 0x7F，不能用 addr > 0x7F 判断。
     *
     * 兼容：
     *   pdev->i2c_slave_address = 0x52  -> 0x29
     *   pdev->i2c_slave_address = 0x29  -> 0x29
     */
    if (addr == 0x52U || addr == 0x53U) {
        addr >>= 1;
    }

    return addr;
}

static VL53L1_Error VL53L1_WaitEvent(uint32_t event)
{
    uint32_t timeout = VL53L1_I2C_TIMEOUT;

    while (!I2C_CheckEvent(I2C1, event)) {
        if (--timeout == 0) {
            I2C_GenerateSTOP(I2C1, ENABLE);
            I2C_ClearITPendingBit(I2C1, I2C_IT_BERR | I2C_IT_ARLO | I2C_IT_AF | I2C_IT_OVR);
            I2C_AcknowledgeConfig(I2C1, ENABLE);
            return (VL53L1_Error)-1;
        }
    }

    return VL53L1_ERROR_NONE;
}

static void VL53L1_I2C_StopAndClear(void)
{
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_ClearITPendingBit(I2C1, I2C_IT_BERR | I2C_IT_ARLO | I2C_IT_AF | I2C_IT_OVR);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

VL53L1_Error VL53L1_CommsInitialise(
    VL53L1_Dev_t *pdev,
    uint8_t       comms_type,
    uint16_t      comms_speed_khz)
{
    if (pdev == 0) {
        return (VL53L1_Error)-1;
    }

    pdev->i2c_slave_address = VL53L1_DEFAULT_ADDR_8BIT;
    pdev->comms_type = comms_type;
    pdev->comms_speed_khz = comms_speed_khz;
    pdev->new_data_ready_poll_duration_ms = 0;

    /* I2C1_Init() 在 main.c 里已经调用，这里不重复初始化总线 */
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_CommsClose(VL53L1_Dev_t *pdev)
{
    (void)pdev;
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WriteMulti(
    VL53L1_Dev_t *pdev,
    uint16_t      index,
    uint8_t      *pdata,
    uint32_t      count)
{
    uint8_t addr;
    uint32_t i;

    if (pdata == 0 || count == 0) {
        return VL53L1_ERROR_NONE;
    }

    if (I2C1_IsBusy()) {
        return (VL53L1_Error)-1;
    }

    addr = VL53L1_Get7BitAddr(pdev);

    I2C_AcknowledgeConfig(I2C1, ENABLE);

    I2C_GenerateSTART(I2C1, ENABLE);
    if (VL53L1_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) != VL53L1_ERROR_NONE) goto error;

    I2C_Send7bitAddress(I2C1, (uint8_t)(addr << 1), I2C_Direction_Transmitter);
    if (VL53L1_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != VL53L1_ERROR_NONE) goto error;

    I2C_SendData(I2C1, (uint8_t)(index >> 8));
    if (VL53L1_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != VL53L1_ERROR_NONE) goto error;

    I2C_SendData(I2C1, (uint8_t)(index & 0xFFU));
    if (VL53L1_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != VL53L1_ERROR_NONE) goto error;

    for (i = 0; i < count; i++) {
        I2C_SendData(I2C1, pdata[i]);
        if (VL53L1_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != VL53L1_ERROR_NONE) goto error;
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    return VL53L1_ERROR_NONE;

error:
    VL53L1_I2C_StopAndClear();
    return (VL53L1_Error)-1;
}

VL53L1_Error VL53L1_ReadMulti(
    VL53L1_Dev_t *pdev,
    uint16_t      index,
    uint8_t      *pdata,
    uint32_t      count)
{
    uint8_t addr;
    uint32_t i;

    if (pdata == 0 || count == 0) {
        return VL53L1_ERROR_NONE;
    }

    if (I2C1_IsBusy()) {
        return (VL53L1_Error)-1;
    }

    addr = VL53L1_Get7BitAddr(pdev);

    I2C_AcknowledgeConfig(I2C1, ENABLE);

    /* 先写入 16-bit 寄存器地址 */
    I2C_GenerateSTART(I2C1, ENABLE);
    if (VL53L1_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) != VL53L1_ERROR_NONE) goto error;

    I2C_Send7bitAddress(I2C1, (uint8_t)(addr << 1), I2C_Direction_Transmitter);
    if (VL53L1_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != VL53L1_ERROR_NONE) goto error;

    I2C_SendData(I2C1, (uint8_t)(index >> 8));
    if (VL53L1_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != VL53L1_ERROR_NONE) goto error;

    I2C_SendData(I2C1, (uint8_t)(index & 0xFFU));
    if (VL53L1_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != VL53L1_ERROR_NONE) goto error;

    /* repeated start 后读数据 */
    I2C_GenerateSTART(I2C1, ENABLE);
    if (VL53L1_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) != VL53L1_ERROR_NONE) goto error;

    I2C_Send7bitAddress(I2C1, (uint8_t)(addr << 1), I2C_Direction_Receiver);
    if (VL53L1_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != VL53L1_ERROR_NONE) goto error;

    for (i = 0; i < count; i++) {
        if (i == (count - 1U)) {
            I2C_AcknowledgeConfig(I2C1, DISABLE);
        }

        if (VL53L1_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED) != VL53L1_ERROR_NONE) goto error;
        pdata[i] = I2C_ReceiveData(I2C1);
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return VL53L1_ERROR_NONE;

error:
    VL53L1_I2C_StopAndClear();
    return (VL53L1_Error)-1;
}

VL53L1_Error VL53L1_WrByte(VL53L1_Dev_t *pdev, uint16_t index, uint8_t data)
{
    return VL53L1_WriteMulti(pdev, index, &data, 1);
}

VL53L1_Error VL53L1_WrWord(VL53L1_Dev_t *pdev, uint16_t index, uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(data >> 8);
    buf[1] = (uint8_t)(data & 0xFFU);
    return VL53L1_WriteMulti(pdev, index, buf, 2);
}

VL53L1_Error VL53L1_WrDWord(VL53L1_Dev_t *pdev, uint16_t index, uint32_t data)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)(data >> 24);
    buf[1] = (uint8_t)(data >> 16);
    buf[2] = (uint8_t)(data >> 8);
    buf[3] = (uint8_t)(data & 0xFFU);
    return VL53L1_WriteMulti(pdev, index, buf, 4);
}

VL53L1_Error VL53L1_RdByte(VL53L1_Dev_t *pdev, uint16_t index, uint8_t *pdata)
{
    return VL53L1_ReadMulti(pdev, index, pdata, 1);
}

VL53L1_Error VL53L1_RdWord(VL53L1_Dev_t *pdev, uint16_t index, uint16_t *pdata)
{
    uint8_t buf[2];
    VL53L1_Error err;

    if (pdata == 0) return (VL53L1_Error)-1;

    err = VL53L1_ReadMulti(pdev, index, buf, 2);
    if (err == VL53L1_ERROR_NONE) {
        *pdata = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    }
    return err;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_Dev_t *pdev, uint16_t index, uint32_t *pdata)
{
    uint8_t buf[4];
    VL53L1_Error err;

    if (pdata == 0) return (VL53L1_Error)-1;

    err = VL53L1_ReadMulti(pdev, index, buf, 4);
    if (err == VL53L1_ERROR_NONE) {
        *pdata = ((uint32_t)buf[0] << 24) |
                 ((uint32_t)buf[1] << 16) |
                 ((uint32_t)buf[2] << 8)  |
                 ((uint32_t)buf[3]);
    }
    return err;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us)
{
    volatile uint32_t n;
    (void)pdev;

    if (wait_us <= 0) return VL53L1_ERROR_NONE;

    /* 粗略延时，初始化阶段够用；更精确可后续改 DWT */
    n = (uint32_t)wait_us * 20U;
    while (n--) {
        __NOP();
    }
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms)
{
    uint32_t start;
    (void)pdev;

    if (wait_ms <= 0) return VL53L1_ERROR_NONE;

    start = GetTick();
    while ((GetTick() - start) < (uint32_t)wait_ms) {
        ;
    }
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
    if (ptimer_freq_hz == 0) return (VL53L1_Error)-1;
    *ptimer_freq_hz = 1000;
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTimerValue(int32_t *ptimer_count)
{
    if (ptimer_count == 0) return (VL53L1_Error)-1;
    *ptimer_count = (int32_t)GetTick();
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTickCount(uint32_t *ptime_ms)
{
    if (ptime_ms == 0) return (VL53L1_Error)-1;
    *ptime_ms = GetTick();
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioSetMode(uint8_t pin, uint8_t mode)
{
    (void)pin;
    (void)mode;
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioSetValue(uint8_t pin, uint8_t value)
{
    (void)pin;
    (void)value;
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioGetValue(uint8_t pin, uint8_t *pvalue)
{
    (void)pin;
    if (pvalue != 0) *pvalue = 1;
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioXshutdown(uint8_t value)
{
    (void)value;
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioCommsSelect(uint8_t value)
{
    (void)value;
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioPowerEnable(uint8_t value)
{
    (void)value;
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioInterruptEnable(void (*function)(void), uint8_t edge_type)
{
    (void)function;
    (void)edge_type;
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioInterruptDisable(void)
{
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitValueMaskEx(
    VL53L1_Dev_t *pdev,
    uint32_t      timeout_ms,
    uint16_t      index,
    uint8_t       value,
    uint8_t       mask,
    uint32_t      poll_delay_ms)
{
    uint32_t start;
    uint8_t data;
    VL53L1_Error err;

    start = GetTick();

    while ((GetTick() - start) < timeout_ms) {
        err = VL53L1_RdByte(pdev, index, &data);
        if (err != VL53L1_ERROR_NONE) {
            return err;
        }

        if ((data & mask) == value) {
            if (pdev != 0) {
                pdev->new_data_ready_poll_duration_ms = GetTick() - start;
            }
            return VL53L1_ERROR_NONE;
        }

        (void)VL53L1_WaitMs(pdev, (int32_t)poll_delay_ms);
    }

    if (pdev != 0) {
        pdev->new_data_ready_poll_duration_ms = GetTick() - start;
    }

    return (VL53L1_Error)-1;
}
