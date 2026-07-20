#include "drv_laser.h"
#include "bsp_gpio.h"
#include "bsp_systick.h"

/* 当前 MOS 模块按高电平有效配置；实机接线仍需在正式验收前复核。 */
#define DRV_LASER_ACTIVE_LEVEL             1U

/*
 * 这是针对当前正方形测试流程的软件保护时长，不代表激光器硬件额定值。
 * 正常描边点亮约 16 s，额外保留 2 s 调度余量。
 */
#define DRV_LASER_MAX_CONTINUOUS_ON_MS     18000U

static uint8_t s_laser_is_on;
static uint8_t s_timeout_tripped;
static uint32_t s_laser_on_start_ms;

static void Drv_Laser_WriteHardware(uint8_t enable)
{
    uint8_t output_level;

    if (enable != 0U) {
        output_level = DRV_LASER_ACTIVE_LEVEL;
    } else {
        output_level = (DRV_LASER_ACTIVE_LEVEL != 0U) ? 0U : 1U;
    }
    BSP_GPIO_Write(BSP_GPIO_LASER_EN, output_level);
}

void Drv_Laser_Init(void)
{
    s_laser_is_on = 0U;
    s_timeout_tripped = 0U;
    s_laser_on_start_ms = BSP_GetTickMs();
    Drv_Laser_WriteHardware(0U);
}

void Drv_Laser_Task(void)
{
    if ((s_laser_is_on != 0U) &&
        (BSP_IsTimeout(
            s_laser_on_start_ms,
            DRV_LASER_MAX_CONTINUOUS_ON_MS
        ) != 0U)) {
        Drv_Laser_Off();
        s_timeout_tripped = 1U;
    }
}

void Drv_Laser_On(void)
{
    if (s_laser_is_on == 0U) {
        s_laser_on_start_ms = BSP_GetTickMs();
    }
    Drv_Laser_WriteHardware(1U);
    s_laser_is_on = 1U;
}

void Drv_Laser_Off(void)
{
    Drv_Laser_WriteHardware(0U);
    s_laser_is_on = 0U;
}

uint8_t Drv_Laser_IsOn(void)
{
    return s_laser_is_on;
}

BSP_Status_t Drv_Laser_GetInfo(Drv_Laser_Info_t *info)
{
    if (info == 0) {
        return BSP_PARAM;
    }

    info->is_on = s_laser_is_on;
    info->timeout_tripped = s_timeout_tripped;
    return BSP_OK;
}

void Drv_Laser_ClearTimeoutFlag(void)
{
    s_timeout_tripped = 0U;
}
