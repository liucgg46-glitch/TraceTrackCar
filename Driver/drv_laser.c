#include "drv_laser.h"
#include "bsp_gpio.h"

static uint8_t s_laser_is_on;

static void Drv_Laser_WriteHardware(uint8_t enable)
{
    uint8_t output_level;

    if (enable != 0U) {
        output_level = DRV_LASER_ACTIVE_LEVEL;
    } else {
        output_level =
            (DRV_LASER_ACTIVE_LEVEL != 0U) ? 0U : 1U;
    }

    BSP_GPIO_Write(BSP_GPIO_LASER_EN, output_level);
}

void Drv_Laser_Init(void)
{
    /*
     * PG2已经由BSP_InitAll()初始化。
     * Driver层只负责保证激光上电默认关闭。
     */
    s_laser_is_on = 0U;
    Drv_Laser_WriteHardware(0U);
}

void Drv_Laser_On(void)
{
    Drv_Laser_WriteHardware(1U);
    s_laser_is_on = 1U;
}

void Drv_Laser_Off(void)
{
    Drv_Laser_WriteHardware(0U);
    s_laser_is_on = 0U;
}

void Drv_Laser_Set(uint8_t enable)
{
    if (enable != 0U) {
        Drv_Laser_On();
    } else {
        Drv_Laser_Off();
    }
}

void Drv_Laser_Toggle(void)
{
    if (s_laser_is_on != 0U) {
        Drv_Laser_Off();
    } else {
        Drv_Laser_On();
    }
}

uint8_t Drv_Laser_IsOn(void)
{
    return s_laser_is_on;
}