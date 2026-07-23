#include "drv_status_light.h"
#include "bsp_gpio.h"

static Drv_StatusLight_Mode_t s_status_light_mode;

static void Drv_StatusLight_Write(
    BSP_GPIO_Id_t gpio,
    uint8_t active_level,
    uint8_t turn_on
)
{
    uint8_t output_level;

    if (turn_on != 0U) {
        output_level = active_level;
    } else {
        output_level = (active_level != 0U) ? 0U : 1U;
    }

    BSP_GPIO_Write(gpio, output_level);
}

void Drv_StatusLight_Init(void)
{
    s_status_light_mode = DRV_STATUS_LIGHT_OFF;
    Drv_StatusLight_Write(
        BSP_GPIO_STATUS_RED,
        BSP_GPIO_STATUS_RED_ACTIVE_LEVEL,
        0U
    );
    Drv_StatusLight_Write(
        BSP_GPIO_STATUS_GREEN,
        BSP_GPIO_STATUS_GREEN_ACTIVE_LEVEL,
        0U
    );
}

BSP_Status_t Drv_StatusLight_SetMode(Drv_StatusLight_Mode_t mode)
{
    if ((mode != DRV_STATUS_LIGHT_OFF) &&
        (mode != DRV_STATUS_LIGHT_RED) &&
        (mode != DRV_STATUS_LIGHT_GREEN)) {
        return BSP_PARAM;
    }

    /* 先熄灭两灯再点亮目标灯，确保任何切换路径都不会出现同时点亮。 */
    Drv_StatusLight_Write(
        BSP_GPIO_STATUS_RED,
        BSP_GPIO_STATUS_RED_ACTIVE_LEVEL,
        0U
    );
    Drv_StatusLight_Write(
        BSP_GPIO_STATUS_GREEN,
        BSP_GPIO_STATUS_GREEN_ACTIVE_LEVEL,
        0U
    );

    if (mode == DRV_STATUS_LIGHT_RED) {
        Drv_StatusLight_Write(
            BSP_GPIO_STATUS_RED,
            BSP_GPIO_STATUS_RED_ACTIVE_LEVEL,
            1U
        );
    } else if (mode == DRV_STATUS_LIGHT_GREEN) {
        Drv_StatusLight_Write(
            BSP_GPIO_STATUS_GREEN,
            BSP_GPIO_STATUS_GREEN_ACTIVE_LEVEL,
            1U
        );
    }

    s_status_light_mode = mode;
    return BSP_OK;
}

void Drv_StatusLight_Off(void)
{
    (void)Drv_StatusLight_SetMode(DRV_STATUS_LIGHT_OFF);
}

void Drv_StatusLight_SetRed(void)
{
    (void)Drv_StatusLight_SetMode(DRV_STATUS_LIGHT_RED);
}

void Drv_StatusLight_SetGreen(void)
{
    (void)Drv_StatusLight_SetMode(DRV_STATUS_LIGHT_GREEN);
}

Drv_StatusLight_Mode_t Drv_StatusLight_GetMode(void)
{
    return s_status_light_mode;
}