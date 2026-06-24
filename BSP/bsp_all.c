#include "bsp_all.h"

BSP_Status_t BSP_InitAll(uint32_t system_core_clock_hz)
{
    BSP_Status_t ret;

    ret = BSP_SysTick_Init(system_core_clock_hz);
    if (ret != BSP_OK) return ret;

    BSP_GPIO_InitAll();
    BSP_PWM_InitAll();
    BSP_Encoder_InitAll();
    BSP_ADC_Init();
    BSP_Key_InitAll();
    BSP_EXTI_InitAll();

    BSP_UART_InitAll();
    BSP_I2C_InitAll();
    BSP_SPI_InitAll();

    return BSP_OK;
}

void BSP_TaskAll(void)
{
    BSP_UART_TaskAll();
    BSP_I2C_TaskAll();
    BSP_SPI_TaskAll();
}
