#include "drv_buzzer.h"
#include "bsp_gpio.h"

static Drv_Buzzer_State_t s_buzzer_state;

static void Drv_Buzzer_Write(Drv_Buzzer_State_t state)
{
    uint8_t output_level;

    if (state == DRV_BUZZER_ON) {
        output_level = BSP_GPIO_BUZZER_ACTIVE_LEVEL;
    } else {
        output_level = (BSP_GPIO_BUZZER_ACTIVE_LEVEL != 0U) ? 0U : 1U;
    }

    BSP_GPIO_Write(BSP_GPIO_BUZZER, output_level);
}

void Drv_Buzzer_Init(void)
{
    s_buzzer_state = DRV_BUZZER_OFF;
    Drv_Buzzer_Write(DRV_BUZZER_OFF);
}

BSP_Status_t Drv_Buzzer_SetState(Drv_Buzzer_State_t state)
{
    if ((state != DRV_BUZZER_OFF) && (state != DRV_BUZZER_ON)) {
        return BSP_PARAM;
    }

    Drv_Buzzer_Write(state);
    s_buzzer_state = state;
    return BSP_OK;
}

void Drv_Buzzer_On(void)
{
    (void)Drv_Buzzer_SetState(DRV_BUZZER_ON);
}

void Drv_Buzzer_Off(void)
{
    (void)Drv_Buzzer_SetState(DRV_BUZZER_OFF);
}

void Drv_Buzzer_Toggle(void)
{
    if (s_buzzer_state == DRV_BUZZER_ON) {
        Drv_Buzzer_Off();
    } else {
        Drv_Buzzer_On();
    }
}

Drv_Buzzer_State_t Drv_Buzzer_GetState(void)
{
    return s_buzzer_state;
}
