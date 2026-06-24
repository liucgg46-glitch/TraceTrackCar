#include "bsp_led.h"

void led_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* 1. Enable Clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
    /* 2. Config GPIO Structure GPIO_Low_Speed*/
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin   = LED_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
}

void led_control(led_state_t status)
{
    if (status == LED_ON) {
        GPIO_ResetBits(LED_PORT, LED_PIN);
    } else {
        GPIO_SetBits(LED_PORT, LED_PIN);
    }
}

/* LED 任务：每 500ms 翻转一次 LED */
void task_led_blink(void) 
{
	GPIO_ToggleBits(GPIOC, GPIO_Pin_13);
}
