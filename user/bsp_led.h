#ifndef _BSP_LED_H_
#define _BSP_LED_H_

#include "stm32f4xx.h"

#define LED_PORT    GPIOC
#define LED_PIN     GPIO_Pin_13

typedef enum led_status_en {
    LED_OFF = 0,
    LED_ON  = 1
} led_state_t;

void led_init(void);
void led_control(led_state_t status);
void task_led_blink(void);

#endif /* _BSP_LED_H_ */
