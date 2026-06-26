#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "bsp_led.h"
#include "bsp_uart.h"
#include "bsp_i2c.h"
#include "bsp_pwm.h"
#include "bsp_all.h"
#include "scheduler.h"
#include "driver_all.h"
#include "app_all.h"

#include "test.h"

int main(void)
{
    SystemInit();

    (void)BSP_InitAll(SystemCoreClock);
	Driver_Init();
	App_Init();

    Scheduler_Init();

    while (1) {
        Scheduler_Run();
    }
}
