#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
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

    /* 初始化GPIO、串口、PWM、I2C等BSP外设 */
    (void)BSP_InitAll(SystemCoreClock);

    /* 初始化电机、编码器、灰度等驱动 */
    Driver_Init();

    /* 初始化循迹、K210通信等应用 */
    App_Init();

    /* 初始化任务调度器 */
    Scheduler_Init();

    while (1)
    {
        /*
         * 持续搬运UART DMA收到的数据。
         * 防止K210连续发送时，只有IDLE中断而处理不及时。
         */
        BSP_UART_TaskAll();

        Scheduler_Run();
    }
}