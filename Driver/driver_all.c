#include "driver_all.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "drv_gray_4051.h"
#include "drv_lcd_tft.h"

void Driver_Init(void)
{
    /* 电机 PWM + 方向 GPIO 组合层。BSP_InitAll() 已经初始化底层 PWM/GPIO。 */
    Motor_Init();

    /* 四轮编码器映射层。BSP_InitAll() 已经初始化底层 TIM 编码器。 */
    Drv_Encoder_Init();

    /* 74HC4051 灰度模块驱动层：只负责 8 路 raw/filt 采样。 */
    Drv_Gray4051_Init();

    Drv_LcdTft_Init();

    /* 后续 Driver 层模块统一从这里继续添加 Init。 */
}
