#include "driver_all.h"
#include "drv_motor.h"
#include "drv_gray_sensor.h"
#include "drv_vl53l1x.h"
#include "drv_icm20948.h"
#include "drv_hx711.h"
#include "drv_lcd_tft.h"
#include "drv_oled_i2c.h"
#include "drv_e220.h"

void Driver_Init(void)
{
    /* USART1 的普通直连或 E220 模式由 BSP/vehicle_config.h 选择。 */
    Drv_E220_Init();

    /* 电机 PWM + 方向 GPIO 组合层。BSP_InitAll() 已经初始化底层 PWM/GPIO。 */
    Motor_Init();

    /* 四轮编码器映射层。BSP_InitAll() 已经初始化底层 TIM 编码器。 */
    /* 灰度模块驱动层：根据 drv_gray_sensor.h 选择 4051 或 MCU-I2C。 */
    Drv_GraySensor_Init();

    /* VL53L1X ToF：只初始化状态机，I2C 配置由 Sensor_Update() 分步推进。 */
    Drv_VL53L1X_Init();

    /* ICM-20948 九轴 IMU：初始化 SPI 状态机，实际配置由 Sensor_Update() 推进。 */
    Drv_ICM20948_Init();

    /* HX711 称重 ADC：初始化缓存和 PD_SCK，采样由 Sensor_Update() 非阻塞推进。 */
    Drv_HX711_Init();

    Drv_LcdTft_Init();
    Drv_OledI2c_Init();

    /* 后续 Driver 层模块统一从这里继续添加 Init。 */
}

void Driver_Task(void)
{
    Drv_E220_Task();
    Drv_LcdTft_Task();
    Drv_OledI2c_Task();
}
