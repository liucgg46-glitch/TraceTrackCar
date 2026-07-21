#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "bsp_common.h"
#include "stm32f4xx_gpio.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 通用 GPIO BSP
 * ============================================================================
 * 定位：只负责 GPIO 输入/输出本身，不写电机方向逻辑、按键逻辑、SPI 协议逻辑。
 *
 * 移植方法：
 *   只改本文件“GPIO 通道配置区”。bsp_gpio.c 不需要改。
 *
 * 当前电赛小车推荐规划 v1.4：
 *   CH1  ：PC13，板载 LED / 调试输出模板
 *   CH2  ：PB12，SPI2 设备 CS 默认片选
 *   CH3~CH10：四电机方向脚，避开 USART2 的 PD5/PD6
 *   CH11~CH13：74HC4051 灰度模块地址选择脚 S0/S1/S2
 *   CH14~CH15：SPI TFT LCD 控制脚 DC/BL
 *   CH16 ：PE5，ICM-20948 SPI2 独立片选 CS
 *   CH17 ：PG2，激光使能
 *   CH18 ：PE6，E220 AUX 输入（仅无线 USART1 模式启用）
 *   CH19~CH20：PG3/PG4，HX711 DOUT/PD_SCK
 *   CH21~CH22：PG5/PG6，红灯/绿灯控制输出
 *
 * 电机方向脚规划：
 *   M1_IN1 = PD0, M1_IN2 = PD1
 *   M2_IN1 = PD2, M2_IN2 = PD3
 *   M3_IN1 = PD4, M3_IN2 = PD7
 *   M4_IN1 = PD8, M4_IN2 = PD9
 *
 * 注意：
 *   - 电机方向含义不要写在 BSP 层，后续 drv_motor.c 再把 CH3~CH10 映射成 FL/FR/RL/RR；
 *   - 如果你的驱动板方向脚接线不同，只改这里的端口和引脚；
 *   - PB12 只作为一个默认 SPI CS，多个 SPI 设备时可以继续增加 GPIO 通道做 CS；
 *   - 这里刻意不占用 PD5/PD6，方便 USART2 继续作为视觉/无线串口备用；
 *   - 74HC4051 的 OUT/SIG/AO 接 ADC，S0/S1/S2 只作为普通 GPIO 输出。
 */

/* CH1：PC13 输出，可作为板载 LED 模板 */
#define BSP_GPIO_CH1_ENABLE          1
#define BSP_GPIO_CH1_PORT            GPIOC
#define BSP_GPIO_CH1_PIN             GPIO_Pin_13
#define BSP_GPIO_CH1_MODE            GPIO_Mode_OUT
#define BSP_GPIO_CH1_OTYPE           GPIO_OType_PP
#define BSP_GPIO_CH1_PUPD            GPIO_PuPd_NOPULL
#define BSP_GPIO_CH1_SPEED           GPIO_Speed_50MHz
#define BSP_GPIO_CH1_INIT_LEVEL      1U

/* CH2：SPI2 默认 CS，PB12，默认拉高不选中 */
#define BSP_GPIO_CH2_ENABLE          1
#define BSP_GPIO_CH2_PORT            GPIOB
#define BSP_GPIO_CH2_PIN             GPIO_Pin_12
#define BSP_GPIO_CH2_MODE            GPIO_Mode_OUT
#define BSP_GPIO_CH2_OTYPE           GPIO_OType_PP
#define BSP_GPIO_CH2_PUPD            GPIO_PuPd_UP
#define BSP_GPIO_CH2_SPEED           GPIO_Speed_50MHz
#define BSP_GPIO_CH2_INIT_LEVEL      1U

/* CH3~CH10：四个电机方向脚模板，后续 drv_motor.c 再做具体车轮映射 */
#define BSP_GPIO_CH3_ENABLE          1
#define BSP_GPIO_CH3_PORT            GPIOD
#define BSP_GPIO_CH3_PIN             GPIO_Pin_0
#define BSP_GPIO_CH3_MODE            GPIO_Mode_OUT
#define BSP_GPIO_CH3_OTYPE           GPIO_OType_PP
#define BSP_GPIO_CH3_PUPD            GPIO_PuPd_NOPULL
#define BSP_GPIO_CH3_SPEED           GPIO_Speed_50MHz
#define BSP_GPIO_CH3_INIT_LEVEL      0U

#define BSP_GPIO_CH4_ENABLE          1
#define BSP_GPIO_CH4_PORT            GPIOD
#define BSP_GPIO_CH4_PIN             GPIO_Pin_1
#define BSP_GPIO_CH4_MODE            GPIO_Mode_OUT
#define BSP_GPIO_CH4_OTYPE           GPIO_OType_PP
#define BSP_GPIO_CH4_PUPD            GPIO_PuPd_NOPULL
#define BSP_GPIO_CH4_SPEED           GPIO_Speed_50MHz
#define BSP_GPIO_CH4_INIT_LEVEL      0U

#define BSP_GPIO_CH5_ENABLE          1
#define BSP_GPIO_CH5_PORT            GPIOD
#define BSP_GPIO_CH5_PIN             GPIO_Pin_2
#define BSP_GPIO_CH5_MODE            GPIO_Mode_OUT
#define BSP_GPIO_CH5_OTYPE           GPIO_OType_PP
#define BSP_GPIO_CH5_PUPD            GPIO_PuPd_NOPULL
#define BSP_GPIO_CH5_SPEED           GPIO_Speed_50MHz
#define BSP_GPIO_CH5_INIT_LEVEL      0U

#define BSP_GPIO_CH6_ENABLE          1
#define BSP_GPIO_CH6_PORT            GPIOD
#define BSP_GPIO_CH6_PIN             GPIO_Pin_3
#define BSP_GPIO_CH6_MODE            GPIO_Mode_OUT
#define BSP_GPIO_CH6_OTYPE           GPIO_OType_PP
#define BSP_GPIO_CH6_PUPD            GPIO_PuPd_NOPULL
#define BSP_GPIO_CH6_SPEED           GPIO_Speed_50MHz
#define BSP_GPIO_CH6_INIT_LEVEL      0U

#define BSP_GPIO_CH7_ENABLE          VEHICLE_REAR_DRIVE_ENABLE
#define BSP_GPIO_CH7_PORT            GPIOD
#define BSP_GPIO_CH7_PIN             GPIO_Pin_4
#define BSP_GPIO_CH7_MODE            GPIO_Mode_OUT
#define BSP_GPIO_CH7_OTYPE           GPIO_OType_PP
#define BSP_GPIO_CH7_PUPD            GPIO_PuPd_NOPULL
#define BSP_GPIO_CH7_SPEED           GPIO_Speed_50MHz
#define BSP_GPIO_CH7_INIT_LEVEL      0U

#define BSP_GPIO_CH8_ENABLE          VEHICLE_REAR_DRIVE_ENABLE
#define BSP_GPIO_CH8_PORT            GPIOD
#define BSP_GPIO_CH8_PIN             GPIO_Pin_7
#define BSP_GPIO_CH8_MODE            GPIO_Mode_OUT
#define BSP_GPIO_CH8_OTYPE           GPIO_OType_PP
#define BSP_GPIO_CH8_PUPD            GPIO_PuPd_NOPULL
#define BSP_GPIO_CH8_SPEED           GPIO_Speed_50MHz
#define BSP_GPIO_CH8_INIT_LEVEL      0U

#define BSP_GPIO_CH9_ENABLE          VEHICLE_REAR_DRIVE_ENABLE
#define BSP_GPIO_CH9_PORT            GPIOD
#define BSP_GPIO_CH9_PIN             GPIO_Pin_8
#define BSP_GPIO_CH9_MODE            GPIO_Mode_OUT
#define BSP_GPIO_CH9_OTYPE           GPIO_OType_PP
#define BSP_GPIO_CH9_PUPD            GPIO_PuPd_NOPULL
#define BSP_GPIO_CH9_SPEED           GPIO_Speed_50MHz
#define BSP_GPIO_CH9_INIT_LEVEL      0U

#define BSP_GPIO_CH10_ENABLE         VEHICLE_REAR_DRIVE_ENABLE
#define BSP_GPIO_CH10_PORT           GPIOD
#define BSP_GPIO_CH10_PIN            GPIO_Pin_9
#define BSP_GPIO_CH10_MODE           GPIO_Mode_OUT
#define BSP_GPIO_CH10_OTYPE          GPIO_OType_PP
#define BSP_GPIO_CH10_PUPD           GPIO_PuPd_NOPULL
#define BSP_GPIO_CH10_SPEED          GPIO_Speed_50MHz
#define BSP_GPIO_CH10_INIT_LEVEL     0U

/* ---------------------------------------------------------------------------
 * CH11~CH13：74HC4051 灰度传感器地址选择脚
 *
 * 适用模块：8 路灰度传感器，74HC4051 多路复用纯模拟版，无 MCU 版本。
 * 连接方式：
 *   灰度 OUT / SIG / AO -> 1 路 ADC，例如 PC0 ADC1_IN10；
 *   S0 / S1 / S2       -> 3 路普通 GPIO 输出。
 *
 * 当前规划：
 *   S0 -> PD10
 *   S1 -> PD11
 *   S2 -> PD12
 *
 * 注意：
 *   这里不写灰度阈值、不写循迹逻辑，只提供 GPIO 输出脚。
 *   选择 0~7 路通道的 Gray4051_Select() 应放在 drv_gray_4051.c 或测试文件中。
 * ------------------------------------------------------------------------- */
#define BSP_GPIO_CH11_ENABLE         1
#define BSP_GPIO_CH11_PORT           GPIOD
#define BSP_GPIO_CH11_PIN            GPIO_Pin_10
#define BSP_GPIO_CH11_MODE           GPIO_Mode_OUT
#define BSP_GPIO_CH11_OTYPE          GPIO_OType_PP
#define BSP_GPIO_CH11_PUPD           GPIO_PuPd_NOPULL
#define BSP_GPIO_CH11_SPEED          GPIO_Speed_50MHz
#define BSP_GPIO_CH11_INIT_LEVEL     0U

#define BSP_GPIO_CH12_ENABLE         1
#define BSP_GPIO_CH12_PORT           GPIOD
#define BSP_GPIO_CH12_PIN            GPIO_Pin_11
#define BSP_GPIO_CH12_MODE           GPIO_Mode_OUT
#define BSP_GPIO_CH12_OTYPE          GPIO_OType_PP
#define BSP_GPIO_CH12_PUPD           GPIO_PuPd_NOPULL
#define BSP_GPIO_CH12_SPEED          GPIO_Speed_50MHz
#define BSP_GPIO_CH12_INIT_LEVEL     0U

#define BSP_GPIO_CH13_ENABLE         1
#define BSP_GPIO_CH13_PORT           GPIOD
#define BSP_GPIO_CH13_PIN            GPIO_Pin_12
#define BSP_GPIO_CH13_MODE           GPIO_Mode_OUT
#define BSP_GPIO_CH13_OTYPE          GPIO_OType_PP
#define BSP_GPIO_CH13_PUPD           GPIO_PuPd_NOPULL
#define BSP_GPIO_CH13_SPEED          GPIO_Speed_50MHz
#define BSP_GPIO_CH13_INIT_LEVEL     0U

/* CH14~CH15: 1.54 inch SPI TFT LCD control pins. */
#define BSP_GPIO_CH14_ENABLE         1
#define BSP_GPIO_CH14_PORT           GPIOC
#define BSP_GPIO_CH14_PIN            GPIO_Pin_10
#define BSP_GPIO_CH14_MODE           GPIO_Mode_OUT
#define BSP_GPIO_CH14_OTYPE          GPIO_OType_PP
#define BSP_GPIO_CH14_PUPD           GPIO_PuPd_NOPULL
#define BSP_GPIO_CH14_SPEED          GPIO_Speed_50MHz
#define BSP_GPIO_CH14_INIT_LEVEL     1U

#define BSP_GPIO_CH15_ENABLE         1
#define BSP_GPIO_CH15_PORT           GPIOC
#define BSP_GPIO_CH15_PIN            GPIO_Pin_11
#define BSP_GPIO_CH15_MODE           GPIO_Mode_OUT
#define BSP_GPIO_CH15_OTYPE          GPIO_OType_PP
#define BSP_GPIO_CH15_PUPD           GPIO_PuPd_NOPULL
#define BSP_GPIO_CH15_SPEED          GPIO_Speed_50MHz
#define BSP_GPIO_CH15_INIT_LEVEL     0U

/* CH16：ICM-20948 独立 SPI2 CS，PE5，默认拉高不选中。
 * SPI2 的 SCK/MISO/MOSI 可与 LCD 共用，但每个设备必须使用独立 CS。 */
#define BSP_GPIO_CH16_ENABLE         1
#define BSP_GPIO_CH16_PORT           GPIOE
#define BSP_GPIO_CH16_PIN            GPIO_Pin_5
#define BSP_GPIO_CH16_MODE           GPIO_Mode_OUT
#define BSP_GPIO_CH16_OTYPE          GPIO_OType_PP
#define BSP_GPIO_CH16_PUPD           GPIO_PuPd_UP
#define BSP_GPIO_CH16_SPEED          GPIO_Speed_50MHz
#define BSP_GPIO_CH16_INIT_LEVEL     1U

/* CH17：激光MOS模块使能，PG2，高电平打开 */
#define BSP_GPIO_CH17_ENABLE         1
#define BSP_GPIO_CH17_PORT           GPIOG
#define BSP_GPIO_CH17_PIN            GPIO_Pin_2
#define BSP_GPIO_CH17_MODE           GPIO_Mode_OUT
#define BSP_GPIO_CH17_OTYPE          GPIO_OType_PP
#define BSP_GPIO_CH17_PUPD           GPIO_PuPd_DOWN
#define BSP_GPIO_CH17_SPEED          GPIO_Speed_50MHz
#define BSP_GPIO_CH17_INIT_LEVEL     0U

/*
 * CH18：E220 AUX 忙闲状态输入，使用 PE6。
 *
 * AUX 是 E220 的输出：低电平表示忙，高电平表示空闲。该引脚不能配置为
 * MCU 输出，也不能在外部下拉。本通道不使用内部上下拉，避免 MCU 在模块
 * 上电期间影响 AUX 电平。
 *
 * 当前源码配置未占用 PE6；实际开发板是否已引出 PE6，仍需结合板级硬件确认。
 */
#define BSP_GPIO_CH18_ENABLE         VEHICLE_UART1_E220_ENABLE
#define BSP_GPIO_CH18_PORT           GPIOE
#define BSP_GPIO_CH18_PIN            GPIO_Pin_6
#define BSP_GPIO_CH18_MODE           GPIO_Mode_IN
#define BSP_GPIO_CH18_OTYPE          GPIO_OType_PP
#define BSP_GPIO_CH18_PUPD           GPIO_PuPd_NOPULL
#define BSP_GPIO_CH18_SPEED          GPIO_Speed_2MHz
#define BSP_GPIO_CH18_INIT_LEVEL     0U

/* CH19~CH20：HX711 称重 ADC 两线数字接口。
 * 当前源码中 PG3/PG4 未被其他已启用外设占用；实际开发板引出位置仍需按板卡核对。
 * DOUT 是 HX711 推挽输出，内部上拉用于模块断线时保持确定高电平。
 * PD_SCK 上电必须为低，避免 HX711 进入掉电模式。
 */
#define BSP_GPIO_CH19_ENABLE         1
#define BSP_GPIO_CH19_PORT           GPIOG
#define BSP_GPIO_CH19_PIN            GPIO_Pin_3
#define BSP_GPIO_CH19_MODE           GPIO_Mode_IN
#define BSP_GPIO_CH19_OTYPE          GPIO_OType_PP
#define BSP_GPIO_CH19_PUPD           GPIO_PuPd_UP
#define BSP_GPIO_CH19_SPEED          GPIO_Speed_2MHz
#define BSP_GPIO_CH19_INIT_LEVEL     0U

#define BSP_GPIO_CH20_ENABLE         1
#define BSP_GPIO_CH20_PORT           GPIOG
#define BSP_GPIO_CH20_PIN            GPIO_Pin_4
#define BSP_GPIO_CH20_MODE           GPIO_Mode_OUT
#define BSP_GPIO_CH20_OTYPE          GPIO_OType_PP
#define BSP_GPIO_CH20_PUPD           GPIO_PuPd_NOPULL
#define BSP_GPIO_CH20_SPEED          GPIO_Speed_50MHz
#define BSP_GPIO_CH20_INIT_LEVEL     0U

/* CH21~CH22：送药小车红绿状态灯控制。
 * PG5/PG6 在当前源码中未被其他外设占用；开发板实际引出位置仍需按板卡确认。
 * 默认按高电平点亮、低电平熄灭接线，上电时两灯均保持熄灭。
 * 若实物模块为低电平有效，应修改对应 ACTIVE_LEVEL，初始化电平会自动取反。 */
#define BSP_GPIO_STATUS_RED_ACTIVE_LEVEL     1U
#define BSP_GPIO_STATUS_GREEN_ACTIVE_LEVEL   1U

#define BSP_GPIO_CH21_ENABLE         1
#define BSP_GPIO_CH21_PORT           GPIOG
#define BSP_GPIO_CH21_PIN            GPIO_Pin_5
#define BSP_GPIO_CH21_MODE           GPIO_Mode_OUT
#define BSP_GPIO_CH21_OTYPE          GPIO_OType_PP
#define BSP_GPIO_CH21_PUPD           GPIO_PuPd_DOWN
#define BSP_GPIO_CH21_SPEED          GPIO_Speed_2MHz
#define BSP_GPIO_CH21_INIT_LEVEL     ((BSP_GPIO_STATUS_RED_ACTIVE_LEVEL != 0U) ? 0U : 1U)

#define BSP_GPIO_CH22_ENABLE         1
#define BSP_GPIO_CH22_PORT           GPIOG
#define BSP_GPIO_CH22_PIN            GPIO_Pin_6
#define BSP_GPIO_CH22_MODE           GPIO_Mode_OUT
#define BSP_GPIO_CH22_OTYPE          GPIO_OType_PP
#define BSP_GPIO_CH22_PUPD           GPIO_PuPd_DOWN
#define BSP_GPIO_CH22_SPEED          GPIO_Speed_2MHz
#define BSP_GPIO_CH22_INIT_LEVEL     ((BSP_GPIO_STATUS_GREEN_ACTIVE_LEVEL != 0U) ? 0U : 1U)

/* 只有 ENABLE=1 的通道会进入枚举，业务代码不要使用魔法数字。 */
typedef enum {
#if BSP_GPIO_CH1_ENABLE
    BSP_GPIO_CH1,
#endif
#if BSP_GPIO_CH2_ENABLE
    BSP_GPIO_CH2,
#endif
#if BSP_GPIO_CH3_ENABLE
    BSP_GPIO_CH3,
#endif
#if BSP_GPIO_CH4_ENABLE
    BSP_GPIO_CH4,
#endif
#if BSP_GPIO_CH5_ENABLE
    BSP_GPIO_CH5,
#endif
#if BSP_GPIO_CH6_ENABLE
    BSP_GPIO_CH6,
#endif
#if BSP_GPIO_CH7_ENABLE
    BSP_GPIO_CH7,
#endif
#if BSP_GPIO_CH8_ENABLE
    BSP_GPIO_CH8,
#endif
#if BSP_GPIO_CH9_ENABLE
    BSP_GPIO_CH9,
#endif
#if BSP_GPIO_CH10_ENABLE
    BSP_GPIO_CH10,
#endif
#if BSP_GPIO_CH11_ENABLE
    BSP_GPIO_CH11,
#endif
#if BSP_GPIO_CH12_ENABLE
    BSP_GPIO_CH12,
#endif
#if BSP_GPIO_CH13_ENABLE
    BSP_GPIO_CH13,
#endif
#if BSP_GPIO_CH14_ENABLE
    BSP_GPIO_CH14,
#endif
#if BSP_GPIO_CH15_ENABLE
    BSP_GPIO_CH15,
#endif
#if BSP_GPIO_CH16_ENABLE
    BSP_GPIO_CH16,
#endif
#if BSP_GPIO_CH17_ENABLE
    BSP_GPIO_CH17,
#endif
#if BSP_GPIO_CH18_ENABLE
    BSP_GPIO_CH18,
#endif
#if BSP_GPIO_CH19_ENABLE
    BSP_GPIO_CH19,
#endif
#if BSP_GPIO_CH20_ENABLE
    BSP_GPIO_CH20,
#endif
#if BSP_GPIO_CH21_ENABLE
    BSP_GPIO_CH21,
#endif
#if BSP_GPIO_CH22_ENABLE
    BSP_GPIO_CH22,
#endif
    BSP_GPIO_COUNT
} BSP_GPIO_Id_t;

/* 设备层别名：灰度 4051 地址选择脚。
 * 后续 drv_gray_4051.c 使用这些别名，不要直接写 CH11/CH12/CH13。 */
#define BSP_GPIO_GRAY_S0   BSP_GPIO_CH11
#define BSP_GPIO_GRAY_S1   BSP_GPIO_CH12
#define BSP_GPIO_GRAY_S2   BSP_GPIO_CH13

/* 1.54 inch SPI TFT LCD aliases. */
#define BSP_GPIO_LCD_CS    BSP_GPIO_CH2
#define BSP_GPIO_LCD_DC    BSP_GPIO_CH14
#define BSP_GPIO_LCD_BL    BSP_GPIO_CH15

/* ICM-20948 SPI2 independent chip-select alias. */
#define BSP_GPIO_ICM20948_CS BSP_GPIO_CH16

#define BSP_GPIO_LASER_EN    BSP_GPIO_CH17

#if BSP_GPIO_CH18_ENABLE
#define BSP_GPIO_E220_AUX     BSP_GPIO_CH18
#endif

/* HX711 数据输出与时钟/掉电控制脚。 */
#define BSP_GPIO_HX711_DOUT    BSP_GPIO_CH19
#define BSP_GPIO_HX711_PD_SCK  BSP_GPIO_CH20

/* 送药小车红绿状态灯控制脚。 */
#define BSP_GPIO_STATUS_RED     BSP_GPIO_CH21
#define BSP_GPIO_STATUS_GREEN   BSP_GPIO_CH22

void       BSP_GPIO_Init(BSP_GPIO_Id_t id);
void       BSP_GPIO_InitAll(void);
void       BSP_GPIO_Write(BSP_GPIO_Id_t id, uint8_t level);
void       BSP_GPIO_Toggle(BSP_GPIO_Id_t id);
uint8_t    BSP_GPIO_Read(BSP_GPIO_Id_t id);
GPIO_TypeDef *BSP_GPIO_GetPort(BSP_GPIO_Id_t id);
uint16_t   BSP_GPIO_GetPin(BSP_GPIO_Id_t id);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_GPIO_H */
