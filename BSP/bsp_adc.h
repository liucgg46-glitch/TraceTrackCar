#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "bsp_common.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 通用 ADC BSP（ADC1 + DMA Circular）
 * ============================================================================
 * 定位：只负责 ADC 原始值采样，不负责灰度阈值、电池电压换算等业务逻辑。
 *
 * 移植方法：
 *   只改本文件配置区：ADC 通道、GPIO、DMA Stream/Channel。
 *   bsp_adc.c 不需要改。
 *
 * 注意：
 *   - 默认使用 ADC1 + DMA2_Stream4 + Channel0，避免和常见 SPI1_RX 的
 *     DMA2_Stream0 冲突；如果你工程里 DMA2_Stream4 被占用，请在这里改。
 *   - ADC 启动后持续 DMA 循环采样，BSP_ADC_GetRaw() 只读取最近一次值，非阻塞。
 */

#define BSP_ADC_PERIPH                    ADC1
#define BSP_ADC_CLOCK_MASK                RCC_APB2Periph_ADC1
#define BSP_ADC_DMA_CLOCK_MASK            RCC_AHB1Periph_DMA2
#define BSP_ADC_DMA_STREAM                DMA2_Stream4
#define BSP_ADC_DMA_STREAM_NUM            4
#define BSP_ADC_DMA_CHANNEL               DMA_Channel_0
#define BSP_ADC_DMA_IRQn                  DMA2_Stream4_IRQn
#define BSP_ADC_DMA_IRQ_HANDLER           DMA2_Stream4_IRQHandler
#define BSP_ADC_RESOLUTION                ADC_Resolution_12b
#define BSP_ADC_SAMPLE_TIME_DEFAULT       ADC_SampleTime_84Cycles

/*
 * 默认只打开 CH1。
 *
 * 对于 74HC4051 多路复用灰度模块：
 *   - 只需要 1 路 ADC 读取 OUT/SIG/AO；
 *   - 8 路灰度通道由 3 路 GPIO 地址线 S0/S1/S2 选择；
 *   - 不要把 CH2~CH8 全部打开。
 *
 * CH2~CH8 只作为普通多路 ADC 模板备用，例如电池电压、独立模拟传感器等。
 */
#define BSP_ADC_CH1_ENABLE                1
#define BSP_ADC_CH1_GPIO_PORT             GPIOC
#define BSP_ADC_CH1_GPIO_PIN              GPIO_Pin_0
#define BSP_ADC_CH1_ADC_CHANNEL           ADC_Channel_10
#define BSP_ADC_CH1_SAMPLE_TIME           BSP_ADC_SAMPLE_TIME_DEFAULT

#define BSP_ADC_CH2_ENABLE                0
#define BSP_ADC_CH2_GPIO_PORT             GPIOC
#define BSP_ADC_CH2_GPIO_PIN              GPIO_Pin_1
#define BSP_ADC_CH2_ADC_CHANNEL           ADC_Channel_11
#define BSP_ADC_CH2_SAMPLE_TIME           BSP_ADC_SAMPLE_TIME_DEFAULT

#define BSP_ADC_CH3_ENABLE                0
#define BSP_ADC_CH3_GPIO_PORT             GPIOC
#define BSP_ADC_CH3_GPIO_PIN              GPIO_Pin_2
#define BSP_ADC_CH3_ADC_CHANNEL           ADC_Channel_12
#define BSP_ADC_CH3_SAMPLE_TIME           BSP_ADC_SAMPLE_TIME_DEFAULT

#define BSP_ADC_CH4_ENABLE                0
#define BSP_ADC_CH4_GPIO_PORT             GPIOC
#define BSP_ADC_CH4_GPIO_PIN              GPIO_Pin_3
#define BSP_ADC_CH4_ADC_CHANNEL           ADC_Channel_13
#define BSP_ADC_CH4_SAMPLE_TIME           BSP_ADC_SAMPLE_TIME_DEFAULT

#define BSP_ADC_CH5_ENABLE                0
#define BSP_ADC_CH5_GPIO_PORT             GPIOC
#define BSP_ADC_CH5_GPIO_PIN              GPIO_Pin_4
#define BSP_ADC_CH5_ADC_CHANNEL           ADC_Channel_14
#define BSP_ADC_CH5_SAMPLE_TIME           BSP_ADC_SAMPLE_TIME_DEFAULT

#define BSP_ADC_CH6_ENABLE                0
#define BSP_ADC_CH6_GPIO_PORT             GPIOC
#define BSP_ADC_CH6_GPIO_PIN              GPIO_Pin_5
#define BSP_ADC_CH6_ADC_CHANNEL           ADC_Channel_15
#define BSP_ADC_CH6_SAMPLE_TIME           BSP_ADC_SAMPLE_TIME_DEFAULT

#define BSP_ADC_CH7_ENABLE                0
#define BSP_ADC_CH7_GPIO_PORT             GPIOA
#define BSP_ADC_CH7_GPIO_PIN              GPIO_Pin_6
#define BSP_ADC_CH7_ADC_CHANNEL           ADC_Channel_6
#define BSP_ADC_CH7_SAMPLE_TIME           BSP_ADC_SAMPLE_TIME_DEFAULT

#define BSP_ADC_CH8_ENABLE                0
#define BSP_ADC_CH8_GPIO_PORT             GPIOA
#define BSP_ADC_CH8_GPIO_PIN              GPIO_Pin_7
#define BSP_ADC_CH8_ADC_CHANNEL           ADC_Channel_7
#define BSP_ADC_CH8_SAMPLE_TIME           BSP_ADC_SAMPLE_TIME_DEFAULT

typedef enum {
#if BSP_ADC_CH1_ENABLE
    BSP_ADC_CH1,
#endif
#if BSP_ADC_CH2_ENABLE
    BSP_ADC_CH2,
#endif
#if BSP_ADC_CH3_ENABLE
    BSP_ADC_CH3,
#endif
#if BSP_ADC_CH4_ENABLE
    BSP_ADC_CH4,
#endif
#if BSP_ADC_CH5_ENABLE
    BSP_ADC_CH5,
#endif
#if BSP_ADC_CH6_ENABLE
    BSP_ADC_CH6,
#endif
#if BSP_ADC_CH7_ENABLE
    BSP_ADC_CH7,
#endif
#if BSP_ADC_CH8_ENABLE
    BSP_ADC_CH8,
#endif
    BSP_ADC_CH_COUNT
} BSP_ADC_Ch_t;

void       BSP_ADC_Init(void);
uint16_t   BSP_ADC_GetRaw(BSP_ADC_Ch_t ch);
BSP_Status_t BSP_ADC_GetRawArray(uint16_t *out_buf, uint8_t max_count, uint8_t *out_count);
void       BSP_ADC_Start(void);
void       BSP_ADC_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ADC_H */
