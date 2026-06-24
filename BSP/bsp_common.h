#ifndef __BSP_COMMON_H
#define __BSP_COMMON_H

#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 通用 BSP 基础定义。
 *
 * 约定：
 *   1. 各外设驱动的硬件资源只在对应 bsp_xxx.h 里配置；
 *   2. bsp_xxx.c 只读取配置表，不直接写死外设号、GPIO、DMA Stream；
 *   3. 非阻塞接口的回调可能运行在中断上下文，只做置标志/拷贝少量数据；
 *   4. BSP_GET_TICK() 必须返回 ms 级系统节拍。默认调用 GetTick()；若工程使用
 *      HAL 库，可把下面的 BSP_GET_TICK() 改成 HAL_GetTick()。
 */

typedef enum {
    BSP_OK = 0,
    BSP_ERROR,
    BSP_BUSY,
    BSP_TIMEOUT,
    BSP_PARAM
} BSP_Status_t;

typedef void (*BSP_ClockCmdFn_t)(uint32_t periph, FunctionalState state);

/*
 * 弱函数标记：用于 App 层默认空实现。
 * 后续 A/B/C 在自己的 .c 文件中实现同名函数时，会自动覆盖这里的弱实现。
 * 这样 Part1 代码可以先独立编译，后续模块也能按固定接口对接。
 */
#ifndef BSP_WEAK
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#define BSP_WEAK __weak
#elif defined(__GNUC__)
#define BSP_WEAK __attribute__((weak))
#else
#define BSP_WEAK
#endif
#endif

#ifndef BSP_GET_TICK
uint32_t GetTick(void);
#define BSP_GET_TICK() GetTick()
#endif

static inline uint32_t BSP_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void BSP_ExitCritical(uint32_t primask)
{
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }
}

static inline uint8_t BSP_DMA_WaitDisable(DMA_Stream_TypeDef *stream, uint32_t timeout)
{
    while (DMA_GetCmdStatus(stream) != DISABLE) {
        if (timeout-- == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static inline void BSP_GPIO_ClockEnable(GPIO_TypeDef *port)
{
    if (port == GPIOA) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    } else if (port == GPIOB) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    } else if (port == GPIOC) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    } else if (port == GPIOD) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    } else if (port == GPIOE) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    } else if (port == GPIOF) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    } else if (port == GPIOG) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    } else if (port == GPIOH) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
    } else if (port == GPIOI) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
#ifdef GPIOJ
    } else if (port == GPIOJ) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOJ, ENABLE);
#endif
#ifdef GPIOK
    } else if (port == GPIOK) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOK, ENABLE);
#endif
    }
}

#define BSP_DMA_FLAG_TC(stream_num)  ((stream_num) == 0 ? DMA_FLAG_TCIF0  : \
                                      (stream_num) == 1 ? DMA_FLAG_TCIF1  : \
                                      (stream_num) == 2 ? DMA_FLAG_TCIF2  : \
                                      (stream_num) == 3 ? DMA_FLAG_TCIF3  : \
                                      (stream_num) == 4 ? DMA_FLAG_TCIF4  : \
                                      (stream_num) == 5 ? DMA_FLAG_TCIF5  : \
                                      (stream_num) == 6 ? DMA_FLAG_TCIF6  : DMA_FLAG_TCIF7)

#define BSP_DMA_FLAG_HT(stream_num)  ((stream_num) == 0 ? DMA_FLAG_HTIF0  : \
                                      (stream_num) == 1 ? DMA_FLAG_HTIF1  : \
                                      (stream_num) == 2 ? DMA_FLAG_HTIF2  : \
                                      (stream_num) == 3 ? DMA_FLAG_HTIF3  : \
                                      (stream_num) == 4 ? DMA_FLAG_HTIF4  : \
                                      (stream_num) == 5 ? DMA_FLAG_HTIF5  : \
                                      (stream_num) == 6 ? DMA_FLAG_HTIF6  : DMA_FLAG_HTIF7)

#define BSP_DMA_FLAG_TE(stream_num)  ((stream_num) == 0 ? DMA_FLAG_TEIF0  : \
                                      (stream_num) == 1 ? DMA_FLAG_TEIF1  : \
                                      (stream_num) == 2 ? DMA_FLAG_TEIF2  : \
                                      (stream_num) == 3 ? DMA_FLAG_TEIF3  : \
                                      (stream_num) == 4 ? DMA_FLAG_TEIF4  : \
                                      (stream_num) == 5 ? DMA_FLAG_TEIF5  : \
                                      (stream_num) == 6 ? DMA_FLAG_TEIF6  : DMA_FLAG_TEIF7)

#define BSP_DMA_FLAG_DME(stream_num) ((stream_num) == 0 ? DMA_FLAG_DMEIF0 : \
                                      (stream_num) == 1 ? DMA_FLAG_DMEIF1 : \
                                      (stream_num) == 2 ? DMA_FLAG_DMEIF2 : \
                                      (stream_num) == 3 ? DMA_FLAG_DMEIF3 : \
                                      (stream_num) == 4 ? DMA_FLAG_DMEIF4 : \
                                      (stream_num) == 5 ? DMA_FLAG_DMEIF5 : \
                                      (stream_num) == 6 ? DMA_FLAG_DMEIF6 : DMA_FLAG_DMEIF7)

#define BSP_DMA_FLAG_FE(stream_num)  ((stream_num) == 0 ? DMA_FLAG_FEIF0  : \
                                      (stream_num) == 1 ? DMA_FLAG_FEIF1  : \
                                      (stream_num) == 2 ? DMA_FLAG_FEIF2  : \
                                      (stream_num) == 3 ? DMA_FLAG_FEIF3  : \
                                      (stream_num) == 4 ? DMA_FLAG_FEIF4  : \
                                      (stream_num) == 5 ? DMA_FLAG_FEIF5  : \
                                      (stream_num) == 6 ? DMA_FLAG_FEIF6  : DMA_FLAG_FEIF7)

#define BSP_DMA_FLAGS_ALL(stream_num) (BSP_DMA_FLAG_TC(stream_num)  | \
                                       BSP_DMA_FLAG_HT(stream_num)  | \
                                       BSP_DMA_FLAG_TE(stream_num)  | \
                                       BSP_DMA_FLAG_DME(stream_num) | \
                                       BSP_DMA_FLAG_FE(stream_num))

#define BSP_DMA_IT_TC(stream_num)  ((stream_num) == 0 ? DMA_IT_TCIF0  : \
                                    (stream_num) == 1 ? DMA_IT_TCIF1  : \
                                    (stream_num) == 2 ? DMA_IT_TCIF2  : \
                                    (stream_num) == 3 ? DMA_IT_TCIF3  : \
                                    (stream_num) == 4 ? DMA_IT_TCIF4  : \
                                    (stream_num) == 5 ? DMA_IT_TCIF5  : \
                                    (stream_num) == 6 ? DMA_IT_TCIF6  : DMA_IT_TCIF7)

#define BSP_DMA_IT_TE(stream_num)  ((stream_num) == 0 ? DMA_IT_TEIF0  : \
                                    (stream_num) == 1 ? DMA_IT_TEIF1  : \
                                    (stream_num) == 2 ? DMA_IT_TEIF2  : \
                                    (stream_num) == 3 ? DMA_IT_TEIF3  : \
                                    (stream_num) == 4 ? DMA_IT_TEIF4  : \
                                    (stream_num) == 5 ? DMA_IT_TEIF5  : \
                                    (stream_num) == 6 ? DMA_IT_TEIF6  : DMA_IT_TEIF7)

#define BSP_DMA_IT_DME(stream_num) ((stream_num) == 0 ? DMA_IT_DMEIF0 : \
                                    (stream_num) == 1 ? DMA_IT_DMEIF1 : \
                                    (stream_num) == 2 ? DMA_IT_DMEIF2 : \
                                    (stream_num) == 3 ? DMA_IT_DMEIF3 : \
                                    (stream_num) == 4 ? DMA_IT_DMEIF4 : \
                                    (stream_num) == 5 ? DMA_IT_DMEIF5 : \
                                    (stream_num) == 6 ? DMA_IT_DMEIF6 : DMA_IT_DMEIF7)

#define BSP_DMA_IT_FE(stream_num)  ((stream_num) == 0 ? DMA_IT_FEIF0  : \
                                    (stream_num) == 1 ? DMA_IT_FEIF1  : \
                                    (stream_num) == 2 ? DMA_IT_FEIF2  : \
                                    (stream_num) == 3 ? DMA_IT_FEIF3  : \
                                    (stream_num) == 4 ? DMA_IT_FEIF4  : \
                                    (stream_num) == 5 ? DMA_IT_FEIF5  : \
                                    (stream_num) == 6 ? DMA_IT_FEIF6  : DMA_IT_FEIF7)

#ifdef __cplusplus
}
#endif

#endif /* __BSP_COMMON_H */
