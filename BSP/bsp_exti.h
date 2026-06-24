#ifndef __BSP_EXTI_H
#define __BSP_EXTI_H

#include "bsp_common.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_syscfg.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 通用 EXTI BSP
 * ============================================================================
 * 定位：只负责外部中断入口和回调分发。具体业务逻辑不要写在 BSP 层。
 *
 * 使用建议：
 *   - 按键如果只是普通按下，优先用 bsp_key 轮询消抖；
 *   - 限位、测速脉冲、超声波 echo 等需要边沿捕获的信号可以用 EXTI；
 *   - 回调在中断上下文中执行，只能置标志/记录时间，不要 printf，不要阻塞。
 *
 * 移植方法：只改本文件配置区，bsp_exti.c 不需要改。
 */

typedef void (*BSP_EXTI_Callback_t)(void *ctx);

#define BSP_EXTI_CH1_ENABLE              1
#define BSP_EXTI_CH1_PORT                GPIOE
#define BSP_EXTI_CH1_PIN                 GPIO_Pin_0
#define BSP_EXTI_CH1_PORT_SOURCE         EXTI_PortSourceGPIOE
#define BSP_EXTI_CH1_PIN_SOURCE          EXTI_PinSource0
#define BSP_EXTI_CH1_LINE                EXTI_Line0
#define BSP_EXTI_CH1_IRQn                EXTI0_IRQn
#define BSP_EXTI_CH1_TRIGGER             EXTI_Trigger_Rising_Falling
#define BSP_EXTI_CH1_PUPD                GPIO_PuPd_UP

#define BSP_EXTI_CH2_ENABLE              0
#define BSP_EXTI_CH2_PORT                GPIOE
#define BSP_EXTI_CH2_PIN                 GPIO_Pin_1
#define BSP_EXTI_CH2_PORT_SOURCE         EXTI_PortSourceGPIOE
#define BSP_EXTI_CH2_PIN_SOURCE          EXTI_PinSource1
#define BSP_EXTI_CH2_LINE                EXTI_Line1
#define BSP_EXTI_CH2_IRQn                EXTI1_IRQn
#define BSP_EXTI_CH2_TRIGGER             EXTI_Trigger_Rising_Falling
#define BSP_EXTI_CH2_PUPD                GPIO_PuPd_UP

#define BSP_EXTI_CH3_ENABLE              0
#define BSP_EXTI_CH3_PORT                GPIOE
#define BSP_EXTI_CH3_PIN                 GPIO_Pin_2
#define BSP_EXTI_CH3_PORT_SOURCE         EXTI_PortSourceGPIOE
#define BSP_EXTI_CH3_PIN_SOURCE          EXTI_PinSource2
#define BSP_EXTI_CH3_LINE                EXTI_Line2
#define BSP_EXTI_CH3_IRQn                EXTI2_IRQn
#define BSP_EXTI_CH3_TRIGGER             EXTI_Trigger_Rising_Falling
#define BSP_EXTI_CH3_PUPD                GPIO_PuPd_UP

#define BSP_EXTI_IRQ_PREEMPT_PRIO        2U
#define BSP_EXTI_IRQ_SUB_PRIO            2U
#define BSP_EXTI_ANY_ENABLE              (BSP_EXTI_CH1_ENABLE || BSP_EXTI_CH2_ENABLE || BSP_EXTI_CH3_ENABLE)

typedef enum {
#if BSP_EXTI_CH1_ENABLE
    BSP_EXTI_CH1,
#endif
#if BSP_EXTI_CH2_ENABLE
    BSP_EXTI_CH2,
#endif
#if BSP_EXTI_CH3_ENABLE
    BSP_EXTI_CH3,
#endif
    BSP_EXTI_COUNT
} BSP_EXTI_Id_t;

void       BSP_EXTI_Init(BSP_EXTI_Id_t id);
void       BSP_EXTI_InitAll(void);
BSP_Status_t BSP_EXTI_AttachCallback(BSP_EXTI_Id_t id, BSP_EXTI_Callback_t cb, void *ctx);
void       BSP_EXTI_DispatchIRQ(uint32_t exti_line);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_EXTI_H */
