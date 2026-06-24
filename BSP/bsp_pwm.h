#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include "bsp_common.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 通用 PWM BSP
 * ============================================================================
 * 定位：只负责 TIM PWM 输出，不负责“哪个电机该转多快”。
 *
 * 移植方法：
 *   只改本文件配置区：TIM、通道、GPIO、AF、预分频、周期。
 *   bsp_pwm.c 不需要改。
 *
 * 建议：
 *   - 直流电机 PWM、舵机 PWM、蜂鸣器 PWM 都可以用这个模块；
 *   - 方向脚不要写在 PWM 里，方向脚用 bsp_gpio，电机逻辑放 drv_motor。
 */

#define BSP_PWM_DEFAULT_ACTIVE_HIGH       1U

/*
 * 当前电赛小车推荐模板：TIM8_CH1~CH4，PC6~PC9，1kHz，ARR=999，占空比 0~1000‰。
 * TIM8 整个留给四路电机 PWM，四个通道共享同一频率和周期。
 */
#define BSP_PWM_CH1_ENABLE                1
#define BSP_PWM_CH1_TIM                   TIM8
#define BSP_PWM_CH1_TIM_CLOCK_FN          RCC_APB2PeriphClockCmd
#define BSP_PWM_CH1_TIM_CLOCK_MASK        RCC_APB2Periph_TIM8
#define BSP_PWM_CH1_GPIO_PORT             GPIOC
#define BSP_PWM_CH1_GPIO_PIN              GPIO_Pin_6
#define BSP_PWM_CH1_GPIO_PINSRC           GPIO_PinSource6
#define BSP_PWM_CH1_GPIO_AF               GPIO_AF_TIM8
#define BSP_PWM_CH1_CHANNEL               1U
#define BSP_PWM_CH1_PRESCALER             (84U - 1U)
#define BSP_PWM_CH1_PERIOD                (1000U - 1U)
#define BSP_PWM_CH1_INIT_COMPARE          0U
#define BSP_PWM_CH1_ACTIVE_HIGH           BSP_PWM_DEFAULT_ACTIVE_HIGH

#define BSP_PWM_CH2_ENABLE                1
#define BSP_PWM_CH2_TIM                   TIM8
#define BSP_PWM_CH2_TIM_CLOCK_FN          RCC_APB2PeriphClockCmd
#define BSP_PWM_CH2_TIM_CLOCK_MASK        RCC_APB2Periph_TIM8
#define BSP_PWM_CH2_GPIO_PORT             GPIOC
#define BSP_PWM_CH2_GPIO_PIN              GPIO_Pin_7
#define BSP_PWM_CH2_GPIO_PINSRC           GPIO_PinSource7
#define BSP_PWM_CH2_GPIO_AF               GPIO_AF_TIM8
#define BSP_PWM_CH2_CHANNEL               2U
#define BSP_PWM_CH2_PRESCALER             (84U - 1U)
#define BSP_PWM_CH2_PERIOD                (1000U - 1U)
#define BSP_PWM_CH2_INIT_COMPARE          0U
#define BSP_PWM_CH2_ACTIVE_HIGH           BSP_PWM_DEFAULT_ACTIVE_HIGH

#define BSP_PWM_CH3_ENABLE                1
#define BSP_PWM_CH3_TIM                   TIM8
#define BSP_PWM_CH3_TIM_CLOCK_FN          RCC_APB2PeriphClockCmd
#define BSP_PWM_CH3_TIM_CLOCK_MASK        RCC_APB2Periph_TIM8
#define BSP_PWM_CH3_GPIO_PORT             GPIOC
#define BSP_PWM_CH3_GPIO_PIN              GPIO_Pin_8
#define BSP_PWM_CH3_GPIO_PINSRC           GPIO_PinSource8
#define BSP_PWM_CH3_GPIO_AF               GPIO_AF_TIM8
#define BSP_PWM_CH3_CHANNEL               3U
#define BSP_PWM_CH3_PRESCALER             (84U - 1U)
#define BSP_PWM_CH3_PERIOD                (1000U - 1U)
#define BSP_PWM_CH3_INIT_COMPARE          0U
#define BSP_PWM_CH3_ACTIVE_HIGH           BSP_PWM_DEFAULT_ACTIVE_HIGH

#define BSP_PWM_CH4_ENABLE                1
#define BSP_PWM_CH4_TIM                   TIM8
#define BSP_PWM_CH4_TIM_CLOCK_FN          RCC_APB2PeriphClockCmd
#define BSP_PWM_CH4_TIM_CLOCK_MASK        RCC_APB2Periph_TIM8
#define BSP_PWM_CH4_GPIO_PORT             GPIOC
#define BSP_PWM_CH4_GPIO_PIN              GPIO_Pin_9
#define BSP_PWM_CH4_GPIO_PINSRC           GPIO_PinSource9
#define BSP_PWM_CH4_GPIO_AF               GPIO_AF_TIM8
#define BSP_PWM_CH4_CHANNEL               4U
#define BSP_PWM_CH4_PRESCALER             (84U - 1U)
#define BSP_PWM_CH4_PERIOD                (1000U - 1U)
#define BSP_PWM_CH4_INIT_COMPARE          0U
#define BSP_PWM_CH4_ACTIVE_HIGH           BSP_PWM_DEFAULT_ACTIVE_HIGH

typedef enum {
#if BSP_PWM_CH1_ENABLE
    BSP_PWM_CH1,
#endif
#if BSP_PWM_CH2_ENABLE
    BSP_PWM_CH2,
#endif
#if BSP_PWM_CH3_ENABLE
    BSP_PWM_CH3,
#endif
#if BSP_PWM_CH4_ENABLE
    BSP_PWM_CH4,
#endif
    BSP_PWM_COUNT
} BSP_PWM_Id_t;

void       BSP_PWM_Init(BSP_PWM_Id_t id);
void       BSP_PWM_InitAll(void);
BSP_Status_t BSP_PWM_SetCompare(BSP_PWM_Id_t id, uint16_t compare);
BSP_Status_t BSP_PWM_SetDutyPermille(BSP_PWM_Id_t id, uint16_t permille);
uint16_t   BSP_PWM_GetPeriod(BSP_PWM_Id_t id);
void       BSP_PWM_Start(BSP_PWM_Id_t id);
void       BSP_PWM_Stop(BSP_PWM_Id_t id);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PWM_H */
