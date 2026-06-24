#ifndef __BSP_SYSTICK_H
#define __BSP_SYSTICK_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * SysTick 毫秒节拍 BSP
 * ============================================================================
 * 设计目标：
 *   1. 全工程统一毫秒时间基准；
 *   2. 所有模块用 BSP_GetTickMs()/BSP_TimeElapsed() 判断时间，不写 delay；
 *   3. 移植到其他 STM32F407 标准库工程时，通常不需要改 .c 文件。
 *
 * 注意：
 *   - 本文件默认提供 SysTick_Handler()。如果工程里的 stm32f4xx_it.c 已经有
 *     SysTick_Handler，请删除那边的空函数，避免重复定义。
 *   - 若你一定要自己实现 SysTick_Handler，可把下面宏改成 0，然后在自己的
 *     SysTick_Handler 中调用 BSP_SysTick_Inc()。
 */
#define BSP_SYSTICK_USE_DEFAULT_HANDLER   1

/* 默认 1ms tick。一般不要改。 */
#define BSP_SYSTICK_HZ                    1000UL

BSP_Status_t BSP_SysTick_Init(uint32_t system_core_clock_hz);
void         BSP_SysTick_Inc(void);
uint32_t     BSP_GetTickMs(void);
uint32_t     GetTick(void);                 /* 兼容 bsp_common.h 里的 BSP_GET_TICK() */
uint8_t      BSP_TimeElapsed(uint32_t *last_time_ms, uint32_t period_ms);
uint8_t      BSP_IsTimeout(uint32_t start_time_ms, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SYSTICK_H */
