#ifndef __BSP_SYSTICK_H
#define __BSP_SYSTICK_H

#include "bsp_common.h"
#include <stdint.h>

uint32_t BSP_GetTickMs(void);
uint32_t GetTick(void);
uint8_t BSP_TimeElapsed(uint32_t *last_time_ms, uint32_t period_ms);
uint8_t BSP_IsTimeout(uint32_t start_time_ms, uint32_t timeout_ms);

#endif /* __BSP_SYSTICK_H */
