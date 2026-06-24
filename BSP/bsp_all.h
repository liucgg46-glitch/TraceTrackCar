#ifndef __BSP_ALL_H
#define __BSP_ALL_H

#include "bsp_common.h"
#include "bsp_systick.h"
#include "bsp_gpio.h"
#include "bsp_pwm.h"
#include "bsp_encoder.h"
#include "bsp_adc.h"
#include "bsp_key.h"
#include "bsp_exti.h"
#include "bsp_uart.h"
#include "bsp_i2c.h"
#include "bsp_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BSP 一键初始化入口。
 * 如果某些模块暂时不用，可以不调用 BSP_InitAll()，改为在 main.c 中单独 Init。
 */
BSP_Status_t BSP_InitAll(uint32_t system_core_clock_hz);
void         BSP_TaskAll(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ALL_H */
