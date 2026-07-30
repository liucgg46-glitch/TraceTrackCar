#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#include "bsp_common.h"
#include "stm32f4xx_gpio.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 非阻塞按键 BSP
 * ============================================================================
 * 定位：只负责按键 GPIO 输入和消抖，不负责菜单、启停小车等业务逻辑。
 *
 * 使用方法：
 *   1. BSP_Key_InitAll();
 *   2. 应用层每 5~10ms 调一次 BSP_Key_UpdateAll()；当前工程由 Key_Update() 统一调用；
 *   3. 用 BSP_Key_WasPressed() 获取“按下沿事件”。
 *
 * 移植方法：只改本文件配置区，bsp_key.c 不需要改。
 */

#define BSP_KEY_DEBOUNCE_MS              20U

#define BSP_KEY1_ENABLE                  1
#define BSP_KEY1_PORT                    GPIOE
#define BSP_KEY1_PIN                     GPIO_Pin_4
#define BSP_KEY1_PUPD                    GPIO_PuPd_UP
#define BSP_KEY1_ACTIVE_LEVEL            0U

#define BSP_KEY2_ENABLE                  1
#define BSP_KEY2_PORT                    GPIOE
#define BSP_KEY2_PIN                     GPIO_Pin_3
#define BSP_KEY2_PUPD                    GPIO_PuPd_UP
#define BSP_KEY2_ACTIVE_LEVEL            0U

#define BSP_KEY3_ENABLE                  1
#define BSP_KEY3_PORT                    GPIOE
#define BSP_KEY3_PIN                     GPIO_Pin_2
#define BSP_KEY3_PUPD                    GPIO_PuPd_UP
#define BSP_KEY3_ACTIVE_LEVEL            0U

#define BSP_KEY4_ENABLE                  1
#define BSP_KEY4_PORT                    GPIOE
#define BSP_KEY4_PIN                     GPIO_Pin_1
#define BSP_KEY4_PUPD                    GPIO_PuPd_UP
#define BSP_KEY4_ACTIVE_LEVEL            0U

/*
 * KEY5：开发板引出的 PA15 按键。因仓库没有开发板原理图，暂按现有
 * KEY1~KEY4 的接法配置为内部上拉、低电平按下；实板电平仍需确认。
 * PA15 复位后兼作 JTDI；配置为按键输入后不能再使用完整 JTAG，
 * 但 PA13/PA14 上的两线 SWD 调试不受影响。
 */
#define BSP_KEY5_ENABLE                  1
#define BSP_KEY5_PORT                    GPIOA
#define BSP_KEY5_PIN                     GPIO_Pin_15
#define BSP_KEY5_PUPD                    GPIO_PuPd_UP
#define BSP_KEY5_ACTIVE_LEVEL            0U

#define BSP_KEY6_ENABLE                  1
#define BSP_KEY6_PORT                    GPIOF
#define BSP_KEY6_PIN                     GPIO_Pin_0
#define BSP_KEY6_PUPD                    GPIO_PuPd_UP
#define BSP_KEY6_ACTIVE_LEVEL            0U

#define BSP_KEY7_ENABLE                  1
#define BSP_KEY7_PORT                    GPIOF
#define BSP_KEY7_PIN                     GPIO_Pin_1
#define BSP_KEY7_PUPD                    GPIO_PuPd_UP
#define BSP_KEY7_ACTIVE_LEVEL            0U

#define BSP_KEY8_ENABLE                  1
#define BSP_KEY8_PORT                    GPIOF
#define BSP_KEY8_PIN                     GPIO_Pin_2
#define BSP_KEY8_PUPD                    GPIO_PuPd_UP
#define BSP_KEY8_ACTIVE_LEVEL            0U

#define BSP_KEY9_ENABLE                  1
#define BSP_KEY9_PORT                    GPIOF
#define BSP_KEY9_PIN                     GPIO_Pin_3
#define BSP_KEY9_PUPD                    GPIO_PuPd_UP
#define BSP_KEY9_ACTIVE_LEVEL            0U

typedef enum {
#if BSP_KEY1_ENABLE
    BSP_KEY1,
#endif
#if BSP_KEY2_ENABLE
    BSP_KEY2,
#endif
#if BSP_KEY3_ENABLE
    BSP_KEY3,
#endif
#if BSP_KEY4_ENABLE
    BSP_KEY4,
#endif
#if BSP_KEY5_ENABLE
    BSP_KEY5,
#endif
#if BSP_KEY6_ENABLE
    BSP_KEY6,
#endif
#if BSP_KEY7_ENABLE
    BSP_KEY7,
#endif
#if BSP_KEY8_ENABLE
    BSP_KEY8,
#endif
#if BSP_KEY9_ENABLE
    BSP_KEY9,
#endif
    BSP_KEY_COUNT
} BSP_Key_Id_t;

typedef enum {
    BSP_KEY_EVENT_NONE = 0,
    BSP_KEY_EVENT_PRESSED,
    BSP_KEY_EVENT_RELEASED
} BSP_KeyEvent_t;

void          BSP_Key_Init(BSP_Key_Id_t id);
void          BSP_Key_InitAll(void);
void          BSP_Key_Update(BSP_Key_Id_t id);
void          BSP_Key_UpdateAll(void);
uint8_t       BSP_Key_IsPressed(BSP_Key_Id_t id);
uint8_t       BSP_Key_WasPressed(BSP_Key_Id_t id);
uint8_t       BSP_Key_WasReleased(BSP_Key_Id_t id);
BSP_KeyEvent_t BSP_Key_GetEvent(BSP_Key_Id_t id);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_KEY_H */
