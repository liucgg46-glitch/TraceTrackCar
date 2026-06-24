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
 *   2. 每 5~10ms 调一次 BSP_Key_UpdateAll();
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
