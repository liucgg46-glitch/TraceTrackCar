#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#include "bsp_common.h"
#include <stdint.h>

#define BSP_KEY1_ENABLE 1
#define BSP_KEY2_ENABLE 1
#define BSP_KEY3_ENABLE 1
#define BSP_KEY4_ENABLE 1
#define BSP_KEY5_ENABLE 1

typedef enum {
    BSP_KEY1 = 0,
    BSP_KEY2,
    BSP_KEY3,
    BSP_KEY4,
    BSP_KEY5,
    BSP_KEY_COUNT
} BSP_Key_Id_t;

uint8_t BSP_Key_IsPressed(BSP_Key_Id_t id);
uint8_t BSP_Key_WasPressed(BSP_Key_Id_t id);

#endif /* __BSP_KEY_H */
