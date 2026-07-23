#ifndef __DRV_STATUS_LIGHT_H
#define __DRV_STATUS_LIGHT_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 红绿状态灯仅允许灭、红、绿三种状态，避免两个指示灯同时点亮。 */
typedef enum {
    DRV_STATUS_LIGHT_OFF = 0,
    DRV_STATUS_LIGHT_RED,
    DRV_STATUS_LIGHT_GREEN
} Drv_StatusLight_Mode_t;

void Drv_StatusLight_Init(void);
BSP_Status_t Drv_StatusLight_SetMode(Drv_StatusLight_Mode_t mode);
void Drv_StatusLight_Off(void);
void Drv_StatusLight_SetRed(void);
void Drv_StatusLight_SetGreen(void);
Drv_StatusLight_Mode_t Drv_StatusLight_GetMode(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_STATUS_LIGHT_H */