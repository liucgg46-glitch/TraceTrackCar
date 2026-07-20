#ifndef __DRV_LASER_H
#define __DRV_LASER_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t is_on;
    uint8_t timeout_tripped;
} Drv_Laser_Info_t;

void Drv_Laser_Init(void);
void Drv_Laser_Task(void);
void Drv_Laser_On(void);
void Drv_Laser_Off(void);
uint8_t Drv_Laser_IsOn(void);
BSP_Status_t Drv_Laser_GetInfo(Drv_Laser_Info_t *info);
void Drv_Laser_ClearTimeoutFlag(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_LASER_H */
