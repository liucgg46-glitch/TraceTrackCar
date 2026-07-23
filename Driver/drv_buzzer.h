#ifndef __DRV_BUZZER_H
#define __DRV_BUZZER_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DRV_BUZZER_OFF = 0,
    DRV_BUZZER_ON
} Drv_Buzzer_State_t;

void Drv_Buzzer_Init(void);
BSP_Status_t Drv_Buzzer_SetState(Drv_Buzzer_State_t state);
void Drv_Buzzer_On(void);
void Drv_Buzzer_Off(void);
void Drv_Buzzer_Toggle(void);
Drv_Buzzer_State_t Drv_Buzzer_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_BUZZER_H */
