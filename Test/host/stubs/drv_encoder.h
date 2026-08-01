#ifndef __DRV_ENCODER_H
#define __DRV_ENCODER_H

#include <stdint.h>

int32_t Drv_Encoder_GetLeftSpeedCps(void);
int32_t Drv_Encoder_GetRightSpeedCps(void);
int32_t Drv_Encoder_GetLeftTotalMm(void);
int32_t Drv_Encoder_GetRightTotalMm(void);

#endif /* __DRV_ENCODER_H */
