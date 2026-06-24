#ifndef __ODOMETER_H
#define __ODOMETER_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * Encoder odometer: odometer
 * ============================================================================
 * Estimates left/right/average distance from encoder totals.
 */

typedef struct {
    int32_t left_mm;
    int32_t right_mm;
    int32_t distance_mm;
    int32_t delta_left_mm;
    int32_t delta_right_mm;
    int32_t delta_distance_mm;
} Odometer_Info_t;

void Odometer_Init(void);
void Odometer_Clear(void);
void Odometer_Update(void);

int32_t Odometer_GetLeftMm(void);
int32_t Odometer_GetRightMm(void);
int32_t Odometer_GetDistanceMm(void);
BSP_Status_t Odometer_GetInfo(Odometer_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __ODOMETER_H */
