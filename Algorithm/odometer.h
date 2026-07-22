#ifndef __ODOMETER_H
#define __ODOMETER_H

#include "project_status.h"
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

/* 调用方传入编码器累计里程，算法内部保存清零基线。 */
void Odometer_Init(int32_t left_total_mm, int32_t right_total_mm);
void Odometer_Clear(int32_t left_total_mm, int32_t right_total_mm);
void Odometer_Update(int32_t left_total_mm, int32_t right_total_mm);

int32_t Odometer_GetLeftMm(void);
int32_t Odometer_GetRightMm(void);
int32_t Odometer_GetDistanceMm(void);
Project_Status_t Odometer_GetInfo(Odometer_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __ODOMETER_H */
