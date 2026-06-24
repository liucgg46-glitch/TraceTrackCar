#ifndef __ODOMETER_H
#define __ODOMETER_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 编码器里程估算：odometer
 * ============================================================================
 * 定位：把 drv_encoder 输出的左右轮累计距离整理成底盘直线里程。
 *
 * 本模块只做里程估算，不直接控制电机，不写 PID。
 * - distance_mm：左右平均距离，用于直走定距；
 * - left/right_mm：左右侧累计距离，用于调试和角度估算；
 * - Odometer_Clear() 会清零 drv_encoder 的累计 count。
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
void Odometer_Update(void);
void Odometer_Clear(void);

int32_t Odometer_GetLeftMm(void);
int32_t Odometer_GetRightMm(void);
int32_t Odometer_GetDistanceMm(void);
BSP_Status_t Odometer_GetInfo(Odometer_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __ODOMETER_H */
