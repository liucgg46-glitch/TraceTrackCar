#ifndef __ANGLE_CONTROL_H
#define __ANGLE_CONTROL_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 角度估算/角度控制辅助：angle_control
 * ============================================================================
 * 当前版本没有 IMU，因此默认使用编码器差速估算 yaw。
 *
 * yaw_deg = (right_distance - left_distance) / wheel_base * 180/pi
 * 约定：yaw_deg > 0 表示左转；yaw_deg < 0 表示右转。
 * 若实车方向相反，改 ANGLE_CONTROL_YAW_REVERSE，不要到处取反。
 */

#define ANGLE_CONTROL_WHEEL_BASE_MM       165.0f  /* 左右轮中心距，先按实车测量修改 */
#define ANGLE_CONTROL_YAW_REVERSE         0       /* yaw 符号反了改 1 */

typedef struct {
    float yaw_deg;
    int32_t left_mm;
    int32_t right_mm;
} AngleControl_Info_t;

void AngleControl_Init(void);
void AngleControl_Update(void);
void AngleControl_Clear(void);
float AngleControl_GetYawDeg(void);
float AngleControl_NormalizeDeg(float angle_deg);
float AngleControl_GetErrorDeg(float target_deg);
BSP_Status_t AngleControl_GetInfo(AngleControl_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __ANGLE_CONTROL_H */
