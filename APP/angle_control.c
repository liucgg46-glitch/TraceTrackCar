#include "angle_control.h"
#include "odometer.h"

#define ANGLE_CONTROL_PI 3.1415926f

static AngleControl_Info_t s_angle;

float AngleControl_NormalizeDeg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

void AngleControl_Init(void)
{
    AngleControl_Clear();
}

void AngleControl_Clear(void)
{
    s_angle.yaw_deg = 0.0f;
    s_angle.left_mm = 0;
    s_angle.right_mm = 0;
}

void AngleControl_Update(void)
{
    float yaw;

    s_angle.left_mm  = Odometer_GetLeftMm();
    s_angle.right_mm = Odometer_GetRightMm();

    yaw = ((float)(s_angle.right_mm - s_angle.left_mm) * 180.0f) /
          (ANGLE_CONTROL_PI * ANGLE_CONTROL_WHEEL_BASE_MM);

#if ANGLE_CONTROL_YAW_REVERSE
    yaw = -yaw;
#endif

    s_angle.yaw_deg = AngleControl_NormalizeDeg(yaw);
}

float AngleControl_GetYawDeg(void)
{
    return s_angle.yaw_deg;
}

float AngleControl_GetErrorDeg(float target_deg)
{
    return AngleControl_NormalizeDeg(target_deg - s_angle.yaw_deg);
}

BSP_Status_t AngleControl_GetInfo(AngleControl_Info_t *info)
{
    if (info == 0) return BSP_PARAM;
    *info = s_angle;
    return BSP_OK;
}
