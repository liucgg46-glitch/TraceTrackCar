#include "heading_estimator.h"
#include "attitude_estimator.h"
#include "odometer.h"

#define HEADING_PI 3.1415926f

static float s_yaw_deg = 0.0f;
static float s_imu_zero_deg = 0.0f;
static uint8_t s_imu_zero_valid = 0U;

static float Heading_Wrap180(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

void Heading_Init(void)
{
    s_yaw_deg = 0.0f;
    s_imu_zero_deg = 0.0f;
    s_imu_zero_valid = 0U;
}

void Heading_Reset(void)
{
    s_yaw_deg = 0.0f;

#if (HEADING_ESTIMATOR_SOURCE == HEADING_SOURCE_FUSED)
    if (Attitude_IsValid() != 0U) {
        s_imu_zero_deg = Attitude_GetYawDeg();
        s_imu_zero_valid = 1U;
    } else {
        /* IMU 尚在上电标定时，第一帧有效姿态自动成为零点。 */
        s_imu_zero_deg = 0.0f;
        s_imu_zero_valid = 0U;
    }
#endif
}

void Heading_Update(void)
{
#if (HEADING_ESTIMATOR_SOURCE == HEADING_SOURCE_ENCODER)
    float left_mm;
    float right_mm;
    float yaw;

    left_mm = (float)Odometer_GetLeftMm();
    right_mm = (float)Odometer_GetRightMm();

    yaw = ((right_mm - left_mm) * 180.0f) /
          (HEADING_PI * HEADING_ENCODER_WHEEL_BASE_MM);

#if HEADING_ENCODER_YAW_REVERSE
    yaw = -yaw;
#endif

    s_yaw_deg = Heading_Wrap180(yaw);

#elif (HEADING_ESTIMATOR_SOURCE == HEADING_SOURCE_FUSED)
    float yaw;

    /* Sensor_Update() 正常负责推进；这里再调用一次也会由 timestamp 自动去重。 */
    (void)Attitude_Update();
    if (Attitude_IsValid() == 0U) {
        return;
    }

    if (s_imu_zero_valid == 0U) {
        s_imu_zero_deg = Attitude_GetYawDeg();
        s_imu_zero_valid = 1U;
    }

    yaw = Attitude_GetYawDeg() - s_imu_zero_deg;

#if HEADING_IMU_YAW_REVERSE
    yaw = -yaw;
#endif

    s_yaw_deg = Heading_Wrap180(yaw);
#endif
}

float Heading_GetYawDeg(void)
{
    return s_yaw_deg;
}

float Heading_GetErrorDeg(float target_deg)
{
    return Heading_Wrap180(target_deg - s_yaw_deg);
}
