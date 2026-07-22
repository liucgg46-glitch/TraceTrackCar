#include "attitude_estimator.h"

#include <math.h>
#include <string.h>

#define ATTITUDE_PI             3.14159265358979323846f
#define ATTITUDE_DEG_TO_RAD     (ATTITUDE_PI / 180.0f)
#define ATTITUDE_RAD_TO_DEG     (180.0f / ATTITUDE_PI)
#define ATTITUDE_EPSILON        1.0e-9f

static Attitude_Info_t s_info;
static Attitude_MagCalibration_t s_mag_cal;

static float s_gyro_yaw_rad;
static float s_yaw_output_offset_deg;
static float s_mag_reference_earth[3];
static float s_mag_cal_min[3];
static float s_mag_cal_max[3];

static uint32_t s_last_timestamp_ms;
static uint32_t s_last_healthy_mag_ms;
static uint16_t s_stationary_samples;
static uint16_t s_mag_good_samples;
static uint8_t s_have_timestamp;
static uint8_t s_mag_reference_valid;

static float Attitude_AbsF(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Attitude_ClampF(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float Attitude_WrapPi(float angle_rad)
{
    while (angle_rad > ATTITUDE_PI) {
        angle_rad -= 2.0f * ATTITUDE_PI;
    }
    while (angle_rad < -ATTITUDE_PI) {
        angle_rad += 2.0f * ATTITUDE_PI;
    }
    return angle_rad;
}

static float Attitude_Wrap180Deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float Attitude_VectorNorm(const float vector[3])
{
    return sqrtf(vector[0] * vector[0] +
                 vector[1] * vector[1] +
                 vector[2] * vector[2]);
}

static uint8_t Attitude_NormalizeVector(float vector[3])
{
    float norm = Attitude_VectorNorm(vector);

    if (norm <= ATTITUDE_EPSILON) {
        return 0U;
    }
    vector[0] /= norm;
    vector[1] /= norm;
    vector[2] /= norm;
    return 1U;
}

static void Attitude_NormalizeQuaternion(void)
{
    float norm = sqrtf(s_info.q[0] * s_info.q[0] +
                       s_info.q[1] * s_info.q[1] +
                       s_info.q[2] * s_info.q[2] +
                       s_info.q[3] * s_info.q[3]);

    if (norm <= ATTITUDE_EPSILON) {
        s_info.q[0] = 1.0f;
        s_info.q[1] = 0.0f;
        s_info.q[2] = 0.0f;
        s_info.q[3] = 0.0f;
        return;
    }

    s_info.q[0] /= norm;
    s_info.q[1] /= norm;
    s_info.q[2] /= norm;
    s_info.q[3] /= norm;
}

static void Attitude_EulerToQuaternion(float roll, float pitch, float yaw)
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    s_info.q[0] = cr * cp * cy + sr * sp * sy;
    s_info.q[1] = sr * cp * cy - cr * sp * sy;
    s_info.q[2] = cr * sp * cy + sr * cp * sy;
    s_info.q[3] = cr * cp * sy - sr * sp * cy;
    Attitude_NormalizeQuaternion();
}

static void Attitude_UpdateEuler(void)
{
    float q0 = s_info.q[0];
    float q1 = s_info.q[1];
    float q2 = s_info.q[2];
    float q3 = s_info.q[3];
    float sin_pitch;

    s_info.roll_deg = atan2f(2.0f * (q0 * q1 + q2 * q3),
                             1.0f - 2.0f * (q1 * q1 + q2 * q2)) * ATTITUDE_RAD_TO_DEG;

    sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    sin_pitch = Attitude_ClampF(sin_pitch, -1.0f, 1.0f);
    s_info.pitch_deg = asinf(sin_pitch) * ATTITUDE_RAD_TO_DEG;

#if (ATTITUDE_SEPARATE_YAW_ENABLE != 0U)
    s_info.yaw_deg = Attitude_Wrap180Deg(
        s_gyro_yaw_rad * ATTITUDE_RAD_TO_DEG - s_yaw_output_offset_deg);
#else
    s_info.yaw_deg = Attitude_Wrap180Deg(
        atan2f(2.0f * (q0 * q3 + q1 * q2),
               1.0f - 2.0f * (q2 * q2 + q3 * q3)) * ATTITUDE_RAD_TO_DEG -
        s_yaw_output_offset_deg);
#endif
}

/* q 表示机体系到地理系的旋转。 */
static void Attitude_RotateBodyToEarth(const float body[3], float earth[3])
{
    float q0 = s_info.q[0];
    float q1 = s_info.q[1];
    float q2 = s_info.q[2];
    float q3 = s_info.q[3];

    earth[0] = (1.0f - 2.0f * (q2 * q2 + q3 * q3)) * body[0] +
               2.0f * (q1 * q2 - q0 * q3) * body[1] +
               2.0f * (q1 * q3 + q0 * q2) * body[2];
    earth[1] = 2.0f * (q1 * q2 + q0 * q3) * body[0] +
               (1.0f - 2.0f * (q1 * q1 + q3 * q3)) * body[1] +
               2.0f * (q2 * q3 - q0 * q1) * body[2];
    earth[2] = 2.0f * (q1 * q3 - q0 * q2) * body[0] +
               2.0f * (q2 * q3 + q0 * q1) * body[1] +
               (1.0f - 2.0f * (q1 * q1 + q2 * q2)) * body[2];
}

static void Attitude_RotateEarthToBody(const float earth[3], float body[3])
{
    float q0 = s_info.q[0];
    float q1 = s_info.q[1];
    float q2 = s_info.q[2];
    float q3 = s_info.q[3];

    body[0] = (1.0f - 2.0f * (q2 * q2 + q3 * q3)) * earth[0] +
              2.0f * (q1 * q2 + q0 * q3) * earth[1] +
              2.0f * (q1 * q3 - q0 * q2) * earth[2];
    body[1] = 2.0f * (q1 * q2 - q0 * q3) * earth[0] +
              (1.0f - 2.0f * (q1 * q1 + q3 * q3)) * earth[1] +
              2.0f * (q2 * q3 + q0 * q1) * earth[2];
    body[2] = 2.0f * (q1 * q3 + q0 * q2) * earth[0] +
              2.0f * (q2 * q3 - q0 * q1) * earth[1] +
              (1.0f - 2.0f * (q1 * q1 + q2 * q2)) * earth[2];
}

static void Attitude_GetEstimatedGravity(float gravity[3])
{
    float q0 = s_info.q[0];
    float q1 = s_info.q[1];
    float q2 = s_info.q[2];
    float q3 = s_info.q[3];

    gravity[0] = 2.0f * (q1 * q3 - q0 * q2);
    gravity[1] = 2.0f * (q0 * q1 + q2 * q3);
    gravity[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}

static void Attitude_Cross(const float first[3],
                           const float second[3],
                           float result[3])
{
    result[0] = first[1] * second[2] - first[2] * second[1];
    result[1] = first[2] * second[0] - first[0] * second[2];
    result[2] = first[0] * second[1] - first[1] * second[0];
}

static float Attitude_Dot(const float first[3], const float second[3])
{
    return first[0] * second[0] +
           first[1] * second[1] +
           first[2] * second[2];
}

#if (ATTITUDE_SEPARATE_YAW_ENABLE == 0U)
static float Attitude_GetQuaternionYawRad(void)
{
    float q0 = s_info.q[0];
    float q1 = s_info.q[1];
    float q2 = s_info.q[2];
    float q3 = s_info.q[3];

    return atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3));
}
#endif

static void Attitude_ResetMagReference(void)
{
    s_mag_reference_earth[0] = 0.0f;
    s_mag_reference_earth[1] = 0.0f;
    s_mag_reference_earth[2] = 0.0f;
    s_mag_reference_valid = 0U;
    s_mag_good_samples = 0U;
    s_last_healthy_mag_ms = 0U;
    s_info.mag_healthy = 0U;
    s_info.mag_used = 0U;
    s_info.mag_reference_uT = 0.0f;
}

static void Attitude_ResetFusionState(void)
{
    uint8_t mag_calibrated = s_mag_cal.valid;

    memset(&s_info, 0, sizeof(s_info));
    s_info.q[0] = 1.0f;
    s_info.mag_calibrated = mag_calibrated;
    s_gyro_yaw_rad = 0.0f;
    s_yaw_output_offset_deg = 0.0f;
    s_last_timestamp_ms = 0U;
    s_stationary_samples = 0U;
    s_have_timestamp = 0U;
    Attitude_ResetMagReference();
}

static uint8_t Attitude_IsCalibrationValid(const Attitude_MagCalibration_t *calibration)
{
    uint8_t axis;

    if ((calibration == 0) || (calibration->valid == 0U)) {
        return 0U;
    }
    for (axis = 0U; axis < 3U; axis++) {
        if ((calibration->scale[axis] < 0.10f) ||
            (calibration->scale[axis] > 10.0f)) {
            return 0U;
        }
    }
    return 1U;
}

static void Attitude_UpdateMagCalibrationSamples(const Attitude_Input_t *input)
{
    float value[3];
    uint8_t axis;

    if ((s_info.mag_calibrating == 0U) ||
        (input->mag_valid == 0U) ||
        (input->mag_updated == 0U)) {
        return;
    }

    value[0] = input->mag_uT.x;
    value[1] = input->mag_uT.y;
    value[2] = input->mag_uT.z;

    for (axis = 0U; axis < 3U; axis++) {
        if (value[axis] < s_mag_cal_min[axis]) s_mag_cal_min[axis] = value[axis];
        if (value[axis] > s_mag_cal_max[axis]) s_mag_cal_max[axis] = value[axis];
    }
    s_info.mag_calibration_samples++;
}

static uint8_t Attitude_PrepareMagnetometer(const Attitude_Input_t *input,
                                             uint8_t motor_active,
                                             float mag_normalized[3])
{
    float raw[3];
    float norm;
    float lower;
    float upper;
    uint8_t magnitude_ok;

    s_info.mag_available = input->mag_valid;

#if (ATTITUDE_MAG_DISABLE_WHEN_MOTOR_ACTIVE != 0U)
    if (motor_active != 0U) {
        s_info.mag_used = 0U;
        return 0U;
    }
#endif

    /* These states make the magnetometer definitively unusable. */
    if ((input->mag_valid == 0U) ||
        (s_mag_cal.valid == 0U) ||
        (s_info.mag_calibrating != 0U)) {
        s_info.mag_healthy = 0U;
        s_info.mag_used = 0U;
        s_mag_good_samples = 0U;
        return 0U;
    }

    /*
     * AK09916 and Accel/Gyro do not necessarily update at the same rate.
     * No fresh sample in this IMU cycle is not a fault: keep the previous
     * healthy/used state and simply skip magnetometer correction this time.
     */
    if (input->mag_updated == 0U) {
        return 0U;
    }

    raw[0] = input->mag_filtered_uT.x;
    raw[1] = input->mag_filtered_uT.y;
    raw[2] = input->mag_filtered_uT.z;

    if (s_mag_cal.valid != 0U) {
        mag_normalized[0] = (raw[0] - s_mag_cal.offset_uT[0]) * s_mag_cal.scale[0];
        mag_normalized[1] = (raw[1] - s_mag_cal.offset_uT[1]) * s_mag_cal.scale[1];
        mag_normalized[2] = (raw[2] - s_mag_cal.offset_uT[2]) * s_mag_cal.scale[2];
    } else {
        mag_normalized[0] = raw[0];
        mag_normalized[1] = raw[1];
        mag_normalized[2] = raw[2];
    }

    norm = Attitude_VectorNorm(mag_normalized);
    s_info.mag_norm_uT = norm;

    magnitude_ok = ((norm >= ATTITUDE_MAG_FIELD_MIN_UT) &&
                    (norm <= ATTITUDE_MAG_FIELD_MAX_UT)) ? 1U : 0U;

    if ((magnitude_ok != 0U) && (s_info.mag_reference_uT > ATTITUDE_EPSILON)) {
        lower = s_info.mag_reference_uT * (1.0f - ATTITUDE_MAG_FIELD_REL_TOLERANCE);
        upper = s_info.mag_reference_uT * (1.0f + ATTITUDE_MAG_FIELD_REL_TOLERANCE);
        magnitude_ok = ((norm >= lower) && (norm <= upper)) ? 1U : 0U;
    }

    if (magnitude_ok == 0U) {
        s_info.mag_healthy = 0U;
        s_info.mag_used = 0U;
        s_mag_good_samples = 0U;
        s_info.mag_reject_count++;
        return 0U;
    }

    if (s_info.mag_reference_uT <= ATTITUDE_EPSILON) {
        s_info.mag_reference_uT = norm;
    }

    if (s_mag_good_samples < ATTITUDE_MAG_ACQUIRE_SAMPLES) {
        s_mag_good_samples++;
    }
    if (s_mag_good_samples < ATTITUDE_MAG_ACQUIRE_SAMPLES) {
        return 0U;
    }

    s_info.mag_healthy = 1U;
    s_last_healthy_mag_ms = input->timestamp_ms;

    if (Attitude_NormalizeVector(mag_normalized) == 0U) {
        s_info.mag_healthy = 0U;
        s_info.mag_used = 0U;
        s_mag_good_samples = 0U;
        return 0U;
    }
    return 1U;
}

static void Attitude_AddMagCorrection(const float mag_body[3],
                                      const float gravity_body[3],
                                      float correction[3])
{
    float predicted_body[3];
    float error[3];
    float yaw_error;
    float direction_dot;

    if (s_mag_reference_valid == 0U) {
        Attitude_RotateBodyToEarth(mag_body, s_mag_reference_earth);
        (void)Attitude_NormalizeVector(s_mag_reference_earth);
        s_mag_reference_valid = 1U;
        s_info.mag_accept_count++;
        return;
    }

    Attitude_RotateEarthToBody(s_mag_reference_earth, predicted_body);
    if (Attitude_NormalizeVector(predicted_body) == 0U) {
        s_info.mag_healthy = 0U;
        s_info.mag_used = 0U;
        s_mag_good_samples = 0U;
        return;
    }

    direction_dot = Attitude_Dot(mag_body, predicted_body);
    if (direction_dot < ATTITUDE_MAG_DIRECTION_MIN_DOT) {
        s_info.mag_healthy = 0U;
        s_info.mag_used = 0U;
        s_mag_good_samples = 0U;
        s_info.mag_reject_count++;
        return;
    }

    Attitude_Cross(mag_body, predicted_body, error);

    /* 只保留绕重力方向的分量，磁力计不参与 Roll/Pitch。 */
    yaw_error = Attitude_Dot(error, gravity_body);
    correction[0] += ATTITUDE_MAHONY_MAG_KP * yaw_error * gravity_body[0];
    correction[1] += ATTITUDE_MAHONY_MAG_KP * yaw_error * gravity_body[1];
    correction[2] += ATTITUDE_MAHONY_MAG_KP * yaw_error * gravity_body[2];

    s_info.mag_reference_uT += ATTITUDE_MAG_REFERENCE_ALPHA *
                               (s_info.mag_norm_uT - s_info.mag_reference_uT);
    s_info.mag_used = 1U;
    s_info.mag_accept_count++;
}

static void Attitude_UpdateStationaryAndBias(const Attitude_Input_t *input,
                                              float accel_norm)
{
    uint8_t stationary_candidate;
    uint8_t axis;

    stationary_candidate =
        ((accel_norm >= ATTITUDE_STATIONARY_ACCEL_MIN_G) &&
         (accel_norm <= ATTITUDE_STATIONARY_ACCEL_MAX_G) &&
         (Attitude_AbsF(input->gyro_filtered_dps.x) <= ATTITUDE_STATIONARY_GYRO_MAX_DPS) &&
         (Attitude_AbsF(input->gyro_filtered_dps.y) <= ATTITUDE_STATIONARY_GYRO_MAX_DPS) &&
         (Attitude_AbsF(input->gyro_filtered_dps.z) <= ATTITUDE_STATIONARY_GYRO_MAX_DPS)) ? 1U : 0U;

    if (stationary_candidate != 0U) {
        if (s_stationary_samples < ATTITUDE_STATIONARY_SAMPLE_COUNT) {
            s_stationary_samples++;
        }
    } else {
        s_stationary_samples = 0U;
    }

    s_info.stationary = (s_stationary_samples >= ATTITUDE_STATIONARY_SAMPLE_COUNT) ? 1U : 0U;
    if (s_info.stationary == 0U) {
        return;
    }

    for (axis = 0U; axis < 3U; axis++) {
        float measurement = (axis == 0U) ? input->gyro_filtered_dps.x :
                            ((axis == 1U) ? input->gyro_filtered_dps.y :
                                           input->gyro_filtered_dps.z);
        s_info.online_gyro_bias_dps[axis] += ATTITUDE_ONLINE_BIAS_ALPHA *
                                             (measurement - s_info.online_gyro_bias_dps[axis]);
        s_info.online_gyro_bias_dps[axis] =
            Attitude_ClampF(s_info.online_gyro_bias_dps[axis],
                            -ATTITUDE_ONLINE_BIAS_MAX_DPS,
                            ATTITUDE_ONLINE_BIAS_MAX_DPS);
    }
}

static void Attitude_IntegrateQuaternion(const float angular_rate[3], float dt)
{
    float q0 = s_info.q[0];
    float q1 = s_info.q[1];
    float q2 = s_info.q[2];
    float q3 = s_info.q[3];
    float half_dt = 0.5f * dt;

    s_info.q[0] += (-q1 * angular_rate[0] - q2 * angular_rate[1] - q3 * angular_rate[2]) * half_dt;
    s_info.q[1] += ( q0 * angular_rate[0] + q2 * angular_rate[2] - q3 * angular_rate[1]) * half_dt;
    s_info.q[2] += ( q0 * angular_rate[1] - q1 * angular_rate[2] + q3 * angular_rate[0]) * half_dt;
    s_info.q[3] += ( q0 * angular_rate[2] + q1 * angular_rate[1] - q2 * angular_rate[0]) * half_dt;
    Attitude_NormalizeQuaternion();
}

void Attitude_Init(void)
{
    memset(&s_mag_cal, 0, sizeof(s_mag_cal));
    s_mag_cal.offset_uT[0] = ATTITUDE_MAG_CAL_OFFSET_X_UT;
    s_mag_cal.offset_uT[1] = ATTITUDE_MAG_CAL_OFFSET_Y_UT;
    s_mag_cal.offset_uT[2] = ATTITUDE_MAG_CAL_OFFSET_Z_UT;
    s_mag_cal.scale[0] = ATTITUDE_MAG_CAL_SCALE_X;
    s_mag_cal.scale[1] = ATTITUDE_MAG_CAL_SCALE_Y;
    s_mag_cal.scale[2] = ATTITUDE_MAG_CAL_SCALE_Z;
    s_mag_cal.valid = ATTITUDE_MAG_CAL_DEFAULT_VALID;
    if (Attitude_IsCalibrationValid(&s_mag_cal) == 0U) {
        s_mag_cal.valid = 0U;
    }
    Attitude_ResetFusionState();
}

void Attitude_Reset(void)
{
    Attitude_ResetFusionState();
}

BSP_Status_t Attitude_Update(const Attitude_Input_t *input,
                             uint8_t motor_active)
{
    float accel[3];
    float accel_norm;
    float gravity[3];
    float accel_error[3];
    float mag[3];
    float correction[3] = {0.0f, 0.0f, 0.0f};
    float angular_rate[3];
    float dt;
    float roll;
    float pitch;
    uint32_t elapsed_ms;
    uint8_t accel_usable;
    uint8_t mag_usable;

    if (input == 0) {
        s_info.valid = 0U;
        return BSP_PARAM;
    }
    if ((s_have_timestamp != 0U) &&
        (input->timestamp_ms == s_last_timestamp_ms)) {
        return BSP_BUSY;
    }

    accel[0] = input->accel_filtered_g.x;
    accel[1] = input->accel_filtered_g.y;
    accel[2] = input->accel_filtered_g.z;
    accel_norm = Attitude_VectorNorm(accel);

    Attitude_UpdateMagCalibrationSamples(input);
    Attitude_UpdateStationaryAndBias(input, accel_norm);

    if (s_have_timestamp == 0U) {
        roll = atan2f(accel[1], accel[2]);
        pitch = atan2f(-accel[0],
                       sqrtf(accel[1] * accel[1] + accel[2] * accel[2]));
        Attitude_EulerToQuaternion(roll, pitch, 0.0f);
        Attitude_UpdateEuler();
        s_gyro_yaw_rad = 0.0f;
        s_info.encoder_yaw_deg = 0.0f;
        s_info.encoder_yaw_rate_dps = 0.0f;
        s_info.encoder_used = 0U;
        s_info.encoder_heading_valid = 0U;
        s_last_timestamp_ms = input->timestamp_ms;
        s_have_timestamp = 1U;
        s_info.timestamp_ms = input->timestamp_ms;
        s_info.initialized = 1U;
        s_info.valid = 1U;
        s_info.update_count++;

        mag_usable = Attitude_PrepareMagnetometer(input,
                                                  motor_active,
                                                  mag);
        if ((ATTITUDE_MAG_YAW_CORRECTION_ENABLE != 0U) && (mag_usable != 0U)) {
            Attitude_GetEstimatedGravity(gravity);
            Attitude_AddMagCorrection(mag, gravity, correction);
        } else {
            s_info.mag_used = 0U;
        }
        return BSP_OK;
    }

    elapsed_ms = (uint32_t)(input->timestamp_ms - s_last_timestamp_ms);
    dt = (float)elapsed_ms * 0.001f;
    if ((dt < ATTITUDE_MIN_DT_S) || (dt > ATTITUDE_MAX_DT_S)) {
        dt = ATTITUDE_NOMINAL_DT_S;
    }
    s_last_timestamp_ms = input->timestamp_ms;

    angular_rate[0] = (input->gyro_filtered_dps.x - s_info.online_gyro_bias_dps[0]) * ATTITUDE_DEG_TO_RAD;
    angular_rate[1] = (input->gyro_filtered_dps.y - s_info.online_gyro_bias_dps[1]) * ATTITUDE_DEG_TO_RAD;
    angular_rate[2] = (input->gyro_filtered_dps.z - s_info.online_gyro_bias_dps[2]) * ATTITUDE_DEG_TO_RAD;

#if (ATTITUDE_SEPARATE_YAW_ENABLE != 0U)
    /*
     * Keep yaw completely independent from Mahony accel feedback, encoders
     * and the magnetometer. Axis mapping/sign is already applied by the IMU
     * driver, so this is the calibrated vehicle-frame gyro Z rate.
     */
    s_gyro_yaw_rad = Attitude_WrapPi(s_gyro_yaw_rad + angular_rate[2] * dt);
#endif

    Attitude_GetEstimatedGravity(gravity);

    accel_usable = ((accel_norm >= ATTITUDE_ACCEL_CORRECTION_MIN_G) &&
                    (accel_norm <= ATTITUDE_ACCEL_CORRECTION_MAX_G) &&
                    (Attitude_NormalizeVector(accel) != 0U)) ? 1U : 0U;
    if (accel_usable != 0U) {
        Attitude_Cross(accel, gravity, accel_error);
        correction[0] += ATTITUDE_MAHONY_ACCEL_KP * accel_error[0];
        correction[1] += ATTITUDE_MAHONY_ACCEL_KP * accel_error[1];
        correction[2] += ATTITUDE_MAHONY_ACCEL_KP * accel_error[2];
    }

    /* Encoder yaw is intentionally not read or integrated in this module. */
    s_info.encoder_yaw_deg = 0.0f;
    s_info.encoder_yaw_rate_dps = 0.0f;
    s_info.encoder_used = 0U;
    s_info.encoder_heading_valid = 0U;

    mag_usable = Attitude_PrepareMagnetometer(input,
                                              motor_active,
                                              mag);
    if ((ATTITUDE_MAG_YAW_CORRECTION_ENABLE != 0U) && (mag_usable != 0U)) {
        Attitude_AddMagCorrection(mag, gravity, correction);
    } else if ((s_info.mag_healthy != 0U) &&
               ((uint32_t)(input->timestamp_ms - s_last_healthy_mag_ms) >
                ATTITUDE_MAG_STALE_TIMEOUT_MS)) {
        s_info.mag_healthy = 0U;
        s_info.mag_used = 0U;
        s_mag_good_samples = 0U;
    }
    if (ATTITUDE_MAG_YAW_CORRECTION_ENABLE == 0U) {
        s_info.mag_used = 0U;
    }

#if (ATTITUDE_SEPARATE_YAW_ENABLE != 0U)
    if ((ATTITUDE_MAG_YAW_CORRECTION_ENABLE != 0U) &&
        (s_info.mag_used != 0U)) {
        /* Apply only the magnetometer's earth-vertical correction to output yaw. */
        s_gyro_yaw_rad = Attitude_WrapPi(
            s_gyro_yaw_rad + Attitude_Dot(correction, gravity) * dt);
    }
#endif

    angular_rate[0] += correction[0];
    angular_rate[1] += correction[1];
    angular_rate[2] += correction[2];
    Attitude_IntegrateQuaternion(angular_rate, dt);
    Attitude_UpdateEuler();

    s_info.timestamp_ms = input->timestamp_ms;
    s_info.initialized = 1U;
    s_info.valid = 1U;
    s_info.update_count++;
    return BSP_OK;
}

void Attitude_Invalidate(void)
{
    s_info.valid = 0U;
}

BSP_Status_t Attitude_GetInfo(Attitude_Info_t *info)
{
    uint32_t primask;

    if (info == 0) {
        return BSP_PARAM;
    }
    primask = BSP_EnterCritical();
    memcpy(info, &s_info, sizeof(*info));
    BSP_ExitCritical(primask);
    return (info->valid != 0U) ? BSP_OK : BSP_ERROR;
}

float Attitude_GetRollDeg(void)
{
    return s_info.roll_deg;
}

float Attitude_GetPitchDeg(void)
{
    return s_info.pitch_deg;
}

float Attitude_GetYawDeg(void)
{
    return s_info.yaw_deg;
}

uint8_t Attitude_IsValid(void)
{
    return s_info.valid;
}

void Attitude_ZeroYaw(void)
{
#if (ATTITUDE_SEPARATE_YAW_ENABLE != 0U)
    s_yaw_output_offset_deg = s_gyro_yaw_rad * ATTITUDE_RAD_TO_DEG;
#else
    s_yaw_output_offset_deg = Attitude_GetQuaternionYawRad() * ATTITUDE_RAD_TO_DEG;
#endif
    Attitude_UpdateEuler();
}

void Attitude_MagCalibrationStart(void)
{
    uint8_t axis;

    for (axis = 0U; axis < 3U; axis++) {
        s_mag_cal_min[axis] = 1000000.0f;
        s_mag_cal_max[axis] = -1000000.0f;
    }
    s_info.mag_calibration_samples = 0U;
    s_info.mag_calibrating = 1U;
    Attitude_ResetMagReference();
}

BSP_Status_t Attitude_MagCalibrationFinish(Attitude_MagCalibration_t *result)
{
    Attitude_MagCalibration_t candidate;
    float radius[3];
    float average_radius;
    uint8_t axis;

    if (s_info.mag_calibrating == 0U) {
        return BSP_ERROR;
    }
    s_info.mag_calibrating = 0U;

    if (s_info.mag_calibration_samples < ATTITUDE_MAG_CAL_MIN_SAMPLES) {
        return BSP_ERROR;
    }

    memset(&candidate, 0, sizeof(candidate));
    for (axis = 0U; axis < 3U; axis++) {
        float span = s_mag_cal_max[axis] - s_mag_cal_min[axis];
        if (span < ATTITUDE_MAG_CAL_MIN_SPAN_UT) {
            return BSP_ERROR;
        }
        candidate.offset_uT[axis] = 0.5f * (s_mag_cal_max[axis] + s_mag_cal_min[axis]);
        radius[axis] = 0.5f * span;
    }

    average_radius = (radius[0] + radius[1] + radius[2]) / 3.0f;
    for (axis = 0U; axis < 3U; axis++) {
        candidate.scale[axis] = average_radius / radius[axis];
    }
    candidate.valid = 1U;

    if (Attitude_SetMagCalibration(&candidate) != BSP_OK) {
        return BSP_ERROR;
    }
    if (result != 0) {
        *result = s_mag_cal;
    }
    return BSP_OK;
}

BSP_Status_t Attitude_SetMagCalibration(const Attitude_MagCalibration_t *calibration)
{
    if (calibration == 0) {
        return BSP_PARAM;
    }

    if (calibration->valid == 0U) {
        s_mag_cal = *calibration;
        s_mag_cal.valid = 0U;
        s_info.mag_calibrated = 0U;
        Attitude_ResetMagReference();
        return BSP_OK;
    }

    if (Attitude_IsCalibrationValid(calibration) == 0U) {
        return BSP_PARAM;
    }

    s_mag_cal = *calibration;
    s_mag_cal.valid = 1U;
    s_info.mag_calibrated = 1U;
    Attitude_ResetMagReference();
    return BSP_OK;
}

BSP_Status_t Attitude_GetMagCalibration(Attitude_MagCalibration_t *calibration)
{
    if (calibration == 0) {
        return BSP_PARAM;
    }
    *calibration = s_mag_cal;
    return BSP_OK;
}
