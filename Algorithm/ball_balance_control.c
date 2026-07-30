#include "ball_balance_control.h"

#define BALL_BALANCE_POSITION_FILTER_ALPHA       0.35f
#define BALL_BALANCE_VELOCITY_FILTER_ALPHA       0.30f
#define BALL_BALANCE_INTEGRAL_LIMIT_MM_S         120.0f

static uint8_t s_enabled;
static uint8_t s_measurement_valid;
static uint8_t s_data_timeout;
static uint8_t s_position_filter_valid;
static uint8_t s_velocity_filter_valid;
static uint8_t s_has_sequence;
static uint8_t s_last_sequence;
static int16_t s_target_mm_x10;
static int16_t s_raw_position_mm_x10;
static float s_filtered_position_mm;
static float s_velocity_mm_s;
static float s_integral;
static float s_kp;
static float s_ki;
static float s_kd;
static float s_p_term;
static float s_i_term;
static float s_d_term;
static float s_pid_output_deg;
static uint16_t s_command_angle_x10;
static uint32_t s_last_sample_ms;
static uint32_t s_last_update_ms;
static uint32_t s_update_count;
static uint32_t s_valid_sample_count;
static uint32_t s_invalid_sample_count;
static uint32_t s_timeout_count;
static uint32_t s_output_limit_count;

static float BallBalance_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float BallBalance_LimitFloat(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static uint16_t BallBalance_LimitAngleX10(uint16_t angle_x10)
{
#if (BALL_BALANCE_ABS_SAFE_MIN_X10 > 0U)
    if (angle_x10 < BALL_BALANCE_ABS_SAFE_MIN_X10) {
        return BALL_BALANCE_ABS_SAFE_MIN_X10;
    }
#endif
    if (angle_x10 > BALL_BALANCE_ABS_SAFE_MAX_X10) {
        return BALL_BALANCE_ABS_SAFE_MAX_X10;
    }
    return angle_x10;
}

static int16_t BallBalance_FloatMmToX10(float value_mm)
{
    float scaled;
    int32_t value;

    scaled = value_mm * 10.0f;
    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }

    value = (int32_t)scaled;
    if (value > 32767) {
        return 32767;
    }
    if (value < -32768) {
        return (int16_t)-32768;
    }
    return (int16_t)value;
}

static uint16_t BallBalance_FloatDegToX10(float value_deg)
{
    float scaled;
    int32_t value;

    scaled = value_deg * 10.0f;
    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }

    value = (int32_t)scaled;
    if (value < 0) {
        return 0U;
    }
    if (value > 1800) {
        return 1800U;
    }
    return (uint16_t)value;
}

static void BallBalance_ClearRuntime(void)
{
    s_measurement_valid = 0U;
    s_data_timeout = 0U;
    s_position_filter_valid = 0U;
    s_velocity_filter_valid = 0U;
    s_has_sequence = 0U;
    s_last_sequence = 0U;
    s_raw_position_mm_x10 = 0;
    s_filtered_position_mm = 0.0f;
    s_velocity_mm_s = 0.0f;
    s_integral = 0.0f;
    s_p_term = 0.0f;
    s_i_term = 0.0f;
    s_d_term = 0.0f;
    s_pid_output_deg = 0.0f;
    s_command_angle_x10 = BALL_BALANCE_NEUTRAL_ANGLE_X10;
    s_last_sample_ms = 0U;
    s_last_update_ms = 0U;
}

static void BallBalance_ResetToNeutral(void)
{
    s_p_term = 0.0f;
    s_i_term = 0.0f;
    s_d_term = 0.0f;
    s_pid_output_deg = 0.0f;
    s_integral = 0.0f;
    s_velocity_mm_s = 0.0f;
    s_velocity_filter_valid = 0U;
    s_command_angle_x10 = BALL_BALANCE_NEUTRAL_ANGLE_X10;
}

static uint32_t BallBalance_IncrementU32(uint32_t value)
{
    return (value < 0xFFFFFFFFUL) ? (value + 1UL) : value;
}

static uint16_t BallBalance_ApplySlew(uint16_t target_x10)
{
    uint16_t next_x10;

    if (target_x10 > s_command_angle_x10) {
        next_x10 = (uint16_t)(s_command_angle_x10 +
                              BALL_BALANCE_SLEW_X10_PER_UPDATE);
        if ((next_x10 < s_command_angle_x10) || (next_x10 > target_x10)) {
            next_x10 = target_x10;
        }
        if (next_x10 != target_x10) {
            s_output_limit_count = BallBalance_IncrementU32(
                s_output_limit_count
            );
        }
        return next_x10;
    }

    if (target_x10 < s_command_angle_x10) {
        if (s_command_angle_x10 <= BALL_BALANCE_SLEW_X10_PER_UPDATE) {
            next_x10 = target_x10;
        } else {
            next_x10 = (uint16_t)(s_command_angle_x10 -
                                  BALL_BALANCE_SLEW_X10_PER_UPDATE);
            if (next_x10 < target_x10) {
                next_x10 = target_x10;
            }
        }
        if (next_x10 != target_x10) {
            s_output_limit_count = BallBalance_IncrementU32(
                s_output_limit_count
            );
        }
        return next_x10;
    }

    return target_x10;
}

void BallBalance_Control_Init(void)
{
    s_enabled = 0U;
    s_target_mm_x10 = 0;
    s_kp = BALL_BALANCE_KP;
    s_ki = BALL_BALANCE_KI;
    s_kd = BALL_BALANCE_KD;
    s_update_count = 0U;
    s_valid_sample_count = 0U;
    s_invalid_sample_count = 0U;
    s_timeout_count = 0U;
    s_output_limit_count = 0U;
    BallBalance_ClearRuntime();
}

void BallBalance_Control_Reset(void)
{
    s_enabled = 0U;
    s_target_mm_x10 = 0;
    s_kp = BALL_BALANCE_KP;
    s_ki = BALL_BALANCE_KI;
    s_kd = BALL_BALANCE_KD;
    s_update_count = 0U;
    s_valid_sample_count = 0U;
    s_invalid_sample_count = 0U;
    s_timeout_count = 0U;
    s_output_limit_count = 0U;
    BallBalance_ClearRuntime();
}

void BallBalance_Control_SetEnabled(uint8_t enabled)
{
    if (enabled == 0U) {
        s_enabled = 0U;
        BallBalance_ClearRuntime();
        return;
    }

    if (s_enabled == 0U) {
        BallBalance_ClearRuntime();
    }
    s_enabled = 1U;
}

uint8_t BallBalance_Control_IsEnabled(void)
{
    return s_enabled;
}

void BallBalance_Control_SetTargetMmX10(int16_t target_mm_x10)
{
    s_target_mm_x10 = target_mm_x10;
}

int16_t BallBalance_Control_GetTargetMmX10(void)
{
    return s_target_mm_x10;
}

void BallBalance_Control_SetGains(float kp, float ki, float kd)
{
    s_kp = kp;
    s_ki = ki;
    s_kd = kd;
}

void BallBalance_Control_PushMeasurement(int16_t position_mm_x10,
                                         uint8_t sequence,
                                         uint32_t timestamp_ms)
{
    uint8_t previous_valid;
    uint32_t dt_ms;
    float raw_position_mm;
    float previous_filtered_mm;
    float raw_velocity_mm_s;

    if ((s_has_sequence != 0U) && (sequence == s_last_sequence)) {
        return;
    }

    previous_valid = (uint8_t)((s_measurement_valid != 0U) &&
                               (s_data_timeout == 0U));
    previous_filtered_mm = s_filtered_position_mm;
    raw_position_mm = (float)position_mm_x10 / 10.0f;

    if ((s_position_filter_valid == 0U) || (previous_valid == 0U)) {
        s_filtered_position_mm = raw_position_mm;
        s_position_filter_valid = 1U;
    } else {
        s_filtered_position_mm +=
            BALL_BALANCE_POSITION_FILTER_ALPHA *
            (raw_position_mm - s_filtered_position_mm);
    }

    dt_ms = timestamp_ms - s_last_sample_ms;
    if ((previous_valid != 0U) &&
        (dt_ms > 0U) &&
        (dt_ms <= BALL_BALANCE_DATA_TIMEOUT_MS)) {
        raw_velocity_mm_s =
            (s_filtered_position_mm - previous_filtered_mm) *
            1000.0f / (float)dt_ms;
    } else {
        raw_velocity_mm_s = 0.0f;
        s_velocity_filter_valid = 0U;
    }

    if (s_velocity_filter_valid == 0U) {
        s_velocity_mm_s = raw_velocity_mm_s;
        s_velocity_filter_valid = 1U;
    } else {
        s_velocity_mm_s +=
            BALL_BALANCE_VELOCITY_FILTER_ALPHA *
            (raw_velocity_mm_s - s_velocity_mm_s);
    }

    s_raw_position_mm_x10 = position_mm_x10;
    s_last_sequence = sequence;
    s_has_sequence = 1U;
    s_last_sample_ms = timestamp_ms;
    s_measurement_valid = 1U;
    s_data_timeout = 0U;
    s_valid_sample_count = BallBalance_IncrementU32(s_valid_sample_count);
}

void BallBalance_Control_InvalidateMeasurement(uint32_t timestamp_ms)
{
    s_measurement_valid = 0U;
    s_data_timeout = 0U;
    s_velocity_filter_valid = 0U;
    s_velocity_mm_s = 0.0f;
    s_last_sample_ms = timestamp_ms;
    s_invalid_sample_count = BallBalance_IncrementU32(s_invalid_sample_count);
}

void BallBalance_Control_Update(uint32_t now_ms)
{
    uint32_t dt_ms;
    float dt_s;
    float target_mm;
    float error_mm;
    float control_error_mm;
    float candidate_integral;
    float output_deg;
    float limited_output_deg;
    uint16_t desired_angle_x10;

    s_update_count = BallBalance_IncrementU32(s_update_count);

    if (s_enabled == 0U) {
        BallBalance_ResetToNeutral();
        return;
    }

    if (s_measurement_valid != 0U) {
        if ((uint32_t)(now_ms - s_last_sample_ms) >=
            BALL_BALANCE_DATA_TIMEOUT_MS) {
            s_measurement_valid = 0U;
            s_data_timeout = 1U;
            s_timeout_count = BallBalance_IncrementU32(s_timeout_count);
            BallBalance_ResetToNeutral();
            return;
        }
    }

    if (s_measurement_valid == 0U) {
        BallBalance_ResetToNeutral();
        return;
    }

    dt_ms = now_ms - s_last_update_ms;
    if (s_last_update_ms == 0U) {
        dt_ms = BALL_BALANCE_UPDATE_PERIOD_MS;
    }
    s_last_update_ms = now_ms;

    if ((dt_ms == 0U) || (dt_ms > BALL_BALANCE_DATA_TIMEOUT_MS)) {
        dt_s = 0.0f;
    } else {
        dt_s = (float)dt_ms / 1000.0f;
    }

    target_mm = (float)s_target_mm_x10 / 10.0f;
    error_mm = s_filtered_position_mm - target_mm;
    control_error_mm = error_mm;
    if (BallBalance_AbsFloat(control_error_mm) <
        BALL_BALANCE_POSITION_DEADBAND_MM) {
        control_error_mm = 0.0f;
    }

    candidate_integral = s_integral;
    if ((dt_s > 0.0f) &&
        (BallBalance_AbsFloat(control_error_mm) <=
         BALL_BALANCE_INTEGRAL_ACTIVE_MM)) {
        candidate_integral += control_error_mm * dt_s;
        candidate_integral = BallBalance_LimitFloat(
            candidate_integral,
            -BALL_BALANCE_INTEGRAL_LIMIT_MM_S,
            BALL_BALANCE_INTEGRAL_LIMIT_MM_S
        );
    }

    s_p_term = s_kp * control_error_mm;
    s_i_term = s_ki * candidate_integral;
    s_d_term = s_kd * s_velocity_mm_s;
    output_deg = s_p_term + s_i_term + s_d_term;
    limited_output_deg = BallBalance_LimitFloat(output_deg, -5.0f, 5.0f);

    if (limited_output_deg != output_deg) {
        s_output_limit_count = BallBalance_IncrementU32(s_output_limit_count);
        if (((output_deg > limited_output_deg) && (control_error_mm > 0.0f)) ||
            ((output_deg < limited_output_deg) && (control_error_mm < 0.0f))) {
            s_i_term = s_ki * s_integral;
            output_deg = s_p_term + s_i_term + s_d_term;
            limited_output_deg = BallBalance_LimitFloat(
                output_deg,
                -5.0f,
                5.0f
            );
        } else {
            s_integral = candidate_integral;
        }
    } else {
        s_integral = candidate_integral;
    }

    s_pid_output_deg = limited_output_deg;
    desired_angle_x10 = BallBalance_FloatDegToX10(
        ((float)BALL_BALANCE_NEUTRAL_ANGLE_X10 / 10.0f) +
        s_pid_output_deg
    );
    if (desired_angle_x10 < BALL_BALANCE_OUTPUT_MIN_X10) {
        desired_angle_x10 = BALL_BALANCE_OUTPUT_MIN_X10;
        s_output_limit_count = BallBalance_IncrementU32(s_output_limit_count);
    }
    if (desired_angle_x10 > BALL_BALANCE_OUTPUT_MAX_X10) {
        desired_angle_x10 = BALL_BALANCE_OUTPUT_MAX_X10;
        s_output_limit_count = BallBalance_IncrementU32(s_output_limit_count);
    }

    desired_angle_x10 = BallBalance_LimitAngleX10(desired_angle_x10);
    s_command_angle_x10 = BallBalance_ApplySlew(desired_angle_x10);
}

Project_Status_t BallBalance_Control_GetInfo(
    BallBalance_ControlInfo_t *info
)
{
    float target_mm;
    float error_mm;

    if (info == 0) {
        return PROJECT_PARAM;
    }

    target_mm = (float)s_target_mm_x10 / 10.0f;
    error_mm = s_filtered_position_mm - target_mm;

    info->enabled = s_enabled;
    info->measurement_valid = s_measurement_valid;
    info->data_timeout = s_data_timeout;
    info->target_mm_x10 = s_target_mm_x10;
    info->raw_position_mm_x10 = s_raw_position_mm_x10;
    info->filtered_position_mm_x10 =
        BallBalance_FloatMmToX10(s_filtered_position_mm);
    info->error_mm_x10 = BallBalance_FloatMmToX10(error_mm);
    info->velocity_mm_s = s_velocity_mm_s;
    info->integral = s_integral;
    info->kp = s_kp;
    info->ki = s_ki;
    info->kd = s_kd;
    info->p_term = s_p_term;
    info->i_term = s_i_term;
    info->d_term = s_d_term;
    info->pid_output_deg = s_pid_output_deg;
    info->command_angle_x10 = s_command_angle_x10;
    info->last_sequence = s_last_sequence;
    info->last_sample_ms = s_last_sample_ms;
    info->update_count = s_update_count;
    info->valid_sample_count = s_valid_sample_count;
    info->invalid_sample_count = s_invalid_sample_count;
    info->timeout_count = s_timeout_count;
    info->output_limit_count = s_output_limit_count;
    return PROJECT_OK;
}
