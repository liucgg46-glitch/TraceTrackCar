#include "ball_balance_control.h"

static BallBalance_ControlInfo_t s_control;
static float s_command_angle_deg;
static float s_command_speed_deg_s;
static float s_filtered_velocity_mm_s;
static float s_filtered_disturbance_mm_s2;
static float s_filtered_dynamic_angle_deg;
static float s_breakaway_angle_deg;
static float s_breakaway_last_measured_position_mm;
static uint8_t s_target_locked;
static uint8_t s_lock_tracking;
static uint8_t s_breakaway_measurement_valid;
static uint8_t s_breakaway_progress_count;
static int8_t s_breakaway_motion_sign;
static uint32_t s_lock_start_ms;

static float BallBalance_Control_AbsF(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float BallBalance_Control_LimitF(float value,
                                        float minimum,
                                        float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int8_t BallBalance_Control_SignF(float value)
{
    if (value > 0.0f) {
        return 1;
    }
    if (value < 0.0f) {
        return -1;
    }
    return 0;
}

static uint32_t BallBalance_Control_IncrementU32(uint32_t value)
{
    return (value < 0xFFFFFFFFUL) ? (value + 1UL) : value;
}

static float BallBalance_Control_FilterAlpha(float dt_s,
                                              float time_constant_s)
{
    if (time_constant_s <= 0.0f) {
        return 1.0f;
    }
    return BallBalance_Control_LimitF(
        dt_s / (time_constant_s + dt_s),
        0.0f,
        1.0f
    );
}

static uint16_t BallBalance_Control_DegToX10(float angle_deg)
{
    float scaled = angle_deg * 10.0f;

    scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
    if (scaled < 0.0f) {
        return 0U;
    }
    if (scaled > 1800.0f) {
        return 1800U;
    }
    return (uint16_t)scaled;
}

static void BallBalance_Control_ClearBreakaway(void)
{
    s_breakaway_angle_deg = 0.0f;
    s_breakaway_last_measured_position_mm = 0.0f;
    s_breakaway_measurement_valid = 0U;
    s_breakaway_progress_count = 0U;
    s_breakaway_motion_sign = 0;
    s_control.breakaway_elapsed_ms = 0U;
}

static void BallBalance_Control_ReleaseBreakaway(float dt_s)
{
    float release_step;

    s_control.breakaway_elapsed_ms = 0U;
    release_step = BALL_BALANCE_BREAKAWAY_RELEASE_DEG_S * dt_s;

    if (s_breakaway_angle_deg > release_step) {
        s_breakaway_angle_deg -= release_step;
    } else if (s_breakaway_angle_deg < -release_step) {
        s_breakaway_angle_deg += release_step;
    } else {
        s_breakaway_angle_deg = 0.0f;
    }
}

static uint8_t BallBalance_Control_UpdateBreakawayProgress(
    const BallBalance_ControlInput_t *input,
    int8_t ball_motion_sign
)
{
    float measured_delta_mm;

    if ((input->position_measurement_valid == 0U) ||
        (ball_motion_sign == 0)) {
        return 0U;
    }

    if (s_breakaway_measurement_valid == 0U) {
        s_breakaway_last_measured_position_mm =
            input->measured_position_mm;
        s_breakaway_measurement_valid = 1U;
        s_breakaway_progress_count = 0U;
        return 0U;
    }

    measured_delta_mm =
        input->measured_position_mm -
        s_breakaway_last_measured_position_mm;
    s_breakaway_last_measured_position_mm =
        input->measured_position_mm;

    if ((measured_delta_mm * (float)ball_motion_sign) >=
        BALL_BALANCE_BREAKAWAY_PROGRESS_MM) {
        if (s_breakaway_progress_count <
            BALL_BALANCE_BREAKAWAY_PROGRESS_COUNT) {
            s_breakaway_progress_count++;
        }
    } else if ((measured_delta_mm * (float)ball_motion_sign) <
               0.0f) {
        s_breakaway_progress_count = 0U;
    }

    return (s_breakaway_progress_count >=
            BALL_BALANCE_BREAKAWAY_PROGRESS_COUNT) ? 1U : 0U;
}

static void BallBalance_Control_ResetTargetLock(void)
{
    s_target_locked = 0U;
    s_lock_tracking = 0U;
    s_lock_start_ms = 0U;
}

static uint8_t BallBalance_Control_UpdateTargetLock(
    const BallBalance_ControlInput_t *input,
    float target_error_mm,
    float filtered_velocity_mm_s
)
{
    if ((input->control_enabled == 0U) ||
        (input->data_valid == 0U)) {
        BallBalance_Control_ResetTargetLock();
        return 0U;
    }

    if (s_target_locked != 0U) {
        if (BallBalance_Control_AbsF(target_error_mm) >
            BALL_BALANCE_TARGET_LOCK_EXIT_ERROR_MM) {
            s_target_locked = 0U;
            s_lock_tracking = 0U;
        } else {
            return 1U;
        }
    }

    if ((BallBalance_Control_AbsF(target_error_mm) <=
         BALL_BALANCE_TARGET_LOCK_ENTER_ERROR_MM) &&
        (BallBalance_Control_AbsF(filtered_velocity_mm_s) <=
         BALL_BALANCE_TARGET_LOCK_SPEED_MM_S)) {
        if (s_lock_tracking == 0U) {
            s_lock_tracking = 1U;
            s_lock_start_ms = input->now_ms;
        } else if ((uint32_t)(input->now_ms - s_lock_start_ms) >=
                   BALL_BALANCE_TARGET_LOCK_TIME_MS) {
            s_target_locked = 1U;
            return 1U;
        }
    } else {
        s_lock_tracking = 0U;
    }
    return 0U;
}

static float BallBalance_Control_UpdateBreakaway(
    const BallBalance_ControlInput_t *input,
    float reference_error_mm,
    float filtered_velocity_mm_s
)
{
    float desired_angle_sign;
    float update_step;
    uint8_t progress_confirmed;
    int8_t ball_motion_sign;
    uint32_t dt_ms;

    if ((input->control_enabled == 0U) ||
        (input->data_valid == 0U)) {
        BallBalance_Control_ClearBreakaway();
        return 0.0f;
    }

    ball_motion_sign =
        BallBalance_Control_SignF(reference_error_mm);
    if ((ball_motion_sign != 0) &&
        (s_breakaway_motion_sign != 0) &&
        (ball_motion_sign != s_breakaway_motion_sign)) {
        BallBalance_Control_ClearBreakaway();
    }
    if (ball_motion_sign != 0) {
        s_breakaway_motion_sign = ball_motion_sign;
    }

    if (BallBalance_Control_AbsF(reference_error_mm) <=
        BALL_BALANCE_BREAKAWAY_CLEAR_ERROR_MM) {
        BallBalance_Control_ReleaseBreakaway(input->dt_s);
        return s_breakaway_angle_deg;
    }

    progress_confirmed =
        BallBalance_Control_UpdateBreakawayProgress(
            input,
            ball_motion_sign
        );

    if (input->allow_breakaway_growth == 0U) {
        BallBalance_Control_ReleaseBreakaway(input->dt_s);
        return s_breakaway_angle_deg;
    }

    desired_angle_sign = (float)BallBalance_Control_SignF(
        BallBalance_Model_AccelToDynamicAngleDeg(
            (float)ball_motion_sign
        )
    );

    if (progress_confirmed != 0U) {
        BallBalance_Control_ReleaseBreakaway(input->dt_s);
        return s_breakaway_angle_deg;
    }

    if ((reference_error_mm * filtered_velocity_mm_s > 0.0f) &&
        (BallBalance_Control_AbsF(filtered_velocity_mm_s) >=
         BALL_BALANCE_BREAKAWAY_RELEASE_SPEED_MM_S)) {
        BallBalance_Control_ReleaseBreakaway(input->dt_s);
        return s_breakaway_angle_deg;
    }

    if ((desired_angle_sign != 0.0f) &&
        (BallBalance_Control_AbsF(reference_error_mm) >
         BALL_BALANCE_BREAKAWAY_MIN_ERROR_MM) &&
        (BallBalance_Control_AbsF(filtered_velocity_mm_s) <
         BALL_BALANCE_BREAKAWAY_STUCK_SPEED_MM_S)) {
        dt_ms = (uint32_t)(input->dt_s * 1000.0f + 0.5f);
        if (dt_ms > BALL_BALANCE_CONTROL_PERIOD_MS) {
            dt_ms = BALL_BALANCE_CONTROL_PERIOD_MS;
        }
        if (s_control.breakaway_elapsed_ms <
            (0xFFFFFFFFUL - dt_ms)) {
            s_control.breakaway_elapsed_ms += dt_ms;
        }

        if ((s_control.breakaway_elapsed_ms >=
             BALL_BALANCE_BREAKAWAY_DWELL_MS) &&
            (s_breakaway_angle_deg == 0.0f)) {
            s_breakaway_angle_deg =
                desired_angle_sign *
                BALL_BALANCE_BREAKAWAY_START_DEG;
            s_control.breakaway_update_count =
                BallBalance_Control_IncrementU32(
                    s_control.breakaway_update_count
                );
        } else if (s_breakaway_angle_deg != 0.0f) {
            update_step =
                BALL_BALANCE_BREAKAWAY_GROWTH_DEG_S * input->dt_s;
            s_breakaway_angle_deg +=
                desired_angle_sign * update_step;
            s_control.breakaway_update_count =
                BallBalance_Control_IncrementU32(
                    s_control.breakaway_update_count
                );
        }
    } else {
        BallBalance_Control_ReleaseBreakaway(input->dt_s);
    }

    s_breakaway_angle_deg = BallBalance_Control_LimitF(
        s_breakaway_angle_deg,
        -BALL_BALANCE_BREAKAWAY_MAX_DEG,
        BALL_BALANCE_BREAKAWAY_MAX_DEG
    );
    return s_breakaway_angle_deg;
}

static float BallBalance_Control_ApplyMotionProfile(
    float requested_angle_deg,
    float dt_s,
    uint8_t *limited
)
{
    float angle_error;
    float desired_speed;
    float speed_step;
    float next_angle;

    *limited = 0U;
    angle_error = requested_angle_deg - s_command_angle_deg;
    desired_speed = BallBalance_Control_LimitF(
        angle_error / BALL_BALANCE_SERVO_TRACK_TIME_S,
        -BALL_BALANCE_SERVO_MAX_SPEED_DEG_S,
        BALL_BALANCE_SERVO_MAX_SPEED_DEG_S
    );
    speed_step = BALL_BALANCE_SERVO_MAX_ACCEL_DEG_S2 * dt_s;
    s_command_speed_deg_s += BallBalance_Control_LimitF(
        desired_speed - s_command_speed_deg_s,
        -speed_step,
        speed_step
    );
    next_angle =
        s_command_angle_deg + s_command_speed_deg_s * dt_s;

    /*
     * 轨迹跨过请求角时直接贴合并清零速度，避免二阶整形在目标两侧摆动。
     */
    if ((angle_error == 0.0f) ||
        ((requested_angle_deg - next_angle) * angle_error <= 0.0f)) {
        s_command_angle_deg = requested_angle_deg;
        s_command_speed_deg_s = 0.0f;
    } else {
        s_command_angle_deg = next_angle;
    }

    if ((s_command_angle_deg != requested_angle_deg) ||
        (s_command_speed_deg_s != desired_speed)) {
        *limited = 1U;
    }
    return s_command_angle_deg;
}

void BallBalance_Control_Init(void)
{
    s_control.initialized = 1U;
    s_control.update_count = 0U;
    s_control.output_limit_count = 0U;
    s_control.breakaway_update_count = 0U;
    BallBalance_Control_Reset();
}

void BallBalance_Control_Reset(void)
{
    BallBalance_ControlInput_t zero_input = {0};
    BallBalance_ControlOutput_t zero_output = {0};

    s_command_angle_deg = BALL_BALANCE_LEVEL_ANGLE_DEG;
    s_command_speed_deg_s = 0.0f;
    s_filtered_velocity_mm_s = 0.0f;
    s_filtered_disturbance_mm_s2 = 0.0f;
    s_filtered_dynamic_angle_deg = 0.0f;
    BallBalance_Control_ClearBreakaway();
    BallBalance_Control_ResetTargetLock();
    zero_output.requested_servo_angle_deg =
        BALL_BALANCE_LEVEL_ANGLE_DEG;
    zero_output.servo_angle_deg = BALL_BALANCE_LEVEL_ANGLE_DEG;
    zero_output.command_angle_x10 = BALL_BALANCE_LEVEL_ANGLE_X10;
    s_control.input = zero_input;
    s_control.output = zero_output;
}

Project_Status_t BallBalance_Control_Update(
    const BallBalance_ControlInput_t *input,
    BallBalance_ControlOutput_t *output
)
{
    BallBalance_ControlOutput_t result = {0};
    float position_gain;
    float velocity_gain;
    float filter_alpha;
    float base_servo_angle;
    float target_error_mm;
    float requested_angle;

    if ((input == 0) || (output == 0) || (input->dt_s <= 0.0f)) {
        return PROJECT_PARAM;
    }

    result.position_error_mm =
        input->reference_position_mm -
        input->estimated_position_mm;
    target_error_mm =
        input->target_position_mm -
        input->estimated_position_mm;
    if (input->control_enabled == 0U) {
        BallBalance_Control_ClearBreakaway();
        BallBalance_Control_ResetTargetLock();
        s_filtered_velocity_mm_s = 0.0f;
        s_filtered_disturbance_mm_s2 = 0.0f;
        s_filtered_dynamic_angle_deg = 0.0f;
        requested_angle = BALL_BALANCE_LEVEL_ANGLE_DEG;
    } else if (input->data_valid == 0U) {
        BallBalance_Control_ClearBreakaway();
        BallBalance_Control_ResetTargetLock();
        s_filtered_velocity_mm_s = 0.0f;
        s_filtered_disturbance_mm_s2 = 0.0f;
        s_filtered_dynamic_angle_deg = 0.0f;
        s_command_speed_deg_s = 0.0f;
        requested_angle = s_command_angle_deg;
    } else {
        filter_alpha = BallBalance_Control_FilterAlpha(
            input->dt_s,
            BALL_BALANCE_VELOCITY_FILTER_TIME_S
        );
        s_filtered_velocity_mm_s +=
            filter_alpha *
            (input->estimated_velocity_mm_s -
             s_filtered_velocity_mm_s);
        if (BallBalance_Control_AbsF(
                s_filtered_velocity_mm_s) <=
            BALL_BALANCE_VELOCITY_DEADBAND_MM_S) {
            result.filtered_velocity_mm_s = 0.0f;
        } else {
            result.filtered_velocity_mm_s =
                s_filtered_velocity_mm_s;
        }
        result.velocity_error_mm_s =
            input->reference_velocity_mm_s -
            result.filtered_velocity_mm_s;

        filter_alpha = BallBalance_Control_FilterAlpha(
            input->dt_s,
            BALL_BALANCE_DISTURBANCE_FILTER_TIME_S
        );
        s_filtered_disturbance_mm_s2 +=
            filter_alpha *
            (input->estimated_disturbance_mm_s2 -
             s_filtered_disturbance_mm_s2);
        result.filtered_disturbance_mm_s2 =
            s_filtered_disturbance_mm_s2;

        if (BallBalance_Control_UpdateTargetLock(
                input,
                target_error_mm,
                result.filtered_velocity_mm_s
            ) != 0U) {
            result.target_locked = 1U;
        }
        position_gain =
            BALL_BALANCE_NATURAL_FREQ_RAD_S *
            BALL_BALANCE_NATURAL_FREQ_RAD_S;
        velocity_gain =
            2.0f *
            BALL_BALANCE_DAMPING_RATIO *
            BALL_BALANCE_NATURAL_FREQ_RAD_S;
        result.reference_accel_feedforward_mm_s2 =
            BALL_BALANCE_REFERENCE_ACCEL_FEEDFORWARD_GAIN *
            input->reference_acceleration_mm_s2;
        result.desired_acceleration_mm_s2 =
            result.reference_accel_feedforward_mm_s2 +
            position_gain * result.position_error_mm +
            velocity_gain * result.velocity_error_mm_s;
        result.required_control_acceleration_mm_s2 =
            result.desired_acceleration_mm_s2 -
            BALL_BALANCE_DISTURBANCE_COMPENSATION_GAIN *
            result.filtered_disturbance_mm_s2 -
            input->vehicle_disturbance_mm_s2;
        result.requested_dynamic_angle_deg =
            BallBalance_Model_AccelToDynamicAngleDeg(
                result.required_control_acceleration_mm_s2
            );
        result.limited_dynamic_angle_deg =
            BallBalance_Control_LimitF(
                result.requested_dynamic_angle_deg,
                -BALL_BALANCE_DYNAMIC_HARD_LIMIT_DEG,
                BALL_BALANCE_DYNAMIC_HARD_LIMIT_DEG
            );
        result.dynamic_limited =
            (result.limited_dynamic_angle_deg !=
             result.requested_dynamic_angle_deg) ? 1U : 0U;
        filter_alpha = BallBalance_Control_FilterAlpha(
            input->dt_s,
            BALL_BALANCE_DYNAMIC_FILTER_TIME_S
        );
        s_filtered_dynamic_angle_deg +=
            filter_alpha *
            (result.limited_dynamic_angle_deg -
             s_filtered_dynamic_angle_deg);
        result.limited_dynamic_angle_deg =
            BallBalance_Control_LimitF(
                s_filtered_dynamic_angle_deg,
                -BALL_BALANCE_DYNAMIC_HARD_LIMIT_DEG,
                BALL_BALANCE_DYNAMIC_HARD_LIMIT_DEG
            );
        s_filtered_dynamic_angle_deg =
            result.limited_dynamic_angle_deg;
        base_servo_angle =
            input->equilibrium_angle_deg +
            result.limited_dynamic_angle_deg;
        result.breakaway_angle_deg =
            BallBalance_Control_UpdateBreakaway(
                input,
                result.position_error_mm,
                result.filtered_velocity_mm_s
            );
        requested_angle =
            base_servo_angle +
            result.breakaway_angle_deg;
    }

    result.requested_servo_angle_deg = requested_angle;
    requested_angle = BallBalance_Control_LimitF(
        requested_angle,
        BALL_BALANCE_SERVO_PHYSICAL_MIN_DEG,
        BALL_BALANCE_SERVO_PHYSICAL_MAX_DEG
    );
    result.absolute_limited =
        (requested_angle != result.requested_servo_angle_deg) ? 1U : 0U;
    result.servo_angle_deg =
        BallBalance_Control_ApplyMotionProfile(
            requested_angle,
            input->dt_s,
            &result.motion_limited
        );
    result.servo_speed_deg_s = s_command_speed_deg_s;
    result.command_angle_x10 =
        BallBalance_Control_DegToX10(result.servo_angle_deg);
    result.applied_dynamic_angle_deg =
        result.servo_angle_deg - input->equilibrium_angle_deg;
    result.breakaway_active =
        (s_breakaway_angle_deg != 0.0f) ? 1U : 0U;

    if ((result.absolute_limited != 0U) ||
        (result.motion_limited != 0U) ||
        (result.dynamic_limited != 0U)) {
        s_control.output_limit_count =
            BallBalance_Control_IncrementU32(
                s_control.output_limit_count
            );
    }
    s_control.update_count =
        BallBalance_Control_IncrementU32(s_control.update_count);
    s_control.input = *input;
    s_control.output = result;
    *output = result;
    return PROJECT_OK;
}

Project_Status_t BallBalance_Control_GetInfo(
    BallBalance_ControlInfo_t *info
)
{
    if (info == 0) {
        return PROJECT_PARAM;
    }
    *info = s_control;
    return PROJECT_OK;
}
