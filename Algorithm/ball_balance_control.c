#include "ball_balance_control.h"

static BallBalance_ControlInfo_t s_control;
static float s_command_angle_deg;
static float s_command_speed_deg_s;
static float s_normal_dynamic_angle_deg;
static float s_filtered_velocity_mm_s;
static float s_filtered_disturbance_mm_s2;
static float s_filtered_dynamic_angle_deg;
static float s_breakaway_angle_deg;
static float s_breakaway_ramp_position_mm;
static float s_breakaway_start_angle_deg;
static float s_breakaway_start_position_mm;
static float s_breakaway_start_error_mm;
static float s_breakaway_progress_mm;
static float s_breakaway_target_position_mm;
static float s_breakaway_last_measured_position_mm;
static float s_hold_servo_angle_deg;
static float s_tracked_target_position_mm;
static uint8_t s_target_locked;
static uint8_t s_lock_tracking;
static uint8_t s_hold_active;
static uint8_t s_velocity_deadband_active;
static uint8_t s_breakaway_measurement_valid;
static uint8_t s_breakaway_forward_count;
static uint8_t s_breakaway_wait_tracking;
static uint8_t s_breakaway_forced_decay;
static uint8_t s_target_tracking_valid;
static int8_t s_breakaway_error_sign;
static BallBalance_BreakawayState_t s_breakaway_state;
static uint32_t s_lock_start_ms;
static uint32_t s_breakaway_state_start_ms;
static uint32_t s_breakaway_wait_start_ms;

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
    s_breakaway_ramp_position_mm = 0.0f;
    s_breakaway_start_angle_deg = 0.0f;
    s_breakaway_start_position_mm = 0.0f;
    s_breakaway_start_error_mm = 0.0f;
    s_breakaway_progress_mm = 0.0f;
    s_breakaway_target_position_mm = 0.0f;
    s_breakaway_last_measured_position_mm = 0.0f;
    s_breakaway_measurement_valid = 0U;
    s_breakaway_forward_count = 0U;
    s_breakaway_wait_tracking = 0U;
    s_breakaway_forced_decay = 0U;
    s_breakaway_error_sign = 0;
    s_breakaway_state = BREAKAWAY_IDLE;
    s_control.breakaway_elapsed_ms = 0U;
    s_breakaway_state_start_ms = 0U;
    s_breakaway_wait_start_ms = 0U;
}

static float BallBalance_Control_MoveTowardZero(float value,
                                                float maximum_step)
{
    if (value > maximum_step) {
        return value - maximum_step;
    }
    if (value < -maximum_step) {
        return value + maximum_step;
    }
    return 0.0f;
}

static void BallBalance_Control_EnterBreakawayCooldown(uint32_t now_ms)
{
    s_breakaway_angle_deg = 0.0f;
    s_breakaway_progress_mm = 0.0f;
    s_breakaway_measurement_valid = 0U;
    s_breakaway_forward_count = 0U;
    s_breakaway_wait_tracking = 0U;
    s_breakaway_forced_decay = 0U;
    s_breakaway_error_sign = 0;
    s_breakaway_state = BREAKAWAY_COOLDOWN;
    s_breakaway_state_start_ms = now_ms;
    s_control.breakaway_elapsed_ms = 0U;
}

static void BallBalance_Control_StartBreakawayClear(uint32_t now_ms)
{
    if ((s_breakaway_state == BREAKAWAY_COOLDOWN) &&
        (s_breakaway_angle_deg == 0.0f)) {
        return;
    }
    s_control.breakaway_elapsed_ms = 0U;
    s_breakaway_wait_tracking = 0U;
    s_breakaway_measurement_valid = 0U;
    s_breakaway_forward_count = 0U;
    if (s_breakaway_angle_deg == 0.0f) {
        BallBalance_Control_EnterBreakawayCooldown(now_ms);
    } else {
        s_breakaway_state = BREAKAWAY_DECAY;
        s_breakaway_forced_decay = 1U;
        s_breakaway_state_start_ms = now_ms;
    }
}

static uint8_t BallBalance_Control_TargetChanged(
    const BallBalance_ControlInput_t *input
)
{
    if (s_target_tracking_valid == 0U) {
        s_tracked_target_position_mm = input->target_position_mm;
        s_target_tracking_valid = 1U;
        return 0U;
    }
    if (BallBalance_Control_AbsF(
            input->target_position_mm -
            s_tracked_target_position_mm) >=
        BALL_BALANCE_TARGET_CHANGE_EPSILON_MM) {
        s_tracked_target_position_mm = input->target_position_mm;
        return 1U;
    }
    return 0U;
}

static float BallBalance_Control_CurrentMeasuredPosition(
    const BallBalance_ControlInput_t *input
)
{
    return (input->position_measurement_valid != 0U) ?
           input->measured_position_mm :
           input->estimated_position_mm;
}

static void BallBalance_Control_EnterBreakawayRamp(
    const BallBalance_ControlInput_t *input,
    int8_t error_sign
)
{
    float position_mm =
        BallBalance_Control_CurrentMeasuredPosition(input);

    s_breakaway_state = BREAKAWAY_RAMP;
    s_breakaway_state_start_ms = input->now_ms;
    s_breakaway_ramp_position_mm = position_mm;
    s_breakaway_last_measured_position_mm = position_mm;
    s_breakaway_target_position_mm = input->target_position_mm;
    s_breakaway_progress_mm = 0.0f;
    s_breakaway_error_sign = error_sign;
    s_breakaway_measurement_valid =
        input->position_measurement_valid;
    s_breakaway_forward_count = 0U;
    s_breakaway_wait_tracking = 0U;
    s_breakaway_forced_decay = 0U;
    s_control.breakaway_elapsed_ms = 0U;
}

static void BallBalance_Control_EnterBreakawayDecay(
    const BallBalance_ControlInput_t *input
)
{
    float position_mm =
        BallBalance_Control_CurrentMeasuredPosition(input);

    s_breakaway_state = BREAKAWAY_DECAY;
    s_breakaway_state_start_ms = input->now_ms;
    s_breakaway_start_angle_deg = s_breakaway_angle_deg;
    s_breakaway_start_position_mm = position_mm;
    s_breakaway_start_error_mm =
        s_breakaway_target_position_mm - position_mm;
    s_breakaway_progress_mm = 0.0f;
    s_breakaway_wait_tracking = 0U;
    s_breakaway_forced_decay = 0U;
    s_control.breakaway_elapsed_ms = 0U;
}

static uint8_t BallBalance_Control_UpdateRampProgress(
    const BallBalance_ControlInput_t *input
)
{
    float measured_delta_mm;

    if ((input->position_measurement_valid == 0U) ||
        (s_breakaway_error_sign == 0)) {
        return 0U;
    }

    if (s_breakaway_measurement_valid == 0U) {
        s_breakaway_last_measured_position_mm =
            input->measured_position_mm;
        s_breakaway_measurement_valid = 1U;
        s_breakaway_forward_count = 0U;
        return 0U;
    }

    measured_delta_mm =
        input->measured_position_mm -
        s_breakaway_last_measured_position_mm;
    s_breakaway_last_measured_position_mm =
        input->measured_position_mm;
    s_breakaway_progress_mm =
        (input->measured_position_mm -
         s_breakaway_ramp_position_mm) *
        (float)s_breakaway_error_sign;
    if (s_breakaway_progress_mm < 0.0f) {
        s_breakaway_progress_mm = 0.0f;
    }

    if ((measured_delta_mm * (float)s_breakaway_error_sign) >=
        BALL_BALANCE_BREAKAWAY_FORWARD_STEP_MIN_MM) {
        if (s_breakaway_forward_count < 0xFFU) {
            s_breakaway_forward_count++;
        }
    } else {
        s_breakaway_forward_count = 0U;
    }

    return ((s_breakaway_progress_mm >=
             BALL_BALANCE_BREAKAWAY_START_PROGRESS_MM) &&
            (s_breakaway_forward_count >=
             BALL_BALANCE_BREAKAWAY_FORWARD_FRAME_COUNT)) ? 1U : 0U;
}

static float BallBalance_Control_LimitBreakawayForSafety(
    float breakaway_angle_deg,
    float base_servo_angle_deg
)
{
    float available_angle_deg;

    if (breakaway_angle_deg >= 0.0f) {
        available_angle_deg =
            BALL_BALANCE_SERVO_SAFE_MAX_DEG - base_servo_angle_deg;
        if (available_angle_deg < 0.0f) {
            available_angle_deg = 0.0f;
        }
        return BallBalance_Control_LimitF(
            breakaway_angle_deg,
            0.0f,
            available_angle_deg
        );
    }

    available_angle_deg =
        BALL_BALANCE_SERVO_SAFE_MIN_DEG - base_servo_angle_deg;
    if (available_angle_deg > 0.0f) {
        available_angle_deg = 0.0f;
    }
    return BallBalance_Control_LimitF(
        breakaway_angle_deg,
        available_angle_deg,
        0.0f
    );
}

static float BallBalance_Control_UpdateDecayTarget(
    float target_error_mm,
    float filtered_velocity_mm_s
)
{
    float start_error_abs;
    float progress_ratio;
    float remaining_ratio;
    float progress_weight;
    float position_factor;
    float toward_speed_mm_s;
    float speed_scale_mm_s;
    float speed_factor;
    float target_magnitude;

    start_error_abs = BallBalance_Control_AbsF(
        s_breakaway_start_error_mm
    );
    if (start_error_abs < BALL_BALANCE_BREAKAWAY_MIN_ERROR_MM) {
        start_error_abs = BALL_BALANCE_BREAKAWAY_MIN_ERROR_MM;
    }
    progress_ratio = BallBalance_Control_LimitF(
        s_breakaway_progress_mm / start_error_abs,
        0.0f,
        1.0f
    );
    remaining_ratio = BallBalance_Control_LimitF(
        BallBalance_Control_AbsF(target_error_mm) / start_error_abs,
        0.0f,
        1.0f
    );
    progress_weight = BallBalance_Control_LimitF(
        BALL_BALANCE_BREAKAWAY_DECAY_PROGRESS_WEIGHT,
        0.0f,
        1.0f
    );
    position_factor =
        progress_weight *
        (1.0f - progress_ratio) +
        (1.0f - progress_weight) *
        remaining_ratio;
    toward_speed_mm_s =
        filtered_velocity_mm_s * (float)s_breakaway_error_sign;
    if (toward_speed_mm_s < 0.0f) {
        toward_speed_mm_s = 0.0f;
    }
    speed_scale_mm_s =
        BALL_BALANCE_BREAKAWAY_DECAY_SPEED_SCALE_MM_S;
    if (speed_scale_mm_s > 0.0f) {
        speed_factor = 1.0f /
            (1.0f + toward_speed_mm_s / speed_scale_mm_s);
    } else {
        speed_factor = 1.0f;
    }
    target_magnitude =
        BallBalance_Control_AbsF(s_breakaway_start_angle_deg) *
        position_factor * speed_factor;
    return BallBalance_Control_LimitF(
        target_magnitude,
        0.0f,
        BallBalance_Control_AbsF(s_breakaway_angle_deg)
    );
}

static void BallBalance_Control_ResetTargetLock(void)
{
    s_target_locked = 0U;
    s_lock_tracking = 0U;
    s_hold_active = 0U;
    s_lock_start_ms = 0U;
}

static uint8_t BallBalance_Control_UpdateTargetLock(
    const BallBalance_ControlInput_t *input,
    float target_error_mm,
    float filtered_velocity_mm_s,
    uint8_t target_changed
)
{
    if (s_hold_active != 0U) {
        if ((input->control_enabled == 0U) ||
            (input->data_valid == 0U) ||
            (input->allow_breakaway_growth == 0U) ||
            (target_changed != 0U) ||
            (BallBalance_Control_AbsF(target_error_mm) >
             BALL_BALANCE_TARGET_LOCK_EXIT_ERROR_MM)) {
            BallBalance_Control_ResetTargetLock();
            s_command_speed_deg_s = 0.0f;
            return 0U;
        }
        return 1U;
    }

    if ((input->control_enabled == 0U) ||
        (input->data_valid == 0U) ||
        (input->allow_breakaway_growth == 0U) ||
        (target_changed != 0U)) {
        BallBalance_Control_ResetTargetLock();
        return 0U;
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
            s_hold_active = 1U;
            s_hold_servo_angle_deg = s_command_angle_deg;
            s_command_speed_deg_s = 0.0f;
            BallBalance_Control_EnterBreakawayCooldown(input->now_ms);
            return 1U;
        }
    } else {
        s_lock_tracking = 0U;
    }
    return 0U;
}

static float BallBalance_Control_UpdateBreakaway(
    const BallBalance_ControlInput_t *input,
    float target_error_mm,
    float filtered_velocity_mm_s,
    float base_servo_angle_deg,
    uint8_t target_changed
)
{
    float angle_sign;
    float current_magnitude;
    float target_magnitude;
    float next_magnitude;
    float position_mm;
    int8_t error_sign = BallBalance_Control_SignF(target_error_mm);
    uint32_t elapsed_ms;

    if ((input->control_enabled == 0U) ||
        (input->data_valid == 0U) ||
        (input->allow_breakaway_growth == 0U) ||
        (target_changed != 0U) ||
        (BallBalance_Control_AbsF(target_error_mm) <=
         BALL_BALANCE_TARGET_LOCK_ENTER_ERROR_MM) ||
        ((s_breakaway_error_sign != 0) &&
         (error_sign != 0) &&
         (error_sign != s_breakaway_error_sign))) {
        BallBalance_Control_StartBreakawayClear(input->now_ms);
    }

    switch (s_breakaway_state) {
        case BREAKAWAY_IDLE:
            if ((input->control_enabled != 0U) &&
                (input->data_valid != 0U) &&
                (input->allow_breakaway_growth != 0U) &&
                (BallBalance_Control_AbsF(target_error_mm) >
                 BALL_BALANCE_BREAKAWAY_MIN_ERROR_MM) &&
                (BallBalance_Control_AbsF(filtered_velocity_mm_s) <
                 BALL_BALANCE_BREAKAWAY_STUCK_SPEED_MM_S) &&
                (error_sign != 0)) {
                if (s_breakaway_wait_tracking == 0U) {
                    s_breakaway_wait_tracking = 1U;
                    s_breakaway_wait_start_ms = input->now_ms;
                }
                elapsed_ms = (uint32_t)(
                    input->now_ms - s_breakaway_wait_start_ms
                );
                s_control.breakaway_elapsed_ms = elapsed_ms;
                if (elapsed_ms >=
                    BALL_BALANCE_BREAKAWAY_IDLE_DWELL_MS) {
                    BallBalance_Control_EnterBreakawayRamp(
                        input,
                        error_sign
                    );
                }
            } else {
                s_breakaway_wait_tracking = 0U;
                s_control.breakaway_elapsed_ms = 0U;
            }
            break;

        case BREAKAWAY_RAMP:
            angle_sign = (float)BallBalance_Control_SignF(
                BallBalance_Model_AccelToDynamicAngleDeg(
                    (float)s_breakaway_error_sign
                )
            );
            s_breakaway_angle_deg +=
                angle_sign *
                BALL_BALANCE_BREAKAWAY_GROWTH_DEG_S *
                input->dt_s;
            s_breakaway_angle_deg =
                BallBalance_Control_LimitBreakawayForSafety(
                    s_breakaway_angle_deg,
                    base_servo_angle_deg
                );
            s_control.breakaway_update_count =
                BallBalance_Control_IncrementU32(
                    s_control.breakaway_update_count
                );
            if (BallBalance_Control_UpdateRampProgress(input) != 0U) {
                BallBalance_Control_EnterBreakawayDecay(input);
            }
            break;

        case BREAKAWAY_DECAY:
            position_mm = input->estimated_position_mm;
            s_breakaway_progress_mm =
                (position_mm - s_breakaway_start_position_mm) *
                (float)s_breakaway_error_sign;
            if (s_breakaway_progress_mm < 0.0f) {
                s_breakaway_progress_mm = 0.0f;
            }
            if (s_breakaway_forced_decay != 0U) {
                s_breakaway_angle_deg =
                    BallBalance_Control_MoveTowardZero(
                        s_breakaway_angle_deg,
                        BALL_BALANCE_BREAKAWAY_CLEAR_RATE_DEG_S *
                        input->dt_s
                    );
            } else {
                target_magnitude =
                    BallBalance_Control_UpdateDecayTarget(
                        target_error_mm,
                        filtered_velocity_mm_s
                    );
                current_magnitude = BallBalance_Control_AbsF(
                    s_breakaway_angle_deg
                );
                next_magnitude = current_magnitude -
                    BALL_BALANCE_BREAKAWAY_DECAY_RATE_DEG_S *
                    input->dt_s;
                if (next_magnitude < target_magnitude) {
                    next_magnitude = target_magnitude;
                }
                if (next_magnitude < 0.0f) {
                    next_magnitude = 0.0f;
                }
                s_breakaway_angle_deg =
                    (float)BallBalance_Control_SignF(
                        s_breakaway_angle_deg
                    ) * next_magnitude;
            }

            if (s_breakaway_angle_deg == 0.0f) {
                BallBalance_Control_EnterBreakawayCooldown(
                    input->now_ms
                );
            } else if ((s_breakaway_forced_decay == 0U) &&
                       (input->allow_breakaway_growth != 0U) &&
                       (BallBalance_Control_AbsF(target_error_mm) >
                        BALL_BALANCE_BREAKAWAY_MIN_ERROR_MM) &&
                       (BallBalance_Control_AbsF(
                            filtered_velocity_mm_s) <
                        BALL_BALANCE_BREAKAWAY_STUCK_SPEED_MM_S)) {
                if (s_breakaway_wait_tracking == 0U) {
                    s_breakaway_wait_tracking = 1U;
                    s_breakaway_wait_start_ms = input->now_ms;
                }
                elapsed_ms = (uint32_t)(
                    input->now_ms - s_breakaway_wait_start_ms
                );
                s_control.breakaway_elapsed_ms = elapsed_ms;
                if (elapsed_ms >=
                    BALL_BALANCE_BREAKAWAY_RESTART_DWELL_MS) {
                    BallBalance_Control_EnterBreakawayRamp(
                        input,
                        error_sign
                    );
                }
            } else {
                s_breakaway_wait_tracking = 0U;
                s_control.breakaway_elapsed_ms = 0U;
            }
            break;

        case BREAKAWAY_COOLDOWN:
            s_breakaway_angle_deg = 0.0f;
            if ((input->control_enabled != 0U) &&
                (input->data_valid != 0U) &&
                ((uint32_t)(input->now_ms -
                            s_breakaway_state_start_ms) >=
                 BALL_BALANCE_BREAKAWAY_COOLDOWN_MS)) {
                s_breakaway_state = BREAKAWAY_IDLE;
                s_breakaway_wait_tracking = 0U;
                s_control.breakaway_elapsed_ms = 0U;
            }
            break;

        default:
            BallBalance_Control_ClearBreakaway();
            break;
    }

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
    s_normal_dynamic_angle_deg = 0.0f;
    s_filtered_velocity_mm_s = 0.0f;
    s_filtered_disturbance_mm_s2 = 0.0f;
    s_filtered_dynamic_angle_deg = 0.0f;
    s_hold_servo_angle_deg = BALL_BALANCE_LEVEL_ANGLE_DEG;
    s_tracked_target_position_mm = 0.0f;
    s_target_tracking_valid = 0U;
    s_velocity_deadband_active = 1U;
    BallBalance_Control_ClearBreakaway();
    BallBalance_Control_ResetTargetLock();
    zero_output.requested_servo_angle_deg =
        BALL_BALANCE_LEVEL_ANGLE_DEG;
    zero_output.servo_angle_deg = BALL_BALANCE_LEVEL_ANGLE_DEG;
    zero_output.hold_servo_angle_deg = BALL_BALANCE_LEVEL_ANGLE_DEG;
    zero_output.breakaway_state = BREAKAWAY_IDLE;
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
    uint8_t target_changed;

    if ((input == 0) || (output == 0) || (input->dt_s <= 0.0f)) {
        return PROJECT_PARAM;
    }

    result.position_error_mm =
        input->reference_position_mm -
        input->estimated_position_mm;
    target_error_mm =
        input->target_position_mm -
        input->estimated_position_mm;
    target_changed = BallBalance_Control_TargetChanged(input);
    if (input->control_enabled == 0U) {
        (void)BallBalance_Control_UpdateTargetLock(
            input,
            target_error_mm,
            0.0f,
            target_changed
        );
        s_filtered_velocity_mm_s = 0.0f;
        s_filtered_disturbance_mm_s2 = 0.0f;
        s_filtered_dynamic_angle_deg = 0.0f;
        s_normal_dynamic_angle_deg = 0.0f;
        s_velocity_deadband_active = 1U;
        (void)BallBalance_Control_UpdateBreakaway(
            input,
            target_error_mm,
            0.0f,
            BALL_BALANCE_LEVEL_ANGLE_DEG,
            target_changed
        );
        requested_angle = BALL_BALANCE_LEVEL_ANGLE_DEG;
    } else if (input->data_valid == 0U) {
        (void)BallBalance_Control_UpdateTargetLock(
            input,
            target_error_mm,
            0.0f,
            target_changed
        );
        s_filtered_velocity_mm_s = 0.0f;
        s_filtered_disturbance_mm_s2 = 0.0f;
        s_filtered_dynamic_angle_deg = 0.0f;
        s_normal_dynamic_angle_deg = 0.0f;
        s_velocity_deadband_active = 1U;
        s_command_speed_deg_s = 0.0f;
        (void)BallBalance_Control_UpdateBreakaway(
            input,
            target_error_mm,
            0.0f,
            s_command_angle_deg,
            target_changed
        );
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
        if (s_velocity_deadband_active != 0U) {
            if (BallBalance_Control_AbsF(
                    s_filtered_velocity_mm_s) >=
                BALL_BALANCE_VELOCITY_DEADBAND_EXIT_MM_S) {
                s_velocity_deadband_active = 0U;
            }
        } else if (BallBalance_Control_AbsF(
                       s_filtered_velocity_mm_s) <=
                   BALL_BALANCE_VELOCITY_DEADBAND_MM_S) {
            s_velocity_deadband_active = 1U;
        }
        if (s_velocity_deadband_active != 0U) {
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
                result.filtered_velocity_mm_s,
                target_changed
            ) != 0U) {
            result.target_locked = 1U;
        }
        if (s_hold_active != 0U) {
            result.hold_active = 1U;
            result.limited_dynamic_angle_deg =
                s_normal_dynamic_angle_deg;
            requested_angle = s_hold_servo_angle_deg;
        } else {
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

            /* 正常反馈角单独使用死区，脱困角不经过该死区。 */
            if (BallBalance_Control_AbsF(
                    result.limited_dynamic_angle_deg -
                    s_normal_dynamic_angle_deg) >=
                BALL_BALANCE_SERVO_COMMAND_DEADBAND_DEG) {
                s_normal_dynamic_angle_deg =
                    result.limited_dynamic_angle_deg;
            }
            result.limited_dynamic_angle_deg =
                s_normal_dynamic_angle_deg;
            base_servo_angle =
                input->equilibrium_angle_deg +
                result.limited_dynamic_angle_deg;
            result.breakaway_angle_deg =
                BallBalance_Control_UpdateBreakaway(
                    input,
                    target_error_mm,
                    s_filtered_velocity_mm_s,
                    base_servo_angle,
                    target_changed
                );
            requested_angle =
                base_servo_angle +
                result.breakaway_angle_deg;
        }
    }

    result.breakaway_angle_deg = s_breakaway_angle_deg;
    result.breakaway_start_angle_deg =
        s_breakaway_start_angle_deg;
    result.breakaway_progress_mm = s_breakaway_progress_mm;
    result.breakaway_state = s_breakaway_state;
    result.hold_active = s_hold_active;
    result.target_locked = s_hold_active;
    result.hold_servo_angle_deg = s_hold_servo_angle_deg;
    result.requested_servo_angle_deg = requested_angle;
    requested_angle = BallBalance_Control_LimitF(
        requested_angle,
        BALL_BALANCE_SERVO_SAFE_MIN_DEG,
        BALL_BALANCE_SERVO_SAFE_MAX_DEG
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
