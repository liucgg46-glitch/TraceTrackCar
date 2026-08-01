#include "ball_balance_control.h"

#include <math.h>

static BallBalance_ControlInfo_t s_control;
static float s_command_angle_deg;
static float s_command_speed_deg_s;
static float s_target_velocity_mm_s;
static float s_filtered_velocity_mm_s;
static float s_velocity_integral_angle_deg;
static float s_hold_servo_angle_deg;
static float s_tracked_target_position_mm;
static float s_last_nonzero_target_direction;
static uint8_t s_target_tracking_valid;
static uint8_t s_hold_active;
static uint8_t s_lock_tracking;
static uint32_t s_lock_start_ms;

static float BallBalance_Control_AbsF(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float BallBalance_Control_SignF(float value)
{
    if (value > 0.0f) {
        return 1.0f;
    }
    if (value < 0.0f) {
        return -1.0f;
    }
    return 0.0f;
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

static float BallBalance_Control_MoveToward(float current,
                                            float target,
                                            float maximum_step)
{
    float delta = target - current;

    if (delta > maximum_step) {
        return current + maximum_step;
    }
    if (delta < -maximum_step) {
        return current - maximum_step;
    }
    return target;
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

static void BallBalance_Control_ClearHoldTracking(void)
{
    s_hold_active = 0U;
    s_lock_tracking = 0U;
    s_lock_start_ms = 0U;
}

static void BallBalance_Control_ClearClosedLoopState(
    float filtered_velocity_mm_s
)
{
    s_target_velocity_mm_s = 0.0f;
    s_filtered_velocity_mm_s = filtered_velocity_mm_s;
    s_velocity_integral_angle_deg = 0.0f;
    BallBalance_Control_ClearHoldTracking();
}

static void BallBalance_Control_EnterHold(void)
{
    s_hold_active = 1U;
    s_lock_tracking = 0U;
    s_hold_servo_angle_deg = s_command_angle_deg;
    s_command_speed_deg_s = 0.0f;
}

static void BallBalance_Control_UpdateHold(
    const BallBalance_ControlInput_t *input,
    BallBalance_ControlOutput_t *result,
    uint8_t target_changed
)
{
    if (s_hold_active != 0U) {
        if ((input->control_enabled == 0U) ||
            (target_changed != 0U) ||
            (BallBalance_Control_AbsF(result->position_error_mm) >
             BALL_BALANCE_TARGET_LOCK_EXIT_ERROR_MM)) {
            BallBalance_Control_ClearHoldTracking();
        }
        return;
    }

    if ((input->control_enabled == 0U) ||
        (input->data_valid == 0U) ||
        (target_changed != 0U)) {
        s_lock_tracking = 0U;
        return;
    }

    if ((BallBalance_Control_AbsF(result->position_error_mm) <=
         BALL_BALANCE_TARGET_LOCK_ENTER_ERROR_MM) &&
        (BallBalance_Control_AbsF(result->filtered_velocity_mm_s) <=
         BALL_BALANCE_TARGET_LOCK_SPEED_MM_S) &&
        (BallBalance_Control_AbsF(s_command_speed_deg_s) <=
         BALL_BALANCE_TARGET_LOCK_SERVO_SPEED_DEG_S)) {
        if (s_lock_tracking == 0U) {
            s_lock_tracking = 1U;
            s_lock_start_ms = input->now_ms;
        } else if ((uint32_t)(input->now_ms - s_lock_start_ms) >=
                   BALL_BALANCE_TARGET_LOCK_TIME_MS) {
            BallBalance_Control_EnterHold();
        }
    } else {
        s_lock_tracking = 0U;
    }
}

static uint8_t BallBalance_Control_UpdateTargetVelocity(
    BallBalance_ControlOutput_t *result,
    float dt_s,
    uint8_t target_changed
)
{
    float raw_target_velocity_mm_s;
    float raw_target_direction;
    float abs_error_mm;
    float remaining_distance_mm;
    float target_speed_mm_s;
    float final_approach_speed_mm_s;
    float maximum_step_mm_s;
    uint8_t direction_reversed = 0U;

    abs_error_mm = BallBalance_Control_AbsF(
        result->position_error_mm
    );
    if (abs_error_mm <= BALL_BALANCE_POSITION_DEADBAND_MM) {
        raw_target_velocity_mm_s = 0.0f;
    } else {
        remaining_distance_mm =
            abs_error_mm - BALL_BALANCE_POSITION_DEADBAND_MM;
        target_speed_mm_s = sqrtf(
            2.0f *
            BALL_BALANCE_BRAKE_ACCEL_MM_S2 *
            remaining_distance_mm
        );
        final_approach_speed_mm_s =
            BALL_BALANCE_FINAL_APPROACH_KP_S *
            remaining_distance_mm;
        /*
         * 平方根制动曲线在死区边缘斜率过大，静摩擦下容易积累过多倾角后冲过目标。
         * 线性上限只压低近目标速度，远距离仍保持原制动曲线。
         */
        if (target_speed_mm_s > final_approach_speed_mm_s) {
            target_speed_mm_s = final_approach_speed_mm_s;
        }
        raw_target_velocity_mm_s =
            (result->position_error_mm >= 0.0f) ?
            target_speed_mm_s :
            -target_speed_mm_s;
    }
    raw_target_velocity_mm_s = BallBalance_Control_LimitF(
        raw_target_velocity_mm_s,
        -BALL_BALANCE_TARGET_VELOCITY_MAX_MM_S,
        BALL_BALANCE_TARGET_VELOCITY_MAX_MM_S
    );

    raw_target_direction =
        BallBalance_Control_SignF(raw_target_velocity_mm_s);
    if (raw_target_direction != 0.0f) {
        if ((target_changed == 0U) &&
            (s_last_nonzero_target_direction != 0.0f) &&
            (raw_target_direction !=
             s_last_nonzero_target_direction)) {
            /*
             * 钢球越过目标后立即卸掉旧方向的速度命令，避免加速度限幅让旧命令
             * 在过零后继续维持数个周期。舵机角仍由运动轨迹负责平滑切换。
             */
            s_target_velocity_mm_s = 0.0f;
            direction_reversed = 1U;
        }
        s_last_nonzero_target_direction = raw_target_direction;
    }

    maximum_step_mm_s =
        BALL_BALANCE_TARGET_ACCEL_MAX_MM_S2 * dt_s;
    s_target_velocity_mm_s = BallBalance_Control_MoveToward(
        s_target_velocity_mm_s,
        raw_target_velocity_mm_s,
        maximum_step_mm_s
    );
    result->target_velocity_mm_s = s_target_velocity_mm_s;
    return direction_reversed;
}

static void BallBalance_Control_UpdateVelocityPi(
    const BallBalance_ControlInput_t *input,
    BallBalance_ControlOutput_t *result
)
{
    float proportional_angle_deg;
    float integral_delta_angle_deg;
    float stiction_delta_angle_deg;
    float candidate_integral_angle_deg;
    float current_servo_request_deg;
    float candidate_servo_request_deg;
    float servo_direction;
    float target_speed_abs_mm_s;
    float velocity_along_target_mm_s;
    float target_direction;

    result->velocity_error_mm_s =
        result->target_velocity_mm_s -
        result->filtered_velocity_mm_s;
    proportional_angle_deg =
        BALL_BALANCE_VELOCITY_KP * result->velocity_error_mm_s;
    integral_delta_angle_deg =
        BALL_BALANCE_VELOCITY_KI *
        result->velocity_error_mm_s *
        input->dt_s;
    target_direction =
        BallBalance_Control_SignF(result->target_velocity_mm_s);
    target_speed_abs_mm_s =
        BallBalance_Control_AbsF(result->target_velocity_mm_s);
    velocity_along_target_mm_s =
        target_direction * result->filtered_velocity_mm_s;
    stiction_delta_angle_deg =
        target_direction *
        BALL_BALANCE_STICTION_RAMP_DEG_S *
        input->dt_s;

    /*
     * 小球因静摩擦或局部卡滞没有跟上目标速度时，积分角至少按固定斜率连续推进。
     * 这样舵机命令每个周期都有可见的小步变化，而不是等普通KI积分攒够0.1度后跳变。
     */
    if ((target_speed_abs_mm_s >=
         BALL_BALANCE_STICTION_TARGET_MIN_SPEED_MM_S) &&
        (velocity_along_target_mm_s <=
         (target_speed_abs_mm_s -
          BALL_BALANCE_STICTION_VELOCITY_MARGIN_MM_S)) &&
        (BallBalance_Control_SignF(integral_delta_angle_deg) ==
         target_direction) &&
        (BallBalance_Control_AbsF(integral_delta_angle_deg) <
         BallBalance_Control_AbsF(stiction_delta_angle_deg))) {
        integral_delta_angle_deg = stiction_delta_angle_deg;
    }

    /*
     * 钢球已经超过目标速度时，立即卸掉同方向的驱动积分，让比例项提前制动。
     * 只清除仍在推球的积分；已经用于反向制动的积分保持不变。
     */
    if ((target_direction != 0.0f) &&
        (velocity_along_target_mm_s >=
         (target_speed_abs_mm_s +
          BALL_BALANCE_STICTION_VELOCITY_MARGIN_MM_S)) &&
        (BallBalance_Control_SignF(
             s_velocity_integral_angle_deg) == target_direction)) {
        s_velocity_integral_angle_deg = 0.0f;
    }

    candidate_integral_angle_deg =
        s_velocity_integral_angle_deg + integral_delta_angle_deg;
    current_servo_request_deg =
        input->equilibrium_angle_deg +
        BALL_BALANCE_SERVO_TO_ACCEL_SIGN *
        (proportional_angle_deg + s_velocity_integral_angle_deg);
    candidate_servo_request_deg =
        input->equilibrium_angle_deg +
        BALL_BALANCE_SERVO_TO_ACCEL_SIGN *
        (proportional_angle_deg + candidate_integral_angle_deg);
    servo_direction =
        BALL_BALANCE_SERVO_TO_ACCEL_SIGN *
        integral_delta_angle_deg;

    result->integral_blocked = 0U;
    if ((candidate_servo_request_deg >=
         BALL_BALANCE_SERVO_PHYSICAL_MAX_DEG) &&
        (servo_direction > 0.0f) &&
        (current_servo_request_deg >=
         BALL_BALANCE_SERVO_PHYSICAL_MAX_DEG)) {
        result->integral_blocked = 1U;
    } else if ((candidate_servo_request_deg <=
                BALL_BALANCE_SERVO_PHYSICAL_MIN_DEG) &&
               (servo_direction < 0.0f) &&
               (current_servo_request_deg <=
                BALL_BALANCE_SERVO_PHYSICAL_MIN_DEG)) {
        result->integral_blocked = 1U;
    } else {
        s_velocity_integral_angle_deg =
            candidate_integral_angle_deg;
    }

    result->proportional_angle_deg = proportional_angle_deg;
    result->velocity_integral_angle_deg =
        s_velocity_integral_angle_deg;
    result->dynamic_angle_deg =
        BALL_BALANCE_SERVO_TO_ACCEL_SIGN *
        (result->proportional_angle_deg +
         result->velocity_integral_angle_deg);
}

void BallBalance_Control_Init(void)
{
    s_control.initialized = 1U;
    s_control.update_count = 0U;
    s_control.output_limit_count = 0U;
    BallBalance_Control_Reset();
}

void BallBalance_Control_Reset(void)
{
    BallBalance_ControlInput_t zero_input = {0};
    BallBalance_ControlOutput_t zero_output = {0};

    s_command_angle_deg = BALL_BALANCE_LEVEL_ANGLE_DEG;
    s_command_speed_deg_s = 0.0f;
    s_target_velocity_mm_s = 0.0f;
    s_filtered_velocity_mm_s = 0.0f;
    s_velocity_integral_angle_deg = 0.0f;
    s_hold_servo_angle_deg = BALL_BALANCE_LEVEL_ANGLE_DEG;
    s_tracked_target_position_mm = 0.0f;
    s_last_nonzero_target_direction = 0.0f;
    s_target_tracking_valid = 0U;
    BallBalance_Control_ClearHoldTracking();

    zero_output.equilibrium_angle_deg = BALL_BALANCE_LEVEL_ANGLE_DEG;
    zero_output.requested_servo_angle_deg =
        BALL_BALANCE_LEVEL_ANGLE_DEG;
    zero_output.servo_angle_deg = BALL_BALANCE_LEVEL_ANGLE_DEG;
    zero_output.hold_servo_angle_deg = BALL_BALANCE_LEVEL_ANGLE_DEG;
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
    float filter_alpha;
    float requested_angle;
    float limited_angle;
    uint8_t target_changed;
    uint8_t direction_reversed;

    if ((input == 0) || (output == 0) || (input->dt_s <= 0.0f)) {
        return PROJECT_PARAM;
    }

    result.target_position_mm = input->target_position_mm;
    result.estimated_position_mm = input->estimated_position_mm;
    result.position_error_mm =
        input->target_position_mm -
        input->estimated_position_mm;
    result.equilibrium_angle_deg = input->equilibrium_angle_deg;
    result.hold_servo_angle_deg = s_hold_servo_angle_deg;

    target_changed = BallBalance_Control_TargetChanged(input);

    if (input->control_enabled == 0U) {
        /*
         * 控制关闭时清除闭环记忆，但不重置当前舵机命令角，
         * 让舵机仍按速度、加速度限制平滑回到水平角。
         */
        BallBalance_Control_ClearClosedLoopState(
            input->estimated_velocity_mm_s
        );
        requested_angle = BALL_BALANCE_LEVEL_ANGLE_DEG;
    } else if (input->data_valid == 0U) {
        /*
         * 数据失效时冻结目标速度和积分，保持当前舵机命令等待APP超时降级。
         */
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
        result.filtered_velocity_mm_s = s_filtered_velocity_mm_s;

        direction_reversed = BallBalance_Control_UpdateTargetVelocity(
            &result,
            input->dt_s,
            target_changed
        );
        if (direction_reversed != 0U) {
            /* 过零时清除旧方向积分，避免钢球已过目标后舵机仍继续推球。 */
            s_velocity_integral_angle_deg = 0.0f;
        }
        BallBalance_Control_UpdateHold(
            input,
            &result,
            target_changed
        );

        if (s_hold_active != 0U) {
            requested_angle = s_hold_servo_angle_deg;
        } else {
            /*
             * 串级调参顺序：
             * 1. 先将VELOCITY_KI设为0，只调VELOCITY_KP，使实际速度能够跟随目标速度且不过度振荡。
             * 2. 再逐渐增加VELOCITY_KI，直到能够自然克服静摩擦。
             * 3. 再调整POSITION_KP和TARGET_VELOCITY_MAX，决定整体运行速度。
             * 4. 最后调整HOLD阈值。
             */
            BallBalance_Control_UpdateVelocityPi(input, &result);
            requested_angle =
                input->equilibrium_angle_deg +
                result.dynamic_angle_deg;
        }
    }

    result.target_velocity_mm_s = s_target_velocity_mm_s;
    result.filtered_velocity_mm_s = s_filtered_velocity_mm_s;
    result.velocity_integral_angle_deg =
        s_velocity_integral_angle_deg;
    result.hold_active = s_hold_active;
    result.target_locked = s_hold_active;
    result.hold_servo_angle_deg = s_hold_servo_angle_deg;
    result.requested_servo_angle_deg = requested_angle;
    limited_angle = BallBalance_Control_LimitF(
        requested_angle,
        BALL_BALANCE_SERVO_PHYSICAL_MIN_DEG,
        BALL_BALANCE_SERVO_PHYSICAL_MAX_DEG
    );
    result.absolute_limited =
        (limited_angle != requested_angle) ? 1U : 0U;
    result.servo_angle_deg =
        BallBalance_Control_ApplyMotionProfile(
            limited_angle,
            input->dt_s,
            &result.motion_limited
        );
    result.servo_speed_deg_s = s_command_speed_deg_s;
    result.command_angle_x10 =
        BallBalance_Control_DegToX10(result.servo_angle_deg);
    result.applied_dynamic_angle_deg =
        result.servo_angle_deg - input->equilibrium_angle_deg;

    if ((result.absolute_limited != 0U) ||
        (result.motion_limited != 0U)) {
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
