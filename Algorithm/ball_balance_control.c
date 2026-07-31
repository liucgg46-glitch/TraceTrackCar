#include "ball_balance_control.h"

static BallBalance_ControlInfo_t s_control;
static float s_command_angle_deg;
static float s_stiction_magnitude_deg;
static int8_t s_stiction_direction;
static uint32_t s_last_stiction_step_ms;

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

static void BallBalance_Control_ClearStiction(void)
{
    s_stiction_magnitude_deg = 0.0f;
    s_stiction_direction = 0;
    s_control.stuck_elapsed_ms = 0U;
    s_last_stiction_step_ms = 0U;
}

static float BallBalance_Control_UpdateStiction(
    const BallBalance_ControlInput_t *input,
    float position_error_mm
)
{
    float desired_acceleration_sign;
    float stiction_angle_sign;
    int8_t requested_direction;
    uint32_t dt_ms;

    if ((input->control_enabled == 0U) ||
        (input->data_valid == 0U)) {
        BallBalance_Control_ClearStiction();
        return 0.0f;
    }

    requested_direction = BallBalance_Control_SignF(position_error_mm);
    if ((requested_direction != 0) &&
        (s_stiction_direction != 0) &&
        (requested_direction != s_stiction_direction)) {
        BallBalance_Control_ClearStiction();
    }

    if ((BallBalance_Control_AbsF(position_error_mm) <=
         BALL_BALANCE_SETTLE_ERROR_MM) ||
        (requested_direction == 0)) {
        BallBalance_Control_ClearStiction();
        return 0.0f;
    }

    if (BallBalance_Control_AbsF(input->estimated_velocity_mm_s) >=
        BALL_BALANCE_STICTION_RELEASE_SPEED_MM_S) {
        s_control.stuck_elapsed_ms = 0U;
        if (s_stiction_magnitude_deg >
            BALL_BALANCE_STICTION_STEP_DEG) {
            s_stiction_magnitude_deg -=
                BALL_BALANCE_STICTION_STEP_DEG;
        } else {
            s_stiction_magnitude_deg = 0.0f;
        }
    } else if ((input->allow_stiction_growth != 0U) &&
               (BallBalance_Control_AbsF(position_error_mm) >
                BALL_BALANCE_STUCK_ERROR_MM) &&
               (BallBalance_Control_AbsF(
                    input->estimated_velocity_mm_s) <
                BALL_BALANCE_STUCK_SPEED_MM_S)) {
        dt_ms = (uint32_t)(input->dt_s * 1000.0f + 0.5f);
        if (dt_ms > BALL_BALANCE_CONTROL_PERIOD_MS) {
            dt_ms = BALL_BALANCE_CONTROL_PERIOD_MS;
        }
        if (s_control.stuck_elapsed_ms <
            (0xFFFFFFFFUL - dt_ms)) {
            s_control.stuck_elapsed_ms += dt_ms;
        }
        s_stiction_direction = requested_direction;

        if ((s_control.stuck_elapsed_ms >=
             BALL_BALANCE_STUCK_TIME_MS) &&
            (s_stiction_magnitude_deg == 0.0f)) {
            s_stiction_magnitude_deg =
                BALL_BALANCE_STICTION_START_DEG;
            s_last_stiction_step_ms = input->now_ms;
            s_control.stiction_step_count =
                BallBalance_Control_IncrementU32(
                    s_control.stiction_step_count
                );
        } else if ((s_stiction_magnitude_deg > 0.0f) &&
                   ((uint32_t)(input->now_ms -
                               s_last_stiction_step_ms) >=
                    BALL_BALANCE_STICTION_STEP_PERIOD_MS)) {
            s_stiction_magnitude_deg +=
                BALL_BALANCE_STICTION_STEP_DEG;
            s_stiction_magnitude_deg =
                BallBalance_Control_LimitF(
                    s_stiction_magnitude_deg,
                    0.0f,
                    BALL_BALANCE_STICTION_MAX_DEG
                );
            s_last_stiction_step_ms = input->now_ms;
            s_control.stiction_step_count =
                BallBalance_Control_IncrementU32(
                    s_control.stiction_step_count
                );
        }
    } else {
        s_control.stuck_elapsed_ms = 0U;
    }

    if ((s_stiction_magnitude_deg <= 0.0f) ||
        (s_stiction_direction == 0)) {
        return 0.0f;
    }

    desired_acceleration_sign = (float)s_stiction_direction;
    stiction_angle_sign = BallBalance_Control_SignF(
        BallBalance_Model_AccelToDynamicAngleDeg(
            desired_acceleration_sign
        )
    );
    return stiction_angle_sign * s_stiction_magnitude_deg;
}

static float BallBalance_Control_ApplySlew(float requested_angle_deg,
                                           uint8_t *limited)
{
    float difference = requested_angle_deg - s_command_angle_deg;

    *limited = 0U;
    if (difference > BALL_BALANCE_ANGLE_SLEW_DEG_PER_UPDATE) {
        s_command_angle_deg +=
            BALL_BALANCE_ANGLE_SLEW_DEG_PER_UPDATE;
        *limited = 1U;
    } else if (difference <
               -BALL_BALANCE_ANGLE_SLEW_DEG_PER_UPDATE) {
        s_command_angle_deg -=
            BALL_BALANCE_ANGLE_SLEW_DEG_PER_UPDATE;
        *limited = 1U;
    } else {
        s_command_angle_deg = requested_angle_deg;
    }
    return s_command_angle_deg;
}

void BallBalance_Control_Init(void)
{
    s_control.initialized = 1U;
    s_control.update_count = 0U;
    s_control.output_limit_count = 0U;
    s_control.stiction_step_count = 0U;
    BallBalance_Control_Reset();
}

void BallBalance_Control_Reset(void)
{
    BallBalance_ControlInput_t zero_input = {0};
    BallBalance_ControlOutput_t zero_output = {0};

    s_command_angle_deg = BALL_BALANCE_LEVEL_ANGLE_DEG;
    BallBalance_Control_ClearStiction();
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
    float requested_angle;

    if ((input == 0) || (output == 0) || (input->dt_s <= 0.0f)) {
        return PROJECT_PARAM;
    }

    result.position_error_mm =
        input->reference_position_mm -
        input->estimated_position_mm;
    result.velocity_error_mm_s =
        input->reference_velocity_mm_s -
        input->estimated_velocity_mm_s;

    if ((input->control_enabled != 0U) &&
        (input->data_valid != 0U)) {
        position_gain =
            BALL_BALANCE_NATURAL_FREQ_RAD_S *
            BALL_BALANCE_NATURAL_FREQ_RAD_S;
        velocity_gain =
            2.0f *
            BALL_BALANCE_DAMPING_RATIO *
            BALL_BALANCE_NATURAL_FREQ_RAD_S;
        result.desired_acceleration_mm_s2 =
            input->reference_acceleration_mm_s2 +
            position_gain * result.position_error_mm +
            velocity_gain * result.velocity_error_mm_s;
        result.required_control_acceleration_mm_s2 =
            result.desired_acceleration_mm_s2 -
            input->estimated_disturbance_mm_s2 -
            input->vehicle_disturbance_mm_s2;
        result.requested_dynamic_angle_deg =
            BallBalance_Model_AccelToDynamicAngleDeg(
                result.required_control_acceleration_mm_s2
            );
        result.limited_dynamic_angle_deg =
            BallBalance_Control_LimitF(
                result.requested_dynamic_angle_deg,
                -BALL_BALANCE_DYNAMIC_ANGLE_LIMIT_DEG,
                BALL_BALANCE_DYNAMIC_ANGLE_LIMIT_DEG
            );
        result.dynamic_limited =
            (result.requested_dynamic_angle_deg !=
             result.limited_dynamic_angle_deg) ? 1U : 0U;
        result.stiction_angle_deg =
            BallBalance_Control_UpdateStiction(
                input,
                result.position_error_mm
            );
        requested_angle =
            input->equilibrium_angle_deg +
            result.limited_dynamic_angle_deg +
            result.stiction_angle_deg;
    } else {
        BallBalance_Control_ClearStiction();
        requested_angle = BALL_BALANCE_LEVEL_ANGLE_DEG;
    }

    result.requested_servo_angle_deg = requested_angle;
    requested_angle = BallBalance_Control_LimitF(
        requested_angle,
        BALL_BALANCE_ABS_SAFE_MIN_DEG,
        BALL_BALANCE_ABS_SAFE_MAX_DEG
    );
    result.absolute_limited =
        (requested_angle != result.requested_servo_angle_deg) ? 1U : 0U;
    result.servo_angle_deg =
        BallBalance_Control_ApplySlew(
            requested_angle,
            &result.slew_limited
        );
    result.command_angle_x10 =
        BallBalance_Control_DegToX10(result.servo_angle_deg);
    result.applied_dynamic_angle_deg =
        result.servo_angle_deg - input->equilibrium_angle_deg;
    result.stiction_active =
        (s_stiction_magnitude_deg > 0.0f) ? 1U : 0U;

    if ((result.dynamic_limited != 0U) ||
        (result.absolute_limited != 0U) ||
        (result.slew_limited != 0U)) {
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
