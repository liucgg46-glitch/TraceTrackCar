#include "ball_reference_generator.h"

#include "ball_balance_config.h"

static BallReference_Info_t s_reference;
static uint8_t s_braking;

static float BallReference_AbsF(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float BallReference_SignF(float value)
{
    if (value > 0.0f) {
        return 1.0f;
    }
    if (value < 0.0f) {
        return -1.0f;
    }
    return 0.0f;
}

static float BallReference_LimitF(float value,
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

void BallReference_Init(float initial_position_mm)
{
    s_reference.initialized = 1U;
    s_reference.paused = 1U;
    s_reference.target_position_mm = initial_position_mm;
    s_reference.reference_position_mm = initial_position_mm;
    s_reference.reference_velocity_mm_s = 0.0f;
    s_reference.reference_acceleration_mm_s2 = 0.0f;
    s_braking = 0U;
}

void BallReference_SetTargetMm(float target_mm)
{
    if (target_mm != s_reference.target_position_mm) {
        s_braking = 0U;
    }
    s_reference.target_position_mm = target_mm;
}

void BallReference_Pause(void)
{
    s_reference.paused = 1U;
    s_reference.reference_velocity_mm_s = 0.0f;
    s_reference.reference_acceleration_mm_s2 = 0.0f;
}

void BallReference_Resume(void)
{
    s_reference.paused = 0U;
}

void BallReference_Update(float dt_s)
{
    float remaining;
    float direction;
    float velocity_direction;
    float braking_distance;
    float jerk_braking_margin;
    float positive_acceleration;
    float transition_time;
    float acceleration_distance;
    float desired_acceleration;
    float acceleration_step;
    float previous_position;
    float next_position;

    if ((s_reference.initialized == 0U) ||
        (s_reference.paused != 0U) ||
        (dt_s <= 0.0f)) {
        return;
    }

    remaining =
        s_reference.target_position_mm -
        s_reference.reference_position_mm;
    if ((BallReference_AbsF(remaining) <=
         BALL_REFERENCE_SNAP_POSITION_MM) &&
        (BallReference_AbsF(s_reference.reference_velocity_mm_s) <=
         BALL_REFERENCE_SNAP_SPEED_MM_S) &&
        (BallReference_AbsF(s_reference.reference_acceleration_mm_s2) <=
         BALL_REFERENCE_SNAP_ACCEL_MM_S2)) {
        s_reference.reference_position_mm =
            s_reference.target_position_mm;
        s_reference.reference_velocity_mm_s = 0.0f;
        s_reference.reference_acceleration_mm_s2 = 0.0f;
        return;
    }

    direction = BallReference_SignF(remaining);
    velocity_direction =
        BallReference_SignF(s_reference.reference_velocity_mm_s);
    braking_distance =
        (s_reference.reference_velocity_mm_s *
         s_reference.reference_velocity_mm_s) /
        (2.0f * BALL_REFERENCE_MAX_ACCEL_MM_S2);
    /*
     * 加速度受jerk限制，开始制动后不能瞬间反向。预留把当前运动状态
     * 平滑切换到最大制动所需的距离，避免接近目标时高速吸附。
     */
    positive_acceleration =
        velocity_direction *
        s_reference.reference_acceleration_mm_s2;
    if (positive_acceleration < 0.0f) {
        positive_acceleration = 0.0f;
    }
    transition_time =
        (positive_acceleration +
         BALL_REFERENCE_MAX_ACCEL_MM_S2) /
        BALL_REFERENCE_MAX_JERK_MM_S3;
    acceleration_distance =
        0.5f * positive_acceleration *
        transition_time * transition_time -
        (BALL_REFERENCE_MAX_JERK_MM_S3 *
         transition_time * transition_time * transition_time) /
        6.0f;
    if (acceleration_distance < 0.0f) {
        acceleration_distance = 0.0f;
    }
    jerk_braking_margin =
        BallReference_AbsF(s_reference.reference_velocity_mm_s) *
        transition_time +
        acceleration_distance;
    braking_distance += jerk_braking_margin;

    if ((velocity_direction != 0.0f) &&
        (velocity_direction != direction)) {
        s_braking = 0U;
        desired_acceleration =
            direction * BALL_REFERENCE_MAX_ACCEL_MM_S2;
    } else {
        if ((s_braking == 0U) &&
            (BallReference_AbsF(remaining) <= braking_distance)) {
            s_braking = 1U;
        }

        if ((s_braking != 0U) &&
            (BallReference_AbsF(
                 s_reference.reference_velocity_mm_s) >
             BALL_REFERENCE_SNAP_SPEED_MM_S)) {
            desired_acceleration =
                -velocity_direction *
                BALL_REFERENCE_MAX_ACCEL_MM_S2;
        } else if ((s_braking != 0U) &&
                   (BallReference_AbsF(remaining) >
                    BALL_REFERENCE_SNAP_POSITION_MM)) {
            s_braking = 0U;
            desired_acceleration =
                direction * BALL_REFERENCE_MAX_ACCEL_MM_S2;
        } else if (s_braking != 0U) {
            desired_acceleration = 0.0f;
        } else {
            desired_acceleration =
                direction * BALL_REFERENCE_MAX_ACCEL_MM_S2;
        }
    }

    acceleration_step =
        BALL_REFERENCE_MAX_JERK_MM_S3 * dt_s;
    s_reference.reference_acceleration_mm_s2 +=
        BallReference_LimitF(
            desired_acceleration -
                s_reference.reference_acceleration_mm_s2,
            -acceleration_step,
            acceleration_step
        );
    s_reference.reference_acceleration_mm_s2 =
        BallReference_LimitF(
            s_reference.reference_acceleration_mm_s2,
            -BALL_REFERENCE_MAX_ACCEL_MM_S2,
            BALL_REFERENCE_MAX_ACCEL_MM_S2
        );

    previous_position = s_reference.reference_position_mm;
    s_reference.reference_velocity_mm_s +=
        s_reference.reference_acceleration_mm_s2 * dt_s;
    s_reference.reference_velocity_mm_s =
        BallReference_LimitF(
            s_reference.reference_velocity_mm_s,
            -BALL_REFERENCE_MAX_SPEED_MM_S,
            BALL_REFERENCE_MAX_SPEED_MM_S
        );
    if (((remaining > 0.0f) &&
         (s_reference.reference_velocity_mm_s < 0.0f)) ||
        ((remaining < 0.0f) &&
         (s_reference.reference_velocity_mm_s > 0.0f))) {
        /*
         * 制动余量偏保守时允许在目标前短暂停住，但不允许参考位置
         * 反向远离目标；下一周期会重新生成向目标方向的平滑加速度。
         */
        s_reference.reference_velocity_mm_s = 0.0f;
        s_reference.reference_acceleration_mm_s2 = 0.0f;
        s_braking = 0U;
    }
    next_position =
        previous_position +
        s_reference.reference_velocity_mm_s * dt_s;

    /* 跨过目标时立即吸附，禁止越过目标后继续向错误方向加速。 */
    if (((s_reference.target_position_mm - previous_position) *
         (s_reference.target_position_mm - next_position)) <= 0.0f) {
        s_reference.reference_position_mm =
            s_reference.target_position_mm;
        s_reference.reference_velocity_mm_s = 0.0f;
        s_reference.reference_acceleration_mm_s2 = 0.0f;
    } else {
        s_reference.reference_position_mm = next_position;
    }
}

Project_Status_t BallReference_GetInfo(BallReference_Info_t *info)
{
    if (info == 0) {
        return PROJECT_PARAM;
    }
    *info = s_reference;
    return PROJECT_OK;
}
