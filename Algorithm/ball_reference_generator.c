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

static float BallReference_EstimateStopDistance(float velocity_mm_s,
                                                 float acceleration_mm_s2,
                                                 float dt_s)
{
    float distance_mm = 0.0f;
    float next_velocity_mm_s;
    float acceleration_step;
    uint16_t index;

    if (velocity_mm_s <= 0.0f) {
        return 0.0f;
    }

    acceleration_step = BALL_REFERENCE_MAX_JERK_MM_S3 * dt_s;
    for (index = 0U; index < 1000U; index++) {
        acceleration_mm_s2 += BallReference_LimitF(
            -BALL_REFERENCE_MAX_ACCEL_MM_S2 - acceleration_mm_s2,
            -acceleration_step,
            acceleration_step
        );
        next_velocity_mm_s =
            velocity_mm_s + acceleration_mm_s2 * dt_s;
        if (next_velocity_mm_s <= 0.0f) {
            /*
             * 最后一个周期按当前速度计入，故意保守预留一点制动距离，
             * 防止离散积分在终点前仍保持较高速度。
             */
            distance_mm += velocity_mm_s * dt_s;
            break;
        }
        distance_mm +=
            0.5f * (velocity_mm_s + next_velocity_mm_s) * dt_s;
        velocity_mm_s = next_velocity_mm_s;
    }
    return distance_mm;
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
    float remaining_abs;
    float velocity_along_path;
    float acceleration_along_path;
    float stopping_distance;
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
    remaining_abs = BallReference_AbsF(remaining);

    velocity_along_path =
        direction * s_reference.reference_velocity_mm_s;
    acceleration_along_path =
        direction * s_reference.reference_acceleration_mm_s2;
    stopping_distance = BallReference_EstimateStopDistance(
        velocity_along_path,
        acceleration_along_path,
        dt_s
    );

    if (velocity_along_path < 0.0f) {
        s_braking = 0U;
        desired_acceleration =
            direction * BALL_REFERENCE_MAX_ACCEL_MM_S2;
    } else {
        if ((s_braking == 0U) &&
            (remaining_abs <=
             stopping_distance *
                 BALL_REFERENCE_BRAKE_DISTANCE_FACTOR +
             velocity_along_path * dt_s +
             BALL_REFERENCE_SNAP_POSITION_MM)) {
            s_braking = 1U;
        }

        if (s_braking != 0U) {
            desired_acceleration =
                -direction * BALL_REFERENCE_MAX_ACCEL_MM_S2;
        } else if (velocity_along_path >=
                   BALL_REFERENCE_MAX_SPEED_MM_S) {
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
    if ((s_braking != 0U) &&
        (remaining_abs <=
         BALL_REFERENCE_FINAL_APPROACH_POSITION_MM) &&
        (BallReference_AbsF(
             s_reference.reference_velocity_mm_s) <=
         BALL_REFERENCE_FINAL_APPROACH_SPEED_MM_S)) {
        /*
         * 已经完成主制动且只剩很小位置差时直接结束参考轨迹，
         * 避免再次起步、再次刹车引起舵机方向反复切换。
         */
        s_reference.reference_position_mm =
            s_reference.target_position_mm;
        s_reference.reference_velocity_mm_s = 0.0f;
        s_reference.reference_acceleration_mm_s2 = 0.0f;
        s_braking = 0U;
        return;
    }
    if (((remaining > 0.0f) &&
         (s_reference.reference_velocity_mm_s < 0.0f)) ||
        ((remaining < 0.0f) &&
         (s_reference.reference_velocity_mm_s > 0.0f))) {
        /* 目标反向时先刹停，不允许参考位置继续远离新目标。 */
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
