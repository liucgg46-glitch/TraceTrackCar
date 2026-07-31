#include "ball_state_estimator.h"

#include "ball_balance_config.h"

static float s_state[3];
static float s_covariance[3][3];
static BallStateEstimator_Info_t s_info;

static float BallStateEstimator_AbsF(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static uint32_t BallStateEstimator_IncrementU32(uint32_t value)
{
    return (value < 0xFFFFFFFFUL) ? (value + 1UL) : value;
}

static void BallStateEstimator_SyncInfo(void)
{
    s_info.position_mm = s_state[0];
    s_info.velocity_mm_s = s_state[1];
    s_info.disturbance_mm_s2 = s_state[2];
}

void BallStateEstimator_Init(void)
{
    s_state[0] = 0.0f;
    s_state[1] = 0.0f;
    s_state[2] = 0.0f;
    s_covariance[0][0] = 0.0f;
    s_covariance[0][1] = 0.0f;
    s_covariance[0][2] = 0.0f;
    s_covariance[1][0] = 0.0f;
    s_covariance[1][1] = 0.0f;
    s_covariance[1][2] = 0.0f;
    s_covariance[2][0] = 0.0f;
    s_covariance[2][1] = 0.0f;
    s_covariance[2][2] = 0.0f;
    s_info.initialized = 0U;
    s_info.measurement_valid = 0U;
    s_info.innovation_mm = 0.0f;
    s_info.innovation_rejected = 0U;
    s_info.prediction_count = 0U;
    s_info.measurement_count = 0U;
    s_info.reject_count = 0U;
    BallStateEstimator_SyncInfo();
}

void BallStateEstimator_Reset(float position_mm)
{
    s_state[0] = position_mm;
    s_state[1] = 0.0f;
    s_state[2] = 0.0f;

    s_covariance[0][0] = BALL_ESTIMATOR_INITIAL_P_POSITION;
    s_covariance[0][1] = 0.0f;
    s_covariance[0][2] = 0.0f;
    s_covariance[1][0] = 0.0f;
    s_covariance[1][1] = BALL_ESTIMATOR_INITIAL_P_VELOCITY;
    s_covariance[1][2] = 0.0f;
    s_covariance[2][0] = 0.0f;
    s_covariance[2][1] = 0.0f;
    s_covariance[2][2] = BALL_ESTIMATOR_INITIAL_P_DISTURBANCE;

    s_info.initialized = 1U;
    s_info.measurement_valid = 1U;
    s_info.innovation_mm = 0.0f;
    s_info.innovation_rejected = 0U;
    BallStateEstimator_SyncInfo();
}

void BallStateEstimator_Predict(float dynamic_angle_deg,
                                float vehicle_disturbance_mm_s2,
                                float dt_s)
{
    float transition[3][3];
    float intermediate[3][3];
    float predicted_covariance[3][3];
    float total_acceleration;
    float dt_squared;
    uint8_t row;
    uint8_t column;
    uint8_t index;

    if ((s_info.initialized == 0U) || (dt_s <= 0.0f)) {
        return;
    }

    dt_squared = dt_s * dt_s;
    total_acceleration =
        BallBalance_Model_DynamicAngleToAccelMmS2(dynamic_angle_deg) +
        s_state[2] +
        vehicle_disturbance_mm_s2;

    s_state[0] += s_state[1] * dt_s +
                  0.5f * total_acceleration * dt_squared;
    s_state[1] += total_acceleration * dt_s;

    transition[0][0] = 1.0f;
    transition[0][1] = dt_s;
    transition[0][2] = 0.5f * dt_squared;
    transition[1][0] = 0.0f;
    transition[1][1] = 1.0f;
    transition[1][2] = dt_s;
    transition[2][0] = 0.0f;
    transition[2][1] = 0.0f;
    transition[2][2] = 1.0f;

    for (row = 0U; row < 3U; row++) {
        for (column = 0U; column < 3U; column++) {
            intermediate[row][column] = 0.0f;
            for (index = 0U; index < 3U; index++) {
                intermediate[row][column] +=
                    transition[row][index] * s_covariance[index][column];
            }
        }
    }

    for (row = 0U; row < 3U; row++) {
        for (column = 0U; column < 3U; column++) {
            predicted_covariance[row][column] = 0.0f;
            for (index = 0U; index < 3U; index++) {
                predicted_covariance[row][column] +=
                    intermediate[row][index] * transition[column][index];
            }
        }
    }
    predicted_covariance[0][0] += BALL_ESTIMATOR_Q_POSITION;
    predicted_covariance[1][1] += BALL_ESTIMATOR_Q_VELOCITY;
    predicted_covariance[2][2] += BALL_ESTIMATOR_Q_DISTURBANCE;

    for (row = 0U; row < 3U; row++) {
        for (column = 0U; column < 3U; column++) {
            s_covariance[row][column] =
                predicted_covariance[row][column];
        }
    }

    s_info.measurement_valid = 0U;
    s_info.prediction_count = BallStateEstimator_IncrementU32(
        s_info.prediction_count
    );
    BallStateEstimator_SyncInfo();
}

Project_Status_t BallStateEstimator_UpdatePosition(float position_mm)
{
    float innovation;
    float innovation_covariance;
    float gain[3];
    float old_row_zero[3];
    uint8_t row;
    uint8_t column;

    if (s_info.initialized == 0U) {
        BallStateEstimator_Reset(position_mm);
        s_info.measurement_count = BallStateEstimator_IncrementU32(
            s_info.measurement_count
        );
        return PROJECT_OK;
    }

    innovation = position_mm - s_state[0];
    s_info.innovation_mm = innovation;
    if (BallStateEstimator_AbsF(innovation) >
        BALL_ESTIMATOR_INNOVATION_LIMIT_MM) {
        s_info.measurement_valid = 0U;
        s_info.innovation_rejected = 1U;
        s_info.reject_count = BallStateEstimator_IncrementU32(
            s_info.reject_count
        );
        return PROJECT_ERROR;
    }

    innovation_covariance =
        s_covariance[0][0] + BALL_ESTIMATOR_R_POSITION;
    if (innovation_covariance <= 0.0f) {
        return PROJECT_ERROR;
    }

    gain[0] = s_covariance[0][0] / innovation_covariance;
    gain[1] = s_covariance[1][0] / innovation_covariance;
    gain[2] = s_covariance[2][0] / innovation_covariance;
    old_row_zero[0] = s_covariance[0][0];
    old_row_zero[1] = s_covariance[0][1];
    old_row_zero[2] = s_covariance[0][2];

    s_state[0] += gain[0] * innovation;
    s_state[1] += gain[1] * innovation;
    s_state[2] += gain[2] * innovation;

    for (row = 0U; row < 3U; row++) {
        for (column = 0U; column < 3U; column++) {
            s_covariance[row][column] -=
                gain[row] * old_row_zero[column];
        }
    }

    /* 抑制浮点舍入造成的非对称，保持后续增益计算稳定。 */
    s_covariance[0][1] =
        0.5f * (s_covariance[0][1] + s_covariance[1][0]);
    s_covariance[1][0] = s_covariance[0][1];
    s_covariance[0][2] =
        0.5f * (s_covariance[0][2] + s_covariance[2][0]);
    s_covariance[2][0] = s_covariance[0][2];
    s_covariance[1][2] =
        0.5f * (s_covariance[1][2] + s_covariance[2][1]);
    s_covariance[2][1] = s_covariance[1][2];

    s_info.measurement_valid = 1U;
    s_info.innovation_rejected = 0U;
    s_info.measurement_count = BallStateEstimator_IncrementU32(
        s_info.measurement_count
    );
    BallStateEstimator_SyncInfo();
    return PROJECT_OK;
}

Project_Status_t BallStateEstimator_GetInfo(BallStateEstimator_Info_t *info)
{
    if (info == 0) {
        return PROJECT_PARAM;
    }
    *info = s_info;
    return PROJECT_OK;
}
