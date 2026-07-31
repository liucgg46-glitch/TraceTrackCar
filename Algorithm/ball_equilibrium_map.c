#include "ball_equilibrium_map.h"

static BallEquilibriumMap_Info_t s_map;

static uint8_t BallEquilibriumMap_AngleSafe(float angle_deg)
{
    return ((angle_deg >= BALL_BALANCE_ABS_SAFE_MIN_DEG) &&
            (angle_deg <= BALL_BALANCE_ABS_SAFE_MAX_DEG)) ? 1U : 0U;
}

void BallEquilibriumMap_Init(void)
{
    static const float default_position_mm[
        BALL_EQUILIBRIUM_MAP_POINT_COUNT
    ] = {
        BALL_EQUILIBRIUM_POS_0_MM,
        BALL_EQUILIBRIUM_POS_1_MM,
        BALL_EQUILIBRIUM_POS_2_MM,
        BALL_EQUILIBRIUM_POS_3_MM,
        BALL_EQUILIBRIUM_POS_4_MM,
        BALL_EQUILIBRIUM_POS_5_MM,
        BALL_EQUILIBRIUM_POS_6_MM
    };
    uint8_t index;

    s_map.point_count = BALL_EQUILIBRIUM_MAP_POINT_COUNT;
    for (index = 0U;
         index < BALL_EQUILIBRIUM_MAP_POINT_COUNT;
         index++) {
        s_map.position_mm[index] = default_position_mm[index];
        s_map.angle_deg[index] = BALL_BALANCE_LEVEL_ANGLE_DEG;
    }
}

float BallEquilibriumMap_GetAngleDeg(float position_mm)
{
    float span;
    float ratio;
    uint8_t index;

    if (position_mm <= s_map.position_mm[0]) {
        return s_map.angle_deg[0];
    }
    if (position_mm >=
        s_map.position_mm[BALL_EQUILIBRIUM_MAP_POINT_COUNT - 1U]) {
        return
            s_map.angle_deg[BALL_EQUILIBRIUM_MAP_POINT_COUNT - 1U];
    }

    for (index = 0U;
         index < (BALL_EQUILIBRIUM_MAP_POINT_COUNT - 1U);
         index++) {
        if (position_mm <= s_map.position_mm[index + 1U]) {
            span =
                s_map.position_mm[index + 1U] -
                s_map.position_mm[index];
            ratio =
                (position_mm - s_map.position_mm[index]) / span;
            return s_map.angle_deg[index] +
                   ratio *
                   (s_map.angle_deg[index + 1U] -
                    s_map.angle_deg[index]);
        }
    }

    return BALL_BALANCE_LEVEL_ANGLE_DEG;
}

Project_Status_t BallEquilibriumMap_SetPoint(uint8_t index,
                                             float position_mm,
                                             float angle_deg)
{
    if ((index >= BALL_EQUILIBRIUM_MAP_POINT_COUNT) ||
        (BallEquilibriumMap_AngleSafe(angle_deg) == 0U)) {
        return PROJECT_PARAM;
    }
    if ((index > 0U) &&
        (position_mm <= s_map.position_mm[index - 1U])) {
        return PROJECT_PARAM;
    }
    if ((index + 1U < BALL_EQUILIBRIUM_MAP_POINT_COUNT) &&
        (position_mm >= s_map.position_mm[index + 1U])) {
        return PROJECT_PARAM;
    }

    s_map.position_mm[index] = position_mm;
    s_map.angle_deg[index] = angle_deg;
    return PROJECT_OK;
}

Project_Status_t BallEquilibriumMap_GetInfo(BallEquilibriumMap_Info_t *info)
{
    if (info == 0) {
        return PROJECT_PARAM;
    }
    *info = s_map;
    return PROJECT_OK;
}
