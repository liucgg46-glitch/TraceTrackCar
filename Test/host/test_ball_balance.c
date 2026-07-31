#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "ball_balance_config.h"
#include "ball_balance_control.h"
#include "ball_equilibrium_map.h"
#include "ball_reference_generator.h"
#include "ball_state_estimator.h"

static int s_failed;

static void CheckTrue(const char *name, int condition)
{
    if (!condition) {
        printf("  FAIL: %s\n", name);
        s_failed = 1;
    }
}

static void TestControlDirectionAndLimits(void)
{
    BallBalance_ControlInput_t input = {0};
    BallBalance_ControlOutput_t output;
    uint32_t index;

    printf("[TEST] ball control direction, limit and stiction\n");
    input.control_enabled = 1U;
    input.data_valid = 1U;
    input.allow_stiction_growth = 1U;
    input.dt_s = BALL_BALANCE_CONTROL_PERIOD_S;
    input.reference_position_mm = 50.0f;
    input.equilibrium_angle_deg = BALL_BALANCE_LEVEL_ANGLE_DEG;

    BallBalance_Control_Init();
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("positive target uses smaller servo angle",
              output.limited_dynamic_angle_deg < 0.0f);
    CheckTrue("dynamic angle limited",
              fabsf(output.limited_dynamic_angle_deg) <=
              BALL_BALANCE_DYNAMIC_ANGLE_LIMIT_DEG);
    CheckTrue("slew limit applied",
              fabsf(output.servo_angle_deg -
                    BALL_BALANCE_LEVEL_ANGLE_DEG) <=
              BALL_BALANCE_ANGLE_SLEW_DEG_PER_UPDATE + 0.001f);

    BallBalance_Control_Reset();
    input.reference_position_mm = -50.0f;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("negative target uses larger servo angle",
              output.limited_dynamic_angle_deg > 0.0f);

    BallBalance_Control_Reset();
    input.reference_position_mm = 20.0f;
    input.estimated_velocity_mm_s = 0.0f;
    for (index = 0U; index < 80U; index++) {
        input.now_ms = index * BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("stiction activates only after dwell",
              output.stiction_active != 0U);
    CheckTrue("stiction remains bounded",
              fabsf(output.stiction_angle_deg) <=
              BALL_BALANCE_STICTION_MAX_DEG);

    input.allow_stiction_growth = 0U;
    input.estimated_velocity_mm_s =
        BALL_BALANCE_STICTION_RELEASE_SPEED_MM_S + 1.0f;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("stiction releases while moving",
              fabsf(output.stiction_angle_deg) <
              BALL_BALANCE_STICTION_MAX_DEG);
    printf(s_failed ? "  FAIL\n" : "  PASS\n");
}

static void TestReferenceTrajectory(void)
{
    BallReference_Info_t info;
    float previous;
    float previous_velocity;
    float reach_velocity = 1000.0f;
    uint32_t index;

    printf("[TEST] ball jerk-limited reference trajectory\n");
    BallReference_Init(0.0f);
    BallReference_SetTargetMm(50.0f);
    BallReference_Resume();
    previous = 0.0f;
    (void)BallReference_GetInfo(&info);
    for (index = 0U; index < 2000U; index++) {
        previous_velocity = info.reference_velocity_mm_s;
        BallReference_Update(BALL_BALANCE_CONTROL_PERIOD_S);
        (void)BallReference_GetInfo(&info);
        if ((reach_velocity > 999.0f) &&
            (info.reference_position_mm >= 49.999f)) {
            reach_velocity = fabsf(previous_velocity);
        }
        CheckTrue("reference does not reverse before target",
                  info.reference_position_mm + 0.001f >= previous);
        CheckTrue("reference speed bounded",
                  fabsf(info.reference_velocity_mm_s) <=
                  BALL_REFERENCE_MAX_SPEED_MM_S + 0.001f);
        CheckTrue("reference acceleration bounded",
                  fabsf(info.reference_acceleration_mm_s2) <=
                  BALL_REFERENCE_MAX_ACCEL_MM_S2 + 0.001f);
        CheckTrue("reference does not overshoot",
                  info.reference_position_mm <= 50.001f);
        previous = info.reference_position_mm;
    }
    CheckTrue("reference reaches target",
              fabsf(info.reference_position_mm - 50.0f) < 0.001f);
    CheckTrue("reference stops at target",
              fabsf(info.reference_velocity_mm_s) < 0.001f);
    CheckTrue("reference reaches target at low speed",
              reach_velocity <= 5.0f);
    if (reach_velocity > 5.0f) {
        printf("  reach velocity = %.3f mm/s\n",
               (double)reach_velocity);
    }
    printf(s_failed ? "  FAIL\n" : "  PASS\n");
}

static void TestEstimatorAndEquilibriumMap(void)
{
    BallStateEstimator_Info_t estimator;
    float angle;
    uint32_t index;

    printf("[TEST] ball estimator and equilibrium map\n");
    BallStateEstimator_Init();
    BallStateEstimator_Reset(0.0f);
    for (index = 0U; index < 100U; index++) {
        BallStateEstimator_Predict(
            -1.0f,
            0.0f,
            BALL_BALANCE_CONTROL_PERIOD_S
        );
    }
    (void)BallStateEstimator_GetInfo(&estimator);
    CheckTrue("negative dynamic angle accelerates positive",
              estimator.position_mm > 0.0f);
    CheckTrue("large innovation rejected",
              BallStateEstimator_UpdatePosition(1000.0f) ==
              PROJECT_ERROR);
    (void)BallStateEstimator_GetInfo(&estimator);
    CheckTrue("innovation reject recorded",
              estimator.innovation_rejected != 0U);

    BallEquilibriumMap_Init();
    CheckTrue("map accepts calibrated center",
              BallEquilibriumMap_SetPoint(
                  3U,
                  0.0f,
                  BALL_BALANCE_LEVEL_ANGLE_DEG + 1.0f
              ) == PROJECT_OK);
    angle = BallEquilibriumMap_GetAngleDeg(0.0f);
    CheckTrue("map returns calibrated center",
              fabsf(angle -
                    (BALL_BALANCE_LEVEL_ANGLE_DEG + 1.0f)) <
              0.001f);
    CheckTrue("map rejects unsafe angle",
              BallEquilibriumMap_SetPoint(3U, 0.0f, 20.0f) ==
              PROJECT_PARAM);
    printf(s_failed ? "  FAIL\n" : "  PASS\n");
}

int main(void)
{
    TestControlDirectionAndLimits();
    TestReferenceTrajectory();
    TestEstimatorAndEquilibriumMap();
    return s_failed ? 1 : 0;
}
