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

static void TestCascadeControl(void)
{
    BallBalance_ControlInput_t input = {0};
    BallBalance_ControlOutput_t output;
    float previous_requested;
    float previous_integral;
    float blocked_integral;
    float hold_servo_angle;
    uint32_t index;

    printf("[TEST] ball position velocity cascade controller\n");
    input.control_enabled = 1U;
    input.data_valid = 1U;
    input.dt_s = BALL_BALANCE_CONTROL_PERIOD_S;
    input.equilibrium_angle_deg = BALL_BALANCE_LEVEL_ANGLE_DEG;

    BallBalance_Control_Init();
    input.target_position_mm = 50.0f;
    input.estimated_position_mm = 0.0f;
    input.estimated_velocity_mm_s = 0.0f;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("positive target generates positive target velocity",
              output.target_velocity_mm_s > 0.0f);
    CheckTrue("target velocity is acceleration limited",
              fabsf(output.target_velocity_mm_s -
                    (BALL_BALANCE_TARGET_ACCEL_MAX_MM_S2 *
                     BALL_BALANCE_CONTROL_PERIOD_S)) < 0.001f);
    CheckTrue("positive target uses smaller servo angle",
              output.dynamic_angle_deg < 0.0f);
    CheckTrue("motion profile acceleration limited",
              fabsf(output.servo_angle_deg -
                    BALL_BALANCE_LEVEL_ANGLE_DEG) <=
              (BALL_BALANCE_SERVO_MAX_ACCEL_DEG_S2 *
               BALL_BALANCE_CONTROL_PERIOD_S *
               BALL_BALANCE_CONTROL_PERIOD_S) + 0.001f);

    previous_requested = output.requested_servo_angle_deg;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("normal PI output has no 0.4 degree command deadband",
              fabsf(output.requested_servo_angle_deg -
                    previous_requested) > 0.001f);

    BallBalance_Control_Reset();
    input.target_position_mm = -50.0f;
    input.estimated_position_mm = 0.0f;
    input.estimated_velocity_mm_s = 0.0f;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("negative target uses larger servo angle",
              output.dynamic_angle_deg > 0.0f);

    BallBalance_Control_Reset();
    input.target_position_mm = 2.0f;
    input.estimated_position_mm = 0.0f;
    input.estimated_velocity_mm_s = 0.0f;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("position deadband zeros target velocity",
              output.target_velocity_mm_s == 0.0f);

    BallBalance_Control_Reset();
    input.target_position_mm = 5.0f;
    input.estimated_position_mm = 0.0f;
    input.estimated_velocity_mm_s = 0.0f;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("final approach limits near-target velocity",
              fabsf(output.target_velocity_mm_s -
                    (BALL_BALANCE_FINAL_APPROACH_KP_S *
                     (5.0f - BALL_BALANCE_POSITION_DEADBAND_MM))) <
              0.001f);

    BallBalance_Control_Reset();
    input.target_position_mm = 200.0f;
    for (index = 0U; index < 200U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("target velocity is clamped",
              fabsf(output.target_velocity_mm_s) <=
              BALL_BALANCE_TARGET_VELOCITY_MAX_MM_S + 0.001f);

    previous_integral = output.velocity_integral_angle_deg;
    input.target_position_mm = -200.0f;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("target change does not clear velocity integral",
              fabsf(output.velocity_integral_angle_deg) >
              fabsf(previous_integral) - 1.0f);

    BallBalance_Control_Reset();
    input.target_position_mm = 0.0f;
    input.estimated_position_mm = 50.0f;
    input.estimated_velocity_mm_s = 0.0f;
    for (index = 0U; index < 20U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("approach accumulates integral in original direction",
              output.velocity_integral_angle_deg < 0.0f);
    input.estimated_position_mm = -5.0f;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("target crossing immediately reverses target velocity",
              output.target_velocity_mm_s > 0.0f);
    CheckTrue("target crossing unloads old velocity integral",
              output.velocity_integral_angle_deg >= 0.0f);

    BallBalance_Control_Reset();
    input.target_position_mm = 50.0f;
    input.estimated_position_mm = 0.0f;
    input.estimated_velocity_mm_s = 0.0f;
    for (index = 0U; index < 20U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("low-speed approach accumulates drive integral",
              output.velocity_integral_angle_deg > 0.0f);
    input.estimated_velocity_mm_s = 1000.0f;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("overspeed unloads drive integral before crossing",
              output.velocity_integral_angle_deg <= 0.0f);

    BallBalance_Control_Reset();
    input.equilibrium_angle_deg = 179.5f;
    input.target_position_mm = -200.0f;
    input.estimated_position_mm = 0.0f;
    input.estimated_velocity_mm_s = 0.0f;
    for (index = 0U; index < 400U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
        if (output.integral_blocked != 0U) {
            break;
        }
    }
    CheckTrue("anti windup blocks integration at 180 degree limit",
              output.integral_blocked != 0U);
    blocked_integral = output.velocity_integral_angle_deg;
    for (index = 0U; index < 20U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("blocked integral remains frozen",
              fabsf(output.velocity_integral_angle_deg -
                    blocked_integral) < 0.001f);

    input.estimated_velocity_mm_s = -1000.0f;
    for (index = 0U; index < 80U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
        if (output.integral_blocked == 0U) {
            break;
        }
    }
    CheckTrue("integral recovers when error leaves limit",
              output.integral_blocked == 0U);

    previous_integral = output.velocity_integral_angle_deg;
    input.data_valid = 0U;
    for (index = 0U; index < 20U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("data invalid freezes integral",
              fabsf(output.velocity_integral_angle_deg -
                    previous_integral) < 0.001f);

    input.control_enabled = 0U;
    input.estimated_velocity_mm_s = 12.3f;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("control disabled clears target velocity",
              output.target_velocity_mm_s == 0.0f);
    CheckTrue("control disabled seeds filtered velocity",
              fabsf(output.filtered_velocity_mm_s -
                    input.estimated_velocity_mm_s) < 0.001f);
    CheckTrue("control disabled clears integral",
              output.velocity_integral_angle_deg == 0.0f);
    CheckTrue("control disabled clears hold",
              output.hold_active == 0U);
    input.control_enabled = 1U;
    input.data_valid = 1U;
    input.equilibrium_angle_deg = BALL_BALANCE_LEVEL_ANGLE_DEG;

    BallBalance_Control_Reset();
    input.target_position_mm = 0.0f;
    input.estimated_position_mm = 0.0f;
    input.estimated_velocity_mm_s = 0.0f;
    for (index = 0U; index < 30U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("controller enters hold after stable dwell",
              output.hold_active != 0U);
    hold_servo_angle = output.hold_servo_angle_deg;
    previous_integral = output.velocity_integral_angle_deg;

    for (index = 0U; index < 20U; index++) {
        input.estimated_position_mm =
            ((index & 1U) != 0U) ? 2.0f : -2.0f;
        input.estimated_velocity_mm_s =
            ((index & 1U) != 0U) ? 2.0f : -2.0f;
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
        CheckTrue("hold keeps recorded servo angle",
                  fabsf(output.servo_angle_deg -
                        hold_servo_angle) < 0.001f);
        CheckTrue("hold freezes integral",
                  fabsf(output.velocity_integral_angle_deg -
                        previous_integral) < 0.001f);
    }

    input.data_valid = 0U;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("short data gap does not exit hold by itself",
              output.hold_active != 0U);
    input.data_valid = 1U;

    input.target_position_mm = 1.0f;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("target change exits hold",
              output.hold_active == 0U);
    CheckTrue("hold exit resumes through motion profile",
              fabsf(output.servo_angle_deg - hold_servo_angle) <=
              (BALL_BALANCE_SERVO_MAX_ACCEL_DEG_S2 *
               BALL_BALANCE_CONTROL_PERIOD_S *
               BALL_BALANCE_CONTROL_PERIOD_S) + 0.001f);

    for (index = 0U; index < 60U; index++) {
        input.estimated_position_mm = 1.0f;
        input.estimated_velocity_mm_s = 0.0f;
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("hold reenters at changed target",
              output.hold_active != 0U);
    input.estimated_position_mm =
        BALL_BALANCE_TARGET_LOCK_EXIT_ERROR_MM + 2.0f;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("position deviation exits hold",
              output.hold_active == 0U);

    printf(s_failed ? "  FAIL\n" : "  PASS\n");
}

static void TestReferenceTrajectory(void)
{
    BallReference_Info_t info;
    float previous;
    float previous_velocity;
    float reach_velocity = 1000.0f;
    int8_t acceleration_sign = 0;
    int8_t previous_acceleration_sign = 0;
    uint32_t acceleration_reversal_count = 0U;
    uint32_t reach_step = 2000U;
    uint32_t reverse_reach_step = 2000U;
    uint32_t index;

    printf("[TEST] ball jerk-limited reference trajectory retained\n");
    BallReference_Init(0.0f);
    BallReference_SetTargetMm(50.0f);
    BallReference_Resume();
    previous = 0.0f;
    (void)BallReference_GetInfo(&info);
    for (index = 0U; index < 2000U; index++) {
        previous_velocity = info.reference_velocity_mm_s;
        BallReference_Update(BALL_BALANCE_CONTROL_PERIOD_S);
        (void)BallReference_GetInfo(&info);
        if (info.reference_acceleration_mm_s2 > 1.0f) {
            acceleration_sign = 1;
        } else if (info.reference_acceleration_mm_s2 < -1.0f) {
            acceleration_sign = -1;
        } else {
            acceleration_sign = 0;
        }
        if ((acceleration_sign != 0) &&
            (previous_acceleration_sign != 0) &&
            (acceleration_sign != previous_acceleration_sign)) {
            acceleration_reversal_count++;
        }
        if (acceleration_sign != 0) {
            previous_acceleration_sign = acceleration_sign;
        }
        if ((reach_velocity > 999.0f) &&
            (info.reference_position_mm >= 49.999f)) {
            reach_velocity = fabsf(previous_velocity);
            reach_step = index;
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
              reach_velocity <= 10.0f);
    CheckTrue("reference does not repeatedly accelerate and brake",
              acceleration_reversal_count <= 2U);
    CheckTrue("reference reaches 50mm within 1.5s",
              reach_step <= 150U);

    BallReference_SetTargetMm(-50.0f);
    for (index = 0U; index < 2000U; index++) {
        BallReference_Update(BALL_BALANCE_CONTROL_PERIOD_S);
        (void)BallReference_GetInfo(&info);
        if (info.reference_position_mm <= -49.999f) {
            reverse_reach_step = index;
            break;
        }
    }
    CheckTrue("reference moves +50mm to -50mm within 2.5s",
              reverse_reach_step <= 250U);
    CheckTrue("reversed reference stops at target",
              fabsf(info.reference_velocity_mm_s) < 0.001f);
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
    CheckTrue("map rejects angle outside physical travel",
              BallEquilibriumMap_SetPoint(3U, 0.0f, -1.0f) ==
              PROJECT_PARAM);
    printf(s_failed ? "  FAIL\n" : "  PASS\n");
}

int main(void)
{
    TestCascadeControl();
    TestReferenceTrajectory();
    TestEstimatorAndEquilibriumMap();
    return s_failed ? 1 : 0;
}
