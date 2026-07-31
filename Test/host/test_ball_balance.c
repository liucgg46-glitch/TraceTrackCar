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

static void TestControlDirectionAndBreakaway(void)
{
    BallBalance_ControlInput_t input = {0};
    BallBalance_ControlOutput_t output;
    float breakaway_before_motion;
    float held_angle_deg;
    float previous_speed;
    uint32_t index;

    printf("[TEST] ball control direction and continuous breakaway\n");
    input.control_enabled = 1U;
    input.data_valid = 1U;
    input.allow_breakaway_growth = 1U;
    input.dt_s = BALL_BALANCE_CONTROL_PERIOD_S;
    input.target_position_mm = 50.0f;
    input.reference_position_mm = 50.0f;
    input.equilibrium_angle_deg = BALL_BALANCE_LEVEL_ANGLE_DEG;

    BallBalance_Control_Init();
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("positive target uses smaller servo angle",
              output.limited_dynamic_angle_deg < 0.0f);
    CheckTrue("motion profile acceleration limited",
              fabsf(output.servo_angle_deg -
                    BALL_BALANCE_LEVEL_ANGLE_DEG) <=
              (BALL_BALANCE_SERVO_MAX_ACCEL_DEG_S2 *
               BALL_BALANCE_CONTROL_PERIOD_S *
               BALL_BALANCE_CONTROL_PERIOD_S) + 0.001f);
    CheckTrue("motion profile starts smoothly",
              fabsf(output.servo_speed_deg_s) <=
              (BALL_BALANCE_SERVO_MAX_ACCEL_DEG_S2 *
               BALL_BALANCE_CONTROL_PERIOD_S) + 0.001f);

    BallBalance_Control_Reset();
    input.target_position_mm = -50.0f;
    input.reference_position_mm = -50.0f;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("negative target uses larger servo angle",
              output.limited_dynamic_angle_deg > 0.0f);

    /*
     * 实物回归场景：钢球位于中心右侧36mm、目标为中心时，
     * 默认反馈角必须明显大于原先约1.2度的无效小角度。
     */
    BallBalance_Control_Reset();
    input.target_position_mm = 0.0f;
    input.reference_position_mm = 0.0f;
    input.estimated_position_mm = 36.0f;
    input.estimated_velocity_mm_s = 0.0f;
    input.allow_breakaway_growth = 0U;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("36mm center error commands useful angle",
              output.requested_dynamic_angle_deg >= 6.5f);
    CheckTrue("36mm center correction direction",
              output.limited_dynamic_angle_deg > 0.0f);
    for (index = 0U; index < 20U; index++) {
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("36mm center correction reaches widened range",
              output.limited_dynamic_angle_deg >= 4.5f);
    previous_speed = output.servo_speed_deg_s;
    for (index = 0U; index < 20U; index++) {
        (void)BallBalance_Control_Update(&input, &output);
        CheckTrue("servo motion acceleration remains bounded",
                  fabsf(output.servo_speed_deg_s - previous_speed) <=
                  (BALL_BALANCE_SERVO_MAX_ACCEL_DEG_S2 *
                   BALL_BALANCE_CONTROL_PERIOD_S) + 0.001f);
        previous_speed = output.servo_speed_deg_s;
    }
    CheckTrue("servo motion profile reaches useful angle quickly",
              fabsf(output.servo_angle_deg -
                    output.requested_servo_angle_deg) < 0.5f);
    held_angle_deg = output.servo_angle_deg;
    input.data_valid = 0U;
    for (index = 0U; index < 50U; index++) {
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("data loss holds current servo angle",
              fabsf(output.servo_angle_deg - held_angle_deg) <
              0.001f);
    CheckTrue("data loss does not snap back to level",
              fabsf(output.servo_angle_deg -
                    BALL_BALANCE_LEVEL_ANGLE_DEG) > 1.0f);
    input.data_valid = 1U;

    BallBalance_Control_Reset();
    input.target_position_mm = -10.0f;
    input.reference_position_mm = -10.0f;
    input.estimated_position_mm = 0.0f;
    input.estimated_velocity_mm_s = 0.0f;
    input.allow_breakaway_growth = 1U;
    input.position_measurement_valid = 0U;
    for (index = 0U; index < 200U; index++) {
        input.now_ms = index * BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("breakaway activates after dwell",
              output.breakaway_active != 0U);
    CheckTrue("breakaway grows beyond former six degree limit",
              fabsf(output.breakaway_angle_deg) > 15.0f);
    breakaway_before_motion =
        fabsf(output.breakaway_angle_deg);

    input.allow_breakaway_growth = 1U;
    input.estimated_velocity_mm_s = 30.0f;
    input.position_measurement_valid = 1U;
    for (index = 0U; index < 50U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        input.measured_position_mm = 0.2f * (float)index;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("wrong-way motion does not release breakaway",
              fabsf(output.breakaway_angle_deg) >
              breakaway_before_motion);
    breakaway_before_motion =
        fabsf(output.breakaway_angle_deg);

    input.estimated_velocity_mm_s = -30.0f;
    for (index = 0U; index < 30U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        input.measured_position_mm = -0.25f * (float)(index + 1U);
        input.estimated_position_mm = input.measured_position_mm;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("breakaway stops growing after motion",
              fabsf(output.breakaway_angle_deg) <
              breakaway_before_motion);

    BallBalance_Control_Reset();
    input.target_position_mm = 0.0f;
    input.reference_position_mm = 0.0f;
    input.estimated_position_mm = -120.0f;
    input.measured_position_mm = -120.0f;
    input.position_measurement_valid = 1U;
    input.allow_breakaway_growth = 1U;
    previous_speed = 181.0f;
    for (index = 0U; index < 300U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        input.estimated_velocity_mm_s =
            ((index & 1U) != 0U) ? 180.0f : -180.0f;
        (void)BallBalance_Control_Update(&input, &output);
        if (output.breakaway_active != 0U) {
            CheckTrue("stuck far error does not reverse request",
                      output.requested_servo_angle_deg <=
                      previous_speed + 0.001f);
            previous_speed = output.requested_servo_angle_deg;
        }
    }
    CheckTrue("fake estimator speed does not cancel breakaway",
              output.breakaway_active != 0U);

    BallBalance_Control_Reset();
    input.target_position_mm = 100.0f;
    input.reference_position_mm = 100.0f;
    input.estimated_position_mm = 0.0f;
    input.estimated_velocity_mm_s = 0.0f;
    input.position_measurement_valid = 0U;
    for (index = 0U; index < 100U; index++) {
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("dynamic feedback is softened before edge push",
              fabsf(output.limited_dynamic_angle_deg) < 16.0f);
    CheckTrue("command remains inside physical servo travel",
              output.command_angle_x10 <= 1800U);
    printf(s_failed ? "  FAIL\n" : "  PASS\n");
}

static void TestControlNoiseAndTargetLock(void)
{
    BallBalance_ControlInput_t input = {0};
    BallBalance_ControlOutput_t output;
    uint16_t locked_command_x10;
    float maximum_servo_deviation = 0.0f;
    float deviation;
    uint32_t index;

    printf("[TEST] stationary noise rejection and target lock\n");
    input.control_enabled = 1U;
    input.data_valid = 1U;
    input.dt_s = BALL_BALANCE_CONTROL_PERIOD_S;
    input.target_position_mm = 0.0f;
    input.equilibrium_angle_deg = BALL_BALANCE_LEVEL_ANGLE_DEG;

    BallBalance_Control_Init();
    for (index = 0U; index < 400U; index++) {
        input.now_ms = index * BALL_BALANCE_CONTROL_PERIOD_MS;
        input.estimated_velocity_mm_s =
            ((index & 1U) != 0U) ? 12.0f : -12.0f;
        input.estimated_disturbance_mm_s2 =
            ((index & 1U) != 0U) ? 80.0f : -80.0f;
        (void)BallBalance_Control_Update(&input, &output);
        deviation = fabsf(
            output.servo_angle_deg -
            BALL_BALANCE_LEVEL_ANGLE_DEG
        );
        if (deviation > maximum_servo_deviation) {
            maximum_servo_deviation = deviation;
        }
    }
    CheckTrue("stationary measurement noise does not rock servo",
              maximum_servo_deviation < 0.8f);
    CheckTrue("controller locks after reaching target",
              output.target_locked != 0U);
    locked_command_x10 = output.command_angle_x10;

    for (index = 0U; index < 200U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        input.estimated_position_mm =
            ((index & 1U) != 0U) ? 2.0f : -2.0f;
        input.estimated_velocity_mm_s =
            ((index & 1U) != 0U) ? 20.0f : -20.0f;
        (void)BallBalance_Control_Update(&input, &output);
        CheckTrue("locked servo ignores in-band estimator noise",
                  output.command_angle_x10 == locked_command_x10);
    }
    CheckTrue("target lock remains active inside hysteresis",
              output.target_locked != 0U);

    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    input.estimated_position_mm =
        BALL_BALANCE_TARGET_LOCK_EXIT_ERROR_MM + 1.0f;
    input.estimated_velocity_mm_s = 0.0f;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("target lock releases after real position deviation",
              output.target_locked == 0U);
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
    if (acceleration_reversal_count > 2U) {
        printf("  acceleration reversals = %lu\n",
               (unsigned long)acceleration_reversal_count);
    }
    CheckTrue("reference reaches 50mm within 1.5s",
              reach_step <= 150U);
    if (reach_step > 150U) {
        printf("  50mm reach time = %.3f s\n",
               (double)reach_step *
                   BALL_BALANCE_CONTROL_PERIOD_S);
    }
    if (reach_velocity > 10.0f) {
        printf("  reach velocity = %.3f mm/s\n",
               (double)reach_velocity);
    }

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
    if (reverse_reach_step > 250U) {
        printf("  100mm reverse reach time = %.3f s\n",
               (double)reverse_reach_step *
                   BALL_BALANCE_CONTROL_PERIOD_S);
    }
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
    TestControlDirectionAndBreakaway();
    TestControlNoiseAndTargetLock();
    TestReferenceTrajectory();
    TestEstimatorAndEquilibriumMap();
    return s_failed ? 1 : 0;
}
