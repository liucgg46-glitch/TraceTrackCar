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
    float angle_before_transition;
    float angle_before_restart;
    float previous_angle;
    float previous_request;
    float held_angle_deg;
    float previous_speed;
    uint32_t index;

    printf("[TEST] ball control direction and breakaway state machine\n");
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
    input.target_position_mm = -20.0f;
    input.reference_position_mm = -20.0f;
    input.estimated_position_mm = 0.0f;
    input.estimated_velocity_mm_s = 4.0f;
    input.allow_breakaway_growth = 1U;
    input.position_measurement_valid = 0U;
    input.now_ms = 0U;
    for (index = 0U; index < 100U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("speed above 3mm/s blocks breakaway dwell",
              output.breakaway_state == BREAKAWAY_IDLE);

    BallBalance_Control_Reset();
    input.estimated_velocity_mm_s = 0.0f;
    input.allow_breakaway_growth = 1U;
    input.position_measurement_valid = 0U;
    input.now_ms = 0U;
    for (index = 0U; index < 30U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("breakaway stays idle before 300ms dwell",
              output.breakaway_state == BREAKAWAY_IDLE);
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("breakaway enters ramp after dwell",
              output.breakaway_state == BREAKAWAY_RAMP);
    CheckTrue("ramp transition has no angle jump",
              output.breakaway_angle_deg == 0.0f);

    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    previous_request = output.requested_servo_angle_deg;
    previous_angle = output.breakaway_angle_deg;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("ramp grows continuously from zero",
              fabsf(output.breakaway_angle_deg - previous_angle) <=
              (BALL_BALANCE_BREAKAWAY_GROWTH_DEG_S *
               BALL_BALANCE_CONTROL_PERIOD_S) + 0.001f);
    CheckTrue("breakaway bypasses normal 0.4 degree deadband",
              fabsf(output.requested_servo_angle_deg -
                    previous_request) > 0.05f);

    for (index = 0U; index < 210U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("ramp has no former fixed 20 degree limit",
              fabsf(output.breakaway_angle_deg) > 20.0f);
    CheckTrue("ramp request stays inside calibrated safety range",
              (output.requested_servo_angle_deg >=
               BALL_BALANCE_SERVO_SAFE_MIN_DEG - 0.001f) &&
              (output.requested_servo_angle_deg <=
               BALL_BALANCE_SERVO_SAFE_MAX_DEG + 0.001f));

    input.position_measurement_valid = 1U;
    input.measured_position_mm = 0.0f;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    input.measured_position_mm = 1.0f;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    input.measured_position_mm = 2.0f;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("wrong-way movement remains in ramp",
              output.breakaway_state == BREAKAWAY_RAMP);

    input.measured_position_mm = -0.8f;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("one forward frame is insufficient",
              output.breakaway_state == BREAKAWAY_RAMP);
    angle_before_transition = output.breakaway_angle_deg;
    input.measured_position_mm = -1.6f;
    input.estimated_position_mm = -1.6f;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("cumulative 1.5mm and two frames enter decay",
              output.breakaway_state == BREAKAWAY_DECAY);
    CheckTrue("decay starts from actual ramp angle",
              fabsf(output.breakaway_start_angle_deg -
                    output.breakaway_angle_deg) < 0.001f);
    CheckTrue("decay transition does not halve angle",
              fabsf(output.breakaway_angle_deg) >=
              fabsf(angle_before_transition));

    previous_angle = fabsf(output.breakaway_angle_deg);
    input.position_measurement_valid = 0U;
    input.estimated_velocity_mm_s = -12.0f;
    for (index = 0U; index < 10U; index++) {
        input.estimated_position_mm -= 0.5f;
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
        CheckTrue("decay angle is monotonic",
                  fabsf(output.breakaway_angle_deg) <=
                  previous_angle + 0.001f);
        previous_angle = fabsf(output.breakaway_angle_deg);
    }
    CheckTrue("decay reports forward progress",
              output.breakaway_progress_mm > 0.0f);

    input.estimated_velocity_mm_s = 0.0f;
    angle_before_restart = fabsf(output.breakaway_angle_deg);
    for (index = 0U; index < 80U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
        if (output.breakaway_state == BREAKAWAY_RAMP) {
            break;
        }
    }
    CheckTrue("decay restop resumes ramp after 300ms",
              output.breakaway_state == BREAKAWAY_RAMP);
    CheckTrue("restop keeps remaining breakaway angle",
              fabsf(output.breakaway_angle_deg) > 0.0f);
    CheckTrue("restop does not restart from zero",
              fabsf(output.breakaway_angle_deg) <=
              angle_before_restart + 0.001f);
    previous_angle = fabsf(output.breakaway_angle_deg);
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("resumed ramp continues increasing",
              fabsf(output.breakaway_angle_deg) > previous_angle);

    previous_angle = output.breakaway_angle_deg;
    input.target_position_mm = 20.0f;
    input.reference_position_mm = 20.0f;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("target change starts smooth fast decay",
              output.breakaway_state == BREAKAWAY_DECAY);
    CheckTrue("target change does not clear angle instantly",
              (output.breakaway_angle_deg != 0.0f) &&
              (output.breakaway_angle_deg * previous_angle > 0.0f) &&
              (fabsf(output.breakaway_angle_deg) <
               fabsf(previous_angle)));
    for (index = 0U; index < 300U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
        if (output.breakaway_state == BREAKAWAY_COOLDOWN) {
            break;
        }
    }
    CheckTrue("fast decay reaches cooldown",
              output.breakaway_state == BREAKAWAY_COOLDOWN);
    for (index = 0U; index < 24U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
        CheckTrue("250ms cooldown blocks ramp",
                  output.breakaway_state == BREAKAWAY_COOLDOWN);
    }

    BallBalance_Control_Reset();
    input.target_position_mm = 100.0f;
    input.reference_position_mm = 100.0f;
    input.estimated_position_mm = 0.0f;
    input.estimated_velocity_mm_s = 0.0f;
    input.position_measurement_valid = 0U;
    for (index = 0U; index < 100U; index++) {
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("dynamic feedback respects configured limit",
              fabsf(output.limited_dynamic_angle_deg) <=
              BALL_BALANCE_DYNAMIC_HARD_LIMIT_DEG + 0.001f);
    CheckTrue("command remains inside physical servo travel",
              output.command_angle_x10 <= 1800U);
    printf(s_failed ? "  FAIL\n" : "  PASS\n");
}

static void TestControlNoiseAndTargetLock(void)
{
    BallBalance_ControlInput_t input = {0};
    BallBalance_ControlOutput_t output;
    uint16_t held_command_x10;
    uint16_t locked_command_x10;
    float hold_servo_angle_deg;
    float maximum_servo_deviation = 0.0f;
    float deviation;
    uint32_t index;

    printf("[TEST] stationary noise rejection and target lock\n");
    input.control_enabled = 1U;
    input.data_valid = 1U;
    input.allow_breakaway_growth = 1U;
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
    CheckTrue("target lock enters real hold state",
              output.hold_active != 0U);
    locked_command_x10 = output.command_angle_x10;
    hold_servo_angle_deg = output.hold_servo_angle_deg;

    for (index = 0U; index < 200U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        input.estimated_position_mm =
            ((index & 1U) != 0U) ? 2.0f : -2.0f;
        input.estimated_velocity_mm_s =
            ((index & 1U) != 0U) ? 20.0f : -20.0f;
        (void)BallBalance_Control_Update(&input, &output);
        CheckTrue("locked servo ignores in-band estimator noise",
                  output.command_angle_x10 == locked_command_x10);
        CheckTrue("hold output remains exact recorded angle",
                  fabsf(output.servo_angle_deg -
                        hold_servo_angle_deg) < 0.001f);
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
    CheckTrue("hold exit resumes through motion profile",
              fabsf(output.servo_angle_deg -
                    hold_servo_angle_deg) <=
              (BALL_BALANCE_SERVO_MAX_ACCEL_DEG_S2 *
               BALL_BALANCE_CONTROL_PERIOD_S *
               BALL_BALANCE_CONTROL_PERIOD_S) + 0.001f);

    BallBalance_Control_Reset();
    input.target_position_mm = 0.0f;
    input.reference_position_mm = 0.0f;
    input.estimated_position_mm = 0.0f;
    input.estimated_velocity_mm_s = 0.0f;
    input.data_valid = 1U;
    input.control_enabled = 1U;
    for (index = 0U; index < 27U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("hold reenters for exit-condition tests",
              output.hold_active != 0U);
    input.target_position_mm = 1.0f;
    input.reference_position_mm = 1.0f;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("target change exits hold inside position hysteresis",
              output.hold_active == 0U);

    input.estimated_position_mm = 1.0f;
    for (index = 0U; index < 27U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("hold reenters at changed target",
              output.hold_active != 0U);
    input.data_valid = 0U;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("invalid vision exits hold",
              output.hold_active == 0U);
    input.data_valid = 1U;
    for (index = 0U; index < 27U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("hold reenters after vision recovery",
              output.hold_active != 0U);
    input.control_enabled = 0U;
    input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
    (void)BallBalance_Control_Update(&input, &output);
    CheckTrue("control disable exits hold",
              output.hold_active == 0U);
    input.control_enabled = 1U;

    BallBalance_Control_Reset();
    input.target_position_mm = 0.0f;
    input.reference_position_mm = 0.0f;
    input.estimated_position_mm = 0.0f;
    input.estimated_disturbance_mm_s2 = 0.0f;
    input.estimated_velocity_mm_s = 4.0f;
    for (index = 0U; index < 60U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("velocity deadband holds until exit threshold",
              output.filtered_velocity_mm_s == 0.0f);
    input.estimated_velocity_mm_s = 8.0f;
    for (index = 0U; index < 60U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("velocity deadband exits above hysteresis",
              output.filtered_velocity_mm_s >=
              BALL_BALANCE_VELOCITY_DEADBAND_EXIT_MM_S);
    input.estimated_velocity_mm_s = 4.0f;
    for (index = 0U; index < 60U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("velocity deadband stays open above enter threshold",
              output.filtered_velocity_mm_s >
              BALL_BALANCE_VELOCITY_DEADBAND_MM_S);
    input.estimated_velocity_mm_s = 0.0f;
    for (index = 0U; index < 60U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("velocity deadband reenters below enter threshold",
              output.filtered_velocity_mm_s == 0.0f);

    BallBalance_Control_Reset();
    input.target_position_mm = 10.0f;
    input.reference_position_mm = 0.0f;
    input.allow_breakaway_growth = 0U;
    input.estimated_position_mm = 0.0f;
    input.estimated_velocity_mm_s = 0.0f;
    input.equilibrium_angle_deg = BALL_BALANCE_LEVEL_ANGLE_DEG;
    for (index = 0U; index < 30U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    held_command_x10 = output.command_angle_x10;
    input.reference_position_mm = 1.0f;
    for (index = 0U; index < 30U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("servo command deadband rejects tiny request",
              output.command_angle_x10 == held_command_x10);
    input.reference_position_mm = 3.0f;
    for (index = 0U; index < 30U; index++) {
        input.now_ms += BALL_BALANCE_CONTROL_PERIOD_MS;
        (void)BallBalance_Control_Update(&input, &output);
    }
    CheckTrue("servo command deadband accepts accumulated request",
              output.command_angle_x10 != held_command_x10);
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
