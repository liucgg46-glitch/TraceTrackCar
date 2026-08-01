#include "task_fsm.h"

#include "ball_balance_app.h"
#include "bsp_key.h"
#include "bsp_systick.h"
#include "chassis.h"
#include "drv_gray_sensor.h"
#include "drv_encoder.h"
#include "k210_comm.h"
#include "line_follow_app.h"
#include "motion_action.h"
#include "route_manager.h"
#include "route_profile_h_oval.h"
#include "task_profile_h2_round_stop.h"

#define MISSION_CUSTOM_STEP_MM_X10             50
#define MISSION_CUSTOM_MIN_MM_X10              (-1000)
#define MISSION_CUSTOM_MAX_MM_X10              1000
#define MISSION_TARGET_O_MM_X10                0
#define MISSION_TARGET_PLUS_50_MM_X10          500
#define MISSION_TARGET_MINUS_50_MM_X10         (-500)
#define MISSION_ARM_TIMEOUT_MS                 5000U
#define MISSION_BALL_ERROR_LIMIT_MM_X10        100
#define MISSION_ROUTE_START_ROOM               0U
#define MISSION_M3_PLUS_REACH_POSITION_MM      45.0f
#define MISSION_M3_PLUS_MIN_VELOCITY_MM_S      (-5.0f)
#define MISSION_M3_PLUS_CONFIRM_FRAME_COUNT    2U

/* 任务2终点接近与主动反向制动参数。 */
#define MISSION_M2_FINISH_APPROACH_SPEED_CPS        2000
#define MISSION_M2_FINISH_APPROACH_MIN_SPEED_CPS    1600
#define MISSION_M2_REVERSE_BRAKE_SPEED_CPS           900
#define MISSION_M2_BRAKE_STOP_THRESHOLD_CPS          250
#define MISSION_M2_REVERSE_BRAKE_MAX_TIME_MS         180U

/* 任务4/5/6独立速度斜坡参数，实车调速只需修改本区域。 */
#define MISSION_DYNAMIC_START_SPEED_CPS               1000
#define MISSION_DYNAMIC_MAX_SPEED_CPS                 2400
#define MISSION_DYNAMIC_CURVE_SPEED_CPS               1900
#define MISSION_DYNAMIC_MIN_TRACK_SPEED_CPS           1000
#define MISSION_DYNAMIC_ACCEL_RATE_CPS_PER_S           1000.0f
#define MISSION_DYNAMIC_DECEL_RATE_CPS_PER_S           1200.0f
#define MISSION_M4_PASS_B_DISTANCE_MM                  120
#define MISSION_H56_DECEL_START_DISTANCE_MM            5200
#define MISSION_H56_PASS_A_DISTANCE_MM                 120
#define MISSION_DYNAMIC_STOP_THRESHOLD_CPS             80

static Mission_Info_t s_mission;
static uint32_t s_state_enter_ms;
static uint32_t s_substate_enter_ms;
static uint8_t s_last_key_ready;
static uint8_t s_m3_plus_confirm_count;
static uint32_t s_m3_last_processed_sample_ms;
static uint8_t s_m3_sample_tracking_valid;
static uint8_t s_m2_finish_approach_applied;
static uint8_t s_m2_brake_active;
static uint32_t s_m2_brake_start_ms;
static float s_dynamic_speed_cps;
static uint32_t s_dynamic_speed_last_ms;
static int32_t s_dynamic_start_distance_mm;
static int32_t s_dynamic_brake_start_distance_mm;
static uint8_t s_dynamic_stop_requested;

static int16_t Mission_Abs16(int16_t value)
{
    return (value < 0) ? (int16_t)(-value) : value;
}

static int16_t Mission_LimitTarget(int16_t target_mm_x10)
{
    if (target_mm_x10 < MISSION_CUSTOM_MIN_MM_X10) {
        return MISSION_CUSTOM_MIN_MM_X10;
    }
    if (target_mm_x10 > MISSION_CUSTOM_MAX_MM_X10) {
        return MISSION_CUSTOM_MAX_MM_X10;
    }
    return target_mm_x10;
}

static void Mission_StopVehicleSafely(void)
{
    LineFollow_StopPreserveRoute();
    Motion_Stop();
    Chassis_EmergencyStop();
}

static int32_t Mission_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t Mission_GetEncoderDistanceMm(void)
{
    return (Drv_Encoder_GetLeftTotalMm() +
            Drv_Encoder_GetRightTotalMm()) / 2;
}

static int16_t Mission_MinSpeed(int16_t first, int16_t second)
{
    return (first < second) ? first : second;
}

static void Mission_ApplyDynamicSpeed(void)
{
    int16_t current_speed_cps;

    current_speed_cps = (int16_t)(s_dynamic_speed_cps + 0.5f);
    (void)LineFollow_SetSpeedProfile(
        current_speed_cps,
        Mission_MinSpeed(current_speed_cps,
                         MISSION_DYNAMIC_CURVE_SPEED_CPS),
        Mission_MinSpeed(current_speed_cps,
                         MISSION_DYNAMIC_MIN_TRACK_SPEED_CPS));
    s_mission.dynamic_speed_cps = current_speed_cps;
}

static void Mission_ResetDynamicSpeed(void)
{
    s_dynamic_speed_cps = (float)MISSION_DYNAMIC_START_SPEED_CPS;
    s_dynamic_speed_last_ms = BSP_GetTickMs();
    s_dynamic_start_distance_mm = Mission_GetEncoderDistanceMm();
    s_dynamic_brake_start_distance_mm = s_dynamic_start_distance_mm;
    s_dynamic_stop_requested = 0U;
    s_mission.dynamic_start_distance_mm = s_dynamic_start_distance_mm;
    s_mission.dynamic_brake_start_distance_mm =
        s_dynamic_brake_start_distance_mm;
    Mission_ApplyDynamicSpeed();
}

static void Mission_UpdateDynamicSpeed(int16_t target_speed_cps)
{
    uint32_t now_ms;
    uint32_t dt_ms;
    float step_cps;

    now_ms = BSP_GetTickMs();
    dt_ms = (uint32_t)(now_ms - s_dynamic_speed_last_ms);
    s_dynamic_speed_last_ms = now_ms;

    if (s_dynamic_speed_cps < (float)target_speed_cps) {
        step_cps = MISSION_DYNAMIC_ACCEL_RATE_CPS_PER_S *
                   ((float)dt_ms / 1000.0f);
        s_dynamic_speed_cps += step_cps;
        if (s_dynamic_speed_cps > (float)target_speed_cps) {
            s_dynamic_speed_cps = (float)target_speed_cps;
        }
    } else if (s_dynamic_speed_cps > (float)target_speed_cps) {
        step_cps = MISSION_DYNAMIC_DECEL_RATE_CPS_PER_S *
                   ((float)dt_ms / 1000.0f);
        s_dynamic_speed_cps -= step_cps;
        if (s_dynamic_speed_cps < (float)target_speed_cps) {
            s_dynamic_speed_cps = (float)target_speed_cps;
        }
    }

    Mission_ApplyDynamicSpeed();
}

static uint8_t Mission_DynamicSpeedAtMinimum(void)
{
    return (s_dynamic_speed_cps <=
            (float)MISSION_DYNAMIC_MIN_TRACK_SPEED_CPS) ? 1U : 0U;
}

static uint8_t Mission_IsVehicleStopped(void)
{
    return ((Mission_Abs32(Drv_Encoder_GetLeftSpeedCps()) <=
             MISSION_DYNAMIC_STOP_THRESHOLD_CPS) &&
            (Mission_Abs32(Drv_Encoder_GetRightSpeedCps()) <=
             MISSION_DYNAMIC_STOP_THRESHOLD_CPS)) ? 1U : 0U;
}

static void Mission_RecordBrakeStartDistance(void)
{
    s_dynamic_brake_start_distance_mm = Mission_GetEncoderDistanceMm();
    s_mission.dynamic_brake_start_distance_mm =
        s_dynamic_brake_start_distance_mm;
}

static void Mission_CloseScore(void)
{
    if (s_mission.score_closed != 0U) {
        return;
    }

    s_mission.score_elapsed_ms =
        (uint32_t)(BSP_GetTickMs() - s_mission.start_ms);
    s_mission.score_closed = 1U;
}

static void Mission_RequestDynamicStop(void)
{
    if (s_dynamic_stop_requested != 0U) {
        return;
    }

    LineFollow_StopPreserveRoute();
    Chassis_EmergencyStop();
    s_dynamic_stop_requested = 1U;
}

static void Mission_ClearMode2BrakeState(void)
{
    s_m2_finish_approach_applied = 0U;
    s_m2_brake_active = 0U;
    s_m2_brake_start_ms = 0U;
}

static void Mission_Mode2UpdateFinishApproach(void)
{
    HOvalRoute_Info_t route;

    if (s_m2_finish_approach_applied != 0U) {
        return;
    }
    if (HRoute_GetH2Info(&route) != PROJECT_OK) {
        return;
    }
    if (route.finish_armed == 0U) {
        return;
    }

    /* Only MODE2 changes to this lower finish-approach speed. */
    if (LineFollow_SetSpeedProfile(
            MISSION_M2_FINISH_APPROACH_SPEED_CPS,
            MISSION_M2_FINISH_APPROACH_MIN_SPEED_CPS,
            MISSION_M2_FINISH_APPROACH_MIN_SPEED_CPS) == BSP_OK) {
        s_m2_finish_approach_applied = 1U;
    }
}

static uint8_t Mission_Mode2StartActiveBrake(void)
{
    /*
     * Release line-follow control first, then let MODE2 temporarily use
     * the MOTION owner to apply a limited reverse speed command.
     */
    Mission_StopVehicleSafely();
    if (Chassis_AcquireControl(CHASSIS_OWNER_MOTION) != BSP_OK) {
        return 0U;
    }
    if (Chassis_SetSpeed(CHASSIS_OWNER_MOTION,
                         (int16_t)(-MISSION_M2_REVERSE_BRAKE_SPEED_CPS),
                         0) != BSP_OK) {
        Chassis_EmergencyStop();
        return 0U;
    }

    s_m2_brake_active = 1U;
    s_m2_brake_start_ms = BSP_GetTickMs();
    return 1U;
}

static int8_t Mission_Mode2UpdateActiveBrake(void)
{
    uint32_t elapsed_ms;
    int32_t left_speed_cps;
    int32_t right_speed_cps;
    int32_t average_speed_cps;

    if (s_m2_brake_active == 0U) {
        return -1;
    }

    elapsed_ms =
        (uint32_t)(BSP_GetTickMs() - s_m2_brake_start_ms);
    left_speed_cps = Drv_Encoder_GetLeftSpeedCps();
    right_speed_cps = Drv_Encoder_GetRightSpeedCps();
    average_speed_cps =
        (left_speed_cps + right_speed_cps) / 2;

    /*
     * Stop reverse torque before the vehicle begins moving backward.
     * The maximum time is a safety fallback when encoder data is noisy.
     */
    if (((Mission_Abs32(left_speed_cps) <=
          MISSION_M2_BRAKE_STOP_THRESHOLD_CPS) &&
         (Mission_Abs32(right_speed_cps) <=
          MISSION_M2_BRAKE_STOP_THRESHOLD_CPS)) ||
        (average_speed_cps <= 0) ||
        (elapsed_ms >= MISSION_M2_REVERSE_BRAKE_MAX_TIME_MS)) {
        Chassis_EmergencyStop();
        s_m2_brake_active = 0U;
        return 1;
    }

    /* Refresh the command before the chassis command watchdog expires. */
    if (Chassis_SetSpeed(CHASSIS_OWNER_MOTION,
                         (int16_t)(-MISSION_M2_REVERSE_BRAKE_SPEED_CPS),
                         0) != BSP_OK) {
        Chassis_EmergencyStop();
        s_m2_brake_active = 0U;
        return -1;
    }

    return 0;
}

static void Mission_SetSubstate(Mission_SubState_t substate)
{
    if (s_mission.substate == (uint8_t)substate) {
        return;
    }
    s_mission.substate = (uint8_t)substate;
    s_substate_enter_ms = BSP_GetTickMs();
}

static void Mission_ClearMode3PlusConfirm(void)
{
    s_m3_plus_confirm_count = 0U;
    s_m3_last_processed_sample_ms = 0U;
    s_m3_sample_tracking_valid = 0U;
    s_mission.m3_plus_confirm_count = 0U;
    s_mission.m3_last_processed_sample_ms = 0U;
}

static void Mission_EnterState(Mission_State_t state,
                               Mission_Result_t result,
                               uint16_t fault_code)
{
    uint32_t now_ms;

    now_ms = BSP_GetTickMs();
    if (s_mission.state != state) {
        s_state_enter_ms = now_ms;
    }
    s_mission.state = state;
    s_mission.result = result;
    s_mission.fault_code = fault_code;
    s_mission.running =
        (state == MISSION_STATE_RUNNING) ? 1U : 0U;
    s_mission.finished =
        ((state == MISSION_STATE_FINISH) ||
         (state == MISSION_STATE_ABORT) ||
         (state == MISSION_STATE_FAULT)) ? 1U : 0U;
}

static void Mission_UpdateLimits(void)
{
    switch (s_mission.mode) {
    case MISSION_MODE_2_LINE_LAP:
        s_mission.score_limit_ms = 20000U;
        s_mission.safety_timeout_ms = 30000U;
        break;
    case MISSION_MODE_3_BALL_TRANSFER:
        s_mission.score_limit_ms = 5000U;
        s_mission.safety_timeout_ms = 15000U;
        break;
    case MISSION_MODE_4_LINE_TO_B_WITH_BALL_O:
        s_mission.score_limit_ms = 8000U;
        s_mission.safety_timeout_ms = 15000U;
        break;
    case MISSION_MODE_5_LINE_LAP_WITH_BALL_O:
    case MISSION_MODE_6_LINE_LAP_WITH_CUSTOM_BALL:
    default:
        s_mission.score_limit_ms = 30000U;
        s_mission.safety_timeout_ms = 45000U;
        break;
    }
}

static void Mission_SetBallTarget(int16_t target_mm_x10,
                                  uint8_t enable)
{
    s_mission.active_ball_target_mm_x10 = target_mm_x10;
    BallBalance_App_SetTargetMmX10(target_mm_x10);
    BallBalance_App_SetVehicleFeedforwardEnabled(
        MISSION_BALL_VEHICLE_FEEDFORWARD_DEFAULT_ENABLE);
    s_mission.vehicle_feedforward_enabled =
        MISSION_BALL_VEHICLE_FEEDFORWARD_DEFAULT_ENABLE;
    if (enable != 0U) {
        BallBalance_App_Enable();
    } else {
        BallBalance_App_Disable();
    }
}

static void Mission_ResetRuntime(void)
{
    s_mission.armed_ready = 0U;
    s_mission.route_ready = 0U;
    s_mission.ball_ready = 0U;
    s_mission.running = 0U;
    s_mission.finished = 0U;
    s_mission.current_ball_position_mm_x10 = 0;
    s_mission.current_ball_error_mm_x10 = 0;
    s_mission.max_ball_error_mm_x10 = 0;
    s_mission.ball_filtered_velocity_mm_s_x10 = 0;
    s_mission.ball_settled = 0U;
    s_mission.start_ms = 0U;
    s_mission.elapsed_ms = 0U;
    s_mission.score_elapsed_ms = 0U;
    s_mission.score_closed = 0U;
    s_mission.route_events = 0U;
    s_mission.fault_code = MISSION_FAULT_NONE;
    s_dynamic_speed_cps = (float)MISSION_DYNAMIC_START_SPEED_CPS;
    s_dynamic_speed_last_ms = 0U;
    s_dynamic_start_distance_mm = 0;
    s_dynamic_brake_start_distance_mm = 0;
    s_dynamic_stop_requested = 0U;
    s_mission.dynamic_speed_cps = MISSION_DYNAMIC_START_SPEED_CPS;
    s_mission.dynamic_start_distance_mm = 0;
    s_mission.dynamic_brake_start_distance_mm = 0;
    Mission_ClearMode2BrakeState();
    Mission_ClearMode3PlusConfirm();
    Mission_UpdateLimits();
}

static void Mission_ResetRouteEvents(void)
{
    RouteManager_ClearEvents(0xFFFFFFFFUL);
    s_mission.route_events = 0U;
}

static void Mission_EnterFault(Mission_Result_t result,
                               Mission_FaultCode_t fault)
{
    Mission_StopVehicleSafely();
    Mission_ClearMode3PlusConfirm();
    if ((fault == MISSION_FAULT_BALL) ||
        (fault == MISSION_FAULT_K210_TIMEOUT) ||
        (fault == MISSION_FAULT_SERVO)) {
        BallBalance_App_Disable();
    }
    Mission_EnterState(MISSION_STATE_FAULT, result, (uint16_t)fault);
}

static void Mission_Finish(Mission_Result_t result)
{
    /*
     * 正常完成时不统一追加急停。
     * 任务2已完成主动制动，任务3全程静止，
     * 任务4/5/6也已在各自BRAKE子状态确认停车。
     * 各任务在进入FINISH前均已完成本任务要求的停车流程。
     */
    Mission_EnterState(MISSION_STATE_FINISH, result, MISSION_FAULT_NONE);

    if (s_mission.mode == MISSION_MODE_2_LINE_LAP) {
        Mission_SetBallTarget(MISSION_TARGET_O_MM_X10, 0U);
    } else if (s_mission.mode == MISSION_MODE_3_BALL_TRANSFER) {
        Mission_SetBallTarget(MISSION_TARGET_MINUS_50_MM_X10, 1U);
    } else if ((s_mission.mode == MISSION_MODE_4_LINE_TO_B_WITH_BALL_O) ||
               (s_mission.mode == MISSION_MODE_5_LINE_LAP_WITH_BALL_O)) {
        Mission_SetBallTarget(MISSION_TARGET_O_MM_X10, 1U);
    } else {
        Mission_SetBallTarget(s_mission.custom_ball_target_mm_x10, 1U);
    }
}

static uint8_t Mission_IsBallUsable(const BallBalance_AppInfo_t *ball)
{
    if (ball == 0) {
        return 0U;
    }
    if ((ball->enabled == 0U) ||
        (ball->servo_fault != 0U) ||
        (ball->state == BALL_BALANCE_APP_FAULT) ||
        (ball->data_timeout != 0U)) {
        return 0U;
    }
    return ((ball->state == BALL_BALANCE_APP_ACTIVE) ||
            (ball->tracking_ready != 0U)) ? 1U : 0U;
}

static uint8_t Mission_IsBallReady(const BallBalance_AppInfo_t *ball)
{
    if (Mission_IsBallUsable(ball) == 0U) {
        return 0U;
    }
    return (ball->settled != 0U) ? 1U : 0U;
}

static void Mission_UpdateBallSnapshot(void)
{
    BallBalance_AppInfo_t ball;
    int16_t error_x10;
    int16_t abs_error_x10;

    if (BallBalance_App_GetInfo(&ball) != BSP_OK) {
        s_mission.ball_ready = 0U;
        return;
    }

    s_mission.current_ball_position_mm_x10 =
        (int16_t)(ball.estimator.position_mm * 10.0f);
    error_x10 =
        (int16_t)(s_mission.current_ball_position_mm_x10 -
                  s_mission.active_ball_target_mm_x10);
    s_mission.current_ball_error_mm_x10 = error_x10;

    if ((s_mission.running != 0U) &&
        (s_mission.score_closed == 0U)) {
        abs_error_x10 = Mission_Abs16(error_x10);
        if (abs_error_x10 > s_mission.max_ball_error_mm_x10) {
            s_mission.max_ball_error_mm_x10 = abs_error_x10;
        }
    }

    s_mission.vehicle_feedforward_enabled =
        ball.vehicle_feedforward_enabled;
    s_mission.ball_filtered_velocity_mm_s_x10 =
        (int16_t)(ball.control.output.filtered_velocity_mm_s * 10.0f);
    s_mission.ball_settled = ball.settled;
    s_mission.ball_ready = Mission_IsBallReady(&ball);
}

static uint8_t Mission_CheckMode3PlusReach(void)
{
    BallBalance_AppInfo_t ball;
    uint8_t reached;

    if (BallBalance_App_GetInfo(&ball) != BSP_OK) {
        Mission_ClearMode3PlusConfirm();
        return 0U;
    }

    s_mission.current_ball_position_mm_x10 =
        (int16_t)(ball.estimator.position_mm * 10.0f);
    s_mission.ball_filtered_velocity_mm_s_x10 =
        (int16_t)(ball.control.output.filtered_velocity_mm_s * 10.0f);
    s_mission.ball_settled = ball.settled;

    if ((Mission_IsBallUsable(&ball) == 0U) ||
        (ball.last_sample_valid == 0U) ||
        (ball.last_valid_sample_ms == 0U)) {
        Mission_ClearMode3PlusConfirm();
        return 0U;
    }

    if ((s_m3_sample_tracking_valid != 0U) &&
        (ball.last_valid_sample_ms ==
         s_m3_last_processed_sample_ms)) {
        return 0U;
    }

    s_m3_sample_tracking_valid = 1U;
    s_m3_last_processed_sample_ms =
        ball.last_valid_sample_ms;
    s_mission.m3_last_processed_sample_ms =
        s_m3_last_processed_sample_ms;

    reached =
        ((ball.estimator.position_mm >=
          MISSION_M3_PLUS_REACH_POSITION_MM) &&
         (ball.control.output.filtered_velocity_mm_s >=
          MISSION_M3_PLUS_MIN_VELOCITY_MM_S)) ? 1U : 0U;

    if (reached != 0U) {
        if (s_m3_plus_confirm_count <
            MISSION_M3_PLUS_CONFIRM_FRAME_COUNT) {
            s_m3_plus_confirm_count++;
        }
    } else {
        s_m3_plus_confirm_count = 0U;
    }

    s_mission.m3_plus_confirm_count =
        s_m3_plus_confirm_count;

    return (s_m3_plus_confirm_count >=
            MISSION_M3_PLUS_CONFIRM_FRAME_COUNT) ? 1U : 0U;
}

static uint8_t Mission_IsRouteReady(void)
{
    if ((Drv_GraySensor_IsOnline() == 0U) ||
        (Chassis_GetFault() != CHASSIS_FAULT_NONE)) {
        return 0U;
    }
    return 1U;
}

static uint8_t Mission_StartLineFollow(void)
{
    BSP_Status_t status;

    if (LineFollow_SetSpeedProfile(H2_RUN_SPEED_CPS,
                                   H2_CURVE_SPEED_CPS,
                                   H2_CURVE_SPEED_CPS) != BSP_OK) {
        return 0U;
    }
    status = LineFollow_Start();
    return (status == BSP_OK) ? 1U : 0U;
}

static void Mission_StartRunning(Mission_SubState_t substate)
{
    uint32_t now_ms;

    now_ms = BSP_GetTickMs();
    Mission_ResetRouteEvents();
    s_mission.start_ms = now_ms;
    s_mission.elapsed_ms = 0U;
    s_mission.score_elapsed_ms = 0U;
    s_mission.score_closed = 0U;
    s_mission.max_ball_error_mm_x10 = 0;
    Mission_ClearMode3PlusConfirm();
    Mission_SetSubstate(substate);
    Mission_EnterState(MISSION_STATE_RUNNING,
                       MISSION_RESULT_NONE,
                       MISSION_FAULT_NONE);
}

static void Mission_CheckRunningFaults(uint8_t need_route,
                                       uint8_t need_ball)
{
    BallBalance_AppInfo_t ball;

    if (s_mission.elapsed_ms > s_mission.safety_timeout_ms) {
        Mission_EnterFault(MISSION_RESULT_TIMEOUT,
                           MISSION_FAULT_TIMEOUT);
        return;
    }
    if (need_route != 0U) {
        if ((s_mission.route_events &
             (ROUTE_EVENT_FAILED |
              ROUTE_EVENT_LINE_LOST |
              ROUTE_EVENT_ACTION_ERROR)) != 0U) {
            Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                               MISSION_FAULT_ROUTE);
            return;
        }
        if ((LineFollow_GetState() != LINE_FOLLOW_RUN) &&
            ((s_mission.substate == MISSION_SUB_M2_WAIT_LEAVE_A) ||
             (s_mission.substate == MISSION_SUB_M2_RUNNING_LAP) ||
             (s_mission.substate == MISSION_SUB_M4_LEAVE_A) ||
             (s_mission.substate == MISSION_SUB_M4_RUNNING_TO_B) ||
             (s_mission.substate == MISSION_SUB_M4_PASS_B_CONFIRM) ||
             (s_mission.substate == MISSION_SUB_M5_RUNNING_LAP) ||
             (s_mission.substate == MISSION_SUB_M6_RUNNING_LAP))) {
            Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                               MISSION_FAULT_ROUTE);
            return;
        }
        if (Chassis_GetFault() != CHASSIS_FAULT_NONE) {
            Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                               MISSION_FAULT_CHASSIS);
            return;
        }
    }
    if (need_ball != 0U) {
        if (BallBalance_App_GetInfo(&ball) != BSP_OK) {
            Mission_EnterFault(MISSION_RESULT_BALL_FAULT,
                               MISSION_FAULT_BALL);
        } else if (ball.servo_fault != 0U) {
            Mission_EnterFault(MISSION_RESULT_BALL_FAULT,
                               MISSION_FAULT_SERVO);
        } else if (ball.data_timeout != 0U) {
            Mission_EnterFault(MISSION_RESULT_BALL_FAULT,
                               MISSION_FAULT_K210_TIMEOUT);
        } else if (Mission_IsBallUsable(&ball) == 0U) {
            Mission_EnterFault(MISSION_RESULT_BALL_FAULT,
                               MISSION_FAULT_BALL);
        }
    }
}

static Mission_Result_t Mission_ResultByScore(void)
{
    if (s_mission.score_elapsed_ms > s_mission.score_limit_ms) {
        return MISSION_RESULT_TIME_FAIL;
    }
    if (((s_mission.mode == MISSION_MODE_4_LINE_TO_B_WITH_BALL_O) ||
         (s_mission.mode == MISSION_MODE_5_LINE_LAP_WITH_BALL_O) ||
         (s_mission.mode == MISSION_MODE_6_LINE_LAP_WITH_CUSTOM_BALL)) &&
        (s_mission.max_ball_error_mm_x10 >
         MISSION_BALL_ERROR_LIMIT_MM_X10)) {
        return MISSION_RESULT_BALL_ERROR_FAIL;
    }
    return MISSION_RESULT_PASS;
}

static void Mission_HandleIdleKeys(void)
{
#if BSP_KEY1_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY1) != 0U) {
        if (s_mission.mode <= MISSION_MODE_2_LINE_LAP) {
            s_mission.mode = MISSION_MODE_6_LINE_LAP_WITH_CUSTOM_BALL;
        } else {
            s_mission.mode = (Mission_Mode_t)((uint8_t)s_mission.mode - 1U);
        }
        Mission_UpdateLimits();
    }
#endif
#if BSP_KEY2_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY2) != 0U) {
        if (s_mission.mode >= MISSION_MODE_6_LINE_LAP_WITH_CUSTOM_BALL) {
            s_mission.mode = MISSION_MODE_2_LINE_LAP;
        } else {
            s_mission.mode = (Mission_Mode_t)((uint8_t)s_mission.mode + 1U);
        }
        Mission_UpdateLimits();
    }
#endif
#if BSP_KEY3_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY3) != 0U) {
        s_mission.custom_ball_target_mm_x10 =
            Mission_LimitTarget(
                (int16_t)(s_mission.custom_ball_target_mm_x10 -
                          MISSION_CUSTOM_STEP_MM_X10));
    }
#endif
#if BSP_KEY4_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY4) != 0U) {
        s_mission.custom_ball_target_mm_x10 =
            Mission_LimitTarget(
                (int16_t)(s_mission.custom_ball_target_mm_x10 +
                          MISSION_CUSTOM_STEP_MM_X10));
    }
#endif
#if BSP_KEY5_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY5) != 0U) {
        Mission_ResetRuntime();
        Mission_SetSubstate(MISSION_SUB_ARM_PREPARE);
        Mission_EnterState(MISSION_STATE_ARMED,
                           MISSION_RESULT_NONE,
                           MISSION_FAULT_NONE);
    }
#endif
}

static void Mission_HandleArmed(void)
{
    BallBalance_AppInfo_t ball;
    uint32_t now_ms;
    uint8_t key5_pressed;

    now_ms = BSP_GetTickMs();
    key5_pressed = 0U;
#if BSP_KEY5_ENABLE
    key5_pressed = BSP_Key_WasPressed(BSP_KEY5);
#endif

    Mission_StopVehicleSafely();
    s_mission.route_ready = Mission_IsRouteReady();

    if (s_mission.substate == MISSION_SUB_ARM_PREPARE) {
        Mission_ResetRouteEvents();
        if (s_mission.mode == MISSION_MODE_2_LINE_LAP) {
            Mission_SetBallTarget(MISSION_TARGET_O_MM_X10, 0U);
        } else if (s_mission.mode == MISSION_MODE_6_LINE_LAP_WITH_CUSTOM_BALL) {
            Mission_SetBallTarget(s_mission.custom_ball_target_mm_x10, 1U);
        } else {
            Mission_SetBallTarget(MISSION_TARGET_O_MM_X10, 1U);
        }
        Mission_SetSubstate(MISSION_SUB_ARM_WAIT_READY);
        return;
    }

    if (s_mission.mode == MISSION_MODE_2_LINE_LAP) {
        s_mission.ball_ready = 1U;
        s_mission.armed_ready = s_mission.route_ready;
    } else {
        if (BallBalance_App_GetInfo(&ball) == BSP_OK) {
            s_mission.ball_ready = Mission_IsBallReady(&ball);
        } else {
            s_mission.ball_ready = 0U;
        }
        if (s_mission.mode == MISSION_MODE_3_BALL_TRANSFER) {
            s_mission.armed_ready = s_mission.ball_ready;
        } else {
            s_mission.armed_ready =
                ((s_mission.ball_ready != 0U) &&
                 (s_mission.route_ready != 0U)) ? 1U : 0U;
        }
    }

    if ((s_mission.armed_ready == 0U) &&
        ((uint32_t)(now_ms - s_state_enter_ms) > MISSION_ARM_TIMEOUT_MS)) {
        s_mission.result = MISSION_RESULT_NOT_READY;
        s_mission.fault_code = MISSION_FAULT_NOT_READY;
    }

    if (key5_pressed == 0U) {
        return;
    }
    if (s_mission.armed_ready == 0U) {
        s_mission.result = MISSION_RESULT_NOT_READY;
        s_mission.fault_code = MISSION_FAULT_NOT_READY;
        return;
    }

    if (s_mission.mode == MISSION_MODE_2_LINE_LAP) {
        Mission_StartRunning(MISSION_SUB_M2_START);
    } else if (s_mission.mode == MISSION_MODE_3_BALL_TRANSFER) {
        Mission_StartRunning(MISSION_SUB_M3_SET_PLUS_50);
    } else if (s_mission.mode == MISSION_MODE_4_LINE_TO_B_WITH_BALL_O) {
        Mission_StartRunning(MISSION_SUB_M4_START);
    } else if (s_mission.mode == MISSION_MODE_5_LINE_LAP_WITH_BALL_O) {
        Mission_StartRunning(MISSION_SUB_M5_START);
    } else {
        Mission_StartRunning(MISSION_SUB_M6_START);
    }
}

static void Mission_HandleMode2(void)
{
    int8_t brake_result;

    brake_result = 0;
    switch ((Mission_SubState_t)s_mission.substate) {
    case MISSION_SUB_M2_START:
        Mission_ClearMode2BrakeState();
        Mission_SetBallTarget(MISSION_TARGET_O_MM_X10, 0U);
        if (Mission_StartLineFollow() != 0U) {
            Mission_SetSubstate(MISSION_SUB_M2_WAIT_LEAVE_A);
        } else {
            Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                               MISSION_FAULT_ROUTE);
        }
        break;

    case MISSION_SUB_M2_WAIT_LEAVE_A:
        if ((s_mission.route_events & ROUTE_EVENT_LEFT_A) != 0U) {
            Mission_SetSubstate(MISSION_SUB_M2_RUNNING_LAP);
        }
        break;

    case MISSION_SUB_M2_RUNNING_LAP:
        Mission_Mode2UpdateFinishApproach();
        if ((s_mission.route_events & ROUTE_EVENT_LAP_COMPLETE) != 0U) {
            if (Mission_Mode2StartActiveBrake() != 0U) {
                Mission_SetSubstate(MISSION_SUB_M2_BRAKE);
            } else {
                Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                                   MISSION_FAULT_CHASSIS);
            }
        }
        break;

    case MISSION_SUB_M2_BRAKE:
        brake_result = Mission_Mode2UpdateActiveBrake();
        if (brake_result > 0) {
            Mission_SetSubstate(MISSION_SUB_M2_DONE);
        } else if (brake_result < 0) {
            Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                               MISSION_FAULT_CHASSIS);
        }
        break;

    case MISSION_SUB_M2_DONE:
        Mission_Finish(Mission_ResultByScore());
        break;

    default:
        Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                           MISSION_FAULT_INTERNAL);
        break;
    }
}

static void Mission_HandleMode3(void)
{
    switch ((Mission_SubState_t)s_mission.substate) {
    case MISSION_SUB_M3_SET_PLUS_50:
        Mission_StopVehicleSafely();
        Mission_ClearMode3PlusConfirm();
        Mission_SetBallTarget(MISSION_TARGET_PLUS_50_MM_X10, 1U);
        Mission_SetSubstate(MISSION_SUB_M3_WAIT_PLUS_50);
        break;
    case MISSION_SUB_M3_WAIT_PLUS_50:
        if ((s_mission.active_ball_target_mm_x10 ==
             MISSION_TARGET_PLUS_50_MM_X10) &&
            (Mission_CheckMode3PlusReach() != 0U)) {
            Mission_SetSubstate(MISSION_SUB_M3_SET_MINUS_50);
        }
        break;
    case MISSION_SUB_M3_SET_MINUS_50:
        Mission_SetBallTarget(MISSION_TARGET_MINUS_50_MM_X10, 1U);
        Mission_SetSubstate(MISSION_SUB_M3_WAIT_MINUS_50);
        break;
    case MISSION_SUB_M3_WAIT_MINUS_50:
        if ((s_mission.active_ball_target_mm_x10 ==
             MISSION_TARGET_MINUS_50_MM_X10) &&
            (BallBalance_App_IsSettled() != 0U)) {
            Mission_SetSubstate(MISSION_SUB_M3_DONE);
        }
        break;
    case MISSION_SUB_M3_DONE:
        Mission_Finish(Mission_ResultByScore());
        break;
    default:
        Mission_EnterFault(MISSION_RESULT_BALL_FAULT,
                           MISSION_FAULT_INTERNAL);
        break;
    }
}

static void Mission_HandleMode4(void)
{
    int32_t current_distance_mm;

    switch ((Mission_SubState_t)s_mission.substate) {
    case MISSION_SUB_M4_START:
        Mission_SetBallTarget(MISSION_TARGET_O_MM_X10, 1U);
        Mission_ResetDynamicSpeed();
        if (LineFollow_Start() == BSP_OK) {
            Mission_SetSubstate(MISSION_SUB_M4_LEAVE_A);
        } else {
            Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                               MISSION_FAULT_ROUTE);
        }
        break;

    case MISSION_SUB_M4_LEAVE_A:
        Mission_UpdateDynamicSpeed(MISSION_DYNAMIC_MAX_SPEED_CPS);
        if ((s_mission.route_events & ROUTE_EVENT_LEFT_A) != 0U) {
            Mission_SetSubstate(MISSION_SUB_M4_RUNNING_TO_B);
        }
        break;

    case MISSION_SUB_M4_RUNNING_TO_B:
        Mission_UpdateDynamicSpeed(MISSION_DYNAMIC_MAX_SPEED_CPS);
        if ((s_mission.route_events & ROUTE_EVENT_PASSED_B) != 0U) {
            Mission_CloseScore();
            Mission_RecordBrakeStartDistance();
            Mission_SetSubstate(MISSION_SUB_M4_PASS_B_CONFIRM);
        }
        break;

    case MISSION_SUB_M4_PASS_B_CONFIRM:
        Mission_UpdateDynamicSpeed(MISSION_DYNAMIC_MIN_TRACK_SPEED_CPS);
        current_distance_mm = Mission_GetEncoderDistanceMm();
        if ((current_distance_mm - s_dynamic_brake_start_distance_mm) >=
            MISSION_M4_PASS_B_DISTANCE_MM) {
            Mission_SetSubstate(MISSION_SUB_M4_BRAKE);
        }
        break;

    case MISSION_SUB_M4_BRAKE:
        Mission_UpdateDynamicSpeed(MISSION_DYNAMIC_MIN_TRACK_SPEED_CPS);
        if (Mission_DynamicSpeedAtMinimum() != 0U) {
            Mission_RequestDynamicStop();
        }
        if ((s_dynamic_stop_requested != 0U) &&
            (Mission_IsVehicleStopped() != 0U)) {
            Mission_SetSubstate(MISSION_SUB_M4_DONE);
        }
        break;

    case MISSION_SUB_M4_DONE:
        Mission_Finish(Mission_ResultByScore());
        break;
    default:
        Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                           MISSION_FAULT_INTERNAL);
        break;
    }
}

static void Mission_HandleMode5(uint8_t custom)
{
    Mission_SubState_t substate;
    int16_t target;
    int32_t current_distance_mm;
    int32_t relative_distance_mm;

    substate = (Mission_SubState_t)s_mission.substate;
    target = (custom != 0U) ?
        s_mission.custom_ball_target_mm_x10 :
        MISSION_TARGET_O_MM_X10;

    if ((substate == MISSION_SUB_M5_START) ||
        (substate == MISSION_SUB_M6_START)) {
        Mission_SetBallTarget(target, 1U);
        Mission_ResetDynamicSpeed();
        if (LineFollow_Start() == BSP_OK) {
            Mission_SetSubstate((custom != 0U) ?
                MISSION_SUB_M6_RUNNING_LAP :
                MISSION_SUB_M5_RUNNING_LAP);
        } else {
            Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                               MISSION_FAULT_ROUTE);
        }
        return;
    }

    if ((substate == MISSION_SUB_M5_RUNNING_LAP) ||
        (substate == MISSION_SUB_M6_RUNNING_LAP)) {
        current_distance_mm = Mission_GetEncoderDistanceMm();
        relative_distance_mm =
            current_distance_mm - s_dynamic_start_distance_mm;
        if (relative_distance_mm >=
            MISSION_H56_DECEL_START_DISTANCE_MM) {
            Mission_UpdateDynamicSpeed(
                MISSION_DYNAMIC_MIN_TRACK_SPEED_CPS);
        } else {
            Mission_UpdateDynamicSpeed(MISSION_DYNAMIC_MAX_SPEED_CPS);
        }

        if ((s_mission.route_events & ROUTE_EVENT_LAP_COMPLETE) != 0U) {
            Mission_CloseScore();
            Mission_RecordBrakeStartDistance();
            Mission_SetSubstate((custom != 0U) ?
                MISSION_SUB_M6_BRAKE :
                MISSION_SUB_M5_BRAKE);
        }
        return;
    }

    if ((substate == MISSION_SUB_M5_BRAKE) ||
        (substate == MISSION_SUB_M6_BRAKE)) {
        Mission_UpdateDynamicSpeed(MISSION_DYNAMIC_MIN_TRACK_SPEED_CPS);
        current_distance_mm = Mission_GetEncoderDistanceMm();
        if (((current_distance_mm - s_dynamic_brake_start_distance_mm) >=
             MISSION_H56_PASS_A_DISTANCE_MM) &&
            (Mission_DynamicSpeedAtMinimum() != 0U)) {
            Mission_RequestDynamicStop();
        }
        if ((s_dynamic_stop_requested != 0U) &&
            (Mission_IsVehicleStopped() != 0U)) {
            Mission_SetSubstate((custom != 0U) ?
                MISSION_SUB_M6_DONE :
                MISSION_SUB_M5_DONE);
        }
        return;
    }

    if ((substate == MISSION_SUB_M5_DONE) ||
        (substate == MISSION_SUB_M6_DONE)) {
        Mission_Finish(Mission_ResultByScore());
        return;
    }

    Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                       MISSION_FAULT_INTERNAL);
}

static void Mission_HandleRunning(void)
{
    uint8_t need_route;
    uint8_t need_ball;

#if BSP_KEY5_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY5) != 0U) {
        Mission_StopVehicleSafely();
        Mission_ClearMode3PlusConfirm();
        Mission_EnterState(MISSION_STATE_ABORT,
                           MISSION_RESULT_USER_ABORT,
                           MISSION_FAULT_NONE);
        return;
    }
#endif

    if (s_mission.start_ms != 0U) {
        s_mission.elapsed_ms =
            (uint32_t)(BSP_GetTickMs() - s_mission.start_ms);
        if (s_mission.score_closed == 0U) {
            s_mission.score_elapsed_ms = s_mission.elapsed_ms;
        }
    }
    s_mission.route_events = RouteManager_GetEvents();
    need_route = (s_mission.mode == MISSION_MODE_3_BALL_TRANSFER) ?
        0U : 1U;
    need_ball = (s_mission.mode == MISSION_MODE_2_LINE_LAP) ?
        0U : 1U;
    Mission_CheckRunningFaults(need_route, need_ball);
    if (s_mission.state != MISSION_STATE_RUNNING) {
        return;
    }

    switch (s_mission.mode) {
    case MISSION_MODE_2_LINE_LAP:
        Mission_HandleMode2();
        break;
    case MISSION_MODE_3_BALL_TRANSFER:
        Mission_HandleMode3();
        break;
    case MISSION_MODE_4_LINE_TO_B_WITH_BALL_O:
        Mission_HandleMode4();
        break;
    case MISSION_MODE_5_LINE_LAP_WITH_BALL_O:
        Mission_HandleMode5(0U);
        break;
    case MISSION_MODE_6_LINE_LAP_WITH_CUSTOM_BALL:
        Mission_HandleMode5(1U);
        break;
    default:
        Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                           MISSION_FAULT_INTERNAL);
        break;
    }
}

static void Mission_HandleTerminalKeys(void)
{
#if BSP_KEY5_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY5) != 0U) {
        Mission_StopVehicleSafely();
        Mission_ResetRuntime();
        Mission_SetSubstate(MISSION_SUB_IDLE);
        Mission_EnterState(MISSION_STATE_IDLE,
                           MISSION_RESULT_NONE,
                           MISSION_FAULT_NONE);
    }
#endif
}

void TaskFSM_Init(void)
{
    s_mission.initialized = 1U;
    s_mission.mode = MISSION_MODE_2_LINE_LAP;
    s_mission.state = MISSION_STATE_BOOT;
    s_mission.substate = (uint8_t)MISSION_SUB_IDLE;
    s_mission.result = MISSION_RESULT_NONE;
    s_mission.custom_ball_target_mm_x10 = 0;
    s_mission.active_ball_target_mm_x10 = 0;
    s_state_enter_ms = BSP_GetTickMs();
    s_substate_enter_ms = s_state_enter_ms;
    s_last_key_ready = 0U;
    Mission_ResetRuntime();
}

void TaskFSM_Reset(void)
{
    Mission_StopVehicleSafely();
    BallBalance_App_Disable();
    TaskFSM_Init();
}

void TaskFSM_Update(void)
{
    if (s_mission.initialized == 0U) {
        TaskFSM_Init();
    }

    Mission_UpdateBallSnapshot();
    s_mission.route_events = RouteManager_GetEvents();

    switch (s_mission.state) {
    case MISSION_STATE_BOOT:
        Mission_StopVehicleSafely();
        BallBalance_App_SetVehicleFeedforwardEnabled(0U);
        BallBalance_App_Disable();
        Mission_ResetRouteEvents();
        Mission_SetSubstate(MISSION_SUB_IDLE);
        Mission_EnterState(MISSION_STATE_IDLE,
                           MISSION_RESULT_NONE,
                           MISSION_FAULT_NONE);
        break;
    case MISSION_STATE_IDLE:
        if (s_last_key_ready == 0U) {
            if ((BSP_Key_IsPressed(BSP_KEY1) == 0U) &&
                (BSP_Key_IsPressed(BSP_KEY2) == 0U) &&
                (BSP_Key_IsPressed(BSP_KEY3) == 0U) &&
                (BSP_Key_IsPressed(BSP_KEY4) == 0U) &&
                (BSP_Key_IsPressed(BSP_KEY5) == 0U)) {
                s_last_key_ready = 1U;
            }
        } else {
            Mission_HandleIdleKeys();
        }
        break;
    case MISSION_STATE_ARMED:
        Mission_HandleArmed();
        break;
    case MISSION_STATE_RUNNING:
        Mission_HandleRunning();
        break;
    case MISSION_STATE_FINISH:
    case MISSION_STATE_ABORT:
    case MISSION_STATE_FAULT:
        Mission_HandleTerminalKeys();
        break;
    default:
        Mission_EnterFault(MISSION_RESULT_ROUTE_FAULT,
                           MISSION_FAULT_INTERNAL);
        break;
    }
}

BSP_Status_t TaskFSM_GetInfo(Mission_Info_t *info)
{
    if (info == 0) {
        return BSP_PARAM;
    }
    *info = s_mission;
    return BSP_OK;
}
