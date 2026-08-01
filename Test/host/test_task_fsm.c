#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ball_balance_app.h"
#include "bsp_key.h"
#include "chassis.h"
#include "drv_gray_sensor.h"
#include "line_follow_app.h"
#include "motion_action.h"
#include "route_manager.h"
#include "task_fsm.h"

static int s_failed;
static uint32_t s_now_ms;
static uint8_t s_key_pressed[BSP_KEY_COUNT];
static uint8_t s_key_was_pressed[BSP_KEY_COUNT];
static BallBalance_AppInfo_t s_ball;
static int16_t s_last_ball_target_mm_x10;
static uint8_t s_ball_enable_count;
static uint8_t s_ball_disable_count;
static uint8_t s_line_follow_start_count;
static LineFollow_State_t s_line_follow_state;
static uint32_t s_route_events;

static void CheckTrue(const char *name, int condition)
{
    if (!condition) {
        printf("  FAIL: %s\n", name);
        s_failed = 1;
    }
}

uint32_t GetTick(void)
{
    return s_now_ms;
}

uint32_t BSP_GetTickMs(void)
{
    return s_now_ms;
}

uint8_t BSP_TimeElapsed(uint32_t *last_time_ms, uint32_t period_ms)
{
    if ((uint32_t)(s_now_ms - *last_time_ms) >= period_ms) {
        *last_time_ms = s_now_ms;
        return 1U;
    }
    return 0U;
}

uint8_t BSP_IsTimeout(uint32_t start_time_ms, uint32_t timeout_ms)
{
    return ((uint32_t)(s_now_ms - start_time_ms) >= timeout_ms) ?
        1U :
        0U;
}

uint8_t BSP_Key_IsPressed(BSP_Key_Id_t id)
{
    return s_key_pressed[id];
}

uint8_t BSP_Key_WasPressed(BSP_Key_Id_t id)
{
    uint8_t pressed = s_key_was_pressed[id];

    s_key_was_pressed[id] = 0U;
    return pressed;
}

void Chassis_EmergencyStop(void)
{
}

Chassis_Fault_t Chassis_GetFault(void)
{
    return CHASSIS_FAULT_NONE;
}

uint8_t Drv_GraySensor_IsOnline(void)
{
    return 1U;
}

BSP_Status_t LineFollow_Start(void)
{
    s_line_follow_start_count++;
    s_line_follow_state = LINE_FOLLOW_RUN;
    return BSP_OK;
}

void LineFollow_StopPreserveRoute(void)
{
    s_line_follow_state = LINE_FOLLOW_STOP;
}

BSP_Status_t LineFollow_SetSpeedProfile(int16_t base_speed_cps,
                                        int16_t cross_speed_cps,
                                        int16_t min_track_speed_cps)
{
    (void)base_speed_cps;
    (void)cross_speed_cps;
    (void)min_track_speed_cps;
    return BSP_OK;
}

BSP_Status_t LineFollow_SetControlProfile(
    const LineTrack_ControlProfile_t *profile)
{
    return (profile != 0) ? BSP_OK : BSP_PARAM;
}

void LineFollow_ResetControlProfile(void)
{
}

LineFollow_State_t LineFollow_GetState(void)
{
    return s_line_follow_state;
}

void Motion_Stop(void)
{
}

uint32_t RouteManager_GetEvents(void)
{
    return s_route_events;
}

void RouteManager_ClearEvents(uint32_t events)
{
    s_route_events &= ~events;
}

void BallBalance_App_Init(void)
{
}

void BallBalance_App_Enable(void)
{
    s_ball.enabled = 1U;
    s_ball_enable_count++;
}

void BallBalance_App_Disable(void)
{
    s_ball.enabled = 0U;
    s_ball_disable_count++;
}

uint8_t BallBalance_App_IsEnabled(void)
{
    return s_ball.enabled;
}

void BallBalance_App_SetTargetMmX10(int16_t target_mm_x10)
{
    s_last_ball_target_mm_x10 = target_mm_x10;
    s_ball.target_mm_x10 = target_mm_x10;
    s_ball.settled = 0U;
}

void BallBalance_App_PushVisionSample(
    const BallBalance_VisionSample_t *sample
)
{
    if (sample != 0) {
        s_ball.last_sample = *sample;
    }
}

void BallBalance_App_SetVehicleDisturbanceMmS2(
    float disturbance_mm_s2,
    uint8_t valid,
    uint32_t timestamp_ms
)
{
    (void)disturbance_mm_s2;
    (void)valid;
    (void)timestamp_ms;
}

void BallBalance_App_SetVehicleFeedforwardEnabled(uint8_t enabled)
{
    s_ball.vehicle_feedforward_enabled = enabled;
}

uint8_t BallBalance_App_IsSettled(void)
{
    return s_ball.settled;
}

void BallBalance_App_Update(void)
{
}

BSP_Status_t BallBalance_App_GetInfo(BallBalance_AppInfo_t *info)
{
    if (info == 0) {
        return BSP_PARAM;
    }
    *info = s_ball;
    return BSP_OK;
}

static void ResetHarness(void)
{
    memset(s_key_pressed, 0, sizeof(s_key_pressed));
    memset(s_key_was_pressed, 0, sizeof(s_key_was_pressed));
    memset(&s_ball, 0, sizeof(s_ball));
    s_now_ms = 0U;
    s_last_ball_target_mm_x10 = 0;
    s_ball_enable_count = 0U;
    s_ball_disable_count = 0U;
    s_line_follow_start_count = 0U;
    s_line_follow_state = LINE_FOLLOW_STOP;
    s_route_events = 0U;

    s_ball.enabled = 1U;
    s_ball.state = BALL_BALANCE_APP_ACTIVE;
    s_ball.tracking_ready = 1U;
    s_ball.last_sample.valid = 1U;
    s_ball.last_sample.state = BALL_BALANCE_VISION_VALID;
    TaskFSM_Init();
}

static void StepMission(uint32_t step_ms)
{
    s_now_ms += step_ms;
    TaskFSM_Update();
}

static void PressKey(BSP_Key_Id_t key)
{
    s_key_pressed[key] = 1U;
    s_key_was_pressed[key] = 1U;
}

static void ReleaseKey(BSP_Key_Id_t key)
{
    s_key_pressed[key] = 0U;
}

static void SetBallSample(float position_mm,
                          float filtered_velocity_mm_s,
                          uint32_t timestamp_ms,
                          uint8_t settled)
{
    s_ball.estimator.position_mm = position_mm;
    s_ball.control.output.filtered_velocity_mm_s =
        filtered_velocity_mm_s;
    s_ball.last_sample.valid = 1U;
    s_ball.last_sample.state = BALL_BALANCE_VISION_VALID;
    s_ball.last_sample.timestamp_ms = timestamp_ms;
    s_ball.last_sample_valid = 1U;
    s_ball.last_valid_sample_ms = timestamp_ms;
    s_ball.settled = settled;
    s_ball.data_timeout = 0U;
    s_ball.servo_fault = 0U;
    s_ball.state = BALL_BALANCE_APP_ACTIVE;
    s_ball.tracking_ready = 1U;
}

static void StartMode3AndEnterWaitPlus(void)
{
    Mission_Info_t info;

    ResetHarness();
    StepMission(10U);
    StepMission(10U);

    PressKey(BSP_KEY2);
    StepMission(10U);
    ReleaseKey(BSP_KEY2);
    StepMission(10U);

    PressKey(BSP_KEY5);
    StepMission(10U);
    ReleaseKey(BSP_KEY5);

    StepMission(10U);
    SetBallSample(0.0f, 0.0f, 10U, 1U);

    PressKey(BSP_KEY5);
    StepMission(10U);
    ReleaseKey(BSP_KEY5);

    StepMission(10U);
    (void)TaskFSM_GetInfo(&info);
    CheckTrue("mode 3 reaches plus target wait state",
              (info.mode == MISSION_MODE_3_BALL_TRANSFER) &&
              (info.state == MISSION_STATE_RUNNING) &&
              (info.substate == MISSION_SUB_M3_WAIT_PLUS_50) &&
              (s_last_ball_target_mm_x10 == 500));
}

static void TestMode3PlusTurnaround(void)
{
    Mission_Info_t info;

    printf("[TEST] task fsm mode 3 plus turnaround\n");
    StartMode3AndEnterWaitPlus();

    SetBallSample(44.0f, 30.0f, 100U, 0U);
    StepMission(10U);
    (void)TaskFSM_GetInfo(&info);
    CheckTrue("44 mm frame does not confirm plus reach",
              info.m3_plus_confirm_count == 0U);

    SetBallSample(56.0f, 20.0f, 110U, 0U);
    StepMission(10U);
    (void)TaskFSM_GetInfo(&info);
    CheckTrue("jump over 50 mm starts plus confirmation",
              info.m3_plus_confirm_count == 1U);
    CheckTrue("plus phase does not require hold",
              info.ball_settled == 0U);

    StepMission(10U);
    (void)TaskFSM_GetInfo(&info);
    CheckTrue("same visual frame is counted only once",
              (info.m3_plus_confirm_count == 1U) &&
              (info.substate == MISSION_SUB_M3_WAIT_PLUS_50));

    SetBallSample(56.0f, 20.0f, 120U, 0U);
    StepMission(10U);
    (void)TaskFSM_GetInfo(&info);
    CheckTrue("two new confirming frames request minus substate",
              info.substate == MISSION_SUB_M3_SET_MINUS_50);

    StepMission(10U);
    (void)TaskFSM_GetInfo(&info);
    CheckTrue("plus confirmation switches target to minus 50 mm",
              (info.substate == MISSION_SUB_M3_WAIT_MINUS_50) &&
              (s_last_ball_target_mm_x10 == -500));

    printf(s_failed ? "  FAIL\n" : "  PASS\n");
}

static void TestMode3ConfirmResetAndFinalHold(void)
{
    Mission_Info_t info;

    printf("[TEST] task fsm mode 3 confirmation reset and final hold\n");
    StartMode3AndEnterWaitPlus();

    SetBallSample(56.0f, 10.0f, 200U, 0U);
    StepMission(10U);
    (void)TaskFSM_GetInfo(&info);
    CheckTrue("first confirming frame increments count",
              info.m3_plus_confirm_count == 1U);

    SetBallSample(44.0f, 10.0f, 210U, 0U);
    StepMission(10U);
    (void)TaskFSM_GetInfo(&info);
    CheckTrue("non confirming new frame clears count",
              info.m3_plus_confirm_count == 0U);

    SetBallSample(56.0f, 10.0f, 220U, 0U);
    StepMission(10U);
    SetBallSample(56.0f, 10.0f, 230U, 0U);
    StepMission(10U);
    StepMission(10U);
    (void)TaskFSM_GetInfo(&info);
    CheckTrue("mode 3 enters minus wait after plus confirmation",
              info.substate == MISSION_SUB_M3_WAIT_MINUS_50);

    SetBallSample(-50.0f, 0.0f, 240U, 0U);
    StepMission(10U);
    StepMission(10U);
    (void)TaskFSM_GetInfo(&info);
    CheckTrue("minus 50 phase does not finish before hold",
              (info.state == MISSION_STATE_RUNNING) &&
              (info.substate == MISSION_SUB_M3_WAIT_MINUS_50));

    SetBallSample(-50.0f, 0.0f, 250U, 1U);
    StepMission(10U);
    (void)TaskFSM_GetInfo(&info);
    CheckTrue("minus 50 hold advances to done substate",
              info.substate == MISSION_SUB_M3_DONE);

    StepMission(10U);
    (void)TaskFSM_GetInfo(&info);
    CheckTrue("minus 50 hold completes mode 3 task",
              (info.state == MISSION_STATE_FINISH) &&
              (info.result == MISSION_RESULT_PASS));

    printf(s_failed ? "  FAIL\n" : "  PASS\n");
}

static void TestMode2StillStartsWithoutBall(void)
{
    Mission_Info_t info;

    printf("[TEST] task fsm mode 2 behavior retained\n");
    ResetHarness();
    s_ball.enabled = 0U;
    s_ball.tracking_ready = 0U;
    s_ball.settled = 0U;

    StepMission(10U);
    StepMission(10U);
    PressKey(BSP_KEY5);
    StepMission(10U);
    ReleaseKey(BSP_KEY5);
    StepMission(10U);
    PressKey(BSP_KEY5);
    StepMission(10U);
    ReleaseKey(BSP_KEY5);
    StepMission(10U);

    (void)TaskFSM_GetInfo(&info);
    CheckTrue("mode 2 starts line follow without ball ready",
              (info.mode == MISSION_MODE_2_LINE_LAP) &&
              (info.state == MISSION_STATE_RUNNING) &&
              (info.substate == MISSION_SUB_M2_WAIT_LEAVE_A) &&
              (s_line_follow_start_count == 1U));

    printf(s_failed ? "  FAIL\n" : "  PASS\n");
}

int main(void)
{
    TestMode3PlusTurnaround();
    TestMode3ConfirmResetAndFinalHold();
    TestMode2StillStartsWithoutBall();
    return s_failed ? 1 : 0;
}
