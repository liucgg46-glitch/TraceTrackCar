#include "route_profile_b_basic.h"

static RouteProfile_Info_t s_route;
static BRoute_State_t s_state;
static uint8_t s_distance_ready;
static uint8_t s_finish_armed;
static uint16_t s_center_samples;
static uint16_t s_tip_lost_samples;
static uint16_t s_gap_reacquire_samples;
static uint16_t s_tip_reacquire_samples;
static uint16_t s_finish_single_samples;
static uint16_t s_finish_black_samples;
static int32_t s_start_distance_mm;
static int32_t s_gap_probe_start_distance_mm;
static int32_t s_tip_exit_distance_mm;
static uint32_t s_start_ms;
static uint32_t s_state_enter_ms;
static uint32_t s_last_center_ms;
static uint32_t s_now_ms;

static int32_t BRoute_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static uint16_t BRoute_IncrementU16(uint16_t value)
{
    return (value < 0xFFFFU) ? (uint16_t)(value + 1U) : value;
}

static void BRoute_ClearOutput(LineTrack_Output_t *out,
                               Route_ActionRequest_t *request)
{
    out->linear_cps = 0;
    out->turn_cps = 0;
    out->valid = 0U;

    request->type = ROUTE_ACTION_NONE;
    request->distance_mm = 0;
    request->angle_deg = 0;
    request->speed_cps = 0;
}

static void BRoute_SetLineOutput(LineTrack_Output_t *out,
                                 int16_t linear_cps,
                                 int16_t turn_cps)
{
    out->linear_cps = linear_cps;
    out->turn_cps = turn_cps;
    out->valid = 1U;
}

static void BRoute_EnterState(BRoute_State_t state)
{
    if (s_state == state) {
        return;
    }

    s_state = state;
    s_route.state = (uint8_t)state;
    s_route.event_confirm_samples = 0U;
    s_state_enter_ms = s_now_ms;
    s_route.transition_count++;

    if (state == B_ROUTE_STATE_ARRIVED) {
        s_route.arrived = 1U;
    } else if (state == B_ROUTE_STATE_ERROR) {
        s_route.error = 1U;
    }
}

/* 中间四路至少一路压线，同时最外侧四路均未压线。 */
static uint8_t BRoute_IsStableCenterLine(
    const LineDetect_Result_t *line)
{
    if (line == 0) {
        return 0U;
    }

    if ((line->type != LINE_TYPE_SINGLE) ||
        (line->black_count == 0U) ||
        (line->black_count > 4U)) {
        return 0U;
    }

    if ((line->black_mask & 0x3CU) == 0U) {
        return 0U;
    }

    if ((line->black_mask & 0xC3U) != 0U) {
        return 0U;
    }

    return 1U;
}

static uint8_t BRoute_HasTrackLine(
    const LineDetect_Result_t *line)
{
    if (line == 0) {
        return 0U;
    }

    return ((line->type != LINE_TYPE_LOST) &&
            (line->black_count != 0U) &&
            (line->black_mask != 0U)) ? 1U : 0U;
}

static uint8_t BRoute_IsFinishLine(
    const LineDetect_Result_t *line)
{
    if (line == 0) {
        return 0U;
    }

    return ((line->type == LINE_TYPE_FULL_BLACK) &&
            (line->black_count == LINE_DETECT_SENSOR_NUM) &&
            (line->black_mask == 0xFFU)) ? 1U : 0U;
}

static Route_ControlMode_t BRoute_FollowLine(
    const LineDetect_Result_t *line,
    LineTrack_Output_t *out)
{
    LineTrack_Compute(line, out, s_now_ms);

    if (out->valid == 0U) {
        BRoute_EnterState(B_ROUTE_STATE_ERROR);
        return ROUTE_CONTROL_ERROR;
    }

    return ROUTE_CONTROL_LINE_TRACK;
}

static uint8_t BRoute_TipGateReady(int32_t distance_mm)
{
    if (s_distance_ready == 0U) {
        return 0U;
    }

    if ((uint32_t)(s_now_ms - s_start_ms) < B_ROUTE_TIP_IGNORE_MS) {
        return 0U;
    }

    if (BRoute_Abs32(distance_mm - s_start_distance_mm) <
        B_ROUTE_TIP_MIN_TRAVEL_MM) {
        return 0U;
    }

    return 1U;
}

static void BRoute_UpdateCenterHistory(
    const LineDetect_Result_t *line)
{
    if (BRoute_IsStableCenterLine(line) != 0U) {
        s_center_samples = BRoute_IncrementU16(s_center_samples);
        if (s_center_samples >= B_ROUTE_TIP_CENTER_CONFIRM_SAMPLES) {
            s_last_center_ms = s_now_ms;
        }
    } else {
        s_center_samples = 0U;
    }
}

static Route_ControlMode_t BRoute_RunToTip(
    const LineDetect_Result_t *line,
    int32_t distance_mm,
    LineTrack_Output_t *out)
{
    BRoute_UpdateCenterHistory(line);

    if ((BRoute_TipGateReady(distance_mm) != 0U) &&
        (line->type == LINE_TYPE_LOST) &&
        (s_last_center_ms != 0U) &&
        ((uint32_t)(s_now_ms - s_last_center_ms) <=
         B_ROUTE_TIP_CENTER_TO_LOST_WINDOW_MS)) {
        /*
         * 第一、二帧丢线先保持低速直行，不让通用找线立即左右扫描。
         * 这样既能平稳跨越题图中的虚线，也为尖头判定建立入口。
         */
        s_tip_lost_samples =
            BRoute_IncrementU16(s_tip_lost_samples);
        s_route.event_confirm_samples = s_tip_lost_samples;
        BRoute_SetLineOutput(out, B_ROUTE_GAP_PROBE_CPS, 0);

        if (s_tip_lost_samples >=
            B_ROUTE_TIP_LOST_CONFIRM_SAMPLES) {
            s_gap_probe_start_distance_mm = distance_mm;
            s_gap_reacquire_samples = 0U;
            BRoute_EnterState(B_ROUTE_STATE_GAP_PROBE);
        }

        return ROUTE_CONTROL_LINE_TRACK;
    }

    s_tip_lost_samples = 0U;
    s_route.event_confirm_samples = 0U;
    return BRoute_FollowLine(line, out);
}

static Route_ControlMode_t BRoute_RunGapProbe(
    const LineDetect_Result_t *line,
    int32_t distance_mm,
    LineTrack_Output_t *out)
{
    uint32_t elapsed_ms;
    int32_t probe_distance_mm;

    elapsed_ms = (uint32_t)(s_now_ms - s_state_enter_ms);
    probe_distance_mm =
        BRoute_Abs32(distance_mm - s_gap_probe_start_distance_mm);

    BRoute_SetLineOutput(out, B_ROUTE_GAP_PROBE_CPS, 0);

    if (BRoute_HasTrackLine(line) != 0U) {
        s_gap_reacquire_samples =
            BRoute_IncrementU16(s_gap_reacquire_samples);
    } else {
        s_gap_reacquire_samples = 0U;
    }

    s_route.event_confirm_samples = s_gap_reacquire_samples;

    if (s_gap_reacquire_samples >=
        B_ROUTE_GAP_REACQUIRE_CONFIRM_SAMPLES) {
        /* 短时间内重新见线：这是虚线间隙，不是三角尖头。 */
        s_center_samples = 0U;
        s_tip_lost_samples = 0U;
        s_gap_reacquire_samples = 0U;
        s_last_center_ms = 0U;
        LineTrack_Reset();
        BRoute_EnterState(B_ROUTE_STATE_RUN_TO_TIP);
        return BRoute_FollowLine(line, out);
    }

    if ((elapsed_ms >= B_ROUTE_GAP_PROBE_MS) ||
        (probe_distance_mm >= B_ROUTE_GAP_PROBE_MAX_MM)) {
        /* 持续丢线：虚线跨越窗口已经结束，判定为向右回折尖头。 */
        LineTrack_Reset();
        s_tip_reacquire_samples = 0U;
        BRoute_EnterState(B_ROUTE_STATE_TIP_RIGHT_TURN);
        BRoute_SetLineOutput(out, 0,
                             (int16_t)(-B_ROUTE_TIP_TURN_CPS));
    }

    return ROUTE_CONTROL_LINE_TRACK;
}

static Route_ControlMode_t BRoute_RunTipTurn(
    const LineDetect_Result_t *line,
    int32_t distance_mm,
    LineTrack_Output_t *out)
{
    uint32_t elapsed_ms;

    elapsed_ms = (uint32_t)(s_now_ms - s_state_enter_ms);

    if (elapsed_ms >= B_ROUTE_TIP_TURN_TIMEOUT_MS) {
        BRoute_EnterState(B_ROUTE_STATE_ERROR);
        return ROUTE_CONTROL_ERROR;
    }

    BRoute_SetLineOutput(out, 0,
                         (int16_t)(-B_ROUTE_TIP_TURN_CPS));

    if ((elapsed_ms >= B_ROUTE_TIP_TURN_MIN_MS) &&
        (BRoute_IsStableCenterLine(line) != 0U)) {
        s_tip_reacquire_samples =
            BRoute_IncrementU16(s_tip_reacquire_samples);
    } else {
        s_tip_reacquire_samples = 0U;
    }

    s_route.event_confirm_samples = s_tip_reacquire_samples;

    if (s_tip_reacquire_samples >=
        B_ROUTE_TIP_REACQUIRE_CONFIRM_SAMPLES) {
        s_route.intersection_count = 1U;
        s_tip_exit_distance_mm = distance_mm;
        s_finish_armed = 0U;
        s_finish_single_samples = 0U;
        s_finish_black_samples = 0U;
        LineTrack_Reset();
        BRoute_EnterState(B_ROUTE_STATE_RUN_TO_FINISH);
        return BRoute_FollowLine(line, out);
    }

    return ROUTE_CONTROL_LINE_TRACK;
}

static void BRoute_UpdateFinishGate(
    const LineDetect_Result_t *line,
    int32_t distance_mm)
{
    if (s_finish_armed != 0U) {
        return;
    }

    if (BRoute_Abs32(distance_mm - s_tip_exit_distance_mm) <
        B_ROUTE_FINISH_MIN_TRAVEL_AFTER_TIP_MM) {
        s_finish_single_samples = 0U;
        return;
    }

    if (BRoute_IsStableCenterLine(line) != 0U) {
        s_finish_single_samples =
            BRoute_IncrementU16(s_finish_single_samples);
        if (s_finish_single_samples >=
            B_ROUTE_FINISH_SINGLE_CONFIRM_SAMPLES) {
            s_finish_armed = 1U;
            s_finish_single_samples = 0U;
        }
    } else {
        s_finish_single_samples = 0U;
    }
}

static Route_ControlMode_t BRoute_RunToFinish(
    const LineDetect_Result_t *line,
    int32_t distance_mm,
    LineTrack_Output_t *out)
{
    BRoute_UpdateFinishGate(line, distance_mm);

    if ((s_finish_armed != 0U) &&
        (BRoute_IsFinishLine(line) != 0U)) {
        /* 第一帧看到停止线就把速度目标置零，确认期间保持停车。 */
        s_finish_black_samples = 1U;
        BRoute_EnterState(B_ROUTE_STATE_FINISH_CONFIRM);
        s_route.event_confirm_samples = s_finish_black_samples;
        BRoute_SetLineOutput(out, 0, 0);
        return ROUTE_CONTROL_LINE_TRACK;
    }

    return BRoute_FollowLine(line, out);
}

static Route_ControlMode_t BRoute_ConfirmFinish(
    const LineDetect_Result_t *line,
    LineTrack_Output_t *out)
{
    BRoute_SetLineOutput(out, 0, 0);

    if (BRoute_IsFinishLine(line) == 0U) {
        /* 单帧全黑按干扰处理，复位PD后继续巡线。 */
        s_finish_black_samples = 0U;
        LineTrack_Reset();
        BRoute_EnterState(B_ROUTE_STATE_RUN_TO_FINISH);
        return BRoute_FollowLine(line, out);
    }

    s_finish_black_samples =
        BRoute_IncrementU16(s_finish_black_samples);
    s_route.event_confirm_samples = s_finish_black_samples;

    if (s_finish_black_samples >=
        B_ROUTE_FINISH_BLACK_CONFIRM_SAMPLES) {
        BRoute_EnterState(B_ROUTE_STATE_ARRIVED);
        return ROUTE_CONTROL_STOP;
    }

    return ROUTE_CONTROL_LINE_TRACK;
}

void BRoute_Init(uint32_t now_ms)
{
    BRoute_Reset(now_ms);
}

void BRoute_Reset(uint32_t now_ms)
{
    s_now_ms = now_ms;
    s_state = B_ROUTE_STATE_RUN_TO_TIP;
    s_distance_ready = 0U;
    s_finish_armed = 0U;
    s_center_samples = 0U;
    s_tip_lost_samples = 0U;
    s_gap_reacquire_samples = 0U;
    s_tip_reacquire_samples = 0U;
    s_finish_single_samples = 0U;
    s_finish_black_samples = 0U;
    s_start_distance_mm = 0;
    s_gap_probe_start_distance_mm = 0;
    s_tip_exit_distance_mm = 0;
    s_start_ms = now_ms;
    s_state_enter_ms = now_ms;
    s_last_center_ms = 0U;

    s_route.state = (uint8_t)s_state;
    s_route.configured = 1U;
    s_route.target_room = 0U;
    s_route.direction = (uint8_t)ROUTE_MISSION_OUTBOUND;
    s_route.room_approach_ready = 0U;
    s_route.visual_stage = 0U;
    s_route.visual_decision_ready = 0U;
    s_route.waiting_visual = 0U;
    s_route.intersection_count = 0U;
    s_route.decisions_completed = 0U;
    s_route.arrived = 0U;
    s_route.error = 0U;
    s_route.event_confirm_samples = 0U;
    s_route.running_ms = 0U;
    s_route.transition_count = 0U;
}

Project_Status_t BRoute_ConfigureMission(
    uint8_t target_room,
    Route_MissionDirection_t direction,
    uint32_t now_ms)
{
    (void)target_room;
    (void)direction;
    (void)now_ms;
    return PROJECT_ERROR;
}

Project_Status_t BRoute_SubmitVisualDecision(
    Route_VisualDirection_t direction)
{
    (void)direction;
    return PROJECT_ERROR;
}

Route_ControlMode_t BRoute_Update(
    const LineDetect_Result_t *line,
    const Route_ActionFeedback_t *feedback,
    LineTrack_Output_t *out,
    Route_ActionRequest_t *request,
    uint32_t now_ms)
{
    if ((line == 0) || (feedback == 0) ||
        (out == 0) || (request == 0)) {
        return ROUTE_CONTROL_ERROR;
    }

    s_now_ms = now_ms;
    s_route.running_ms = (uint32_t)(now_ms - s_start_ms);
    BRoute_ClearOutput(out, request);

    if (s_distance_ready == 0U) {
        s_start_distance_mm = feedback->distance_mm;
        s_gap_probe_start_distance_mm = feedback->distance_mm;
        s_tip_exit_distance_mm = feedback->distance_mm;
        s_distance_ready = 1U;
    }

    switch (s_state) {
    case B_ROUTE_STATE_RUN_TO_TIP:
        return BRoute_RunToTip(line, feedback->distance_mm, out);

    case B_ROUTE_STATE_GAP_PROBE:
        return BRoute_RunGapProbe(line, feedback->distance_mm, out);

    case B_ROUTE_STATE_TIP_RIGHT_TURN:
        return BRoute_RunTipTurn(line, feedback->distance_mm, out);

    case B_ROUTE_STATE_RUN_TO_FINISH:
        return BRoute_RunToFinish(line, feedback->distance_mm, out);

    case B_ROUTE_STATE_FINISH_CONFIRM:
        return BRoute_ConfirmFinish(line, out);

    case B_ROUTE_STATE_ARRIVED:
        return ROUTE_CONTROL_STOP;

    case B_ROUTE_STATE_ERROR:
    default:
        return ROUTE_CONTROL_ERROR;
    }
}

Project_Status_t BRoute_GetInfo(RouteProfile_Info_t *info)
{
    if (info == 0) {
        return PROJECT_PARAM;
    }

    *info = s_route;
    return PROJECT_OK;
}
