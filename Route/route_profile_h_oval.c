#include "route_profile_h_oval.h"

static HOvalRoute_Info_t s_route;
static uint32_t s_start_ms;
static uint32_t s_state_enter_ms;
static uint32_t s_last_update_ms;
static uint32_t s_lost_start_ms;
static uint8_t s_lost_active;
static uint8_t s_leave_clear_samples;
static uint8_t s_leave_normal_samples;
static uint8_t s_finish_clear_seen;
static uint8_t s_finish_confirm_samples;
static uint8_t s_last_start_line_candidate;

static uint32_t HRoute_AbsI16(int16_t value)
{
    return (value >= 0) ? (uint32_t)value : (uint32_t)(-(int32_t)value);
}

static uint32_t HRoute_AddSaturated(uint32_t value,
                                    uint32_t increment,
                                    uint32_t limit)
{
    if (value >= limit) {
        return limit;
    }
    if (increment >= (limit - value)) {
        return limit;
    }
    return value + increment;
}

static uint32_t HRoute_Decay(uint32_t value, uint32_t decrement)
{
    return (value > decrement) ? (value - decrement) : 0U;
}

static uint8_t HRoute_GetBlackSpan(uint8_t mask)
{
    uint8_t first = 0U;
    uint8_t last = 0U;
    uint8_t found = 0U;
    uint8_t index;

    for (index = 0U; index < LINE_DETECT_SENSOR_NUM; index++) {
        if ((mask & (uint8_t)(1U << index)) != 0U) {
            if (found == 0U) {
                first = index;
                found = 1U;
            }
            last = index;
        }
    }

    return (found != 0U) ? (uint8_t)(last - first + 1U) : 0U;
}

static uint8_t HRoute_IsStartLineCandidate(
    const LineDetect_Result_t *line,
    uint8_t black_span)
{
    uint8_t center_black;

    center_black = ((line->black_mask & 0x18U) != 0U) ? 1U : 0U;
    return ((line->black_count >= H2_START_LINE_MIN_BLACK_COUNT) &&
            (black_span >= H2_START_LINE_MIN_SPAN) &&
            (center_black != 0U)) ? 1U : 0U;
}

static uint8_t HRoute_IsNormalLine(const LineDetect_Result_t *line,
                                   uint8_t start_line_candidate)
{
    return ((start_line_candidate == 0U) &&
            (line->type != LINE_TYPE_LOST) &&
            (line->black_count > 0U)) ? 1U : 0U;
}

static uint8_t HRoute_IsRightCurve(const LineDetect_Result_t *line,
                                   const LineTrack_Output_t *out)
{
    if (line->type == LINE_TYPE_LOST) {
        return 0U;
    }

    return ((out->turn_cps <=
             (int16_t)(-H2_CURVE_MIN_TURN_OUTPUT)) ||
            (line->error_x1000 >=
             H2_CURVE_MIN_ERROR_X1000)) ? 1U : 0U;
}

static uint8_t HRoute_IsStraight(const LineDetect_Result_t *line,
                                 const LineTrack_Output_t *out,
                                 uint8_t start_line_candidate)
{
    return ((HRoute_IsNormalLine(line, start_line_candidate) != 0U) &&
            (HRoute_AbsI16(out->turn_cps) <
             (uint32_t)H2_CURVE_MIN_TURN_OUTPUT) &&
            (HRoute_AbsI16(line->error_x1000) <
             (uint32_t)H2_CURVE_MIN_ERROR_X1000)) ? 1U : 0U;
}

static void HRoute_EnterState(HOvalRoute_State_t state, uint32_t now_ms)
{
    if (s_route.state == state) {
        return;
    }

    s_route.state = state;
    s_state_enter_ms = now_ms;
    s_route.state_elapsed_ms = 0U;
    s_route.curve_confirm_ms = 0U;
    s_route.straight_confirm_ms = 0U;
    s_route.transition_count++;
}

static Route_ControlMode_t HRoute_EnterFault(HOvalRoute_Fault_t fault,
                                              LineTrack_Output_t *out,
                                              uint32_t now_ms)
{
    s_route.fault = fault;
    HRoute_EnterState(H_OVAL_ROUTE_FAULT, now_ms);
    out->linear_cps = 0;
    out->turn_cps = 0;
    out->valid = 0U;
    return ROUTE_CONTROL_ERROR;
}

static void HRoute_ClearRequest(Route_ActionRequest_t *request)
{
    request->type = ROUTE_ACTION_NONE;
    request->distance_mm = 0;
    request->angle_deg = 0;
    request->speed_cps = 0;
}

static void HRoute_UpdateCurveEvidence(uint8_t is_curve,
                                       uint32_t delta_ms)
{
    if (is_curve != 0U) {
        s_route.curve_confirm_ms =
            HRoute_AddSaturated(s_route.curve_confirm_ms,
                                delta_ms,
                                H2_CURVE_CONFIRM_MS);
    } else {
        s_route.curve_confirm_ms =
            HRoute_Decay(s_route.curve_confirm_ms,
                         delta_ms * 2U);
    }
}

static void HRoute_UpdateStraightEvidence(uint8_t is_straight,
                                          uint32_t delta_ms)
{
    if (is_straight != 0U) {
        s_route.straight_confirm_ms =
            HRoute_AddSaturated(s_route.straight_confirm_ms,
                                delta_ms,
                                H2_STRAIGHT_CONFIRM_MS);
    } else {
        s_route.straight_confirm_ms =
            HRoute_Decay(s_route.straight_confirm_ms,
                         delta_ms * 2U);
    }
}

static uint8_t HRoute_IsPhaseTimedOut(void)
{
    switch (s_route.state) {
    case H_OVAL_ROUTE_FIRST_STRAIGHT:
    case H_OVAL_ROUTE_FIRST_CURVE:
    case H_OVAL_ROUTE_SECOND_STRAIGHT:
    case H_OVAL_ROUTE_SECOND_CURVE:
        return (s_route.state_elapsed_ms >=
                H2_ROUTE_PHASE_TIMEOUT_MS) ? 1U : 0U;
    default:
        return 0U;
    }
}

static void HRoute_ResetRuntime(uint32_t now_ms,
                                HOvalRoute_State_t state)
{
    s_route.state = state;
    s_route.fault = H_OVAL_ROUTE_FAULT_NONE;
    s_route.left_a = 0U;
    s_route.first_curve_seen = 0U;
    s_route.second_curve_seen = 0U;
    s_route.finish_armed = 0U;
    s_route.finish_candidate = 0U;
    s_route.arrived = 0U;
    s_route.gray_mask = 0U;
    s_route.black_count = 0U;
    s_route.black_span = 0U;
    s_route.line_error = 0;
    s_route.turn_output = 0;
    s_route.curve_confirm_ms = 0U;
    s_route.straight_confirm_ms = 0U;
    s_route.running_ms = 0U;
    s_route.state_elapsed_ms = 0U;
    s_route.encoder_distance_mm = 0;
    s_route.transition_count = 0U;

    s_start_ms = now_ms;
    s_state_enter_ms = now_ms;
    s_last_update_ms = now_ms;
    s_lost_start_ms = now_ms;
    s_lost_active = 0U;
    s_leave_clear_samples = 0U;
    s_leave_normal_samples = 0U;
    s_finish_clear_seen = 0U;
    s_finish_confirm_samples = 0U;
    s_last_start_line_candidate = 0U;
}

void HRoute_Init(uint32_t now_ms)
{
    HRoute_ResetRuntime(now_ms, H_OVAL_ROUTE_IDLE);
}

void HRoute_Reset(uint32_t now_ms)
{
    HRoute_ResetRuntime(now_ms, H_OVAL_ROUTE_LEAVING_A);
}

Project_Status_t HRoute_ConfigureMission(
    uint8_t target_room,
    Route_MissionDirection_t direction,
    uint32_t now_ms)
{
    (void)target_room;
    (void)direction;
    (void)now_ms;
    return PROJECT_ERROR;
}

Project_Status_t HRoute_SubmitVisualDecision(
    Route_VisualDirection_t direction)
{
    (void)direction;
    return PROJECT_ERROR;
}

Route_ControlMode_t HRoute_Update(
    const LineDetect_Result_t *line,
    const Route_ActionFeedback_t *feedback,
    LineTrack_Output_t *out,
    Route_ActionRequest_t *request,
    uint32_t now_ms)
{
    uint32_t delta_ms;
    uint8_t start_line_candidate;
    uint8_t normal_line;
    uint8_t right_curve;
    uint8_t straight;

    if ((line == 0) || (feedback == 0) ||
        (out == 0) || (request == 0)) {
        if (out != 0) {
            out->linear_cps = 0;
            out->turn_cps = 0;
            out->valid = 0U;
        }
        s_route.fault = H_OVAL_ROUTE_FAULT_PARAM;
        s_route.state = H_OVAL_ROUTE_FAULT;
        return ROUTE_CONTROL_ERROR;
    }

    HRoute_ClearRequest(request);
    delta_ms = (uint32_t)(now_ms - s_last_update_ms);
    if (delta_ms > 100U) {
        delta_ms = 100U;
    }
    s_last_update_ms = now_ms;
    s_route.running_ms = (uint32_t)(now_ms - s_start_ms);
    s_route.state_elapsed_ms = (uint32_t)(now_ms - s_state_enter_ms);
    s_route.encoder_distance_mm = feedback->distance_mm;
    s_route.gray_mask = line->black_mask;
    s_route.black_count = line->black_count;
    s_route.black_span = HRoute_GetBlackSpan(line->black_mask);
    s_route.line_error = line->error_x1000;

    LineTrack_Compute(line, out, now_ms);
    s_route.turn_output = out->turn_cps;

    if (s_route.state == H_OVAL_ROUTE_IDLE) {
        out->linear_cps = 0;
        out->turn_cps = 0;
        out->valid = 0U;
        return ROUTE_CONTROL_STOP;
    }
    if (s_route.state == H_OVAL_ROUTE_FAULT) {
        out->linear_cps = 0;
        out->turn_cps = 0;
        out->valid = 0U;
        return ROUTE_CONTROL_ERROR;
    }

    if ((s_route.arrived == 0U) &&
        (s_route.running_ms >= H2_ROUTE_MAX_RUN_TIME_MS)) {
        return HRoute_EnterFault(H_OVAL_ROUTE_FAULT_RUN_TIMEOUT,
                                 out,
                                 now_ms);
    }

    if (line->type == LINE_TYPE_LOST) {
        if (s_lost_active == 0U) {
            s_lost_active = 1U;
            s_lost_start_ms = now_ms;
        } else if ((uint32_t)(now_ms - s_lost_start_ms) >=
                   H2_ROUTE_LINE_LOST_TIMEOUT_MS) {
            return HRoute_EnterFault(H_OVAL_ROUTE_FAULT_LINE_LOST,
                                     out,
                                     now_ms);
        }
    } else {
        s_lost_active = 0U;
    }

    if (HRoute_IsPhaseTimedOut() != 0U) {
        return HRoute_EnterFault(H_OVAL_ROUTE_FAULT_PHASE_TIMEOUT,
                                 out,
                                 now_ms);
    }

    start_line_candidate =
        HRoute_IsStartLineCandidate(line, s_route.black_span);
    normal_line = HRoute_IsNormalLine(line, start_line_candidate);
    right_curve = HRoute_IsRightCurve(line, out);
    straight = HRoute_IsStraight(line, out, start_line_candidate);

    switch (s_route.state) {
    case H_OVAL_ROUTE_LEAVING_A:
        if (start_line_candidate != 0U) {
            s_leave_clear_samples = 0U;
            s_leave_normal_samples = 0U;
        } else {
            if (s_leave_clear_samples < 0xFFU) {
                s_leave_clear_samples++;
            }
            if (normal_line != 0U) {
                if (s_leave_normal_samples < 0xFFU) {
                    s_leave_normal_samples++;
                }
            } else {
                s_leave_normal_samples = 0U;
            }
        }

        if ((s_route.running_ms >= H2_LEAVE_A_MIN_MS) &&
            (s_leave_clear_samples >=
             H2_LEAVE_CLEAR_CONFIRM_COUNT) &&
            (s_leave_normal_samples >=
             H2_LEAVE_CLEAR_CONFIRM_COUNT)) {
            s_route.left_a = 1U;
            HRoute_EnterState(H_OVAL_ROUTE_FIRST_STRAIGHT, now_ms);
        }
        break;

    case H_OVAL_ROUTE_FIRST_STRAIGHT:
        HRoute_UpdateCurveEvidence(right_curve, delta_ms);
        if (s_route.curve_confirm_ms >= H2_CURVE_CONFIRM_MS) {
            s_route.first_curve_seen = 1U;
            HRoute_EnterState(H_OVAL_ROUTE_FIRST_CURVE, now_ms);
        }
        break;

    case H_OVAL_ROUTE_FIRST_CURVE:
        HRoute_UpdateStraightEvidence(straight, delta_ms);
        if (s_route.straight_confirm_ms >=
            H2_STRAIGHT_CONFIRM_MS) {
            HRoute_EnterState(H_OVAL_ROUTE_SECOND_STRAIGHT, now_ms);
        }
        break;

    case H_OVAL_ROUTE_SECOND_STRAIGHT:
        HRoute_UpdateCurveEvidence(right_curve, delta_ms);
        if (s_route.curve_confirm_ms >= H2_CURVE_CONFIRM_MS) {
            s_route.second_curve_seen = 1U;
            HRoute_EnterState(H_OVAL_ROUTE_SECOND_CURVE, now_ms);
        }
        break;

    case H_OVAL_ROUTE_SECOND_CURVE:
        if ((s_route.running_ms >= H2_MIN_FINISH_TIME_MS) &&
            (s_route.left_a != 0U) &&
            (s_route.first_curve_seen != 0U) &&
            (s_route.second_curve_seen != 0U)) {
            s_route.finish_armed = 1U;
            s_finish_clear_seen =
                (start_line_candidate == 0U) ? 1U : 0U;
            HRoute_EnterState(H_OVAL_ROUTE_FINISH_ARMED, now_ms);
        }
        break;

    case H_OVAL_ROUTE_FINISH_ARMED:
        if (start_line_candidate == 0U) {
            s_finish_clear_seen = 1U;
        } else if ((s_finish_clear_seen != 0U) &&
                   (s_last_start_line_candidate == 0U)) {
            s_route.finish_candidate = 1U;
            s_finish_confirm_samples = 1U;
            HRoute_EnterState(H_OVAL_ROUTE_FINISH_CONFIRM, now_ms);
        }
        break;

    case H_OVAL_ROUTE_FINISH_CONFIRM:
        if (start_line_candidate == 0U) {
            s_route.finish_candidate = 0U;
            s_finish_confirm_samples = 0U;
            HRoute_EnterState(H_OVAL_ROUTE_FINISH_ARMED, now_ms);
        } else {
            if (s_finish_confirm_samples < 0xFFU) {
                s_finish_confirm_samples++;
            }
            if (s_finish_confirm_samples >=
                H2_START_LINE_CONFIRM_COUNT) {
                s_route.finish_candidate = 1U;
                s_route.arrived = 1U;
                HRoute_EnterState(H_OVAL_ROUTE_FINISHED, now_ms);
            }
        }
        break;

    case H_OVAL_ROUTE_FINISHED:
        s_route.finish_candidate = 1U;
        break;

    case H_OVAL_ROUTE_IDLE:
    case H_OVAL_ROUTE_FAULT:
    default:
        return HRoute_EnterFault(H_OVAL_ROUTE_FAULT_PARAM,
                                 out,
                                 now_ms);
    }

    s_last_start_line_candidate = start_line_candidate;
    return (out->valid != 0U) ? ROUTE_CONTROL_LINE_TRACK
                              : ROUTE_CONTROL_ERROR;
}

Project_Status_t HRoute_GetInfo(RouteProfile_Info_t *info)
{
    if (info == 0) {
        return PROJECT_PARAM;
    }

    info->state = (uint8_t)s_route.state;
    info->configured = 1U;
    info->target_room = 0U;
    info->direction = (uint8_t)ROUTE_MISSION_OUTBOUND;
    info->room_approach_ready = 0U;
    info->visual_stage = 0U;
    info->visual_decision_ready = 0U;
    info->waiting_visual = 0U;
    info->intersection_count =
        (uint8_t)(s_route.first_curve_seen +
                  s_route.second_curve_seen);
    info->decisions_completed = 0U;
    info->arrived = s_route.arrived;
    info->error = (s_route.fault !=
                   H_OVAL_ROUTE_FAULT_NONE) ? 1U : 0U;
    info->event_confirm_samples =
        (uint16_t)s_finish_confirm_samples;
    info->running_ms = s_route.running_ms;
    info->transition_count = s_route.transition_count;
    return PROJECT_OK;
}

Project_Status_t HRoute_GetH2Info(HOvalRoute_Info_t *info)
{
    if (info == 0) {
        return PROJECT_PARAM;
    }

    *info = s_route;
    return PROJECT_OK;
}
