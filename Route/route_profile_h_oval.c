#include "route_profile_h_oval.h"

static HOvalRoute_Info_t s_route;
static uint32_t s_start_ms;
static uint32_t s_state_enter_ms;
static uint8_t s_leave_clear_samples;
static uint8_t s_finish_clear_seen;
static uint8_t s_finish_confirm_samples;
static uint8_t s_last_start_line_candidate;
static uint8_t s_start_distance_valid;
static int32_t s_start_encoder_distance_mm;

static int32_t HRoute_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
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
    return ((line->black_count >= H2_START_LINE_MIN_BLACK_COUNT) &&
            (black_span >= H2_START_LINE_MIN_SPAN)) ? 1U : 0U;
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
    s_route.b_armed = 0U;
    s_route.passed_b = 0U;
    s_route.b_curve_confirm_count = 0U;
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
    s_route.relative_distance_mm = 0;
    s_route.transition_count = 0U;

    s_start_ms = now_ms;
    s_state_enter_ms = now_ms;
    s_leave_clear_samples = 0U;
    s_finish_clear_seen = 0U;
    s_finish_confirm_samples = 0U;
    s_last_start_line_candidate = 0U;
    s_start_distance_valid = 0U;
    s_start_encoder_distance_mm = 0;
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
    uint8_t start_line_candidate;

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
    s_route.running_ms = (uint32_t)(now_ms - s_start_ms);
    s_route.state_elapsed_ms = (uint32_t)(now_ms - s_state_enter_ms);
    s_route.encoder_distance_mm = feedback->distance_mm;
    if (s_start_distance_valid == 0U) {
        s_start_encoder_distance_mm = feedback->distance_mm;
        s_start_distance_valid = 1U;
    }
    s_route.relative_distance_mm =
        feedback->distance_mm - s_start_encoder_distance_mm;
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

    start_line_candidate =
        HRoute_IsStartLineCandidate(line, s_route.black_span);

    /*
     * H2 只使用时间锁屏蔽起点横线，不再根据弯道阶段、丢线时长或
     * 总运行时间判故障。前10秒始终按普通循迹处理；离开A点且满10秒后，
     * 才允许后续一次横线由“无横线到有横线”的跳变触发停车确认。
     */
    if (start_line_candidate == 0U) {
        s_finish_clear_seen = 1U;
    }

    if ((s_route.left_a != 0U) &&
        (s_route.running_ms >= H2_MIN_FINISH_TIME_MS) &&
        (s_finish_clear_seen != 0U) &&
        (s_route.state != H_OVAL_ROUTE_FINISH_ARMED) &&
        (s_route.state != H_OVAL_ROUTE_FINISH_CONFIRM) &&
        (s_route.state != H_OVAL_ROUTE_FINISHED)) {
        s_route.finish_armed = 1U;
        HRoute_EnterState(H_OVAL_ROUTE_FINISH_ARMED, now_ms);
    }

    switch (s_route.state) {
    case H_OVAL_ROUTE_LEAVING_A:
        if (start_line_candidate != 0U) {
            s_leave_clear_samples = 0U;
        } else {
            if (s_leave_clear_samples < 0xFFU) {
                s_leave_clear_samples++;
            }
        }

        if ((s_route.running_ms >= H2_LEAVE_A_MIN_MS) &&
            (s_leave_clear_samples >=
             H2_LEAVE_CLEAR_CONFIRM_COUNT)) {
            s_route.left_a = 1U;
            HRoute_EnterState(H_OVAL_ROUTE_FIRST_STRAIGHT, now_ms);
        }
        break;

    case H_OVAL_ROUTE_FIRST_STRAIGHT:
    case H_OVAL_ROUTE_FIRST_CURVE:
    case H_OVAL_ROUTE_SECOND_STRAIGHT:
    case H_OVAL_ROUTE_SECOND_CURVE:
        /* 保留旧状态编号兼容现有日志，H2当前不再做弯道阶段判定。 */
        break;

    case H_OVAL_ROUTE_FINISH_ARMED:
        if ((start_line_candidate != 0U) &&
            (s_finish_clear_seen != 0U) &&
            (s_last_start_line_candidate == 0U)) {
            s_route.finish_candidate = 1U;
            s_finish_confirm_samples = 1U;
            s_route.arrived = 1U;
            HRoute_EnterState(H_OVAL_ROUTE_FINISHED, now_ms);
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

    if ((s_route.left_a != 0U) &&
        (s_route.relative_distance_mm >= H4_B_ARM_DISTANCE_MM)) {
        s_route.b_armed = 1U;
    }

    if ((s_route.b_armed != 0U) && (s_route.passed_b == 0U)) {
        if (HRoute_Abs32((int32_t)s_route.turn_output) >=
            H4_B_TURN_THRESHOLD_CPS) {
            if (s_route.b_curve_confirm_count <
                H4_B_CURVE_CONFIRM_COUNT) {
                s_route.b_curve_confirm_count++;
            }
        } else {
            s_route.b_curve_confirm_count = 0U;
        }

        if (s_route.b_curve_confirm_count >=
            H4_B_CURVE_CONFIRM_COUNT) {
            s_route.passed_b = 1U;
        }
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
