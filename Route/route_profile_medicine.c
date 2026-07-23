#include "route_profile_medicine.h"

#define MEDICINE_ROUTE_MAX_DECISIONS 2U

typedef enum {
    MEDICINE_VISUAL_WAIT_FAR_MAIN = 0,
    MEDICINE_VISUAL_WAIT_FAR_CORRIDOR
} MedicineRoute_VisualWait_t;

static RouteProfile_Info_t s_route;
static uint8_t s_configured;
static uint8_t s_target_room;
static Route_MissionDirection_t s_direction;
static Route_VisualDirection_t s_pending_visual;
static int16_t s_active_turn_angle;
static MedicineRoute_State_t s_turn_next_state;
static uint8_t s_action_requested;
static uint8_t s_intersection_armed;
static uint16_t s_single_samples;
static MedicineRoute_VisualWait_t s_visual_wait_context;
static int16_t s_outbound_turns[MEDICINE_ROUTE_MAX_DECISIONS];
static uint8_t s_outbound_turn_count;
static int8_t s_return_turn_index;
static uint8_t s_outbound_main_intersection;
static uint8_t s_return_main_intersections;
static int32_t s_end_gate_start_distance_mm;
static uint8_t s_end_gate_distance_ready;
static uint32_t s_start_ms;
static uint32_t s_state_enter_ms;
static uint32_t s_now_ms;

static int32_t MedicineRoute_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static void MedicineRoute_ClearOutput(LineTrack_Output_t *out,
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

static void MedicineRoute_ResetEventGate(void)
{
    s_intersection_armed = 0U;
    s_single_samples = 0U;
    s_route.event_confirm_samples = 0U;
    s_state_enter_ms = s_now_ms;
}

static void MedicineRoute_ResetEndGate(void)
{
    MedicineRoute_ResetEventGate();
    s_end_gate_start_distance_mm = 0;
    s_end_gate_distance_ready = 0U;
}

static void MedicineRoute_EnterState(MedicineRoute_State_t state)
{
    if (s_route.state == (uint8_t)state) {
        return;
    }

    s_route.state = (uint8_t)state;
    s_route.event_confirm_samples = 0U;
    s_state_enter_ms = s_now_ms;
    s_action_requested = 0U;
    s_route.transition_count++;
    s_route.waiting_visual =
        (state == MEDICINE_ROUTE_STATE_OUTBOUND_WAIT_VISUAL) ? 1U : 0U;

    if (state == MEDICINE_ROUTE_STATE_OUTBOUND_ROOM) {
        s_route.room_approach_ready = 1U;
        s_route.visual_stage = 0U;
        s_route.visual_decision_ready = 0U;
        MedicineRoute_ResetEndGate();
    }
    if ((state == MEDICINE_ROUTE_STATE_OUTBOUND_FAR_CORRIDOR) ||
        (state == MEDICINE_ROUTE_STATE_RETURN_ROOM) ||
        (state == MEDICINE_ROUTE_STATE_RETURN_CORRIDOR)) {
        MedicineRoute_ResetEventGate();
    }
    if (state == MEDICINE_ROUTE_STATE_OUTBOUND_FAR_CORRIDOR) {
        s_route.visual_stage = 3U;
        s_route.visual_decision_ready = 0U;
        s_pending_visual = ROUTE_VISUAL_NONE;
    }
    if (state == MEDICINE_ROUTE_STATE_RETURN_MAIN) {
        MedicineRoute_ResetEndGate();
    }
    if (state == MEDICINE_ROUTE_STATE_ARRIVED) {
        s_route.arrived = 1U;
    }
    if (state == MEDICINE_ROUTE_STATE_ERROR) {
        s_route.error = 1U;
    }
}

static uint8_t MedicineRoute_ConfirmEvent(uint8_t active)
{
    if (active == 0U) {
        s_route.event_confirm_samples = 0U;
        return 0U;
    }

    if (s_route.event_confirm_samples < 0xFFFFU) {
        s_route.event_confirm_samples++;
    }

    return (s_route.event_confirm_samples >=
            MEDICINE_ROUTE_EVENT_CONFIRM_SAMPLES) ? 1U : 0U;
}

/* 门口黑白虚线连续确认两帧后才停车，过滤单帧灰度抖动。 */
static uint8_t MedicineRoute_ConfirmEnd(uint8_t active)
{
    if (active == 0U) {
        s_route.event_confirm_samples = 0U;
        return 0U;
    }

    if (s_route.event_confirm_samples < 0xFFFFU) {
        s_route.event_confirm_samples++;
    }

    return (s_route.event_confirm_samples >=
            MEDICINE_ROUTE_END_CONFIRM_SAMPLES) ? 1U : 0U;
}

static uint8_t MedicineRoute_IsIntersection(
    const LineDetect_Result_t *line)
{
    if (line == 0) {
        return 0U;
    }

    if ((line->type == LINE_TYPE_LEFT_BRANCH) ||
        (line->type == LINE_TYPE_RIGHT_BRANCH) ||
        (line->type == LINE_TYPE_CROSS) ||
        (line->type == LINE_TYPE_FULL_BLACK) ||
        (line->black_count >= 5U)) {
        return 1U;
    }

    return 0U;
}

static uint8_t MedicineRoute_IsStableSingle(
    const LineDetect_Result_t *line)
{
    if (line == 0) {
        return 0U;
    }

    return ((line->type == LINE_TYPE_SINGLE) &&
            (line->black_count >= 1U) &&
            (line->black_count <= 4U)) ? 1U : 0U;
}

static uint8_t MedicineRoute_CountBlackSegments(uint8_t mask)
{
    uint8_t segments;
    uint8_t previous_black;
    uint8_t i;
    uint8_t current_black;

    segments = 0U;
    previous_black = 0U;
    for (i = 0U; i < LINE_DETECT_SENSOR_NUM; i++) {
        current_black = ((mask & (uint8_t)(1U << i)) != 0U) ? 1U : 0U;
        if ((current_black != 0U) && (previous_black == 0U)) {
            segments++;
        }
        previous_black = current_black;
    }

    return segments;
}

/* 赛题规定门口标线为约2 cm宽的黑白相间虚线。 */
static uint8_t MedicineRoute_IsDoorPattern(
    const LineDetect_Result_t *line)
{
    if (line == 0) {
        return 0U;
    }
    if ((line->type == LINE_TYPE_LOST) ||
        (line->type == LINE_TYPE_CROSS) ||
        (line->type == LINE_TYPE_FULL_BLACK)) {
        return 0U;
    }

    return ((line->black_count >= 2U) &&
            (MedicineRoute_CountBlackSegments(line->black_mask) >=
             MEDICINE_ROUTE_DOOR_MIN_SEGMENTS)) ? 1U : 0U;
}

/* 病房送达只认门口黑白虚线，普通丢线不再解释为到达。 */
static uint8_t MedicineRoute_IsRoomEnd(
    const LineDetect_Result_t *line)
{
    if (line == 0) {
        return 0U;
    }

    return MedicineRoute_IsDoorPattern(line);
}

/* 返回药房同样只认门口黑白虚线。 */
static uint8_t MedicineRoute_IsPharmacyEnd(
    const LineDetect_Result_t *line)
{
    if (line == 0) {
        return 0U;
    }

    return MedicineRoute_IsDoorPattern(line);
}

static uint8_t MedicineRoute_UpdateIntersectionGate(
    const LineDetect_Result_t *line)
{
    uint32_t elapsed_ms;

    elapsed_ms = (uint32_t)(s_now_ms - s_state_enter_ms);
    if (s_intersection_armed == 0U) {
        s_route.event_confirm_samples = 0U;
        if ((elapsed_ms < MEDICINE_ROUTE_EVENT_IGNORE_MS) ||
            (MedicineRoute_IsStableSingle(line) == 0U)) {
            s_single_samples = 0U;
            return 0U;
        }

        if (s_single_samples < 0xFFFFU) {
            s_single_samples++;
        }
        if (s_single_samples >= MEDICINE_ROUTE_SINGLE_CONFIRM_SAMPLES) {
            s_intersection_armed = 1U;
            s_single_samples = 0U;
        }
        return 0U;
    }

    return MedicineRoute_ConfirmEvent(MedicineRoute_IsIntersection(line));
}

/*
 * 先确认车辆已经稳定跟上终点支路，再启用门口虚线判定。
 * 转弯过程中的交叉线、短时丢线不会误报为病房或药房终点。
 */
static uint8_t MedicineRoute_UpdateEndGate(
    const LineDetect_Result_t *line,
    uint8_t end_active,
    int32_t distance_mm)
{
    /* 最后一次转弯附近仍可能压着十字路口，距离不足时禁止识别门标。 */
    if (s_end_gate_distance_ready == 0U) {
        s_end_gate_start_distance_mm = distance_mm;
        s_end_gate_distance_ready = 1U;
        s_intersection_armed = 0U;
        s_single_samples = 0U;
        s_route.event_confirm_samples = 0U;
        return 0U;
    }

    if (MedicineRoute_Abs32(
            distance_mm - s_end_gate_start_distance_mm) <
        MEDICINE_ROUTE_END_MIN_TRAVEL_MM) {
        s_intersection_armed = 0U;
        s_single_samples = 0U;
        s_route.event_confirm_samples = 0U;
        return 0U;
    }

    if (s_intersection_armed == 0U) {
        s_route.event_confirm_samples = 0U;
        if (MedicineRoute_IsStableSingle(line) == 0U) {
            s_single_samples = 0U;
            return 0U;
        }

        if (s_single_samples < 0xFFFFU) {
            s_single_samples++;
        }
        if (s_single_samples >= MEDICINE_ROUTE_SINGLE_CONFIRM_SAMPLES) {
            s_intersection_armed = 1U;
            s_single_samples = 0U;
        }
        return 0U;
    }

    return MedicineRoute_ConfirmEnd(end_active);
}

static int16_t MedicineRoute_VisualToAngle(
    Route_VisualDirection_t direction)
{
    return (direction == ROUTE_VISUAL_LEFT) ?
        MEDICINE_ROUTE_RIGHT_ANGLE_DEG :
        (int16_t)(-MEDICINE_ROUTE_RIGHT_ANGLE_DEG);
}

static Route_ControlMode_t MedicineRoute_FollowLine(
    const LineDetect_Result_t *line,
    LineTrack_Output_t *out)
{
    int32_t return_speed_cps;

    LineTrack_Compute(line, out, s_now_ms);
    /*
     * 返程只在正常单线循迹时提速；十字路口、分支和异常线型仍保留
     * LineTrack原有速度，避免提高路口转向和终点识别风险。
     */
    if ((out->valid != 0U) &&
        (s_direction == ROUTE_MISSION_RETURN) &&
        (line != 0) &&
        (line->type == LINE_TYPE_SINGLE) &&
        (out->linear_cps > 0) &&
        (out->linear_cps < MEDICINE_ROUTE_RETURN_MAX_SPEED_CPS)) {
        return_speed_cps =
            (int32_t)out->linear_cps +
            MEDICINE_ROUTE_RETURN_SPEED_BOOST_CPS;
        if (return_speed_cps > MEDICINE_ROUTE_RETURN_MAX_SPEED_CPS) {
            return_speed_cps = MEDICINE_ROUTE_RETURN_MAX_SPEED_CPS;
        }
        out->linear_cps = (int16_t)return_speed_cps;
    }
    if ((out->valid != 0U) &&
        (s_route.visual_stage != 0U) &&
        (s_route.visual_decision_ready == 0U) &&
        (out->linear_cps > MEDICINE_ROUTE_VISION_APPROACH_MAX_CPS)) {
        out->linear_cps = MEDICINE_ROUTE_VISION_APPROACH_MAX_CPS;
    }
    return (out->valid != 0U) ? ROUTE_CONTROL_LINE_TRACK
                              : ROUTE_CONTROL_STOP;
}

static Route_ControlMode_t MedicineRoute_HoldForVision(
    LineTrack_Output_t *out)
{
    out->linear_cps = 0;
    out->turn_cps = 0;
    out->valid = 1U;
    return ROUTE_CONTROL_LINE_TRACK;
}

static Route_ControlMode_t MedicineRoute_ContinueToFar(
    const LineDetect_Result_t *line,
    LineTrack_Output_t *out)
{
    MedicineRoute_EnterState(MEDICINE_ROUTE_STATE_OUTBOUND_MAIN);
    s_route.visual_stage = 2U;
    s_route.visual_decision_ready = 0U;
    s_pending_visual = ROUTE_VISUAL_NONE;
    MedicineRoute_ResetEventGate();
    return MedicineRoute_FollowLine(line, out);
}

static void MedicineRoute_PrepareTurn(
    int16_t angle_deg,
    MedicineRoute_State_t turn_state,
    MedicineRoute_State_t next_state)
{
    s_active_turn_angle = angle_deg;
    s_turn_next_state = next_state;
    MedicineRoute_EnterState(turn_state);
}

static Route_ControlMode_t MedicineRoute_RunTurn(
    const Route_ActionFeedback_t *feedback,
    Route_ActionRequest_t *request,
    const LineDetect_Result_t *line,
    LineTrack_Output_t *out)
{
    if (feedback->state == ROUTE_ACTION_STATE_ERROR) {
        MedicineRoute_EnterState(MEDICINE_ROUTE_STATE_ERROR);
        return ROUTE_CONTROL_ERROR;
    }

    if (s_action_requested == 0U) {
        if (feedback->state != ROUTE_ACTION_STATE_IDLE) {
            MedicineRoute_EnterState(MEDICINE_ROUTE_STATE_ERROR);
            return ROUTE_CONTROL_ERROR;
        }

        request->type = ROUTE_ACTION_TURN_ANGLE;
        request->angle_deg = s_active_turn_angle;
        s_action_requested = 1U;
        return ROUTE_CONTROL_MOTION;
    }

    if (feedback->state == ROUTE_ACTION_STATE_RUNNING) {
        return ROUTE_CONTROL_MOTION;
    }

    if (feedback->state == ROUTE_ACTION_STATE_DONE) {
        MedicineRoute_EnterState(s_turn_next_state);
        return MedicineRoute_FollowLine(line, out);
    }

    MedicineRoute_EnterState(MEDICINE_ROUTE_STATE_ERROR);
    return ROUTE_CONTROL_ERROR;
}

static Project_Status_t MedicineRoute_StoreOutboundDecision(
    Route_VisualDirection_t direction)
{
    if ((direction != ROUTE_VISUAL_LEFT) &&
        (direction != ROUTE_VISUAL_RIGHT)) {
        return PROJECT_PARAM;
    }
    if (s_outbound_turn_count >= MEDICINE_ROUTE_MAX_DECISIONS) {
        return PROJECT_ERROR;
    }

    /* 第一项转向位于主路，保存路口层级供返程终点解锁使用。 */
    if (s_outbound_turn_count == 0U) {
        s_outbound_main_intersection = s_route.intersection_count;
    }

    s_outbound_turns[s_outbound_turn_count] =
        MedicineRoute_VisualToAngle(direction);
    s_outbound_turn_count++;
    s_route.decisions_completed = s_outbound_turn_count;
    s_pending_visual = ROUTE_VISUAL_NONE;
    s_route.visual_decision_ready = 0U;
    s_route.visual_stage = 0U;
    return PROJECT_OK;
}

static Route_ControlMode_t MedicineRoute_HandleOutboundMain(
    const LineDetect_Result_t *line,
    const Route_ActionFeedback_t *feedback,
    LineTrack_Output_t *out,
    Route_ActionRequest_t *request)
{
    Route_VisualDirection_t direction;

    if (MedicineRoute_UpdateIntersectionGate(line) == 0U) {
        return MedicineRoute_FollowLine(line, out);
    }

    if (s_route.intersection_count < 255U) {
        s_route.intersection_count++;
    }
    /* 1、2号病房位置固定，在第一个十字路口直接选路。 */
    if ((s_target_room <= 2U) &&
        (s_route.intersection_count == 1U)) {
        direction = (s_target_room == 1U) ?
            ROUTE_VISUAL_LEFT : ROUTE_VISUAL_RIGHT;
        if (MedicineRoute_StoreOutboundDecision(direction) != PROJECT_OK) {
            MedicineRoute_EnterState(MEDICINE_ROUTE_STATE_ERROR);
            return ROUTE_CONTROL_ERROR;
        }
        MedicineRoute_PrepareTurn(
            s_outbound_turns[0],
            MEDICINE_ROUTE_STATE_OUTBOUND_TURN,
            MEDICINE_ROUTE_STATE_OUTBOUND_ROOM);
        return MedicineRoute_RunTurn(feedback, request, line, out);
    }

    /* 3～8号数字位置随机：越过1、2号路口后先搜索中部两个位置。 */
    if ((s_target_room >= 3U) &&
        (s_route.intersection_count == 1U)) {
        s_route.visual_stage = 1U;
        MedicineRoute_ResetEventGate();
        return MedicineRoute_FollowLine(line, out);
    }

    /* 第二个十字路口对应随机中部病房；路口前没有目标决策就继续搜索远端。 */
    if ((s_target_room >= 3U) &&
        (s_route.intersection_count == 2U)) {
        if (s_pending_visual == ROUTE_VISUAL_STRAIGHT) {
            return MedicineRoute_ContinueToFar(line, out);
        }
        if (s_pending_visual == ROUTE_VISUAL_NONE) {
            return MedicineRoute_ContinueToFar(line, out);
        }

        if (MedicineRoute_StoreOutboundDecision(s_pending_visual) != PROJECT_OK) {
            MedicineRoute_EnterState(MEDICINE_ROUTE_STATE_ERROR);
            return ROUTE_CONTROL_ERROR;
        }
        MedicineRoute_PrepareTurn(
            s_outbound_turns[0],
            MEDICINE_ROUTE_STATE_OUTBOUND_TURN,
            MEDICINE_ROUTE_STATE_OUTBOUND_ROOM);
        return MedicineRoute_RunTurn(feedback, request, line, out);
    }

    /* 中部未发现目标后，第三个十字路口按远端目标所在左右区域选路。 */
    if ((s_target_room >= 3U) &&
        (s_route.intersection_count == 3U)) {
        if (s_pending_visual == ROUTE_VISUAL_NONE) {
            s_visual_wait_context = MEDICINE_VISUAL_WAIT_FAR_MAIN;
            MedicineRoute_EnterState(
                MEDICINE_ROUTE_STATE_OUTBOUND_WAIT_VISUAL);
            return MedicineRoute_HoldForVision(out);
        }

        if (MedicineRoute_StoreOutboundDecision(s_pending_visual) != PROJECT_OK) {
            MedicineRoute_EnterState(MEDICINE_ROUTE_STATE_ERROR);
            return ROUTE_CONTROL_ERROR;
        }
        MedicineRoute_PrepareTurn(
            s_outbound_turns[0],
            MEDICINE_ROUTE_STATE_OUTBOUND_TURN,
            MEDICINE_ROUTE_STATE_OUTBOUND_FAR_CORRIDOR);
        return MedicineRoute_RunTurn(feedback, request, line, out);
    }

    MedicineRoute_EnterState(MEDICINE_ROUTE_STATE_ERROR);
    return ROUTE_CONTROL_ERROR;
}

static Route_ControlMode_t MedicineRoute_HandleFarCorridor(
    const LineDetect_Result_t *line,
    const Route_ActionFeedback_t *feedback,
    LineTrack_Output_t *out,
    Route_ActionRequest_t *request)
{
    if (MedicineRoute_UpdateIntersectionGate(line) == 0U) {
        return MedicineRoute_FollowLine(line, out);
    }

    if (s_pending_visual == ROUTE_VISUAL_NONE) {
        s_visual_wait_context = MEDICINE_VISUAL_WAIT_FAR_CORRIDOR;
        MedicineRoute_EnterState(
            MEDICINE_ROUTE_STATE_OUTBOUND_WAIT_VISUAL);
        return MedicineRoute_HoldForVision(out);
    }

    if (MedicineRoute_StoreOutboundDecision(s_pending_visual) != PROJECT_OK) {
        MedicineRoute_EnterState(MEDICINE_ROUTE_STATE_ERROR);
        return ROUTE_CONTROL_ERROR;
    }

    MedicineRoute_PrepareTurn(
        s_outbound_turns[1],
        MEDICINE_ROUTE_STATE_OUTBOUND_TURN,
        MEDICINE_ROUTE_STATE_OUTBOUND_ROOM);
    return MedicineRoute_RunTurn(feedback, request, line, out);
}

static Route_ControlMode_t MedicineRoute_HandleVisualWait(
    const LineDetect_Result_t *line,
    const Route_ActionFeedback_t *feedback,
    LineTrack_Output_t *out,
    Route_ActionRequest_t *request)
{
    MedicineRoute_State_t next_state;
    uint8_t decision_index;

    if (s_pending_visual == ROUTE_VISUAL_NONE) {
        return MedicineRoute_HoldForVision(out);
    }

    if (s_pending_visual == ROUTE_VISUAL_STRAIGHT) {
        MedicineRoute_EnterState(MEDICINE_ROUTE_STATE_ERROR);
        return ROUTE_CONTROL_ERROR;
    }

    if (MedicineRoute_StoreOutboundDecision(s_pending_visual) != PROJECT_OK) {
        MedicineRoute_EnterState(MEDICINE_ROUTE_STATE_ERROR);
        return ROUTE_CONTROL_ERROR;
    }

    decision_index = (uint8_t)(s_outbound_turn_count - 1U);
    if (s_visual_wait_context == MEDICINE_VISUAL_WAIT_FAR_CORRIDOR) {
        next_state = MEDICINE_ROUTE_STATE_OUTBOUND_ROOM;
    } else {
        next_state = MEDICINE_ROUTE_STATE_OUTBOUND_FAR_CORRIDOR;
    }

    MedicineRoute_PrepareTurn(
        s_outbound_turns[decision_index],
        MEDICINE_ROUTE_STATE_OUTBOUND_TURN,
        next_state);
    return MedicineRoute_RunTurn(feedback, request, line, out);
}

static Route_ControlMode_t MedicineRoute_HandleReturnIntersection(
    const LineDetect_Result_t *line,
    const Route_ActionFeedback_t *feedback,
    LineTrack_Output_t *out,
    Route_ActionRequest_t *request)
{
    int16_t angle_deg;
    MedicineRoute_State_t next_state;

    if (MedicineRoute_UpdateIntersectionGate(line) == 0U) {
        return MedicineRoute_FollowLine(line, out);
    }

    if ((s_return_turn_index < 0) ||
        ((uint8_t)s_return_turn_index >= s_outbound_turn_count)) {
        MedicineRoute_EnterState(MEDICINE_ROUTE_STATE_ERROR);
        return ROUTE_CONTROL_ERROR;
    }

    angle_deg = (int16_t)(-s_outbound_turns[s_return_turn_index]);
    next_state = (s_return_turn_index == 0) ?
        MEDICINE_ROUTE_STATE_RETURN_MAIN :
        MEDICINE_ROUTE_STATE_RETURN_CORRIDOR;
    s_return_turn_index--;

    MedicineRoute_PrepareTurn(angle_deg,
                              MEDICINE_ROUTE_STATE_RETURN_TURN,
                              next_state);
    return MedicineRoute_RunTurn(feedback, request, line, out);
}

static Route_ControlMode_t MedicineRoute_HandleReturnMain(
    const LineDetect_Result_t *line,
    const Route_ActionFeedback_t *feedback,
    LineTrack_Output_t *out)
{
    uint8_t required_intersections;

    if (s_outbound_main_intersection == 0U) {
        MedicineRoute_EnterState(MEDICINE_ROUTE_STATE_ERROR);
        return ROUTE_CONTROL_ERROR;
    }

    /*
     * 去程在主路第1、2、3个路口转向时，返程回到主路后分别还需经过
     * 0、1、2个路口。必经路口未走完前，不允许灰度门标触发停车。
     */
    required_intersections =
        (uint8_t)(s_outbound_main_intersection - 1U);
    if (s_return_main_intersections < required_intersections) {
        if (MedicineRoute_UpdateIntersectionGate(line) != 0U) {
            s_return_main_intersections++;
            MedicineRoute_ResetEventGate();

            /* 最后一个必经路口通过后，从该处重新计算终点保护距离。 */
            if (s_return_main_intersections >= required_intersections) {
                MedicineRoute_ResetEndGate();
            }
        }
        return MedicineRoute_FollowLine(line, out);
    }

    if (MedicineRoute_UpdateEndGate(
            line,
            MedicineRoute_IsPharmacyEnd(line),
            feedback->distance_mm) != 0U) {
        MedicineRoute_EnterState(MEDICINE_ROUTE_STATE_ARRIVED);
        return ROUTE_CONTROL_STOP;
    }
    return MedicineRoute_FollowLine(line, out);
}

void MedicineRoute_Init(uint32_t now_ms)
{
    s_now_ms = now_ms;
    s_configured = 0U;
    s_target_room = 0U;
    s_direction = ROUTE_MISSION_OUTBOUND;
    s_outbound_turns[0] = 0;
    s_outbound_turns[1] = 0;
    s_outbound_turn_count = 0U;
    s_outbound_main_intersection = 0U;
    MedicineRoute_Reset(now_ms);
}

void MedicineRoute_Reset(uint32_t now_ms)
{
    s_now_ms = now_ms;
    s_route.state = (uint8_t)MEDICINE_ROUTE_STATE_IDLE;
    s_route.configured = s_configured;
    s_route.target_room = s_target_room;
    s_route.direction = (uint8_t)s_direction;
    s_route.room_approach_ready = 0U;
    s_route.visual_stage = 0U;
    s_route.visual_decision_ready = 0U;
    s_route.waiting_visual = 0U;
    s_route.intersection_count = 0U;
    s_route.decisions_completed = s_outbound_turn_count;
    s_route.arrived = 0U;
    s_route.error = 0U;
    s_route.event_confirm_samples = 0U;
    s_route.running_ms = 0U;
    s_route.transition_count = 0U;
    s_pending_visual = ROUTE_VISUAL_NONE;
    s_active_turn_angle = 0;
    s_turn_next_state = MEDICINE_ROUTE_STATE_IDLE;
    s_action_requested = 0U;
    s_visual_wait_context = MEDICINE_VISUAL_WAIT_FAR_MAIN;
    s_return_turn_index = (int8_t)s_outbound_turn_count - 1;
    s_return_main_intersections = 0U;
    s_end_gate_start_distance_mm = 0;
    s_end_gate_distance_ready = 0U;
    s_start_ms = s_now_ms;
    s_state_enter_ms = s_start_ms;
    MedicineRoute_ResetEventGate();

    if (s_configured == 0U) {
        return;
    }

    if (s_direction == ROUTE_MISSION_OUTBOUND) {
        s_route.state = (uint8_t)MEDICINE_ROUTE_STATE_OUTBOUND_MAIN;
    } else {
        s_route.state = (uint8_t)MEDICINE_ROUTE_STATE_RETURN_TURN_AROUND;
        s_active_turn_angle = 180;
        s_turn_next_state = MEDICINE_ROUTE_STATE_RETURN_ROOM;
    }
}

Project_Status_t MedicineRoute_ConfigureMission(
    uint8_t target_room,
    Route_MissionDirection_t direction,
    uint32_t now_ms)
{
    if ((target_room < 1U) || (target_room > 8U) ||
        ((direction != ROUTE_MISSION_OUTBOUND) &&
         (direction != ROUTE_MISSION_RETURN))) {
        return PROJECT_PARAM;
    }

    if (direction == ROUTE_MISSION_OUTBOUND) {
        s_outbound_turns[0] = 0;
        s_outbound_turns[1] = 0;
        s_outbound_turn_count = 0U;
        s_outbound_main_intersection = 0U;
    } else {
        if ((target_room != s_target_room) ||
            (s_outbound_turn_count == 0U) ||
            (s_outbound_turn_count > MEDICINE_ROUTE_MAX_DECISIONS)) {
            return PROJECT_ERROR;
        }
    }

    s_configured = 1U;
    s_target_room = target_room;
    s_direction = direction;
    MedicineRoute_Reset(now_ms);
    return PROJECT_OK;
}

Project_Status_t MedicineRoute_SubmitVisualDecision(
    Route_VisualDirection_t direction)
{
    if ((direction != ROUTE_VISUAL_LEFT) &&
        (direction != ROUTE_VISUAL_RIGHT) &&
        (direction != ROUTE_VISUAL_STRAIGHT)) {
        return PROJECT_PARAM;
    }
    if ((s_direction != ROUTE_MISSION_OUTBOUND) ||
        (s_route.visual_stage == 0U)) {
        return PROJECT_ERROR;
    }
    if (s_route.visual_decision_ready != 0U) {
        return PROJECT_BUSY;
    }
    if ((direction == ROUTE_VISUAL_STRAIGHT) &&
        (s_route.visual_stage != 1U)) {
        return PROJECT_PARAM;
    }

    s_pending_visual = direction;
    s_route.visual_decision_ready = 1U;
    return PROJECT_OK;
}

Route_ControlMode_t MedicineRoute_Update(
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
    MedicineRoute_ClearOutput(out, request);
    s_route.running_ms = (uint32_t)(s_now_ms - s_start_ms);

    if (s_configured == 0U) {
        MedicineRoute_EnterState(MEDICINE_ROUTE_STATE_ERROR);
        return ROUTE_CONTROL_ERROR;
    }

    switch ((MedicineRoute_State_t)s_route.state) {
    case MEDICINE_ROUTE_STATE_OUTBOUND_MAIN:
        return MedicineRoute_HandleOutboundMain(
            line, feedback, out, request);

    case MEDICINE_ROUTE_STATE_OUTBOUND_FAR_CORRIDOR:
        return MedicineRoute_HandleFarCorridor(
            line, feedback, out, request);

    case MEDICINE_ROUTE_STATE_OUTBOUND_WAIT_VISUAL:
        return MedicineRoute_HandleVisualWait(
            line, feedback, out, request);

    case MEDICINE_ROUTE_STATE_OUTBOUND_TURN:
    case MEDICINE_ROUTE_STATE_RETURN_TURN:
        return MedicineRoute_RunTurn(feedback, request, line, out);

    case MEDICINE_ROUTE_STATE_OUTBOUND_ROOM:
        if (MedicineRoute_UpdateEndGate(
                line,
                MedicineRoute_IsRoomEnd(line),
                feedback->distance_mm) != 0U) {
            MedicineRoute_EnterState(MEDICINE_ROUTE_STATE_ARRIVED);
            return ROUTE_CONTROL_STOP;
        }
        return MedicineRoute_FollowLine(line, out);

    case MEDICINE_ROUTE_STATE_RETURN_TURN_AROUND:
        return MedicineRoute_RunTurn(feedback, request, line, out);

    case MEDICINE_ROUTE_STATE_RETURN_ROOM:
    case MEDICINE_ROUTE_STATE_RETURN_CORRIDOR:
        return MedicineRoute_HandleReturnIntersection(
            line, feedback, out, request);

    case MEDICINE_ROUTE_STATE_RETURN_MAIN:
        return MedicineRoute_HandleReturnMain(line, feedback, out);

    case MEDICINE_ROUTE_STATE_ARRIVED:
        return ROUTE_CONTROL_STOP;

    case MEDICINE_ROUTE_STATE_IDLE:
    case MEDICINE_ROUTE_STATE_ERROR:
    default:
        return ROUTE_CONTROL_ERROR;
    }
}

Project_Status_t MedicineRoute_GetInfo(RouteProfile_Info_t *info)
{
    if (info == 0) {
        return PROJECT_PARAM;
    }

    *info = s_route;
    return PROJECT_OK;
}
