#include "task_fsm.h"

#include "bsp_systick.h"
#include "chassis.h"
#include "drv_status_light.h"
#include "k210_comm.h"
#include "line_follow_app.h"
#include "motion_action.h"
#include "route_manager.h"
#include "sensor_manager.h"

static TaskFSM_Info_t s_task;
static uint8_t s_target_candidate;
static Route_VisualDirection_t s_visual_candidate;
static uint8_t s_visual_stage_seen;
static uint8_t s_visual_absence_left_digit;
static uint8_t s_visual_absence_right_digit;
static uint8_t s_visual_absence_confirm_frames;
static uint8_t s_weight_candidate_valid;
static TaskFSM_LoadState_t s_weight_candidate;
static uint32_t s_state_enter_ms;
static uint32_t s_init_ms;
static uint32_t s_weight_candidate_since_ms;
static uint32_t s_visual_candidate_last_ms;

static Route_VisualDirection_t TaskFSM_GetVisualSide(uint16_t center_x);

static void TaskFSM_StopVehicle(void)
{
    LineFollow_Stop();
    Motion_Stop();
    Chassis_EmergencyStop();
}

static void TaskFSM_SafeStop(void)
{
    TaskFSM_StopVehicle();
    Drv_StatusLight_Off();
}

static void TaskFSM_EnterState(TaskFSM_State_t state)
{
    if (s_task.state == state) {
        return;
    }

    s_task.state = state;
    s_state_enter_ms = BSP_GetTickMs();
    s_task.state_elapsed_ms = 0U;
    s_task.transition_count++;
}

static void TaskFSM_EnterFault(TaskFSM_Fault_t fault)
{
    s_task.fault = fault;
    TaskFSM_EnterState(TASK_FSM_STATE_FAULT);
    TaskFSM_SafeStop();
}

static void TaskFSM_ResetTargetCandidate(void)
{
    s_target_candidate = 0U;
    s_task.observed_digit = 0U;
    s_task.target_confirm_frames = 0U;
}

static void TaskFSM_ResetVisualCandidate(void)
{
    s_visual_candidate = ROUTE_VISUAL_NONE;
    s_visual_absence_left_digit = 0U;
    s_visual_absence_right_digit = 0U;
    s_visual_absence_confirm_frames = 0U;
    s_task.vision_observed_side = (uint8_t)ROUTE_VISUAL_NONE;
    s_task.vision_confirm_frames = 0U;
    s_task.vision_center_x = 0U;
    s_visual_candidate_last_ms = BSP_GetTickMs();
}

static void TaskFSM_ExpireVisualCandidate(void)
{
    if (((s_task.vision_confirm_frames != 0U) ||
         (s_visual_absence_confirm_frames != 0U)) &&
        ((uint32_t)(BSP_GetTickMs() - s_visual_candidate_last_ms) >
         TASK_FSM_VISION_CONFIRM_TIMEOUT_MS)) {
        TaskFSM_ResetVisualCandidate();
    }
}

/* 中部左右两个位置都稳定识别为非目标数字时，确认目标位于远端。 */
static void TaskFSM_ProcessMiddleAbsence(
    const K210_DigitSnapshot_t *snapshot)
{
    Route_VisualDirection_t side;
    uint8_t left_digit;
    uint8_t right_digit;
    uint8_t left_count;
    uint8_t right_count;
    uint8_t i;

    if ((snapshot == 0) || (s_task.route_visual_stage != 1U)) {
        return;
    }

    left_digit = 0U;
    right_digit = 0U;
    left_count = 0U;
    right_count = 0U;
    for (i = 0U; i < snapshot->count; i++) {
        if ((snapshot->items[i].digit < 3U) ||
            (snapshot->items[i].digit > 8U) ||
            (snapshot->items[i].digit == s_task.target_room) ||
            (snapshot->items[i].confidence <
             TASK_FSM_TARGET_MIN_CONFIDENCE)) {
            continue;
        }

        side = TaskFSM_GetVisualSide(snapshot->items[i].center_x);
        if (side == ROUTE_VISUAL_LEFT) {
            left_digit = snapshot->items[i].digit;
            left_count++;
        } else if (side == ROUTE_VISUAL_RIGHT) {
            right_digit = snapshot->items[i].digit;
            right_count++;
        }
    }

    if ((left_count != 1U) || (right_count != 1U) ||
        (left_digit == right_digit)) {
        TaskFSM_ExpireVisualCandidate();
        return;
    }

    s_visual_candidate = ROUTE_VISUAL_NONE;
    s_task.vision_observed_side = (uint8_t)ROUTE_VISUAL_NONE;
    s_task.vision_confirm_frames = 0U;
    s_task.vision_center_x = 0U;

    if ((s_visual_absence_left_digit != left_digit) ||
        (s_visual_absence_right_digit != right_digit)) {
        s_visual_absence_left_digit = left_digit;
        s_visual_absence_right_digit = right_digit;
        s_visual_absence_confirm_frames = 1U;
    } else if (s_visual_absence_confirm_frames < 255U) {
        s_visual_absence_confirm_frames++;
    }

    s_visual_candidate_last_ms = BSP_GetTickMs();
    if (s_visual_absence_confirm_frames >=
        TASK_FSM_VISION_ABSENCE_CONFIRM_FRAMES) {
        (void)RouteManager_SubmitVisualDecision(ROUTE_VISUAL_STRAIGHT);
    }
}

static uint8_t TaskFSM_ReadSingleTargetDigit(uint8_t *digit)
{
    uint8_t digits[K210_MAX_DIGITS];

    if (digit == 0) {
        return 0U;
    }

    /* 初始处方只使用已经由K210稳定输出的数字，不重复检查置信度和坐标。 */
    if ((K210_Comm_ReadDigits(digits) != 1U) ||
        (digits[0] < 1U) ||
        (digits[0] > 8U)) {
        return 0U;
    }

    *digit = digits[0];
    return 1U;
}

static void TaskFSM_UpdateTargetCandidate(uint8_t digit)
{
    s_task.observed_digit = digit;
    if (s_target_candidate != digit) {
        s_target_candidate = digit;
        s_task.target_confirm_frames = 1U;
        return;
    }

    if (s_task.target_confirm_frames < 255U) {
        s_task.target_confirm_frames++;
    }
}

static Route_VisualDirection_t TaskFSM_GetVisualSide(uint16_t center_x)
{
    Route_VisualDirection_t side;

    if (center_x <
        (TASK_FSM_VISION_CENTER_X - TASK_FSM_VISION_DEADBAND_X)) {
        side = ROUTE_VISUAL_LEFT;
    } else if (center_x >
        (TASK_FSM_VISION_CENTER_X + TASK_FSM_VISION_DEADBAND_X)) {
        side = ROUTE_VISUAL_RIGHT;
    } else {
        return ROUTE_VISUAL_NONE;
    }

#if (TASK_FSM_VISION_X_REVERSED != 0U)
    return (side == ROUTE_VISUAL_LEFT) ?
        ROUTE_VISUAL_RIGHT : ROUTE_VISUAL_LEFT;
#else
    return side;
#endif
}

/*
 * 中部两个数字同时入镜时，按目标与另一张数字纸的相对横向顺序判定左右。
 * 这比固定画面中心更适合数字靠近中心、相机安装略有偏移的情况。
 */
static Route_VisualDirection_t TaskFSM_GetMiddleVisualSide(
    const K210_DigitSnapshot_t *snapshot,
    uint16_t target_center_x)
{
    Route_VisualDirection_t side;
    uint8_t other_count;
    uint8_t i;
    uint16_t other_center_x;

    if (snapshot == 0) {
        return ROUTE_VISUAL_NONE;
    }

    other_count = 0U;
    other_center_x = 0U;
    for (i = 0U; i < snapshot->count; i++) {
        if ((snapshot->items[i].digit >= 3U) &&
            (snapshot->items[i].digit <= 8U) &&
            (snapshot->items[i].digit != s_task.target_room) &&
            (snapshot->items[i].confidence >=
             TASK_FSM_TARGET_MIN_CONFIDENCE)) {
            other_count++;
            other_center_x = snapshot->items[i].center_x;
        }
    }

    if ((other_count != 1U) || (other_center_x == target_center_x)) {
        return TaskFSM_GetVisualSide(target_center_x);
    }

    side = (target_center_x < other_center_x) ?
        ROUTE_VISUAL_LEFT : ROUTE_VISUAL_RIGHT;
#if (TASK_FSM_VISION_X_REVERSED != 0U)
    return (side == ROUTE_VISUAL_LEFT) ?
        ROUTE_VISUAL_RIGHT : ROUTE_VISUAL_LEFT;
#else
    return side;
#endif
}

static void TaskFSM_ProcessRouteVision(
    const K210_DigitSnapshot_t *snapshot)
{
    Route_VisualDirection_t side;
    uint8_t match_count;
    uint8_t i;
    uint16_t center_x;

    if ((snapshot == 0) ||
        (s_task.route_visual_stage == 0U) ||
        (s_task.route_visual_ready != 0U)) {
        TaskFSM_ResetVisualCandidate();
        return;
    }

    if ((snapshot->status != K210_RESULT_NORMAL) ||
        (snapshot->count == 0U) ||
        (snapshot->count > K210_MAX_DIGITS)) {
        TaskFSM_ExpireVisualCandidate();
        return;
    }

    match_count = 0U;
    center_x = 0U;
    for (i = 0U; i < snapshot->count; i++) {
        if ((snapshot->items[i].digit == s_task.target_room) &&
            (snapshot->items[i].confidence >=
             TASK_FSM_TARGET_MIN_CONFIDENCE)) {
            match_count++;
            center_x = snapshot->items[i].center_x;
        }
    }

    if (match_count != 1U) {
        if ((match_count == 0U) &&
            (s_task.vision_confirm_frames == 0U)) {
            TaskFSM_ProcessMiddleAbsence(snapshot);
        }
        TaskFSM_ExpireVisualCandidate();
        return;
    }

    s_visual_absence_left_digit = 0U;
    s_visual_absence_right_digit = 0U;
    s_visual_absence_confirm_frames = 0U;

    side = (s_task.route_visual_stage == 1U) ?
        TaskFSM_GetMiddleVisualSide(snapshot, center_x) :
        TaskFSM_GetVisualSide(center_x);
    if (side == ROUTE_VISUAL_NONE) {
        TaskFSM_ExpireVisualCandidate();
        return;
    }

    s_task.observed_digit = s_task.target_room;
    s_task.vision_observed_side = (uint8_t)side;
    s_task.vision_center_x = center_x;
    if (s_visual_candidate != side) {
        s_visual_candidate = side;
        s_task.vision_confirm_frames = 1U;
        s_visual_candidate_last_ms = BSP_GetTickMs();
        return;
    }

    if (s_task.vision_confirm_frames < 255U) {
        s_task.vision_confirm_frames++;
    }
    s_visual_candidate_last_ms = BSP_GetTickMs();
    if (s_task.vision_confirm_frames >= TASK_FSM_VISION_CONFIRM_FRAMES) {
        (void)RouteManager_SubmitVisualDecision(side);
    }
}

static void TaskFSM_UpdateVision(void)
{
    K210_Comm_Info_t comm_info;
    K210_DigitSnapshot_t snapshot;
    uint8_t digit;

    if (K210_Comm_GetInfo(&comm_info) != BSP_OK) {
        s_task.k210_online = 0U;
        TaskFSM_ResetTargetCandidate();
        TaskFSM_ResetVisualCandidate();
        return;
    }

    s_task.k210_online = comm_info.online;
    if (comm_info.online == 0U) {
        TaskFSM_ResetTargetCandidate();
        TaskFSM_ResetVisualCandidate();
        return;
    }

    if ((s_task.target_confirm_frames != 0U) &&
        ((uint32_t)(BSP_GetTickMs() - s_task.last_snapshot_ms) >
         TASK_FSM_TARGET_CONFIRM_TIMEOUT_MS)) {
        TaskFSM_ResetTargetCandidate();
    }
    TaskFSM_ExpireVisualCandidate();

    if (K210_Comm_GetNewSnapshot(&snapshot) != BSP_OK) {
        return;
    }

    s_task.last_snapshot_ms = BSP_GetTickMs();

    /* 所有状态都消费新快照，防止业务暂停读取时通信层覆盖计数增加。 */
    if (s_task.state == TASK_FSM_STATE_OUTBOUND) {
        TaskFSM_ResetTargetCandidate();
        TaskFSM_ProcessRouteVision(&snapshot);
        return;
    }

    if (s_task.state != TASK_FSM_STATE_WAIT_TARGET) {
        TaskFSM_ResetTargetCandidate();
        TaskFSM_ResetVisualCandidate();
        return;
    }

    /* 目标一旦确认就保持锁定；等待称重就绪期间仍消费快照，但不覆盖目标。 */
    if (s_task.target_locked != 0U) {
        return;
    }

    if (TaskFSM_ReadSingleTargetDigit(&digit) == 0U) {
        TaskFSM_ResetTargetCandidate();
        return;
    }

    TaskFSM_UpdateTargetCandidate(digit);
    if (s_task.target_confirm_frames < TASK_FSM_TARGET_CONFIRM_FRAMES) {
        return;
    }

    /* 先保存目标；进入等待装药由主状态迁移在空载就绪后单独完成。 */
    s_task.target_room = digit;
    s_task.target_locked = 1U;
}

static TaskFSM_LoadState_t TaskFSM_GetWeightDesiredState(float weight_g)
{
    if (weight_g >= TASK_FSM_LOAD_ON_THRESHOLD_G) {
        return TASK_FSM_LOAD_PRESENT;
    }
    if (weight_g <= TASK_FSM_LOAD_OFF_THRESHOLD_G) {
        return TASK_FSM_LOAD_EMPTY;
    }

    /* 位于滞回区时保持当前确认结果，不制造新的候选状态。 */
    return s_task.load_state;
}

static void TaskFSM_UpdateWeight(void)
{
    TaskFSM_LoadState_t desired_state;
    BSP_Status_t weight_status;
    float weight_g;

    weight_status = Sensor_GetPressureGram(&weight_g);
    if (weight_status != BSP_OK) {
        /*
         * HX711单项测试已验证正常。整车启动时使用现有手动去皮接口，
         * 在WAIT_TARGET且托盘应为空载的阶段建立一次零点，避免原始值
         * 波动使驱动的严格稳定窗口一直无法完成。即使目标先锁定，
         * 也只记录目标，不会在空载就绪前进入WAIT_LOAD。
         */
        if ((s_task.state == TASK_FSM_STATE_WAIT_TARGET) &&
            (s_task.empty_seen == 0U) &&
            ((uint32_t)(BSP_GetTickMs() - s_init_ms) >=
             TASK_FSM_STARTUP_TARE_DELAY_MS) &&
            (Sensor_PressureTare() == BSP_OK) &&
            (Sensor_GetPressureGram(&weight_g) == BSP_OK)) {
            s_task.load_state = TASK_FSM_LOAD_EMPTY;
            s_task.empty_seen = 1U;
            s_weight_candidate_valid = 0U;
        } else {
            s_task.weight_valid = 0U;
            s_weight_candidate_valid = 0U;
            return;
        }
    }

    s_task.weight_valid = 1U;
    s_task.weight_g = weight_g;
    desired_state = TaskFSM_GetWeightDesiredState(weight_g);

    if ((desired_state == TASK_FSM_LOAD_UNKNOWN) ||
        (desired_state == s_task.load_state)) {
        s_weight_candidate_valid = 0U;
        return;
    }

    if ((s_weight_candidate_valid == 0U) ||
        (s_weight_candidate != desired_state)) {
        s_weight_candidate = desired_state;
        s_weight_candidate_since_ms = BSP_GetTickMs();
        s_weight_candidate_valid = 1U;
        return;
    }

    if ((uint32_t)(BSP_GetTickMs() - s_weight_candidate_since_ms) <
        TASK_FSM_WEIGHT_CONFIRM_MS) {
        return;
    }

    s_task.load_state = desired_state;
    s_weight_candidate_valid = 0U;
    if (desired_state == TASK_FSM_LOAD_EMPTY) {
        s_task.empty_seen = 1U;
    }
}

static void TaskFSM_UpdateRouteInfo(void)
{
    RouteManager_Info_t route_info;

    if (RouteManager_GetInfo(&route_info) != BSP_OK) {
        s_task.route_state = 0U;
        s_task.route_approach_ready = 0U;
        s_task.route_visual_stage = 0U;
        s_task.route_visual_ready = 0U;
        s_task.route_waiting_visual = 0U;
        s_task.route_intersections = 0U;
        s_task.route_decisions = 0U;
        s_task.route_arrived = 0U;
        return;
    }

    s_task.route_state = route_info.profile_state;
    s_task.route_approach_ready = route_info.room_approach_ready;
    s_task.route_visual_stage = route_info.visual_stage;
    s_task.route_visual_ready = route_info.visual_decision_ready;
    s_task.route_waiting_visual = route_info.waiting_visual;
    s_task.route_intersections = route_info.intersection_count;
    s_task.route_decisions = route_info.decisions_completed;
    s_task.route_arrived = route_info.arrived;

    if (s_visual_stage_seen != s_task.route_visual_stage) {
        s_visual_stage_seen = s_task.route_visual_stage;
        TaskFSM_ResetVisualCandidate();
    }
}

static BSP_Status_t TaskFSM_StartRoute(Route_MissionDirection_t direction)
{
    BSP_Status_t status;

    status = RouteManager_ConfigureMission(s_task.target_room, direction);
    s_task.route_start_status = (uint8_t)status;
    if (status != BSP_OK) {
        return status;
    }

    status = LineFollow_Start();
    s_task.route_start_status = (uint8_t)status;
    if (status == BSP_OK) {
        s_task.route_approach_ready = 0U;
        s_task.route_visual_stage = 0U;
        s_task.route_visual_ready = 0U;
        s_task.route_waiting_visual = 0U;
        s_task.route_arrived = 0U;
    }
    return status;
}

void TaskFSM_Init(void)
{
    s_task.state = TASK_FSM_STATE_WAIT_TARGET;
    s_task.load_state = TASK_FSM_LOAD_UNKNOWN;
    s_task.fault = TASK_FSM_FAULT_NONE;
    s_task.target_room = 0U;
    s_task.target_locked = 0U;
    s_task.observed_digit = 0U;
    s_task.target_confirm_frames = 0U;
    s_task.k210_online = 0U;
    s_task.weight_valid = 0U;
    s_task.empty_seen = 0U;
    s_task.vision_observed_side = (uint8_t)ROUTE_VISUAL_NONE;
    s_task.vision_confirm_frames = 0U;
    s_task.vision_center_x = 0U;
    s_task.route_state = 0U;
    s_task.route_approach_ready = 0U;
    s_task.route_visual_stage = 0U;
    s_task.route_visual_ready = 0U;
    s_task.route_waiting_visual = 0U;
    s_task.route_start_status = (uint8_t)BSP_OK;
    s_task.route_intersections = 0U;
    s_task.route_decisions = 0U;
    s_task.route_arrived = 0U;
    s_task.stop_confirmed = 0U;
    s_task.status_light = (uint8_t)DRV_STATUS_LIGHT_OFF;
    s_task.weight_g = 0.0f;
    s_task.state_elapsed_ms = 0U;
    s_task.last_snapshot_ms = 0U;
    s_task.transition_count = 0U;

    s_target_candidate = 0U;
    s_visual_candidate = ROUTE_VISUAL_NONE;
    s_visual_stage_seen = 0U;
    s_visual_absence_left_digit = 0U;
    s_visual_absence_right_digit = 0U;
    s_visual_absence_confirm_frames = 0U;
    s_weight_candidate_valid = 0U;
    s_weight_candidate = TASK_FSM_LOAD_UNKNOWN;
    s_state_enter_ms = BSP_GetTickMs();
    s_init_ms = s_state_enter_ms;
    s_weight_candidate_since_ms = BSP_GetTickMs();
    s_visual_candidate_last_ms = BSP_GetTickMs();

    TaskFSM_SafeStop();
}

void TaskFSM_Reset(void)
{
    TaskFSM_SafeStop();
    TaskFSM_Init();
}

void TaskFSM_Update(void)
{
    BSP_Status_t start_status;

    TaskFSM_UpdateWeight();
    TaskFSM_UpdateRouteInfo();
    TaskFSM_UpdateVision();
    s_task.status_light = (uint8_t)Drv_StatusLight_GetMode();
    s_task.state_elapsed_ms =
        (uint32_t)(BSP_GetTickMs() - s_state_enter_ms);

    switch (s_task.state) {
    case TASK_FSM_STATE_WAIT_TARGET:
        if ((s_task.target_locked != 0U) &&
            (s_task.weight_valid != 0U) &&
            (s_task.load_state == TASK_FSM_LOAD_EMPTY)) {
            TaskFSM_EnterState(TASK_FSM_STATE_WAIT_LOAD);
        }
        break;

    case TASK_FSM_STATE_WAIT_LOAD:
        if ((s_task.empty_seen != 0U) &&
            (s_task.load_state == TASK_FSM_LOAD_PRESENT)) {
            TaskFSM_EnterState(TASK_FSM_STATE_READY);
        }
        break;

    case TASK_FSM_STATE_READY:
        start_status = TaskFSM_StartRoute(ROUTE_MISSION_OUTBOUND);
        if (start_status == BSP_OK) {
            Drv_StatusLight_Off();
            TaskFSM_ResetTargetCandidate();
            TaskFSM_ResetVisualCandidate();
            TaskFSM_EnterState(TASK_FSM_STATE_OUTBOUND);
        } else if (start_status == BSP_PARAM) {
            TaskFSM_EnterFault(TASK_FSM_FAULT_ROUTE);
        }
        break;

    case TASK_FSM_STATE_OUTBOUND:
        if (s_task.route_arrived != 0U) {
            TaskFSM_StopVehicle();
            s_task.stop_confirmed = 1U;
            Drv_StatusLight_SetRed();
            TaskFSM_EnterState(TASK_FSM_STATE_WAIT_UNLOAD);
        } else if ((LineFollow_GetState() != LINE_FOLLOW_RUN) ||
            (Chassis_GetFault() != CHASSIS_FAULT_NONE)) {
            TaskFSM_EnterFault(
                (Chassis_GetFault() != CHASSIS_FAULT_NONE) ?
                TASK_FSM_FAULT_CHASSIS : TASK_FSM_FAULT_ROUTE);
        }
        break;

    case TASK_FSM_STATE_WAIT_UNLOAD:
        if (s_task.load_state == TASK_FSM_LOAD_EMPTY) {
            Drv_StatusLight_Off();
            TaskFSM_EnterState(TASK_FSM_STATE_RETURN_READY);
        }
        break;

    case TASK_FSM_STATE_RETURN_READY:
        start_status = TaskFSM_StartRoute(ROUTE_MISSION_RETURN);
        if (start_status == BSP_OK) {
            TaskFSM_EnterState(TASK_FSM_STATE_RETURNING);
        } else if (start_status == BSP_PARAM) {
            TaskFSM_EnterFault(TASK_FSM_FAULT_ROUTE);
        }
        break;

    case TASK_FSM_STATE_RETURNING:
        if (s_task.route_arrived != 0U) {
            TaskFSM_StopVehicle();
            s_task.stop_confirmed = 1U;
            Drv_StatusLight_SetGreen();
            TaskFSM_EnterState(TASK_FSM_STATE_COMPLETE);
        } else if ((LineFollow_GetState() != LINE_FOLLOW_RUN) ||
                   (Chassis_GetFault() != CHASSIS_FAULT_NONE)) {
            TaskFSM_EnterFault(
                (Chassis_GetFault() != CHASSIS_FAULT_NONE) ?
                TASK_FSM_FAULT_CHASSIS : TASK_FSM_FAULT_ROUTE);
        }
        break;

    case TASK_FSM_STATE_COMPLETE:
        Chassis_EmergencyStop();
        break;

    case TASK_FSM_STATE_FAULT:
        TaskFSM_SafeStop();
        break;

    default:
        TaskFSM_EnterFault(TASK_FSM_FAULT_SENSOR);
        break;
    }

    s_task.status_light = (uint8_t)Drv_StatusLight_GetMode();
}

BSP_Status_t TaskFSM_GetInfo(TaskFSM_Info_t *info)
{
    if (info == 0) {
        return BSP_PARAM;
    }

    *info = s_task;
    return BSP_OK;
}
