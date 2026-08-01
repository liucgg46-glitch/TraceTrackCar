#include "line_track.h"
#include "control_config.h"

typedef struct {
    int16_t base_speed_cps;
    int16_t cross_speed_cps;
    int16_t min_track_speed_cps;
    int16_t turn_max_cps;
    int16_t search_turn_cps;
    float kp;
    float kd;
    int16_t error_deadband;
    uint16_t error_filter_num;
    uint16_t error_filter_den;
    int16_t speed_full_error;
    int16_t speed_min_error;
    uint16_t lost_confirm_samples;
    uint16_t reacquire_confirm_samples;
    int16_t search_turn_step_cps;
    int16_t search_turn_max_cps;
    int16_t search_min_inner_cps;
    uint32_t search_initial_phase_ms;
    uint32_t search_phase_step_ms;
    uint32_t search_phase_max_ms;
    uint32_t search_timeout_ms;
} LineTrack_InternalConfig_t;

static LineTrack_InternalConfig_t s_cfg;
static LineTrack_Mode_t s_mode;
static int16_t s_raw_error;
static int16_t s_error_filt;
static uint8_t s_error_filter_valid;
static int16_t s_last_error;
static int16_t s_last_linear_cmd;
static int16_t s_last_turn_cmd;
static int16_t s_target_linear;
static int16_t s_target_turn;
static int8_t s_last_line_direction;
static int8_t s_search_direction;
static uint16_t s_lost_samples;
static uint16_t s_reacquire_samples;
static uint16_t s_search_phase;
static uint32_t s_lost_start_ms;
static uint32_t s_search_phase_start_ms;
static uint32_t s_now_ms;

static void LineTrack_SetOutput(int16_t linear,
                                int16_t turn,
                                uint8_t valid,
                                LineTrack_Output_t *out);

static int16_t LineTrack_AbsI16(int16_t value)
{
    return (value >= 0) ? value : (int16_t)(-value);
}

static int16_t LineTrack_LimitI16(int32_t value,
                                  int16_t min_value,
                                  int16_t max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return (int16_t)value;
}

static int16_t LineTrack_LimitFloat(float value,
                                    int16_t min_value,
                                    int16_t max_value)
{
    if (value > (float)max_value) return max_value;
    if (value < (float)min_value) return min_value;
    return (int16_t)value;
}

static uint16_t LineTrack_IncrementU16(uint16_t value)
{
    return (value < 0xFFFFU) ? (uint16_t)(value + 1U) : value;
}

/*
 * 数字量灰度的误差只能在有限位置之间跳变。这里在算法层做一阶滤波，
 * 避免相邻通道切换直接变成左右轮目标的阶跃，同时保持10ms非阻塞更新。
 */
static int16_t LineTrack_FilterError(int16_t raw_error)
{
    int32_t difference;
    int32_t step;

    if (s_error_filter_valid == 0U) {
        s_error_filt = raw_error;
        s_error_filter_valid = 1U;
        return s_error_filt;
    }

    difference = (int32_t)raw_error - (int32_t)s_error_filt;
    step = difference * (int32_t)s_cfg.error_filter_num /
           (int32_t)s_cfg.error_filter_den;

    if ((step == 0) && (difference != 0)) {
        step = (difference > 0) ? 1 : -1;
    }

    s_error_filt = LineTrack_LimitI16((int32_t)s_error_filt + step,
                                      -3500,
                                      3500);
    return s_error_filt;
}

/*
 * 根据线路误差确定找线方向。
 * 返回 +1 表示左转，-1 表示右转。
 */
static int8_t LineTrack_DirectionFromError(int16_t error)
{
    if (error < (int16_t)(-s_cfg.error_deadband)) return 1;
    if (error > s_cfg.error_deadband) return -1;
    return s_last_line_direction;
}

static int16_t LineTrack_GetSearchTurnCps(uint16_t phase)
{
    uint32_t step_count;
    uint32_t max_step_count;

    if ((s_cfg.search_turn_step_cps <= 0) ||
        (s_cfg.search_turn_cps >= s_cfg.search_turn_max_cps)) {
        return s_cfg.search_turn_cps;
    }

    max_step_count = (uint32_t)(s_cfg.search_turn_max_cps -
                                s_cfg.search_turn_cps) /
                     (uint32_t)s_cfg.search_turn_step_cps;
    step_count = (uint32_t)phase;
    if (step_count > max_step_count) {
        step_count = max_step_count;
    }

    return (int16_t)((int32_t)s_cfg.search_turn_cps +
                     (int32_t)step_count *
                     (int32_t)s_cfg.search_turn_step_cps);
}

static uint32_t LineTrack_GetSearchPhaseMs(uint16_t phase)
{
    uint32_t step_count;
    uint32_t max_step_count;

    max_step_count = (s_cfg.search_phase_max_ms -
                      s_cfg.search_initial_phase_ms) /
                     s_cfg.search_phase_step_ms;
    step_count = (uint32_t)phase;
    if (step_count > max_step_count) {
        step_count = max_step_count;
    }

    return s_cfg.search_initial_phase_ms +
           step_count * s_cfg.search_phase_step_ms;
}

static void LineTrack_ClearLossRecovery(void)
{
    s_lost_samples = 0U;
    s_search_phase = 0U;
    s_lost_start_ms = 0U;
    s_search_phase_start_ms = 0U;
}

static void LineTrack_StartSearch(uint32_t now)
{
    s_mode = LINE_TRACK_MODE_SEARCH;
    s_reacquire_samples = 0U;
    s_search_phase = 0U;
    s_search_phase_start_ms = now;
    s_search_direction = LineTrack_DirectionFromError(s_last_error);
    if (s_search_direction == 0) {
        s_search_direction = 1;
    }
}

static void LineTrack_UpdateSearchPhase(uint32_t now)
{
    uint32_t phase_ms;

    phase_ms = LineTrack_GetSearchPhaseMs(s_search_phase);
    while ((uint32_t)(now - s_search_phase_start_ms) >= phase_ms) {
        s_search_phase_start_ms += phase_ms;
        s_search_phase = LineTrack_IncrementU16(s_search_phase);
        s_search_direction = (int8_t)(-s_search_direction);
        phase_ms = LineTrack_GetSearchPhaseMs(s_search_phase);
    }
}

static void LineTrack_OutputSearch(LineTrack_Output_t *out)
{
    int16_t linear_cps;
    int16_t max_turn_cps;
    int16_t turn_cps;

    linear_cps = s_last_linear_cmd;
    if (linear_cps <= 0) {
        linear_cps = s_cfg.base_speed_cps;
    }
    if (linear_cps < s_cfg.min_track_speed_cps) {
        linear_cps = s_cfg.min_track_speed_cps;
    }

    turn_cps = LineTrack_GetSearchTurnCps(s_search_phase);
    max_turn_cps = (int16_t)(linear_cps - s_cfg.search_min_inner_cps);
    if (max_turn_cps < 0) {
        max_turn_cps = 0;
    }
    if (turn_cps > max_turn_cps) {
        turn_cps = max_turn_cps;
    }

    LineTrack_SetOutput(linear_cps,
                        (int16_t)(s_search_direction * turn_cps),
                        1U,
                        out);
}

/*
 * 误差越大，直行速度越低。
 * 只做一次线性计算，不再建立额外的“边缘状态”。
 */
static int16_t LineTrack_GetAdaptiveSpeed(int16_t error)
{
    int16_t abs_error;
    int32_t error_span;
    int32_t speed_span;
    int32_t speed;

    abs_error = LineTrack_AbsI16(error);

    if (abs_error <= s_cfg.speed_full_error) {
        return s_cfg.base_speed_cps;
    }

    if (abs_error >= s_cfg.speed_min_error) {
        return s_cfg.min_track_speed_cps;
    }

    error_span = (int32_t)s_cfg.speed_min_error -
                 (int32_t)s_cfg.speed_full_error;
    if (error_span <= 0) {
        return s_cfg.min_track_speed_cps;
    }

    speed_span = (int32_t)s_cfg.base_speed_cps -
                 (int32_t)s_cfg.min_track_speed_cps;

    speed = (int32_t)s_cfg.base_speed_cps -
            (((int32_t)abs_error - (int32_t)s_cfg.speed_full_error) *
             speed_span / error_span);

    return LineTrack_LimitI16(speed,
                              s_cfg.min_track_speed_cps,
                              s_cfg.base_speed_cps);
}

/*
 * 算法层直接保存目标，不再叠加第二套斜坡；所有上层命令统一由底盘层
 * 的目标斜坡处理，避免两层限速导致响应时间难以判断。
 */
static void LineTrack_SetOutput(int16_t linear,
                                int16_t turn,
                                uint8_t valid,
                                LineTrack_Output_t *out)
{
    linear = LineTrack_LimitI16(linear,
                                0,
                                CONTROL_CHASSIS_TARGET_MAX_CPS);
    turn = LineTrack_LimitI16(turn,
                              (int16_t)(-CONTROL_CHASSIS_TARGET_MAX_CPS),
                              CONTROL_CHASSIS_TARGET_MAX_CPS);

    s_target_linear = linear;
    s_target_turn = turn;
    s_last_linear_cmd = linear;
    s_last_turn_cmd = turn;

    out->linear_cps = linear;
    out->turn_cps = turn;
    out->valid = valid;
}

static void LineTrack_LoadDefaultConfig(void)
{
    s_cfg.base_speed_cps = CONTROL_LINE_BASE_SPEED_CPS;
    s_cfg.cross_speed_cps = CONTROL_LINE_CROSS_SPEED_CPS;
    s_cfg.min_track_speed_cps = CONTROL_LINE_MIN_TRACK_SPEED_CPS;
    LineTrack_ResetControlProfile();
    s_cfg.search_turn_cps = CONTROL_LINE_SEARCH_TURN_CPS;
    s_cfg.lost_confirm_samples = CONTROL_LINE_LOST_CONFIRM_SAMPLES;
    s_cfg.reacquire_confirm_samples =
        CONTROL_LINE_REACQUIRE_CONFIRM_SAMPLES;
    s_cfg.search_turn_step_cps = CONTROL_LINE_SEARCH_TURN_STEP_CPS;
    s_cfg.search_turn_max_cps = CONTROL_LINE_SEARCH_TURN_MAX_CPS;
    s_cfg.search_min_inner_cps = CONTROL_LINE_SEARCH_MIN_INNER_CPS;
    s_cfg.search_initial_phase_ms =
        CONTROL_LINE_SEARCH_INITIAL_PHASE_MS;
    s_cfg.search_phase_step_ms = CONTROL_LINE_SEARCH_PHASE_STEP_MS;
    s_cfg.search_phase_max_ms = CONTROL_LINE_SEARCH_PHASE_MAX_MS;
    s_cfg.search_timeout_ms = CONTROL_LINE_SEARCH_TIMEOUT_MS;
}

void LineTrack_ResetControlProfile(void)
{
    s_cfg.turn_max_cps = CONTROL_LINE_TURN_MAX_CPS;
    s_cfg.kp = CONTROL_LINE_KP;
    s_cfg.kd = CONTROL_LINE_KD;
    s_cfg.error_deadband = CONTROL_LINE_ERROR_DEADBAND;
    s_cfg.error_filter_num = CONTROL_LINE_ERROR_FILTER_NUM;
    s_cfg.error_filter_den = CONTROL_LINE_ERROR_FILTER_DEN;
    s_cfg.speed_full_error = CONTROL_LINE_SPEED_FULL_ERROR;
    s_cfg.speed_min_error = CONTROL_LINE_SPEED_MIN_ERROR;
}

void LineTrack_Init(void)
{
    LineTrack_LoadDefaultConfig();
    LineTrack_Reset();
}

void LineTrack_Reset(void)
{
    s_mode = LINE_TRACK_MODE_TRACK;
    s_raw_error = 0;
    s_error_filt = 0;
    s_error_filter_valid = 0U;
    s_last_error = 0;
    s_last_linear_cmd = 0;
    s_last_turn_cmd = 0;
    s_target_linear = 0;
    s_target_turn = 0;
    s_last_line_direction = 1;
    s_search_direction = 1;
    s_reacquire_samples = 0U;
    s_now_ms = 0U;
    LineTrack_ClearLossRecovery();
}

Project_Status_t LineTrack_SetSpeedProfile(int16_t base_speed_cps,
                                           int16_t cross_speed_cps,
                                           int16_t min_track_speed_cps)
{
    if ((base_speed_cps < 0) ||
        (base_speed_cps > CONTROL_CHASSIS_TARGET_MAX_CPS) ||
        (cross_speed_cps < 0) ||
        (cross_speed_cps > CONTROL_CHASSIS_TARGET_MAX_CPS) ||
        (min_track_speed_cps < 0) ||
        (min_track_speed_cps > base_speed_cps)) {
        return PROJECT_PARAM;
    }

    s_cfg.base_speed_cps = base_speed_cps;
    s_cfg.cross_speed_cps = cross_speed_cps;
    s_cfg.min_track_speed_cps = min_track_speed_cps;
    return PROJECT_OK;
}

Project_Status_t LineTrack_SetControlProfile(
    const LineTrack_ControlProfile_t *profile)
{
    if ((profile == 0) ||
        (profile->turn_max_cps < 0) ||
        (profile->turn_max_cps > CONTROL_CHASSIS_TARGET_MAX_CPS) ||
        (profile->kp < 0.0f) ||
        (profile->kd < 0.0f) ||
        (profile->error_deadband < 0) ||
        (profile->error_filter_num == 0U) ||
        (profile->error_filter_den == 0U) ||
        (profile->error_filter_num > profile->error_filter_den) ||
        (profile->speed_full_error < 0) ||
        (profile->speed_min_error <= profile->speed_full_error)) {
        return PROJECT_PARAM;
    }

    s_cfg.turn_max_cps = profile->turn_max_cps;
    s_cfg.kp = profile->kp;
    s_cfg.kd = profile->kd;
    s_cfg.error_deadband = profile->error_deadband;
    s_cfg.error_filter_num = profile->error_filter_num;
    s_cfg.error_filter_den = profile->error_filter_den;
    s_cfg.speed_full_error = profile->speed_full_error;
    s_cfg.speed_min_error = profile->speed_min_error;
    return PROJECT_OK;
}

Project_Status_t LineTrack_GetInfo(LineTrack_Info_t *info)
{
    if (info == 0) return PROJECT_PARAM;

    info->mode = s_mode;
    info->raw_error = s_raw_error;
    info->filtered_error = s_error_filt;
    info->target_linear_cps = s_target_linear;
    info->target_turn_cps = s_target_turn;
    info->output_linear_cps = s_last_linear_cmd;
    info->output_turn_cps = s_last_turn_cmd;
    info->lost_samples = s_lost_samples;
    info->reacquire_samples = s_reacquire_samples;
    info->search_phase = s_search_phase;
    info->search_direction = s_search_direction;
    info->lost_ms = (s_lost_samples == 0U) ? 0U :
                    (uint32_t)(s_now_ms - s_lost_start_ms);
    info->control_profile.turn_max_cps = s_cfg.turn_max_cps;
    info->control_profile.kp = s_cfg.kp;
    info->control_profile.kd = s_cfg.kd;
    info->control_profile.error_deadband = s_cfg.error_deadband;
    info->control_profile.error_filter_num = s_cfg.error_filter_num;
    info->control_profile.error_filter_den = s_cfg.error_filter_den;
    info->control_profile.speed_full_error = s_cfg.speed_full_error;
    info->control_profile.speed_min_error = s_cfg.speed_min_error;

    return PROJECT_OK;
}

void LineTrack_Compute(const LineDetect_Result_t *line,
                       LineTrack_Output_t *out,
                       uint32_t now_ms)
{
    uint8_t was_recovering;
    int16_t error;
    int16_t d_error;
    int16_t target_linear;
    int16_t target_turn;
    int8_t direction;
    float turn_f;

    if ((line == 0) || (out == 0)) return;

    out->linear_cps = 0;
    out->turn_cps = 0;
    out->valid = 0U;

    s_now_ms = now_ms;
    s_raw_error = line->error_x1000;

    /* 找线超时后保持无效输出，必须由上层重新启动循迹。 */
    if (s_mode == LINE_TRACK_MODE_FAILSAFE) {
        return;
    }

    /*
     * 先连续确认丢线，避免单帧噪声触发原地搜索。进入搜索后从最后一次
     * 线路方向开始，随后交替方向并逐步增加单阶段时间和转向量。
     */
    if (line->type == LINE_TYPE_LOST) {
        if (s_lost_samples == 0U) {
            s_lost_start_ms = s_now_ms;
        }

        s_lost_samples = LineTrack_IncrementU16(s_lost_samples);
        s_reacquire_samples = 0U;

        if ((uint32_t)(s_now_ms - s_lost_start_ms) >=
            s_cfg.search_timeout_ms) {
            s_mode = LINE_TRACK_MODE_FAILSAFE;
            LineTrack_SetOutput(0, 0, 0U, out);
            return;
        }

        if (s_mode != LINE_TRACK_MODE_SEARCH) {
            if (s_lost_samples < s_cfg.lost_confirm_samples) {
                s_mode = LINE_TRACK_MODE_LOST_CONFIRM;
                LineTrack_SetOutput(s_last_linear_cmd,
                                    s_last_turn_cmd,
                                    1U,
                                    out);
                return;
            }
            LineTrack_StartSearch(s_now_ms);
        } else {
            LineTrack_UpdateSearchPhase(s_now_ms);
        }

        LineTrack_OutputSearch(out);
        return;
    }

    was_recovering = (s_mode != LINE_TRACK_MODE_TRACK) ? 1U : 0U;

    /*
     * 搜索过程中看到线路后仍连续确认多帧，但首帧立即按当前线路误差修正，
     * 不再输出0/0造成制动；中间再次丢线会继续原扫描阶段。
     */
    if (s_mode == LINE_TRACK_MODE_SEARCH) {
        s_reacquire_samples =
            LineTrack_IncrementU16(s_reacquire_samples);
        if (s_reacquire_samples >= s_cfg.reacquire_confirm_samples) {
            s_mode = LINE_TRACK_MODE_TRACK;
            LineTrack_ClearLossRecovery();
        }
    } else {
        s_mode = LINE_TRACK_MODE_TRACK;
        LineTrack_ClearLossRecovery();
    }

    /*
     * 十字和全黑区域在基础模式下统一低速直行。
     * 左右分支不在这里强制直行，仍由下面的 P/PD 根据实际误差处理，
     * 避免把右直角误判成宽线后继续向前冲。
     */
    if ((line->type == LINE_TYPE_CROSS) ||
        (line->type == LINE_TYPE_FULL_BLACK)) {
        s_error_filt = 0;
        s_error_filter_valid = 1U;
        s_last_error = 0;
        LineTrack_SetOutput(s_cfg.cross_speed_cps, 0, 1U, out);
        return;
    }

    error = line->error_x1000;
    if ((error > (int16_t)(-s_cfg.error_deadband)) &&
        (error < s_cfg.error_deadband)) {
        error = 0;
    }

    /*
     * 正常循迹时平滑数字量位置跳变；刚从丢线恢复时直接采用当前误差，
     * 避免旧方向残留延迟重新建立转向。
     */
    if (was_recovering != 0U) {
        s_error_filt = error;
        s_error_filter_valid = 1U;
    } else {
        error = LineTrack_FilterError(error);
        if ((error > (int16_t)(-s_cfg.error_deadband)) &&
            (error < s_cfg.error_deadband)) {
            error = 0;
            s_error_filt = 0;
        }
    }
    error = s_error_filt;

    /* 刚刚重新找到线时不使用搜索前的旧误差计算微分。 */
    if (was_recovering != 0U) {
        s_last_error = error;
        d_error = 0;
    } else {
        d_error = (int16_t)(error - s_last_error);
        s_last_error = error;
    }

    direction = LineTrack_DirectionFromError(error);
    if (direction != 0) {
        s_last_line_direction = direction;
    }

    target_linear = LineTrack_GetAdaptiveSpeed(error);
    turn_f = -(s_cfg.kp * (float)error +
               s_cfg.kd * (float)d_error);
    target_turn = LineTrack_LimitFloat(turn_f,
                                       (int16_t)(-s_cfg.turn_max_cps),
                                       s_cfg.turn_max_cps);

    /*
     * 普通循迹时不让内侧车轮反转：
     * |turn| == linear 时内侧轮停止，已经能够形成很强的转向。
     */
    if (target_turn > target_linear) {
        target_turn = target_linear;
    }
    if (target_turn < (int16_t)(-target_linear)) {
        target_turn = (int16_t)(-target_linear);
    }

    LineTrack_SetOutput(target_linear, target_turn, 1U, out);
}
