#include "line_track.h"
#include "control_config.h"

/*
 * 基础版只保留真正需要调节的参数，不再包含边缘模式、宽线模式、
 * 输出斜坡、多阶段反向扫描等互相叠加的控制逻辑。
 */
typedef struct {
    int16_t base_speed_cps;
    int16_t cross_speed_cps;
    int16_t min_track_speed_cps;
    int16_t turn_max_cps;
    int16_t search_turn_cps;
    float kp;
    float kd;
    int16_t error_deadband;
    int16_t speed_full_error;
    int16_t speed_min_error;
    uint32_t search_timeout_ms;
} LineTrack_InternalConfig_t;

static LineTrack_InternalConfig_t s_cfg;
static LineTrack_Mode_t s_mode;
static int16_t s_raw_error;
static int16_t s_error_filt;
static int16_t s_last_error;
static int16_t s_derivative_error;
static int16_t s_adaptive_linear;
static int32_t s_turn_unclamped;
static uint8_t s_turn_saturated;
static int16_t s_last_linear_cmd;
static int16_t s_last_turn_cmd;
static int16_t s_target_linear;
static int16_t s_target_turn;
static int8_t s_last_line_direction;
static int8_t s_search_direction;
static uint16_t s_lost_samples;
static uint32_t s_lost_start_ms;

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
 * 直接保存并输出命令。
 * 基础版故意取消输出斜坡限制，使黑线从边缘回到中间后能够立即回正。
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
    s_cfg.turn_max_cps = CONTROL_LINE_TURN_MAX_CPS;
    s_cfg.search_turn_cps = CONTROL_LINE_SEARCH_TURN_CPS;
    s_cfg.kp = CONTROL_LINE_KP;
    s_cfg.kd = CONTROL_LINE_KD;
    s_cfg.error_deadband = CONTROL_LINE_ERROR_DEADBAND;
    s_cfg.speed_full_error = CONTROL_LINE_SPEED_FULL_ERROR;
    s_cfg.speed_min_error = CONTROL_LINE_SPEED_MIN_ERROR;
    s_cfg.search_timeout_ms = CONTROL_LINE_SEARCH_TIMEOUT_MS;
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
    s_last_error = 0;
    s_derivative_error = 0;
    s_adaptive_linear = 0;
    s_turn_unclamped = 0;
    s_turn_saturated = 0U;
    s_last_linear_cmd = 0;
    s_last_turn_cmd = 0;
    s_target_linear = 0;
    s_target_turn = 0;
    s_last_line_direction = 1;
    s_search_direction = 1;
    s_lost_samples = 0U;
    s_lost_start_ms = 0U;
}

BSP_Status_t LineTrack_GetInfo(LineTrack_Info_t *info)
{
    uint32_t now;

    if (info == 0) return BSP_PARAM;

    now = BSP_GET_TICK();

    info->mode = s_mode;
    info->raw_error = s_raw_error;
    info->filtered_error = s_error_filt;
    info->derivative_error = s_derivative_error;
    info->adaptive_linear_cps = s_adaptive_linear;
    info->turn_unclamped_cps = s_turn_unclamped;
    info->turn_saturated = s_turn_saturated;
    info->target_linear_cps = s_target_linear;
    info->target_turn_cps = s_target_turn;
    info->output_linear_cps = s_last_linear_cmd;
    info->output_turn_cps = s_last_turn_cmd;
    info->lost_samples = s_lost_samples;
    info->reacquire_samples = 0U;
    info->search_phase = 0U;
    info->search_direction = s_search_direction;
    info->lost_ms = (s_lost_samples == 0U) ? 0U :
                    (uint32_t)(now - s_lost_start_ms);

    return BSP_OK;
}

void LineTrack_Compute(const LineDetect_Result_t *line,
                       LineTrack_Output_t *out)
{
    uint32_t now;
    uint8_t was_searching;
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

    now = BSP_GET_TICK();
    s_raw_error = line->error_x1000;
    s_derivative_error = 0;
    s_adaptive_linear = 0;
    s_turn_unclamped = 0;
    s_turn_saturated = 0U;

    /* 找线超时后保持无效输出，必须由上层重新启动循迹。 */
    if (s_mode == LINE_TRACK_MODE_FAILSAFE) {
        return;
    }

    /*
     * 丢线后只做一件事：沿最后一次看到黑线的方向原地找线。
     * 不反复切换方向，不逐级增加速度，行为简单且便于现场判断。
     */
    if (line->type == LINE_TYPE_LOST) {
        if (s_lost_samples == 0U) {
            s_lost_start_ms = now;
            s_search_direction =
                LineTrack_DirectionFromError(s_last_error);
            if (s_search_direction == 0) {
                s_search_direction = 1;
            }
        }

        if (s_lost_samples < 0xFFFFU) {
            s_lost_samples++;
        }

        if ((uint32_t)(now - s_lost_start_ms) >=
            s_cfg.search_timeout_ms) {
            s_mode = LINE_TRACK_MODE_FAILSAFE;
            LineTrack_SetOutput(0, 0, 0U, out);
            return;
        }

        s_mode = LINE_TRACK_MODE_SEARCH;
        s_turn_unclamped = (int32_t)s_search_direction *
                           (int32_t)s_cfg.search_turn_cps;
        LineTrack_SetOutput(0,
                            (int16_t)(s_search_direction *
                                      s_cfg.search_turn_cps),
                            1U,
                            out);
        return;
    }

    was_searching = (s_mode == LINE_TRACK_MODE_SEARCH) ? 1U : 0U;
    s_mode = LINE_TRACK_MODE_TRACK;
    s_lost_samples = 0U;
    s_lost_start_ms = 0U;

    /*
     * 十字和全黑区域在基础模式下统一低速直行。
     * 左右分支不在这里强制直行，仍由下面的 P/PD 根据实际误差处理，
     * 避免把右直角误判成宽线后继续向前冲。
     */
    if ((line->type == LINE_TYPE_CROSS) ||
        (line->type == LINE_TYPE_FULL_BLACK)) {
        s_error_filt = 0;
        s_last_error = 0;
        s_adaptive_linear = s_cfg.cross_speed_cps;
        LineTrack_SetOutput(s_cfg.cross_speed_cps, 0, 1U, out);
        return;
    }

    /*
     * 基础版不做误差低通滤波，直接使用当前帧误差。
     * 这样黑线从最外侧返回中间时，不会继续保留旧方向的大误差。
     */
    error = line->error_x1000;
    if ((error > (int16_t)(-s_cfg.error_deadband)) &&
        (error < s_cfg.error_deadband)) {
        error = 0;
    }

    s_error_filt = error;

    /* 刚刚重新找到线时不使用搜索前的旧误差计算微分。 */
    if (was_searching != 0U) {
        s_last_error = error;
        d_error = 0;
    } else {
        d_error = (int16_t)(error - s_last_error);
        s_last_error = error;
    }
    s_derivative_error = d_error;

    direction = LineTrack_DirectionFromError(error);
    if (direction != 0) {
        s_last_line_direction = direction;
    }

    target_linear = LineTrack_GetAdaptiveSpeed(error);
    s_adaptive_linear = target_linear;
    turn_f = -(s_cfg.kp * (float)error +
               s_cfg.kd * (float)d_error);
    s_turn_unclamped = (int32_t)turn_f;
    if ((turn_f > (float)s_cfg.turn_max_cps) ||
        (turn_f < (float)(-s_cfg.turn_max_cps))) {
        s_turn_saturated = 1U;
    }
    target_turn = LineTrack_LimitFloat(turn_f,
                                       (int16_t)(-s_cfg.turn_max_cps),
                                       s_cfg.turn_max_cps);

    /*
     * 普通循迹时不让内侧车轮反转：
     * |turn| == linear 时内侧轮停止，已经能够形成很强的转向。
     */
    if (target_turn > target_linear) {
        target_turn = target_linear;
        s_turn_saturated = 1U;
    }
    if (target_turn < (int16_t)(-target_linear)) {
        target_turn = (int16_t)(-target_linear);
        s_turn_saturated = 1U;
    }

    LineTrack_SetOutput(target_linear, target_turn, 1U, out);
}
