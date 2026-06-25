#include "line_track.h"

static LineTrack_Config_t s_cfg;
static int16_t s_last_error;

static int16_t LimitI16(int32_t x, int16_t min_v, int16_t max_v)
{
    if (x > max_v) return max_v;
    if (x < min_v) return min_v;
    return (int16_t)x;
}
static int16_t AbsI16(int16_t x)
{
    return (x >= 0) ? x : (int16_t)(-x);
}

static int16_t LineTrack_GetAdaptiveSpeed(int16_t error)
{
    int16_t abs_e = AbsI16(error);

    if (abs_e < 180) {
        return s_cfg.base_speed_cps;
    } else if (abs_e < 450) {
        return (int16_t)(s_cfg.base_speed_cps * 75 / 100);
    } else if (abs_e < 750) {
        return (int16_t)(s_cfg.base_speed_cps * 55 / 100);
    } else {
        return (int16_t)(s_cfg.base_speed_cps * 40 / 100);
    }
}
static int16_t LimitFloatToI16(float x, int16_t min_v, int16_t max_v)
{
    if (x > (float)max_v) return max_v;
    if (x < (float)min_v) return min_v;
    return (int16_t)x;
}

void LineTrack_Init(void)
{
    s_cfg.base_speed_cps = LINE_TRACK_BASE_SPEED_CPS;
    s_cfg.cross_speed_cps = LINE_TRACK_CROSS_SPEED_CPS;
    s_cfg.lost_speed_cps = LINE_TRACK_LOST_SPEED_CPS;
    s_cfg.search_turn_cps = LINE_TRACK_SEARCH_TURN_CPS;
    s_cfg.turn_max_cps = LINE_TRACK_TURN_MAX_CPS;
    s_cfg.kp = LINE_TRACK_KP;
    s_cfg.kd = LINE_TRACK_KD;
    s_last_error = 0;
}

void LineTrack_SetConfig(const LineTrack_Config_t *cfg)
{
    if (cfg == 0) return;
    s_cfg = *cfg;
}

BSP_Status_t LineTrack_GetConfig(LineTrack_Config_t *cfg)
{
    if (cfg == 0) return BSP_PARAM;
    *cfg = s_cfg;
    return BSP_OK;
}

void LineTrack_Compute(const LineDetect_Result_t *line, LineTrack_Output_t *out)
{
    int16_t error;
    int16_t d_error;
    float turn_f;

    if ((line == 0) || (out == 0)) return;

    out->linear_cps = 0;
    out->turn_cps = 0;
    out->valid = 0U;

    if (line->type == LINE_TYPE_LOST) {
        out->linear_cps = s_cfg.lost_speed_cps;

        if (s_last_error >= 0) {
            out->turn_cps = (int16_t)(-s_cfg.search_turn_cps);
        } else {
            out->turn_cps = s_cfg.search_turn_cps;
        }

        out->valid = 1U;
        return;
    }

    error = line->error_x1000;

    /*
     * 很小的误差直接当 0，减少直线左右轻微抖动
     */
    if ((error > -60) && (error < 60)) {
        error = 0;
    }

    d_error = (int16_t)(error - s_last_error);
    s_last_error = error;

    /*
     * 约定：
     * error > 0 表示线在右边，车应右转，所以 turn_cps 为负
     * error < 0 表示线在左边，车应左转，所以 turn_cps 为正
     */
    turn_f = -(s_cfg.kp * (float)error + s_cfg.kd * (float)d_error);

    out->turn_cps = LimitFloatToI16(turn_f,
                                    (int16_t)(-s_cfg.turn_max_cps),
                                    s_cfg.turn_max_cps);

#if LINE_TRACK_SLOW_ON_CROSS
    if ((line->type == LINE_TYPE_CROSS) ||
        (line->type == LINE_TYPE_LEFT_BRANCH) ||
        (line->type == LINE_TYPE_RIGHT_BRANCH) ||
        (line->type == LINE_TYPE_FULL_BLACK)) {
        out->linear_cps = s_cfg.cross_speed_cps;
    } else
#endif
    {
        out->linear_cps = LineTrack_GetAdaptiveSpeed(error);
    }

    out->linear_cps = LimitI16(out->linear_cps, -3000, 3000);
    out->valid = 1U;
}