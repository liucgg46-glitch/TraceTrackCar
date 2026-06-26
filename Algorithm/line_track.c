#include "line_track.h"

static uint8_t s_cross_hold_cnt;
static LineTrack_Config_t s_cfg;
static int16_t s_last_error;
static int16_t s_error_filt;
static int16_t s_last_linear_cmd;
static int16_t s_last_turn_cmd;

static int16_t LimitI16(int32_t x, int16_t min_v, int16_t max_v)
{
    if (x > max_v) return max_v;
    if (x < min_v) return min_v;
    return (int16_t)x;
}

static int16_t LimitFloatToI16(float x, int16_t min_v, int16_t max_v)
{
    if (x > (float)max_v) return max_v;
    if (x < (float)min_v) return min_v;
    return (int16_t)x;
}

static int16_t AbsI16(int16_t x)
{
    return (x >= 0) ? x : (int16_t)(-x);
}

static int16_t LineTrack_GetAdaptiveSpeed(int16_t error)
{
    int16_t abs_err = AbsI16(error);
    int32_t speed;

    /*
     * base = 3000 时：
     * abs_err = 0     -> 3000
     * abs_err = 300   -> 2730
     * abs_err = 600   -> 2460
     * abs_err = 1000  -> 2100
     * abs_err = 1500  -> 1650
     * abs_err = 2000  -> 1200
     * abs_err = 2500  -> 750，随后被限到 850
     */
    speed = (int32_t)s_cfg.base_speed_cps - ((int32_t)abs_err * 90 / 100);

    if (speed > s_cfg.base_speed_cps) {
        speed = s_cfg.base_speed_cps;
    }

    if (speed < 850) {
        speed = 850;
    }

    return (int16_t)speed;
}
/* 交叉区沿用上一次偏差方向，避免在十字 / 8 字交叉处乱选线 */
static int16_t LineTrack_HoldTurnByLastError(uint8_t div)
{
    if (div == 0U) div = 4U;

    if (s_last_error > 300) {
        return (int16_t)(-s_cfg.turn_max_cps / div);  /* 上次偏右，轻微右转 */
    } else if (s_last_error < -300) {
        return (int16_t)(s_cfg.turn_max_cps / div);   /* 上次偏左，轻微左转 */
    } else {
        return 0;
    }
}

static int16_t LineTrack_RampI16(int16_t last, int16_t target, int16_t step)
{
    int32_t diff = (int32_t)target - (int32_t)last;

    if (diff > step) {
        return (int16_t)(last + step);
    } else if (diff < -step) {
        return (int16_t)(last - step);
    } else {
        return target;
    }
}

void LineTrack_Init(void)
{
    s_cfg.base_speed_cps   = LINE_TRACK_BASE_SPEED_CPS;
    s_cfg.cross_speed_cps  = LINE_TRACK_CROSS_SPEED_CPS;
    s_cfg.lost_speed_cps   = LINE_TRACK_LOST_SPEED_CPS;
    s_cfg.search_turn_cps  = LINE_TRACK_SEARCH_TURN_CPS;
    s_cfg.turn_max_cps     = LINE_TRACK_TURN_MAX_CPS;
    s_cfg.kp               = LINE_TRACK_KP;
    s_cfg.kd               = LINE_TRACK_KD;

    s_last_error      = 0;
    s_cross_hold_cnt  = 0U;
    s_error_filt      = 0;
    s_last_linear_cmd = 0;
    s_last_turn_cmd   = 0;
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
    int16_t target_linear;
    int16_t target_turn;

    if ((line == 0) || (out == 0)) return;

    out->linear_cps = 0;
    out->turn_cps = 0;
    out->valid = 0U;

    /* 1. 丢线：按最后误差方向原地找线 */
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

//    /* 2. 交叉保持期：短时间低速穿越，避免 8 字交叉处乱选线 */
//    if (s_cross_hold_cnt > 0U) {
//        s_cross_hold_cnt--;

//        out->linear_cps = s_cfg.cross_speed_cps;
//        out->turn_cps = LineTrack_HoldTurnByLastError(4U);
//        out->valid = 1U;
//        return;
//    }

//    /* 3. CROSS / FULL_BLACK：进入交叉保持 */
//    if ((line->type == LINE_TYPE_CROSS) ||
//        (line->type == LINE_TYPE_FULL_BLACK)) {

//        s_cross_hold_cnt = 25U;   /* 25 * 10ms = 250ms */

//        out->linear_cps = s_cfg.cross_speed_cps;
//        out->turn_cps = LineTrack_HoldTurnByLastError(4U);
//        out->valid = 1U;
//        return;
//    }

//    /*
//     * 4. 直角 / 大弯边缘补救
//     * mask 低位是左侧传感器，高位是右侧传感器。
//     * 如果实际测试发现左右反了，检查 DRV_GRAY_4051_INDEX_REVERSE。
//     */
//    if ((line->black_mask & 0x03U) != 0U) {
//        /* 最左侧 0 / 1 路看到线：强制左转 */
//        out->linear_cps = 450;
//        out->turn_cps = (int16_t)(s_cfg.turn_max_cps * 3 / 4);
//        out->valid = 1U;
//        s_last_error = LINE_DETECT_ERR_LEFT_MAX;
//        return;
//    }

//    if ((line->black_mask & 0xC0U) != 0U) {
//        /* 最右侧 6 / 7 路看到线：强制右转 */
//        out->linear_cps = 450;
//        out->turn_cps = (int16_t)(-s_cfg.turn_max_cps * 3 / 4);
//        out->valid = 1U;
//        s_last_error = LINE_DETECT_ERR_RIGHT_MAX;
//        return;
//    }

//    /*
//     * 5. LEFT_BRANCH / RIGHT_BRANCH：
//     * 对这个 HJduino 8 字交叉赛道，先不要直接强制左 / 右选路。
//     * 先低速保持穿越，避免在中间环形交叉处误入支路。
//     */
//    if ((line->type == LINE_TYPE_LEFT_BRANCH) ||
//        (line->type == LINE_TYPE_RIGHT_BRANCH)) {

//        s_cross_hold_cnt = 12U;   /* 12 * 10ms = 120ms */

//        out->linear_cps = s_cfg.cross_speed_cps;
//        out->turn_cps = LineTrack_HoldTurnByLastError(4U);
//        out->valid = 1U;
//        return;
//    }

    /* 6. 普通单线 PD 循迹 */

    /* error 一阶低通滤波，减少灰度跳变导致的顿挫 */
  s_error_filt = (int16_t)((3 * (int32_t)s_error_filt + line->error_x1000) / 4);

error = s_error_filt;

   if ((error > -80) && (error < 80)) {
    error = 0;
     }

    d_error = (int16_t)(error - s_last_error);
    s_last_error = error;

    turn_f = -(s_cfg.kp * (float)error + s_cfg.kd * (float)d_error);

    target_turn = LimitFloatToI16(turn_f,
                                  (int16_t)(-s_cfg.turn_max_cps),
                                  s_cfg.turn_max_cps);

    target_linear = LineTrack_GetAdaptiveSpeed(error);
    target_linear = LimitI16(target_linear, -3000, 3000);

    /* 每 10ms 限制变化量，让输出更丝滑 */
    out->linear_cps = LineTrack_RampI16(s_last_linear_cmd, target_linear, 100);
    out->turn_cps   = LineTrack_RampI16(s_last_turn_cmd,   target_turn,   140);

    s_last_linear_cmd = out->linear_cps;
    s_last_turn_cmd   = out->turn_cps;

    out->linear_cps = LimitI16(out->linear_cps, -3000, 3000);
    out->turn_cps = LimitI16(out->turn_cps,
                             (int16_t)(-s_cfg.turn_max_cps),
                             s_cfg.turn_max_cps);

    out->valid = 1U;
}