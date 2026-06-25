#include "route_profile_hjduino.h"
#include "line_track.h"

typedef enum {
    HJD_ROUTE_NORMAL_BEFORE_LOOP = 0,
    HJD_ROUTE_ENTER_LOOP_RIGHT,
    HJD_ROUTE_IN_LOOP,
    HJD_ROUTE_EXIT_LOOP,
    HJD_ROUTE_NORMAL_AFTER_LOOP,
    HJD_ROUTE_RIGHT_ANGLE
} HJduinoRoute_State_t;

static HJduinoRoute_State_t s_hjd_state;
static uint16_t s_hjd_cnt;
static uint16_t s_hjd_run_cnt;
static uint8_t s_hjd_lap_count;

void HJduinoRoute_Init(void)
{
    HJduinoRoute_Reset();
}

void HJduinoRoute_Reset(void)
{
    s_hjd_state = HJD_ROUTE_NORMAL_BEFORE_LOOP;
    s_hjd_cnt = 0U;
    s_hjd_run_cnt = 0U;
    s_hjd_lap_count = 0U;
}

static uint8_t HJduinoRoute_IsLoopEntry(const LineDetect_Result_t *line)
{
    if (line == 0) {
        return 0U;
    }

    /*
     * 环岛入口相当于右转进入。
     * 先用右分支 / 交叉 / 右侧边缘作为入口判断。
     */
    if (line->type == LINE_TYPE_RIGHT_BRANCH) {
        return 1U;
    }

    if (line->type == LINE_TYPE_CROSS) {
        return 1U;
    }

    if (line->type == LINE_TYPE_FULL_BLACK) {
        return 1U;
    }

    /*
     * 右侧 6/7 路看到线，也认为可能到达右侧入口。
     * 如果实车发现左右反了，再改成 0x03U。
     */
    if ((line->black_mask & 0xC0U) != 0U) {
        return 1U;
    }

    return 0U;
}

void HJduinoRoute_Update(const LineDetect_Result_t *line, LineTrack_Output_t *out)
{
    if ((line == 0) || (out == 0)) {
        return;
    }

    s_hjd_run_cnt++;

    switch (s_hjd_state) {
    case HJD_ROUTE_NORMAL_BEFORE_LOOP:
        /*
         * 前面直线、波浪、大圆弧都普通循迹。
         * 这里用时间门限防止刚启动时误判。
         * 300 表示 300 * 10ms = 3s。
         */
        if ((s_hjd_run_cnt > 300U) && HJduinoRoute_IsLoopEntry(line)) {
            s_hjd_state = HJD_ROUTE_ENTER_LOOP_RIGHT;
            s_hjd_cnt = 0U;

            out->linear_cps = 550;
            out->turn_cps = -650;
							out->valid = 1U;
            return;
        }

        LineTrack_Compute(line, out);
        break;

    case HJD_ROUTE_ENTER_LOOP_RIGHT:
        /*
         * 强制右转切入环岛。
         * 40 表示 400ms。
         */
        s_hjd_cnt++;

        out->linear_cps = 450;
        out->turn_cps = -750;
        out->valid = 1U;

        if (s_hjd_cnt >= 70U) {
            s_hjd_state = HJD_ROUTE_IN_LOOP;
            s_hjd_cnt = 0U;
        }
        break;

    case HJD_ROUTE_IN_LOOP:
        /*
         * 先进入环岛后继续普通循迹。
         * 后面再加“出环岛”判断。
         */
        LineTrack_Compute(line, out);
        break;

    case HJD_ROUTE_EXIT_LOOP:
        LineTrack_Compute(line, out);
        break;

    case HJD_ROUTE_NORMAL_AFTER_LOOP:
        LineTrack_Compute(line, out);
        break;

    case HJD_ROUTE_RIGHT_ANGLE:
        LineTrack_Compute(line, out);
        break;

    default:
        s_hjd_state = HJD_ROUTE_NORMAL_BEFORE_LOOP;
        s_hjd_cnt = 0U;
        LineTrack_Compute(line, out);
        break;
    }
}