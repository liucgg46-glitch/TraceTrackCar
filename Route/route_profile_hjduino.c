#include "route_profile_hjduino.h"
#include "line_track.h"
#include "line_detect.h"

typedef enum {
    HJD_ROUTE_NORMAL = 0,
    HJD_ROUTE_ENTER_ISLAND
} HJduinoRoute_State_t;

static HJduinoRoute_State_t s_hjd_state;
static uint16_t s_hjd_cnt;
static uint16_t s_hjd_run_cnt;
static uint8_t s_hjd_enter_done;

void HJduinoRoute_Init(void)
{
    HJduinoRoute_Reset();
}

void HJduinoRoute_Reset(void)
{
    s_hjd_state = HJD_ROUTE_NORMAL;
    s_hjd_cnt = 0U;
    s_hjd_run_cnt = 0U;
    s_hjd_enter_done = 0U;
}

static uint8_t HJduinoRoute_IsIslandEntry(const LineDetect_Result_t *line)
{
    if (line == 0) {
        return 0U;
    }

    /*
     * 环岛入口相当于右转进入。
     * 先用右分支 + 右侧传感器作为主要判断。
     */
    if (line->type == LINE_TYPE_RIGHT_BRANCH) {
        return 1U;
    }

    /*
     * 如果右侧 6/7 路看到黑线，认为可能到达环岛入口。
     * 如果实车发现左右反了，把 0xC0U 改成 0x03U。
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
    case HJD_ROUTE_NORMAL:
        /*
         * 前面直线、波浪、大圆弧都普通循迹。
         * s_hjd_run_cnt > 300U 用来避免刚启动或前面普通弯道误判。
         * 300 * 10ms = 3s。
         */
        if ((s_hjd_enter_done == 0U) &&
            (s_hjd_run_cnt > 300U) &&
            HJduinoRoute_IsIslandEntry(line)) {

            s_hjd_state = HJD_ROUTE_ENTER_ISLAND;
            s_hjd_cnt = 0U;

            out->linear_cps = 550;
            out->turn_cps = -650;    /* 右转进环岛 */
            out->valid = 1U;
            return;
        }

        LineTrack_Compute(line, out);
        break;

    case HJD_ROUTE_ENTER_ISLAND:
        /*
         * 强制右转切入环岛。
         * 40 * 10ms = 400ms。
         */
        s_hjd_cnt++;

        out->linear_cps = 550;
        out->turn_cps = -650;
        out->valid = 1U;

        if (s_hjd_cnt >= 40U) {
            s_hjd_state = HJD_ROUTE_NORMAL;
            s_hjd_cnt = 0U;
            s_hjd_enter_done = 1U;   /* 防止后面再次误触发进环岛 */
        }
        break;

    default:
        s_hjd_state = HJD_ROUTE_NORMAL;
        s_hjd_cnt = 0U;
        LineTrack_Compute(line, out);
        break;
    }
}