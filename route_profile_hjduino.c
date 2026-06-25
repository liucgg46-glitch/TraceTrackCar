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
static uint8_t s_hjd_lap_count;

void HJduinoRoute_Init(void)
{
    HJduinoRoute_Reset();
}

void HJduinoRoute_Reset(void)
{
    s_hjd_state = HJD_ROUTE_NORMAL_BEFORE_LOOP;
    s_hjd_cnt = 0U;
    s_hjd_lap_count = 0U;
}

void HJduinoRoute_Update(const LineDetect_Result_t *line, LineTrack_Output_t *out)
{
    /* 第一版先不写复杂赛道逻辑，先保持和普通循迹一致 */
    (void)s_hjd_state;
    (void)s_hjd_cnt;
    (void)s_hjd_lap_count;

    LineTrack_Compute(line, out);
}