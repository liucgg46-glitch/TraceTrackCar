#include "line_follow_app.h"
#include "drv_gray_4051.h"
#include "line_detect.h"
#include "line_track.h"
#include "chassis.h"
#include "route_manager.h"

static LineFollow_Info_t s_lf;

void LineFollow_Init(void)
{
    uint8_t i;

    s_lf.state = LINE_FOLLOW_STOP;
    for (i = 0U; i < LINE_DETECT_SENSOR_NUM; i++) {
        s_lf.raw[i] = 0U;
    }

    LineDetect_Init();
    LineTrack_Init();
		RouteManager_Init();
}

void LineFollow_Start(void)
{
    s_lf.state = LINE_FOLLOW_RUN;
}

void LineFollow_Stop(void)
{
    s_lf.state = LINE_FOLLOW_STOP;
    Chassis_Stop();
}

void LineFollow_Update(void)
{
    const LineDetect_Result_t *res;

    if (Drv_Gray4051_IsValid() == 0U) {
        return;
    }

    (void)Drv_Gray4051_GetFiltArray(s_lf.raw, LINE_DETECT_SENSOR_NUM);
    LineDetect_Update(s_lf.raw);
    res = LineDetect_GetResultPtr();
    s_lf.detect = *res;

   RouteManager_Update(res, &s_lf.output);

    if (s_lf.state == LINE_FOLLOW_RUN) {
        if (s_lf.output.valid) {
            Chassis_SetSpeed(s_lf.output.linear_cps, s_lf.output.turn_cps);
        } else {
            Chassis_Stop();
        }
    }
}

LineFollow_State_t LineFollow_GetState(void)
{
    return s_lf.state;
}

BSP_Status_t LineFollow_GetInfo(LineFollow_Info_t *info)
{
    if (info == 0) return BSP_PARAM;
    *info = s_lf;
    return BSP_OK;
}

void LineTrack_Update(void)
{
    LineFollow_Update();
}
