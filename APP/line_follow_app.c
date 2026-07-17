
#include "line_follow_app.h"
#include "drv_gray_sensor.h"
#include "line_detect.h"
#include "line_track.h"
#include "chassis.h"
#include "route_manager.h"
#include "sensor_manager.h"

static LineFollow_Info_t s_lf;

void LineFollow_Init(void)
{
    uint8_t i;

    s_lf.state = LINE_FOLLOW_STOP;
    for (i = 0U; i < LINE_DETECT_SENSOR_NUM; i++) {
        s_lf.raw[i] = 0U;
    }
    s_lf.output.linear_cps = 0;
    s_lf.output.turn_cps = 0;
    s_lf.output.valid = 0U;

    LineDetect_Init();
    LineTrack_Init();
    RouteManager_Init();
}

void LineFollow_Start(void)
{
    if (Sensor_IsImuReadyForMotion() == 0U) {
        LineFollow_Stop();
        return;
    }

    /* Start each run with a clean route state and no stale motion action. */
    RouteManager_Reset();
    s_lf.output.linear_cps = 0;
    s_lf.output.turn_cps = 0;
    s_lf.output.valid = 0U;
    s_lf.state = LINE_FOLLOW_RUN;
}

void LineFollow_Stop(void)
{
    s_lf.state = LINE_FOLLOW_STOP;
    RouteManager_Reset();
    s_lf.output.linear_cps = 0;
    s_lf.output.turn_cps = 0;
    s_lf.output.valid = 0U;
    Chassis_Stop();
}

void LineFollow_Update(void)
{
    const LineDetect_Result_t *res;
    Route_ControlMode_t control;

    if ((s_lf.state == LINE_FOLLOW_RUN) &&
        (Sensor_IsImuReadyForMotion() == 0U)) {
        LineFollow_Stop();
        return;
    }

    if (Drv_GraySensor_IsOnline() == 0U) {
        if (s_lf.state == LINE_FOLLOW_RUN) {
            LineFollow_Stop();
        }
        return;
    }

    (void)Drv_GraySensor_GetFiltArray(s_lf.raw, LINE_DETECT_SENSOR_NUM);
    LineDetect_Update(s_lf.raw);
    res = LineDetect_GetResultPtr();
    s_lf.detect = *res;

    /* Keep detection available for UI while stopped, but do not advance the
     * route state machine until the vehicle is actually running. */
    if (s_lf.state != LINE_FOLLOW_RUN) {
        s_lf.output.linear_cps = 0;
        s_lf.output.turn_cps = 0;
        s_lf.output.valid = 0U;
        return;
    }

    /*
     * 只通过 RouteManager 输出。
     * 当前 ROUTE_PROFILE_BASIC 内部会调用 LineTrack_Compute。
     * 以后切换到 HJduino 状态机，也从这里统一接管。
     */
    control = RouteManager_Update(res, &s_lf.output);

    if (control == ROUTE_CONTROL_MOTION) {
        /* Motion_Update() exclusively owns the chassis during this state. */
        return;
    }

    if ((control == ROUTE_CONTROL_LINE_TRACK) &&
        (s_lf.output.valid != 0U)) {
        Chassis_SetSpeed(s_lf.output.linear_cps, s_lf.output.turn_cps);
        return;
    }

    /* Invalid route output or action failure is fail-safe. */
    LineFollow_Stop();
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
