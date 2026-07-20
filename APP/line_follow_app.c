
#include "line_follow_app.h"
#include "drv_gray_sensor.h"
#include "line_detect.h"
#include "line_track.h"
#include "chassis.h"
#include "motion_action.h"
#include "route_manager.h"
#include "sensor_manager.h"

static LineFollow_Info_t s_lf;

static Route_ActionState_t LineFollow_GetRouteActionState(void)
{
    switch (Motion_GetState()) {
    case MOTION_RUNNING:
        return ROUTE_ACTION_STATE_RUNNING;
    case MOTION_DONE:
        return ROUTE_ACTION_STATE_DONE;
    case MOTION_ERROR:
        return ROUTE_ACTION_STATE_ERROR;
    case MOTION_IDLE:
    default:
        return ROUTE_ACTION_STATE_IDLE;
    }
}

static BSP_Status_t LineFollow_StartRouteAction(
    const Route_ActionRequest_t *request)
{
    if (request == 0) {
        return BSP_PARAM;
    }

    switch (request->type) {
    case ROUTE_ACTION_GO_DISTANCE:
        return Motion_GoDistance(request->distance_mm, request->speed_cps);
    case ROUTE_ACTION_TURN_ANGLE:
        return Motion_TurnAngle(request->angle_deg);
    case ROUTE_ACTION_NONE:
    default:
        return BSP_PARAM;
    }
}

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
    RouteManager_Init();
}

void LineFollow_Start(void)
{
    if (s_lf.state == LINE_FOLLOW_RUN) {
        LineFollow_Stop();
    }

    if (Sensor_IsImuReadyForMotion() == 0U) {
        return;
    }

    /* 每次启动都清除上一次路线动作和路线状态。 */
    Motion_Stop();
    RouteManager_Reset();
    if (Chassis_AcquireControl(CHASSIS_OWNER_LINE_FOLLOW) != BSP_OK) {
        return;
    }
    s_lf.output.linear_cps = 0;
    s_lf.output.turn_cps = 0;
    s_lf.output.valid = 0U;
    s_lf.state = LINE_FOLLOW_RUN;
}

void LineFollow_Stop(void)
{
    s_lf.state = LINE_FOLLOW_STOP;
    Motion_Stop();
    Chassis_EmergencyStop();
    RouteManager_Reset();
    s_lf.output.linear_cps = 0;
    s_lf.output.turn_cps = 0;
    s_lf.output.valid = 0U;
}

void LineFollow_Update(void)
{
    const LineDetect_Result_t *res;
    Route_ControlMode_t control;
    Route_ActionFeedback_t feedback;
    Route_ActionRequest_t request;

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
    feedback.state = LineFollow_GetRouteActionState();
    control = RouteManager_Update(res,
                                  &feedback,
                                  &s_lf.output,
                                  &request);

    if (control == ROUTE_CONTROL_MOTION) {
        if (request.type != ROUTE_ACTION_NONE) {
            if (Chassis_ReleaseControl(CHASSIS_OWNER_LINE_FOLLOW) != BSP_OK) {
                LineFollow_Stop();
                return;
            }
            if (LineFollow_StartRouteAction(&request) != BSP_OK) {
                LineFollow_Stop();
                return;
            }
        }
        return;
    }

    if ((control == ROUTE_CONTROL_LINE_TRACK) &&
        (s_lf.output.valid != 0U)) {
        if (Chassis_AcquireControl(CHASSIS_OWNER_LINE_FOLLOW) != BSP_OK) {
            LineFollow_Stop();
            return;
        }
        if (Chassis_SetSpeed(CHASSIS_OWNER_LINE_FOLLOW,
                             s_lf.output.linear_cps,
                             s_lf.output.turn_cps) != BSP_OK) {
            LineFollow_Stop();
        }
        return;
    }

    /* 路线输出无效或动作失败时进入安全停车。 */
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
