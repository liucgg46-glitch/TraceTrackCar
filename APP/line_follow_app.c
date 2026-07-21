
#include "line_follow_app.h"
#include "drv_gray_sensor.h"
#include "line_detect.h"
#include "line_track.h"
#include "chassis.h"
#include "motion_action.h"
#include "route_manager.h"

static LineFollow_Info_t s_lf;
static uint8_t s_route_action_active;
static uint8_t s_auto_start_pending;

static void LineFollow_ClearOutput(void)
{
    s_lf.output.linear_cps = 0;
    s_lf.output.turn_cps = 0;
    s_lf.output.valid = 0U;
}

static void LineFollow_ReleaseOwnedControl(void)
{
    if (s_route_action_active != 0U) {
        Motion_Stop();
        s_route_action_active = 0U;
    }

    if (Chassis_GetOwner() == CHASSIS_OWNER_LINE_FOLLOW) {
        (void)Chassis_ReleaseControl(CHASSIS_OWNER_LINE_FOLLOW);
    }
}

/*
 * 异常退出时保留Route和LineTrack诊断状态，便于显示FAILSAFE等原因。
 * 下一次成功启动前，LineFollow_Start()仍会统一复位全部路线状态。
 */
static void LineFollow_Abort(void)
{
    s_lf.state = LINE_FOLLOW_STOP;
    LineFollow_ReleaseOwnedControl();
    LineFollow_ClearOutput();
}

static Route_ActionState_t LineFollow_GetRouteActionState(void)
{
    if (s_route_action_active == 0U) {
        return ROUTE_ACTION_STATE_IDLE;
    }

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
    LineFollow_ClearOutput();
    s_route_action_active = 0U;

    LineDetect_Init();
    RouteManager_Init();
		
}

BSP_Status_t LineFollow_Start(void)
{
    if (s_lf.state == LINE_FOLLOW_RUN) {
        return BSP_BUSY;
    }

    if (Drv_GraySensor_IsOnline() == 0U) {
        return BSP_ERROR;
    }

    if (Chassis_AcquireControl(CHASSIS_OWNER_LINE_FOLLOW) != BSP_OK) {
        return BSP_BUSY;
    }

    /* 成功取得底盘后才复位本模块状态，不影响其他控制者。 */
    RouteManager_Reset();
    LineFollow_ClearOutput();
    s_route_action_active = 0U;
    s_lf.state = LINE_FOLLOW_RUN;
    return BSP_OK;
}

void LineFollow_Stop(void)
{
    s_lf.state = LINE_FOLLOW_STOP;
    LineFollow_ReleaseOwnedControl();
    RouteManager_Reset();
    LineFollow_ClearOutput();
}

void LineFollow_Update(void)
{
    const LineDetect_Result_t *res;
    Route_ControlMode_t control;
    Route_ActionFeedback_t feedback;
    Route_ActionRequest_t request;

    if (Drv_GraySensor_IsOnline() == 0U) {
        if (s_lf.state == LINE_FOLLOW_RUN) {
            LineFollow_Stop();
        }
        return;
    }
				/* 等灰度传感器完成第一次采样后，只自动启动一次 */
		if (s_auto_start_pending != 0U) {
				if (LineFollow_Start() == BSP_OK) {
						s_auto_start_pending = 0U;
				} else {
						return;
				}
		}

    (void)Drv_GraySensor_GetFiltArray(s_lf.raw, LINE_DETECT_SENSOR_NUM);
    LineDetect_Update(s_lf.raw);
    res = LineDetect_GetResultPtr();
    s_lf.detect = *res;

    /* 停车时继续刷新识别结果供界面查看，但不推进赛道状态机。 */
    if (s_lf.state != LINE_FOLLOW_RUN) {
        LineFollow_ClearOutput();
        return;
    }

    /*
     * 只通过 RouteManager 输出。
     * 当前 ROUTE_PROFILE_BASIC 内部会调用 LineTrack_Compute。
     * 后续赛道方案也从这里统一接管，不直接操作底盘或 Motion。
     */
    feedback.state = LineFollow_GetRouteActionState();
    control = RouteManager_Update(res,
                                  &feedback,
                                  &s_lf.output,
                                  &request);

    if (control == ROUTE_CONTROL_MOTION) {
        if (request.type != ROUTE_ACTION_NONE) {
            if (s_route_action_active != 0U) {
                LineFollow_Abort();
                return;
            }
            if (Chassis_ReleaseControl(CHASSIS_OWNER_LINE_FOLLOW) != BSP_OK) {
                LineFollow_Abort();
                return;
            }
            if (LineFollow_StartRouteAction(&request) != BSP_OK) {
                LineFollow_Abort();
                return;
            }
            s_route_action_active = 1U;
        } else if (s_route_action_active == 0U) {
            /* 首次切入动作控制时必须同时给出有效动作请求。 */
            LineFollow_Abort();
        }
        return;
    }

    if ((control == ROUTE_CONTROL_LINE_TRACK) &&
        (s_lf.output.valid != 0U)) {
        if (s_route_action_active != 0U) {
            if (Motion_GetState() != MOTION_DONE) {
                LineFollow_Abort();
                return;
            }
            Motion_Stop();
            s_route_action_active = 0U;
        }
        if (Chassis_AcquireControl(CHASSIS_OWNER_LINE_FOLLOW) != BSP_OK) {
            LineFollow_Abort();
            return;
        }
        if (Chassis_SetSpeed(CHASSIS_OWNER_LINE_FOLLOW,
                             s_lf.output.linear_cps,
                             s_lf.output.turn_cps) != BSP_OK) {
            LineFollow_Abort();
        }
        return;
    }

    /* 路线输出无效或动作失败时进入安全停车。 */
    LineFollow_Abort();
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
