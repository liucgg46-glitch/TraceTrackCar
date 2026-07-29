
#include "line_follow_app.h"
#include "drv_gray_sensor.h"
#include "drv_encoder.h"
#include "line_detect.h"
#include "line_track.h"
#include "chassis.h"
#include "motion_action.h"
#include "route_manager.h"
#include "bsp_systick.h"

static LineFollow_Info_t s_lf;
static uint8_t s_route_action_active;

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
    RouteManager_Init(BSP_GetTickMs());
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
    RouteManager_Reset(BSP_GetTickMs());
    LineFollow_ClearOutput();
    s_route_action_active = 0U;
    s_lf.state = LINE_FOLLOW_RUN;
    return BSP_OK;
}

void LineFollow_Stop(void)
{
    s_lf.state = LINE_FOLLOW_STOP;
    LineFollow_ReleaseOwnedControl();
    RouteManager_Reset(BSP_GetTickMs());
    LineFollow_ClearOutput();
}

void LineFollow_StopPreserveRoute(void)
{
    LineFollow_Abort();
}

BSP_Status_t LineFollow_SetSpeedProfile(int16_t base_speed_cps,
                                        int16_t cross_speed_cps,
                                        int16_t min_track_speed_cps)
{
    return (LineTrack_SetSpeedProfile(base_speed_cps,
                                      cross_speed_cps,
                                      min_track_speed_cps) == PROJECT_OK) ?
           BSP_OK : BSP_PARAM;
}

void LineFollow_Update(void)
{
    const LineDetect_Result_t *res;
    Route_ControlMode_t control;
    Route_ActionFeedback_t feedback;
    Route_ActionRequest_t request;
    MotionState_t motion_state;
    uint32_t now_ms;

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

    /* 停车时继续刷新识别结果供界面查看，但不推进赛道状态机。 */
    if (s_lf.state != LINE_FOLLOW_RUN) {
        LineFollow_ClearOutput();
        return;
    }

    /*
     * 只通过 RouteManager 输出。
     * 当前选择的Route方案内部会按赛道需要调用LineTrack_Compute。
     * APP层只负责控制权仲裁，不承担具体赛道事件和整车任务流程。
     */
    feedback.state = LineFollow_GetRouteActionState();
    feedback.distance_mm =
        (Drv_Encoder_GetLeftTotalMm() +
         Drv_Encoder_GetRightTotalMm()) / 2;
    now_ms = BSP_GetTickMs();
    control = RouteManager_Update(res,
                                  &feedback,
                                  &s_lf.output,
                                  &request,
                                  now_ms);

    if (control == ROUTE_CONTROL_MOTION) {
        if (request.type != ROUTE_ACTION_NONE) {
            /*
             * 允许Route在一个动作DONE后立即提交下一个动作。
             * B题直角需要“前进到旋转中心→定角转弯”连续执行，
             * 中间不把底盘控制权交回普通循迹。
             */
            if (s_route_action_active != 0U) {
                if (Motion_GetState() != MOTION_DONE) {
                    LineFollow_Abort();
                    return;
                }
                Motion_Stop();
                s_route_action_active = 0U;
            } else if (Chassis_GetOwner() == CHASSIS_OWNER_LINE_FOLLOW) {
                if (Chassis_ReleaseControl(
                        CHASSIS_OWNER_LINE_FOLLOW) != BSP_OK) {
                    LineFollow_Abort();
                    return;
                }
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
            motion_state = Motion_GetState();

            /*
             * Route重新见线时可以提前结束正在运行的短距离动作，
             * 立即把控制权交回PD；动作ERROR或异常IDLE仍按故障处理。
             */
            if ((motion_state == MOTION_ERROR) ||
                (motion_state == MOTION_IDLE)) {
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

    if (control == ROUTE_CONTROL_STOP) {
        /*
         * Route层已经完成到达或要求停车。这里只停止循迹并释放控制权，
         * 保留Route诊断快照；到达鸣笛和整车状态迁移由TaskProfile负责。
         */
        LineFollow_Abort();
        return;
    }

    /* 路线错误、输出无效或动作失败时进入安全停车。 */
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
