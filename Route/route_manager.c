#include "route_manager.h"
#include "route_config.h"
#include "route_profile_basic.h"
#include "route_profile_hjduino.h"
#include "motion_action.h"

/* 保存最近一次路线更新后，当前应该由谁控制底盘。 */
static Route_ControlMode_t s_route_control = ROUTE_CONTROL_STOP;

/*
 * 根据 route_config.h 中 ROUTE_PROFILE_SELECT 的值，
 * 在编译期选择并初始化一个路线方案。
 *
 * #if/#elif 属于预处理编译选择，不是运行时 if；
 * 最终固件中只会保留被选中的那套调用路径。
 */
void RouteManager_Init(void)
{
#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    BasicRoute_Init();
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_HJDUINO)
    HJduinoRoute_Init();
#else
#error "Invalid ROUTE_PROFILE_SELECT"
#endif
    s_route_control = ROUTE_CONTROL_STOP;
}

/*
 * 将整个路线模块恢复到初始状态。
 * Motion_Stop() 用来取消路线模块此前启动的非阻塞动作，
 * 然后再复位具体 profile 的内部计数器和状态。
 */
void RouteManager_Reset(void)
{
    /* Cancel a route-owned action before returning chassis ownership. */
    Motion_Stop();
#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    BasicRoute_Reset();
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_HJDUINO)
    HJduinoRoute_Reset();
#else
#error "Invalid ROUTE_PROFILE_SELECT"
#endif
    s_route_control = ROUTE_CONTROL_STOP;
}

/*
 * 路线管理器统一入口。
 * 它不直接解析具体赛道，只把调用转发给当前选中的 profile，
 * 并记录该 profile 返回的底盘控制权。
 */
Route_ControlMode_t RouteManager_Update(const LineDetect_Result_t *line,
                                        LineTrack_Output_t *out)
{
#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC)
    s_route_control = BasicRoute_Update(line, out);
#elif (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_HJDUINO)
    s_route_control = HJduinoRoute_Update(line, out);
#else
#error "Invalid ROUTE_PROFILE_SELECT"
#endif
    return s_route_control;
}

/*
 * 获取路线模块运行信息，主要用于调试打印或界面显示。
 * 先填充所有方案共有的信息，再按所选 profile 补充专用字段。
 */
BSP_Status_t RouteManager_GetInfo(RouteManager_Info_t *info)
{
    /* 调用者必须提供有效的结构体地址。 */
    if (info == 0) {
        return BSP_PARAM;
    }

    /* 先给所有字段设置确定值，防止结构体中残留旧数据。 */
    info->profile = ROUTE_PROFILE_SELECT;
    info->profile_state = 0U;
    info->control_mode = s_route_control;
    info->motion_state = (uint8_t)Motion_GetState();
    info->event_confirm_samples = 0U;
    info->running_ms = 0U;
    info->transition_count = 0U;

#if (ROUTE_PROFILE_SELECT == ROUTE_PROFILE_HJDUINO)
    {
        /* 获取 HJduino profile 的专用状态，再映射到通用信息结构体。 */
        HJduinoRoute_Info_t hjd;
        if (HJduinoRoute_GetInfo(&hjd) != BSP_OK) {
            return BSP_ERROR;
        }
        info->profile_state = (uint8_t)hjd.state;
        info->event_confirm_samples = hjd.entry_confirm_samples;
        info->running_ms = hjd.running_ms;
        info->transition_count = hjd.transition_count;
    }
#endif

    return BSP_OK;
}
