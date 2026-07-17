
#ifndef __ROUTE_MANAGER_H
#define __ROUTE_MANAGER_H

#include "route_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化当前选中的路线方案，并把控制权状态置为 STOP。 */
void RouteManager_Init(void);

/*
 * 复位路线状态机：
 *   1. 停止 route 可能正在执行的 motion 动作；
 *   2. 复位当前 profile；
 *   3. 把控制权状态置为 STOP。
 */
void RouteManager_Reset(void);

/*
 * 路线管理器的核心周期函数。
 * 输入：
 *   line：line_detect 产生的当前线路识别结果；
 *   out ：当控制权属于普通循迹时，写入线速度和转向速度。
 * 返回：本周期底盘应由 STOP、LINE_TRACK、MOTION 还是 ERROR 控制。
 */
Route_ControlMode_t RouteManager_Update(const LineDetect_Result_t *line,
                                        LineTrack_Output_t *out);

/*
 * 路线管理器对外提供的调试信息。
 * 这些字段用于串口/OLED观察状态，不直接参与控制。
 */
typedef struct {
    uint8_t profile;                 /* 当前编译选择的路线方案编号 */
    uint8_t profile_state;           /* 当前路线方案内部状态 */
    Route_ControlMode_t control_mode;/* 最近一次 Update 返回的控制权 */
    uint8_t motion_state;            /* motion_action 当前状态 */
    uint16_t event_confirm_samples;  /* 当前事件连续确认次数 */
    uint32_t running_ms;             /* 当前路线自复位以来运行时间 */
    uint32_t transition_count;       /* 路线状态切换累计次数 */
} RouteManager_Info_t;

/* 复制当前路线调试信息到调用者提供的结构体。 */
BSP_Status_t RouteManager_GetInfo(RouteManager_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif
