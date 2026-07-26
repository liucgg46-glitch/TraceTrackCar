#ifndef __ROUTE_PROFILE_B_BASIC_H
#define __ROUTE_PROFILE_B_BASIC_H

#include "route_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 2026通信系电赛模拟赛B题：基础巡线赛道。
 *
 * 普通路段完全复用 Algorithm/line_track.c 的现有速度和PD参数；
 * 本文件定义直角、虚线、三角尖头和终点停止线的赛道事件参数。
 */

/* 启动后先离开起点，避免把起点附近的异常线型误判成三角尖头。 */
#define B_ROUTE_TIP_IGNORE_MS                         800U
#define B_ROUTE_TIP_MIN_TRAVEL_MM                     200L

/*
 * 虚线/尖头共同特征：车辆先稳定压住中线，随后前方线路消失。
 * 首先以较低速度短暂直行探测；短时间内重新见线即判定为虚线间隙，
 * 只有持续丢线才判定已经到达图中的三角尖头。
 */
#define B_ROUTE_TIP_CENTER_CONFIRM_SAMPLES              3U
#define B_ROUTE_TIP_CENTER_TO_LOST_WINDOW_MS           220U
#define B_ROUTE_TIP_LOST_CONFIRM_SAMPLES                2U
#define B_ROUTE_GAP_PROBE_CPS                         1800
#define B_ROUTE_GAP_PROBE_MS                           160U
#define B_ROUTE_GAP_PROBE_MAX_MM                        60L
#define B_ROUTE_GAP_REACQUIRE_CONFIRM_SAMPLES            2U


/*
 * 第一段虚线之前的两个直角使用独立状态处理。
 * 明确侧边图案连续确认后直接转弯；车身带偏角时允许记录弱侧边特征，
 * 若随后快速丢线，仍按记录的方向进入直角转弯，而不是进入虚线探测。
 */
#define B_ROUTE_CORNER_IGNORE_MS                       500U
#define B_ROUTE_CORNER_CENTER_TO_EVENT_WINDOW_MS       350U
#define B_ROUTE_CORNER_CONFIRM_SAMPLES                   2U
#define B_ROUTE_CORNER_TO_LOST_WINDOW_MS               180U
/*
 * 直角先由Motion按角度粗转，再由灰度低速补转到中线。
 * 这里故意小于几何90度，避免灰度板位于车轴前方时一次转过头；
 * 实车只需要优先调整这个宏，不改状态机代码。
 */
#define B_ROUTE_CORNER_TURN_ANGLE_DEG                   45
#define B_ROUTE_CORNER_REACQUIRE_TURN_CPS              900
#define B_ROUTE_CORNER_REACQUIRE_TIMEOUT_MS           1000U
#define B_ROUTE_CORNER_REACQUIRE_CONFIRM_SAMPLES         3U
#define B_ROUTE_CORNER_RECOVERY_IGNORE_MS              250U
#define B_ROUTE_CORNER_EDGE_ERROR_X1000                2500

/* 图中尖头需要向右回折；turn<0表示右转。 */
#define B_ROUTE_TIP_TURN_CPS                          1600
#define B_ROUTE_TIP_TURN_MIN_MS                       220U
#define B_ROUTE_TIP_TURN_TIMEOUT_MS                  2200U
#define B_ROUTE_TIP_REACQUIRE_CONFIRM_SAMPLES           3U

/* 越过尖头并重新稳定循迹后，才允许识别终点全黑停止线。 */
#define B_ROUTE_FINISH_MIN_TRAVEL_AFTER_TIP_MM         250L
#define B_ROUTE_FINISH_SINGLE_CONFIRM_SAMPLES            5U
#define B_ROUTE_FINISH_BLACK_CONFIRM_SAMPLES             3U

#if ((B_ROUTE_TIP_CENTER_CONFIRM_SAMPLES == 0U) || \
     (B_ROUTE_TIP_LOST_CONFIRM_SAMPLES == 0U) || \
     (B_ROUTE_GAP_PROBE_CPS <= 0) || \
     (B_ROUTE_GAP_PROBE_MS == 0U) || \
     (B_ROUTE_GAP_PROBE_MAX_MM <= 0L) || \
     (B_ROUTE_GAP_REACQUIRE_CONFIRM_SAMPLES == 0U) || \
     (B_ROUTE_CORNER_CENTER_TO_EVENT_WINDOW_MS == 0U) || \
     (B_ROUTE_CORNER_CONFIRM_SAMPLES == 0U) || \
     (B_ROUTE_CORNER_TO_LOST_WINDOW_MS == 0U) || \
     (B_ROUTE_CORNER_TURN_ANGLE_DEG <= 0) || \
     (B_ROUTE_CORNER_TURN_ANGLE_DEG >= 90) || \
     (B_ROUTE_CORNER_REACQUIRE_TURN_CPS <= 0) || \
     (B_ROUTE_CORNER_REACQUIRE_TIMEOUT_MS == 0U) || \
     (B_ROUTE_CORNER_REACQUIRE_CONFIRM_SAMPLES == 0U) || \
     (B_ROUTE_CORNER_EDGE_ERROR_X1000 <= 0) || \
     (B_ROUTE_TIP_TURN_CPS <= 0) || \
     (B_ROUTE_TIP_TURN_MIN_MS == 0U) || \
     (B_ROUTE_TIP_TURN_TIMEOUT_MS <= B_ROUTE_TIP_TURN_MIN_MS) || \
     (B_ROUTE_TIP_REACQUIRE_CONFIRM_SAMPLES == 0U) || \
     (B_ROUTE_FINISH_SINGLE_CONFIRM_SAMPLES == 0U) || \
     (B_ROUTE_FINISH_BLACK_CONFIRM_SAMPLES == 0U))
#error "B route parameters are invalid"
#endif

typedef enum {
    B_ROUTE_STATE_RUN_TO_TIP = 0,
    B_ROUTE_STATE_CORNER_TURN,
    B_ROUTE_STATE_CORNER_REACQUIRE,
    B_ROUTE_STATE_GAP_PROBE,
    B_ROUTE_STATE_TIP_RIGHT_TURN,
    B_ROUTE_STATE_RUN_TO_FINISH,
    B_ROUTE_STATE_FINISH_CONFIRM,
    B_ROUTE_STATE_ARRIVED,
    B_ROUTE_STATE_ERROR
} BRoute_State_t;

void BRoute_Init(uint32_t now_ms);
void BRoute_Reset(uint32_t now_ms);
Project_Status_t BRoute_ConfigureMission(
    uint8_t target_room,
    Route_MissionDirection_t direction,
    uint32_t now_ms);
Project_Status_t BRoute_SubmitVisualDecision(
    Route_VisualDirection_t direction);
Route_ControlMode_t BRoute_Update(
    const LineDetect_Result_t *line,
    const Route_ActionFeedback_t *feedback,
    LineTrack_Output_t *out,
    Route_ActionRequest_t *request,
    uint32_t now_ms);
Project_Status_t BRoute_GetInfo(RouteProfile_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __ROUTE_PROFILE_B_BASIC_H */
