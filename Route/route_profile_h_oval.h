#ifndef __ROUTE_PROFILE_H_OVAL_H
#define __ROUTE_PROFILE_H_OVAL_H

#include "route_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 2026年电赛H题第2项椭圆赛道参数。
 * Route只根据纯数据识别赛道阶段和A点启停线，不直接访问硬件或控制显示。
 */
#define H2_LEAVE_A_MIN_MS                         300U
#define H2_LEAVE_CLEAR_CONFIRM_COUNT              5U
#define H2_START_LINE_MIN_BLACK_COUNT             3U
#define H2_START_LINE_MIN_SPAN                    3U
#define H2_MIN_FINISH_TIME_MS                     10000U

/* B点检测参数需结合实车直线修正量和稳定入弯量继续标定。 */
#define H4_B_ARM_DISTANCE_MM                      1200
#define H4_B_TURN_THRESHOLD_CPS                   500
#define H4_B_CURVE_CONFIRM_COUNT                  4U

typedef enum {
    H_OVAL_ROUTE_IDLE = 0,
    H_OVAL_ROUTE_LEAVING_A,
    H_OVAL_ROUTE_FIRST_STRAIGHT,
    H_OVAL_ROUTE_FIRST_CURVE,
    H_OVAL_ROUTE_SECOND_STRAIGHT,
    H_OVAL_ROUTE_SECOND_CURVE,
    H_OVAL_ROUTE_FINISH_ARMED,
    H_OVAL_ROUTE_FINISH_CONFIRM,
    H_OVAL_ROUTE_FINISHED,
    H_OVAL_ROUTE_FAULT
} HOvalRoute_State_t;

typedef enum {
    H_OVAL_ROUTE_FAULT_NONE = 0,
    H_OVAL_ROUTE_FAULT_LINE_LOST,
    H_OVAL_ROUTE_FAULT_PHASE_TIMEOUT,
    H_OVAL_ROUTE_FAULT_RUN_TIMEOUT,
    H_OVAL_ROUTE_FAULT_PARAM
} HOvalRoute_Fault_t;

typedef struct {
    HOvalRoute_State_t state;
    HOvalRoute_Fault_t fault;
    uint8_t left_a;
    uint8_t first_curve_seen;
    uint8_t second_curve_seen;
    uint8_t finish_armed;
    uint8_t finish_candidate;
    uint8_t arrived;
    uint8_t b_armed;
    uint8_t passed_b;
    uint8_t b_curve_confirm_count;
    uint8_t gray_mask;
    uint8_t black_count;
    uint8_t black_span;
    int16_t line_error;
    int16_t turn_output;
    uint32_t curve_confirm_ms;
    uint32_t straight_confirm_ms;
    uint32_t running_ms;
    uint32_t state_elapsed_ms;
    int32_t encoder_distance_mm;
    int32_t relative_distance_mm;
    uint32_t transition_count;
} HOvalRoute_Info_t;

void HRoute_Init(uint32_t now_ms);
void HRoute_Reset(uint32_t now_ms);
Project_Status_t HRoute_ConfigureMission(
    uint8_t target_room,
    Route_MissionDirection_t direction,
    uint32_t now_ms);
Project_Status_t HRoute_SubmitVisualDecision(
    Route_VisualDirection_t direction);
Route_ControlMode_t HRoute_Update(
    const LineDetect_Result_t *line,
    const Route_ActionFeedback_t *feedback,
    LineTrack_Output_t *out,
    Route_ActionRequest_t *request,
    uint32_t now_ms);
Project_Status_t HRoute_GetInfo(RouteProfile_Info_t *info);
Project_Status_t HRoute_GetH2Info(HOvalRoute_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __ROUTE_PROFILE_H_OVAL_H */
