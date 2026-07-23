#ifndef __ROUTE_PROFILE_MEDICINE_H
#define __ROUTE_PROFILE_MEDICINE_H

#include "route_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 路口和终点必须连续确认，避免单帧灰度抖动改变路线。 */
#define MEDICINE_ROUTE_EVENT_CONFIRM_SAMPLES       3U
#define MEDICINE_ROUTE_END_CONFIRM_SAMPLES         2U
#define MEDICINE_ROUTE_SINGLE_CONFIRM_SAMPLES      3U
#define MEDICINE_ROUTE_DOOR_MIN_SEGMENTS            2U
#define MEDICINE_ROUTE_VISION_APPROACH_MAX_CPS    1600
#define MEDICINE_ROUTE_END_MIN_TRAVEL_MM           250L
#define MEDICINE_ROUTE_RETURN_SPEED_BOOST_CPS       500
#define MEDICINE_ROUTE_RETURN_MAX_SPEED_CPS        3000

#if ((MEDICINE_ROUTE_RETURN_SPEED_BOOST_CPS < 0) || \
     (MEDICINE_ROUTE_RETURN_MAX_SPEED_CPS <= 0))
#error "medicine return speed parameters are invalid"
#endif

/* 直角路口预留循迹回正余量，实车转弯结果稳定后再固定该值。 */
#define MEDICINE_ROUTE_RIGHT_ANGLE_DEG              55

/* 启动或转弯后先离开脚下标志，再允许确认下一个事件。 */
#define MEDICINE_ROUTE_EVENT_IGNORE_MS            400U

typedef enum {
    MEDICINE_ROUTE_STATE_IDLE = 0,
    MEDICINE_ROUTE_STATE_OUTBOUND_MAIN,
    MEDICINE_ROUTE_STATE_OUTBOUND_WAIT_VISUAL,
    MEDICINE_ROUTE_STATE_OUTBOUND_TURN,
    MEDICINE_ROUTE_STATE_OUTBOUND_FAR_CORRIDOR,
    MEDICINE_ROUTE_STATE_OUTBOUND_ROOM,
    MEDICINE_ROUTE_STATE_RETURN_TURN_AROUND,
    MEDICINE_ROUTE_STATE_RETURN_ROOM,
    MEDICINE_ROUTE_STATE_RETURN_TURN,
    MEDICINE_ROUTE_STATE_RETURN_CORRIDOR,
    MEDICINE_ROUTE_STATE_RETURN_MAIN,
    MEDICINE_ROUTE_STATE_ARRIVED,
    MEDICINE_ROUTE_STATE_ERROR
} MedicineRoute_State_t;

void MedicineRoute_Init(uint32_t now_ms);
void MedicineRoute_Reset(uint32_t now_ms);
Project_Status_t MedicineRoute_ConfigureMission(
    uint8_t target_room,
    Route_MissionDirection_t direction,
    uint32_t now_ms);
Project_Status_t MedicineRoute_SubmitVisualDecision(
    Route_VisualDirection_t direction);
Route_ControlMode_t MedicineRoute_Update(
    const LineDetect_Result_t *line,
    const Route_ActionFeedback_t *feedback,
    LineTrack_Output_t *out,
    Route_ActionRequest_t *request,
    uint32_t now_ms);
Project_Status_t MedicineRoute_GetInfo(RouteProfile_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __ROUTE_PROFILE_MEDICINE_H */
