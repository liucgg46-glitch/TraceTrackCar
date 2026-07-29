#ifndef __TASK_PROFILE_H2_ROUND_STOP_H
#define __TASK_PROFILE_H2_ROUND_STOP_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * H题第2项独立速度和停车参数。
 * 3200 count/s约为451 mm/s，弯道最低2300 count/s约为324 mm/s，
 * 按理论6142 mm赛道估算整圈约16.4 s，实际值仍需上板校准。
 */
#define H2_RUN_SPEED_CPS                         2500
#define H2_CURVE_SPEED_CPS                       2500
#define H2_BRAKE_TIME_MS                         1200U
#define H2_STOP_SPEED_THRESHOLD_CPS              80
#define H2_STOP_SETTLE_MS                        200U
#define H2_MAX_STOP_OFFSET_MM                    20
#define H2_LCD_TEXT_UPDATE_MS                    100U
#define H2_KEY_RELEASE_CONFIRM_SAMPLES           10U
#define H2_ENCODER_MAX_REASONABLE_CPS            6000
#define H2_ENCODER_MAX_STEP_MM                   100

typedef enum {
    H2_IDLE = 0,
    H2_WAIT_START,
    H2_LEAVE_A,
    H2_RUNNING,
    H2_FINISH_ARMED,
    H2_FINAL_APPROACH,
    H2_BRAKING,
    H2_STOPPED,
    H2_FAULT
} H2Task_State_t;

typedef enum {
    H2_FAULT_NONE = 0,
    H2_FAULT_GRAY_OFFLINE,
    H2_FAULT_CONTROL_BUSY,
    H2_FAULT_ROUTE_LINE_LOST,
    H2_FAULT_ROUTE_TIMEOUT,
    H2_FAULT_RUN_TIMEOUT,
    H2_FAULT_CHASSIS,
    H2_FAULT_BRAKE_TIMEOUT,
    H2_FAULT_MANUAL_STOP,
    H2_FAULT_INTERNAL
} H2Task_Fault_t;

typedef struct {
    H2Task_State_t state;
    H2Task_Fault_t fault;
    uint8_t keys_armed;
    uint8_t timer_running;
    uint8_t route_state;
    uint8_t finish_armed;
    uint8_t finish_candidate;
    uint8_t encoder_reliable;
    uint8_t line_follow_running;
    uint8_t chassis_fault;
    uint32_t elapsed_ms;
    uint32_t final_time_ms;
    uint32_t state_elapsed_ms;
    uint32_t transition_count;
    int32_t encoder_distance_mm;
    int32_t marker_distance_mm;
    int32_t stop_offset_mm;
    int32_t left_speed_cps;
    int32_t right_speed_cps;
} H2Task_Info_t;

void H2Task_Init(void);
void H2Task_Reset(void);
void H2Task_Update(void);
BSP_Status_t H2Task_GetInfo(H2Task_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_PROFILE_H2_ROUND_STOP_H */
