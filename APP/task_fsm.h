#ifndef __TASK_FSM_H
#define __TASK_FSM_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MISSION_BALL_VEHICLE_FEEDFORWARD_DEFAULT_ENABLE 0U

typedef enum {
    MISSION_MODE_2_LINE_LAP = 2,
    MISSION_MODE_3_BALL_TRANSFER = 3,
    MISSION_MODE_4_LINE_TO_B_WITH_BALL_O = 4,
    MISSION_MODE_5_LINE_LAP_WITH_BALL_O = 5,
    MISSION_MODE_6_LINE_LAP_WITH_CUSTOM_BALL = 6
} Mission_Mode_t;

typedef enum {
    MISSION_STATE_BOOT = 0,
    MISSION_STATE_IDLE,
    MISSION_STATE_ARMED,
    MISSION_STATE_RUNNING,
    MISSION_STATE_FINISH,
    MISSION_STATE_ABORT,
    MISSION_STATE_FAULT
} Mission_State_t;

typedef enum {
    MISSION_RESULT_NONE = 0,
    MISSION_RESULT_PASS,
    MISSION_RESULT_TIME_FAIL,
    MISSION_RESULT_BALL_ERROR_FAIL,
    MISSION_RESULT_TIMEOUT,
    MISSION_RESULT_USER_ABORT,
    MISSION_RESULT_ROUTE_FAULT,
    MISSION_RESULT_BALL_FAULT,
    MISSION_RESULT_NOT_READY
} Mission_Result_t;

typedef enum {
    MISSION_FAULT_NONE = 0,
    MISSION_FAULT_NOT_READY = 1,
    MISSION_FAULT_ROUTE = 2,
    MISSION_FAULT_CHASSIS = 3,
    MISSION_FAULT_LINE_LOST = 4,
    MISSION_FAULT_BALL = 5,
    MISSION_FAULT_K210_TIMEOUT = 6,
    MISSION_FAULT_SERVO = 7,
    MISSION_FAULT_TIMEOUT = 8,
    MISSION_FAULT_ROUTE_EVENT_MISSING = 9,
    MISSION_FAULT_INTERNAL = 10
} Mission_FaultCode_t;

typedef enum {
    MISSION_SUB_IDLE = 0,
    MISSION_SUB_ARM_PREPARE,
    MISSION_SUB_ARM_WAIT_READY,
    MISSION_SUB_M2_START,
    MISSION_SUB_M2_WAIT_LEAVE_A,
    MISSION_SUB_M2_RUNNING_LAP,
    MISSION_SUB_M2_BRAKE,
    MISSION_SUB_M2_DONE,
    MISSION_SUB_M3_SET_PLUS_50,
    MISSION_SUB_M3_WAIT_PLUS_50,
    MISSION_SUB_M3_SET_MINUS_50,
    MISSION_SUB_M3_WAIT_MINUS_50,
    MISSION_SUB_M3_DONE,
    MISSION_SUB_M4_START,
    MISSION_SUB_M4_RUNNING_TO_B,
    MISSION_SUB_M4_BRAKE,
    MISSION_SUB_M4_DONE,
    MISSION_SUB_M5_START,
    MISSION_SUB_M5_RUNNING_LAP,
    MISSION_SUB_M5_BRAKE,
    MISSION_SUB_M5_DONE,
    MISSION_SUB_M6_START,
    MISSION_SUB_M6_RUNNING_LAP,
    MISSION_SUB_M6_BRAKE,
    MISSION_SUB_M6_DONE
} Mission_SubState_t;

typedef struct {
    uint8_t initialized;
    Mission_Mode_t mode;
    Mission_State_t state;
    uint8_t substate;
    Mission_Result_t result;

    uint8_t armed_ready;
    uint8_t route_ready;
    uint8_t ball_ready;
    uint8_t running;
    uint8_t finished;

    int16_t custom_ball_target_mm_x10;
    int16_t active_ball_target_mm_x10;
    int16_t current_ball_position_mm_x10;
    int16_t current_ball_error_mm_x10;
    int16_t max_ball_error_mm_x10;
    int16_t ball_filtered_velocity_mm_s_x10;
    uint8_t ball_settled;
    uint8_t m3_plus_confirm_count;
    uint32_t m3_last_processed_sample_ms;

    uint32_t start_ms;
    uint32_t elapsed_ms;
    uint32_t score_limit_ms;
    uint32_t safety_timeout_ms;
    uint32_t route_events;

    uint16_t fault_code;
    uint8_t vehicle_feedforward_enabled;
} Mission_Info_t;

typedef Mission_Info_t TaskFSM_Info_t;

void TaskFSM_Init(void);
void TaskFSM_Reset(void);
void TaskFSM_Update(void);
BSP_Status_t TaskFSM_GetInfo(Mission_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_FSM_H */
