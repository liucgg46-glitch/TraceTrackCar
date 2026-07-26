#ifndef __TASK_PROFILE_B_BASIC_H
#define __TASK_PROFILE_B_BASIC_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 2026通信系电赛模拟赛B题：基础巡线正式整车任务状态机。
 *
 * APP层只负责整车业务流程：
 *   KEY1启动、KEY4停止、运行监控、到达提示和故障停车。
 * 具体直角、虚线、三角尖头和停止线识别仍由Route层负责。
 */
#define B_TASK_KEY_RELEASE_CONFIRM_SAMPLES          10U
#define B_TASK_ARRIVAL_BUZZER_MS                   600U

typedef enum {
    B_TASK_STATE_WAIT_START = 0,
    B_TASK_STATE_RUNNING,
    B_TASK_STATE_ARRIVAL_BUZZER,
    B_TASK_STATE_COMPLETE,
    B_TASK_STATE_FAULT
} BTask_State_t;

typedef enum {
    B_TASK_FAULT_NONE = 0,
    B_TASK_FAULT_GRAY_OFFLINE,
    B_TASK_FAULT_CONTROL_BUSY,
    B_TASK_FAULT_ROUTE,
    B_TASK_FAULT_CHASSIS
} BTask_Fault_t;

typedef struct {
    BTask_State_t state;
    BTask_Fault_t fault;
    uint8_t keys_armed;
    uint8_t route_state;
    uint8_t route_arrived;
    uint8_t route_error;
    uint8_t line_follow_running;
    uint8_t chassis_fault;
    uint8_t start_status;
    uint32_t state_elapsed_ms;
    uint32_t transition_count;
} BTask_Info_t;

void BTask_Init(void);
void BTask_Reset(void);
void BTask_Update(void);
BSP_Status_t BTask_GetInfo(BTask_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_PROFILE_B_BASIC_H */