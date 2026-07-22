#ifndef __TASK_FSM_H
#define __TASK_FSM_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 目标数字必须来自多个新快照，避免单帧误识别触发车辆。 */
#define TASK_FSM_TARGET_MIN_CONFIDENCE        70U
#define TASK_FSM_TARGET_CONFIRM_FRAMES         3U
#define TASK_FSM_TARGET_CONFIRM_TIMEOUT_MS   800U

/* K210使用QVGA横坐标；中心死区用于避免把正前方数字误判为左右选路。 */
#define TASK_FSM_VISION_CENTER_X             160U
#define TASK_FSM_VISION_DEADBAND_X             5U
#define TASK_FSM_VISION_CONFIRM_FRAMES         2U
#define TASK_FSM_VISION_ABSENCE_CONFIRM_FRAMES 2U
#define TASK_FSM_VISION_CONFIRM_TIMEOUT_MS  1200U
/* 摄像头画面若被水平镜像，实测确认后改为1。当前方向待实车确认。 */
#define TASK_FSM_VISION_X_REVERSED              0U

/*
 * 约200 g药品使用上下两个门槛形成滞回，避免称重波动反复切换。
 * 实际药盒、托盘或传感器结构改变后必须重新实测这两个值。
 */
#define TASK_FSM_LOAD_ON_THRESHOLD_G        120.0f
#define TASK_FSM_LOAD_OFF_THRESHOLD_G        50.0f
#define TASK_FSM_WEIGHT_CONFIRM_MS          500U
/* 整车状态机启动后先用滤波缓存建立空载零点，避免严格自动去皮长期未完成。 */
#define TASK_FSM_STARTUP_TARE_DELAY_MS       500U

#if (TASK_FSM_TARGET_CONFIRM_FRAMES == 0U)
#error "TASK_FSM_TARGET_CONFIRM_FRAMES must be greater than zero"
#endif

typedef enum {
    TASK_FSM_STATE_WAIT_TARGET = 0,
    TASK_FSM_STATE_WAIT_LOAD,
    TASK_FSM_STATE_READY,
    TASK_FSM_STATE_OUTBOUND,
    TASK_FSM_STATE_WAIT_UNLOAD,
    TASK_FSM_STATE_RETURN_READY,
    TASK_FSM_STATE_RETURNING,
    TASK_FSM_STATE_COMPLETE,
    TASK_FSM_STATE_FAULT
} TaskFSM_State_t;

typedef enum {
    TASK_FSM_LOAD_UNKNOWN = 0,
    TASK_FSM_LOAD_EMPTY,
    TASK_FSM_LOAD_PRESENT
} TaskFSM_LoadState_t;

typedef enum {
    TASK_FSM_FAULT_NONE = 0,
    TASK_FSM_FAULT_ROUTE,
    TASK_FSM_FAULT_CHASSIS,
    TASK_FSM_FAULT_SENSOR,
    TASK_FSM_FAULT_TIMEOUT,
    TASK_FSM_FAULT_UNSUPPORTED_TARGET
} TaskFSM_Fault_t;

typedef struct {
    TaskFSM_State_t state;
    TaskFSM_LoadState_t load_state;
    TaskFSM_Fault_t fault;
    uint8_t target_room;
    uint8_t target_locked;
    uint8_t observed_digit;
    uint8_t target_confirm_frames;
    uint8_t k210_online;
    uint8_t weight_valid;
    uint8_t empty_seen;
    uint8_t vision_observed_side;
    uint8_t vision_confirm_frames;
    uint16_t vision_center_x;
    uint8_t route_state;
    uint8_t route_approach_ready;
    uint8_t route_visual_stage;
    uint8_t route_visual_ready;
    uint8_t route_waiting_visual;
    uint8_t route_start_status;
    uint8_t route_intersections;
    uint8_t route_decisions;
    uint8_t route_arrived;
    uint8_t stop_confirmed;
    uint8_t status_light;
    float weight_g;
    uint32_t state_elapsed_ms;
    uint32_t last_snapshot_ms;
    uint32_t transition_count;
} TaskFSM_Info_t;

void TaskFSM_Init(void);
void TaskFSM_Reset(void);

/* 覆盖app_task_port.c中的弱函数，正式任务建议10 ms周期调用。 */
void TaskFSM_Update(void);

BSP_Status_t TaskFSM_GetInfo(TaskFSM_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_FSM_H */
