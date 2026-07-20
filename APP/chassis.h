#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 差速底盘开环 PWM 控制：chassis
 * ============================================================================
 * 定位：根据 linear_speed + turn_speed 生成左右目标值，再按固定比例直接
 * 换算为四路电机 PWM，不使用编码器速度反馈修正。
 *
 * 本模块不直接读 TIM、不直接操作 GPIO 引脚，只调用：
 *   - drv_encoder：保留四轮速度监测和里程累计；
 *   - drv_motor：输出四轮 PWM。
 *
 * 使用约定：
 *   - Chassis_Update() 固定 10ms 调用；
 *   - 控制模块必须先获取控制权，再使用带 owner 的接口提交命令；
 *   - 未持有控制权的模块不能覆盖当前底盘命令；
 *   - 所有速度单位默认用 count/s，先便于调车；后续可切换到 mm/s。
 */

typedef enum {
    CHASSIS_MODE_STOP = 0,
    CHASSIS_MODE_SPEED
} Chassis_Mode_t;

typedef enum {
    CHASSIS_OWNER_NONE = 0,
    CHASSIS_OWNER_LINE_FOLLOW,
    CHASSIS_OWNER_MOTION,
    CHASSIS_OWNER_TEST
} Chassis_ControlOwner_t;

typedef struct {
    Chassis_Mode_t mode;
    Chassis_ControlOwner_t owner;
    int16_t linear_target_cps;
    int16_t turn_target_cps;
    int16_t left_target_cps;
    int16_t right_target_cps;
    int32_t fl_feedback_cps;
    int32_t fr_feedback_cps;
    int32_t rl_feedback_cps;
    int32_t rr_feedback_cps;
    int16_t fl_output;
    int16_t fr_output;
    int16_t rl_output;
    int16_t rr_output;
} Chassis_Info_t;

void         Chassis_Init(void);
/* 同一 owner 重复获取视为成功；已有其他 owner 时返回 BSP_BUSY。 */
BSP_Status_t Chassis_AcquireControl(Chassis_ControlOwner_t owner);
/* 只有当前 owner 可以更新目标，函数只保存命令，不阻塞等待执行。 */
BSP_Status_t Chassis_SetSpeed(Chassis_ControlOwner_t owner,
                              int16_t linear_speed_cps,
                              int16_t turn_speed_cps);
/* 停止输出但保留控制权，供动作稳定确认等过程继续独占底盘。 */
BSP_Status_t Chassis_Stop(Chassis_ControlOwner_t owner);
/* 停止输出并释放控制权，其他模块随后可以重新获取。 */
BSP_Status_t Chassis_ReleaseControl(Chassis_ControlOwner_t owner);
/* 安全停车接口：无条件停止并清除当前控制权。 */
void         Chassis_EmergencyStop(void);
Chassis_Mode_t Chassis_GetMode(void);
Chassis_ControlOwner_t Chassis_GetOwner(void);
BSP_Status_t Chassis_GetInfo(Chassis_Info_t *info);

/* 覆盖 app_task_port.c 里的弱函数，任务表仍然使用 Chassis_Update。 */
void Chassis_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_H */
