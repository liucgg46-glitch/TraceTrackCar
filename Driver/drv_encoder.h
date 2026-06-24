#ifndef __DRV_ENCODER_H
#define __DRV_ENCODER_H

#include "bsp_common.h"
#include "bsp_encoder.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 四轮编码器驱动层：drv_encoder
 * ============================================================================
 * 定位：把 BSP_ENCODER_CH1~CH4 映射成 FL/FR/RL/RR，并可选换算为 mm/s。
 *
 * BSP 层只输出 count/cps；本层负责：
 *   - 车轮映射：CH1~CH4 -> FL/FR/RL/RR；
 *   - 左右平均速度；
 *   - count/s 与 mm/s 的换算；
 *   - 提供给 chassis 的统一反馈接口。
 *
 * 移植方法：
 *   只改本文件配置区：映射、轮径、每圈脉冲数。
 */

typedef enum {
    WHEEL_FL = 0,
    WHEEL_FR,
    WHEEL_RL,
    WHEEL_RR,
    WHEEL_COUNT
} Wheel_Id_t;

/* 默认映射：按 Part1 引脚规划。若实际接线不同，只改这里。 */
#define DRV_ENCODER_FL_BSP_ID       BSP_ENCODER_CH1
#define DRV_ENCODER_FR_BSP_ID       BSP_ENCODER_CH2
#define DRV_ENCODER_RL_BSP_ID       BSP_ENCODER_CH3
#define DRV_ENCODER_RR_BSP_ID       BSP_ENCODER_CH4

/* 轮子参数：这里先给模板值，实际必须按你的编码器和轮径修改。 */
#define DRV_ENCODER_WHEEL_DIAMETER_MM     65.0f
#define DRV_ENCODER_COUNTS_PER_REV        1450.0f

/* 是否使用四路平均。若只想临时用后轮反馈，可改下面两个宏。 */
#define DRV_ENCODER_LEFT_USE_FRONT        1
#define DRV_ENCODER_LEFT_USE_REAR         1
#define DRV_ENCODER_RIGHT_USE_FRONT       1
#define DRV_ENCODER_RIGHT_USE_REAR        1

typedef struct {
    int16_t delta_count;
    int32_t speed_cps;
    int32_t total_count;
    int32_t speed_mm_s;
    int32_t total_mm;
} Drv_Encoder_WheelInfo_t;

void    Drv_Encoder_Init(void);
void    Drv_Encoder_Update(void);

int16_t Drv_Encoder_GetWheelDelta(Wheel_Id_t wheel);
int32_t Drv_Encoder_GetWheelSpeedCps(Wheel_Id_t wheel);
int32_t Drv_Encoder_GetWheelTotalCount(Wheel_Id_t wheel);
int32_t Drv_Encoder_GetWheelSpeedMmS(Wheel_Id_t wheel);
int32_t Drv_Encoder_GetWheelTotalMm(Wheel_Id_t wheel);

int32_t Drv_Encoder_GetLeftSpeedCps(void);
int32_t Drv_Encoder_GetRightSpeedCps(void);
int32_t Drv_Encoder_GetLeftSpeedMmS(void);
int32_t Drv_Encoder_GetRightSpeedMmS(void);
int32_t Drv_Encoder_GetLeftTotalMm(void);
int32_t Drv_Encoder_GetRightTotalMm(void);

void    Drv_Encoder_ClearAllTotal(void);
BSP_Status_t Drv_Encoder_GetWheelInfo(Wheel_Id_t wheel, Drv_Encoder_WheelInfo_t *info);

/* 覆盖 app_task_port.c 里的弱函数，任务表仍然使用 Encoder_Update。 */
void Encoder_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_ENCODER_H */
