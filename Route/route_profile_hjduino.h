#ifndef __ROUTE_PROFILE_HJDUINO_H
#define __ROUTE_PROFILE_HJDUINO_H

#include "route_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * HJduino 专用路线状态机。
 * 当前实现只包含“入环前正常循迹 → 右转进入环 → 环内循迹”，
 * 尚未实现环形出口识别和离环动作。
 */
typedef enum {
    HJD_ROUTE_NORMAL_BEFORE_LOOP = 0, /* 尚未到入口，执行普通循迹 */
    HJD_ROUTE_ENTER_LOOP_RIGHT,       /* 已启动右转动作，等待 motion 完成 */
    HJD_ROUTE_IN_LOOP,                /* 已进入环形区域，继续普通循迹 */
    HJD_ROUTE_ERROR                   /* 路线状态机或 motion 出错 */
} HJduinoRoute_State_t;

/* HJduino 路线的调试信息。 */
typedef struct {
    HJduinoRoute_State_t state;       /* 当前状态机状态 */
    uint16_t entry_confirm_samples;   /* 入口连续确认次数 */
    uint32_t running_ms;              /* 自最近一次 Reset 后运行时间 */
    uint32_t transition_count;        /* 状态切换累计次数 */
} HJduinoRoute_Info_t;

/* 初始化 HJduino 路线状态机。 */
void HJduinoRoute_Init(void);

/* 恢复到“入环前正常循迹”状态，并清零计数。 */
void HJduinoRoute_Reset(void);

/* 每个调度周期执行一次路线状态机。 */
Route_ControlMode_t HJduinoRoute_Update(const LineDetect_Result_t *line,
                                        LineTrack_Output_t *out);

/* 获取状态机调试信息。 */
BSP_Status_t HJduinoRoute_GetInfo(HJduinoRoute_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif
