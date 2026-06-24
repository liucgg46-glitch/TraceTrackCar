#ifndef __LINE_FOLLOW_APP_H
#define __LINE_FOLLOW_APP_H

#include "bsp_common.h"
#include "line_detect.h"
#include "line_track.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 循迹应用层：line_follow_app
 * ============================================================================
 * 定位：负责把 Driver 的灰度数据、Algorithm 的识别/控制结果接到底盘。
 * 不在这里写阈值算法，不在这里写 PID 算法，只做流程编排。
 */

typedef enum {
    LINE_FOLLOW_STOP = 0,
    LINE_FOLLOW_RUN
} LineFollow_State_t;

typedef struct {
    LineFollow_State_t state;
    uint16_t raw[LINE_DETECT_SENSOR_NUM];
    LineDetect_Result_t detect;
    LineTrack_Output_t output;
} LineFollow_Info_t;

void LineFollow_Init(void);
void LineFollow_Start(void);
void LineFollow_Stop(void);
void LineFollow_Update(void);
LineFollow_State_t LineFollow_GetState(void);
BSP_Status_t LineFollow_GetInfo(LineFollow_Info_t *info);

/* 覆盖 app_task_port.c 里的弱函数，任务表仍然使用 LineTrack_Update。 */
void LineTrack_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __LINE_FOLLOW_APP_H */
