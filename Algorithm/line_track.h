#ifndef __LINE_TRACK_H
#define __LINE_TRACK_H

#include "bsp_common.h"
#include "line_detect.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 基础灰度循迹控制器：line_track
 * ============================================================================
 *
 * 输入：line_detect 输出的线路位置误差和线路类型。
 * 输出：底盘直行量 linear_cps、转向量 turn_cps。
 *
 * 方向约定：
 *   error < 0：黑线位于小车左侧，turn > 0，向左修正；
 *   error > 0：黑线位于小车右侧，turn < 0，向右修正。
 *
 * 当前版本只保留三种实际运行状态：
 *   1. TRACK：正常 P/PD 循迹；
 *   2. SEARCH：丢线后沿最后看到黑线的方向原地找线；
 *   3. FAILSAFE：找线超时，输出无效，由上层停车。
 *
 * 本模块不直接操作电机和底盘，也不负责环岛、直角或赛道顺序判断。
 */

typedef enum {
    LINE_TRACK_MODE_TRACK = 0,

    /*
     * 以下三个名称为兼容已有串口日志或旧代码而保留。
     * 当前基础版不会进入这些状态，不包含边缘模式、宽线模式或丢线确认模式。
     */
    LINE_TRACK_MODE_EDGE,
    LINE_TRACK_MODE_WIDE_LINE,
    LINE_TRACK_MODE_LOST_CONFIRM,

    LINE_TRACK_MODE_SEARCH,
    LINE_TRACK_MODE_FAILSAFE
} LineTrack_Mode_t;

typedef struct {
    int16_t linear_cps;  /* 底盘直行目标；当前开环底盘中属于“虚拟速度命令” */
    int16_t turn_cps;    /* 底盘转向目标；正数左转，负数右转 */
    uint8_t valid;       /* 1：输出有效；0：上层应停止底盘 */
} LineTrack_Output_t;

typedef struct {
    LineTrack_Mode_t mode;
    int16_t raw_error;
    int16_t filtered_error;      /* 基础版不做低通滤波，等于 raw_error */
    int16_t target_linear_cps;
    int16_t target_turn_cps;
    int16_t output_linear_cps;
    int16_t output_turn_cps;
    uint16_t lost_samples;
    uint16_t reacquire_samples;  /* 兼容字段，基础版固定为 0 */
    uint16_t search_phase;       /* 兼容字段，基础版固定为 0 */
    int8_t search_direction;
    uint32_t lost_ms;
} LineTrack_Info_t;

void LineTrack_Init(void);
void LineTrack_Reset(void);
BSP_Status_t LineTrack_GetInfo(LineTrack_Info_t *info);
void LineTrack_Compute(const LineDetect_Result_t *line, LineTrack_Output_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __LINE_TRACK_H */
