#ifndef __LINE_TRACK_H
#define __LINE_TRACK_H

#include "project_status.h"
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
 * 当前版本包含四种实际运行状态：
 *   1. TRACK：正常 P/PD 循迹；
 *   2. LOST_CONFIRM：连续确认丢线，过滤单帧干扰；
 *   3. SEARCH：沿最后方向起扫，并分阶段反向扩大找线范围；
 *   4. FAILSAFE：找线超时，输出无效，由上层停车。
 *
 * 本模块不直接操作电机和底盘，也不负责环岛、直角或赛道顺序判断。
 */

typedef enum {
    LINE_TRACK_MODE_TRACK = 0,

    /* 以下两个名称只为兼容已有日志或旧代码保留。 */
    LINE_TRACK_MODE_EDGE,
    LINE_TRACK_MODE_WIDE_LINE,
    LINE_TRACK_MODE_LOST_CONFIRM,

    LINE_TRACK_MODE_SEARCH,
    LINE_TRACK_MODE_FAILSAFE
} LineTrack_Mode_t;

typedef struct {
    int16_t linear_cps;  /* 底盘直行速度目标 */
    int16_t turn_cps;    /* 底盘转向目标；正数左转，负数右转 */
    uint8_t valid;       /* 1：输出有效；0：上层应停止底盘 */
} LineTrack_Output_t;

typedef struct {
    LineTrack_Mode_t mode;
    int16_t raw_error;
    int16_t filtered_error;      /* 当前不做低通滤波，等于经过死区处理的误差 */
    int16_t target_linear_cps;
    int16_t target_turn_cps;
    int16_t output_linear_cps;
    int16_t output_turn_cps;
    uint16_t lost_samples;
    uint16_t reacquire_samples;  /* 当前连续重获帧数；成功后保持确认值到下次丢线或复位 */
    uint16_t search_phase;       /* 当前扫描阶段，0为沿最后线路方向起扫 */
    int8_t search_direction;
    uint32_t lost_ms;
} LineTrack_Info_t;

void LineTrack_Init(void);
void LineTrack_Reset(void);
/*
 * 运行时速度方案只修改纯算法目标，不接触底盘。
 * 用于不同正式任务复用同一套循迹算法和PID参数。
 */
Project_Status_t LineTrack_SetSpeedProfile(int16_t base_speed_cps,
                                           int16_t cross_speed_cps,
                                           int16_t min_track_speed_cps);
Project_Status_t LineTrack_GetInfo(LineTrack_Info_t *info);
void LineTrack_Compute(const LineDetect_Result_t *line,
                       LineTrack_Output_t *out,
                       uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* __LINE_TRACK_H */
