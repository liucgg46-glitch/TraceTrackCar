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
 * 循迹控制算法：line_track
 * ============================================================================
 * 定位：纯算法层，不直接读取灰度、不直接控制底盘。
 * 输入：LineDetect_Result_t；输出：linear_cps / turn_cps。
 *
 * 约定：
 *   error_x1000 > 0 表示黑线偏右，车应右转，因此 turn_cps 为负；
 *   error_x1000 < 0 表示黑线偏左，车应左转，因此 turn_cps 为正。
 */

	/*==================== 循迹速度参数 ====================*/

/* 正常循迹基础前进速度，单位 cps
 * 调高：车整体跑得更快，但弯道更容易冲出线
 * 调低：车更稳，但速度慢
 */
#define LINE_TRACK_BASE_SPEED_CPS        1800

/* 检测到路口、分支、十字时的前进速度，单位 cps
 * 调高：过路口更快，但容易冲过路口、错过判断
 * 调低：路口更稳，方便识别，但整体节奏变慢
 */
#define LINE_TRACK_CROSS_SPEED_CPS       600

/* 丢线时的前进速度，单位 cps
 * 一般建议为 0，表示丢线后不继续往前冲
 * 调高：丢线后还会往前走，可能更容易找回线，也可能直接冲出赛道
 * 调低：更安全；设为 0 最稳
 */
#define LINE_TRACK_LOST_SPEED_CPS        0

/* 丢线找线时的原地搜索转向速度，单位 cps
 * 调高：丢线后转得更快，找线更积极，但容易来回扫过头
 * 调低：找线更慢，更稳，但恢复速度慢
 */
#define LINE_TRACK_SEARCH_TURN_CPS       300

/* 循迹最大转向速度限制，单位 cps
 * 调高：最大修正能力更强，高速时更能拐回来，但容易急摆
 * 调低：转向更柔和，但速度快时可能转不过来
 */
#define LINE_TRACK_TURN_MAX_CPS          650


/*==================== 循迹 PD 参数 ====================*/

/* 比例系数 Kp：根据当前偏差产生转向修正
 * 调高：看到偏差后修正更猛，拐弯更积极，但太高会左右蛇形摆动
 * 调低：修正更柔和，但太低会跟线慢、弯道冲出去
 */
#define LINE_TRACK_KP                    0.8f

/* 微分系数 Kd：根据偏差变化速度抑制摆动
 * 调高：可以减少左右振荡，让车更稳，但太高会反应迟钝、抖动或噪声敏感
 * 调低：车反应更直接，但高速时更容易来回摆
 */
#define LINE_TRACK_KD                    0.12f

/* 路口是否减速。比赛任务层后续会根据路口类型决定转弯/直行。 */
#define LINE_TRACK_SLOW_ON_CROSS         0U

typedef struct {
    int16_t linear_cps;
    int16_t turn_cps;
    uint8_t valid;
} LineTrack_Output_t;

typedef struct {
    int16_t base_speed_cps;
    int16_t cross_speed_cps;
    int16_t lost_speed_cps;
    int16_t search_turn_cps;
    int16_t turn_max_cps;
    float kp;
    float kd;
} LineTrack_Config_t;

void LineTrack_Init(void);
void LineTrack_SetConfig(const LineTrack_Config_t *cfg);
BSP_Status_t LineTrack_GetConfig(LineTrack_Config_t *cfg);

void LineTrack_Compute(const LineDetect_Result_t *line, LineTrack_Output_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __LINE_TRACK_H */
