#ifndef __ROUTE_COMMON_H
#define __ROUTE_COMMON_H

#include "bsp_common.h"
#include "line_detect.h"
#include "line_track.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 路线层的公共控制权枚举。
 *
 * 设计目的：在同一个调度周期内，只允许一个模块向底盘写入速度命令，
 * 防止“循迹模块”和“定角转弯模块”同时调用 Chassis_SetSpeed()，互相覆盖。
 *
 * 注意：这个枚举只是在告诉上层“本周期应该由谁控制底盘”，
 * 它本身不会自动阻止其他模块写底盘；调用 RouteManager_Update() 的上层代码
 * 必须读取返回值并据此决定是否使用 LineTrack_Output_t。
 */
/* Exactly one module may own the chassis command in each scheduler cycle. */
typedef enum {
    ROUTE_CONTROL_STOP = 0,   /* 本周期不应继续运动，应停止底盘 */
    ROUTE_CONTROL_LINE_TRACK, /* 本周期由普通循迹输出控制底盘 */
    ROUTE_CONTROL_MOTION,     /* 本周期由 motion_action 动作模块控制底盘 */
    ROUTE_CONTROL_ERROR       /* 路线状态机出现错误，应进入安全停止 */
} Route_ControlMode_t;

/* 统计 8 位黑线掩码中有多少位为 1，即有多少个灰度探头检测到黑线。 */
uint8_t Route_CountBlackBits(uint8_t mask);

/* 判断当前图案是否像十字、全黑或分支等“大面积黑线区域”。 */
uint8_t Route_IsCrossLike(const LineDetect_Result_t *line);

/* 判断当前结果是否为较稳定的普通单线：类型为 SINGLE，黑点数为 1~4。 */
uint8_t Route_IsStableSingleLine(const LineDetect_Result_t *line);

/* 判断最左侧两个探头（bit0、bit1）中是否至少有一个检测到黑线。 */
uint8_t Route_IsLeftEdge(const LineDetect_Result_t *line);

/* 判断最右侧两个探头（bit6、bit7）中是否至少有一个检测到黑线。 */
uint8_t Route_IsRightEdge(const LineDetect_Result_t *line);

/* 判断中间四个探头（bit2~bit5）中是否至少有一个检测到黑线。 */
uint8_t Route_IsMiddleOnLine(const LineDetect_Result_t *line);

#ifdef __cplusplus
}
#endif

#endif
