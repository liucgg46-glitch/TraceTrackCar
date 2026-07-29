#ifndef __LINE_DETECT_H
#define __LINE_DETECT_H

#include "project_status.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 灰度循迹识别算法：line_detect
 * ============================================================================
 * 定位：纯算法层，不直接读 ADC、不控制 GPIO、不输出电机。
 * 输入：8 路灰度 raw/filt 数据。
 * 输出：黑白二值 mask、循迹误差、路口类型。
 */

#define LINE_DETECT_SENSOR_NUM          8U

/* 黑线对应 ADC 高电平还是低电平。
 * 大多数红外灰度模块：白底反射强，ADC 可能高；黑线反射弱，ADC 可能低。
 * 如果你发现黑线上 raw 更大，把这里改 1。
 */
#define LINE_DETECT_BLACK_IS_HIGH       0U

/* 默认固定阈值。
 * 如果你已经用串口 p 打印出 8 路 TH，可以直接在这里改。
 * 顺序：0~7 对应 Driver 输出的物理通道，必须与各探头的标定值保持一致。
 * 传感器旋转180度时只修改 control_config.h 中的方向开关，不要手工倒序阈值。
 * 默认代码中 LINE_DETECT_BLACK_IS_HIGH = 0U 时：
 *   raw < threshold 判定为黑线；raw > threshold 判定为白底。
 */
#define LINE_DETECT_DEFAULT_THRESHOLD   2000U
// 751 1405 1942 2027 1928 1734 1742 1718  无MCU阈值
//1774 1830 2995 2762 2104 2505 2553 2336  有MCU阈值
// 2460 2302 2328 2397 2435 2484 2605 2500  亚博红外巡线

#define LINE_DETECT_DEFAULT_THRESHOLD_0 2460
#define LINE_DETECT_DEFAULT_THRESHOLD_1 2302
#define LINE_DETECT_DEFAULT_THRESHOLD_2 2328
#define LINE_DETECT_DEFAULT_THRESHOLD_3 2397
#define LINE_DETECT_DEFAULT_THRESHOLD_4 2435
#define LINE_DETECT_DEFAULT_THRESHOLD_5 2484
#define LINE_DETECT_DEFAULT_THRESHOLD_6 2605
#define LINE_DETECT_DEFAULT_THRESHOLD_7 2500

#define LINE_DETECT_DEFAULT_THRESHOLD_ARRAY \
{                                             \
    LINE_DETECT_DEFAULT_THRESHOLD_0,          \
    LINE_DETECT_DEFAULT_THRESHOLD_1,          \
    LINE_DETECT_DEFAULT_THRESHOLD_2,          \
    LINE_DETECT_DEFAULT_THRESHOLD_3,          \
    LINE_DETECT_DEFAULT_THRESHOLD_4,          \
    LINE_DETECT_DEFAULT_THRESHOLD_5,          \
    LINE_DETECT_DEFAULT_THRESHOLD_6,          \
    LINE_DETECT_DEFAULT_THRESHOLD_7           \
}

/* 黑白自动阈值安全间隔，防止黑白采样太近导致误判。 */
#define LINE_DETECT_MIN_DIFF            80U

/* 中线误差权重，单位是“千分位置”。输出范围大约 -3500 ~ +3500。
 * 经过方向开关规范化后，负数表示线路在左，正数表示线路在右。
 */
#define LINE_DETECT_ERR_LEFT_MAX        (-3500)
#define LINE_DETECT_ERR_RIGHT_MAX       (3500)

typedef enum {
    LINE_TYPE_LOST = 0,       /* 没看到黑线 */
    LINE_TYPE_SINGLE,         /* 正常单线 */
    LINE_TYPE_LEFT_BRANCH,    /* 左侧分支/左转路口 */
    LINE_TYPE_RIGHT_BRANCH,   /* 右侧分支/右转路口 */
    LINE_TYPE_CROSS,          /* 十字/大面积黑线 */
    LINE_TYPE_FULL_BLACK      /* 全黑，常用于终点/特殊标志 */
} LineType_t;

typedef struct {
    uint16_t raw[LINE_DETECT_SENSOR_NUM];       /* Driver 物理通道顺序 */
    uint16_t threshold[LINE_DETECT_SENSOR_NUM]; /* Driver 物理通道顺序 */
    uint8_t  black[LINE_DETECT_SENSOR_NUM];     /* Driver 物理通道顺序 */
    uint8_t  black_mask;                        /* 已规范化：bit0左、bit7右 */
    uint8_t  black_count;
    int16_t  error_x1000;
    int16_t  last_error_x1000;
    LineType_t type;
} LineDetect_Result_t;

void LineDetect_Init(void);

void LineDetect_Update(const uint16_t raw[LINE_DETECT_SENSOR_NUM]);
Project_Status_t LineDetect_GetResult(LineDetect_Result_t *out_result);
const LineDetect_Result_t *LineDetect_GetResultPtr(void);

void LineDetect_SetThreshold(uint8_t index, uint16_t threshold);
void LineDetect_SetAllThreshold(uint16_t threshold);
Project_Status_t LineDetect_GetThresholdArray(uint16_t *out_threshold, uint8_t max_count);

/* 白底/黑线两次采样后生成阈值：threshold = (white + black) / 2。 */
void LineDetect_CaptureWhite(const uint16_t raw[LINE_DETECT_SENSOR_NUM]);
void LineDetect_CaptureBlack(const uint16_t raw[LINE_DETECT_SENSOR_NUM]);
void LineDetect_MakeThresholdFromWhiteBlack(void);

#ifdef __cplusplus
}
#endif

#endif /* __LINE_DETECT_H */
