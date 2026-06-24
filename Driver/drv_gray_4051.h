#ifndef __DRV_GRAY_4051_H
#define __DRV_GRAY_4051_H

#include "bsp_common.h"
#include "bsp_adc.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 8 路 74HC4051 灰度传感器驱动层：drv_gray_4051
 * ============================================================================
 * 硬件结构：
 *   8 路灰度模拟量 -> 74HC4051 -> 1 路 ADC
 *   S0/S1/S2       -> 3 路 GPIO
 *
 * 分层定位：
 *   - BSP_ADC 只采 ADC 原始值；
 *   - BSP_GPIO 只控制 S0/S1/S2；
 *   - 本 Driver 只完成 4051 选通和 8 路 raw/filt 数据缓存；
 *   - 黑白阈值、误差计算、路口识别放在 Algorithm/line_detect.c；
 *   - 循迹速度/转向控制放在 Algorithm/line_track.c + App/line_follow_app.c。
 *
 * 调用周期：
 *   Drv_Gray4051_Update() 建议 1ms 调用一次。默认每 1ms 完成一路，8ms 刷新完 8 路。
 */

#define DRV_GRAY_4051_CHANNEL_NUM         8U

/* 4051 OUT/SIG/AO 接到哪一路 BSP_ADC。当前规划：PC0 / ADC1_IN10 / BSP_ADC_CH1。 */
#define DRV_GRAY_4051_ADC_CH              BSP_ADC_CH1

/* 选通后等待 ADC 稳定的时间。1ms 对普通灰度模块足够，太大则刷新变慢。 */
#define DRV_GRAY_4051_SETTLE_MS           1U

/* 一阶滤波：0=关闭；1~7=越大越平滑，响应越慢。
 * filt = filt + (raw - filt) / 2^SHIFT
 */
#define DRV_GRAY_4051_FILTER_SHIFT        2U

/* 是否反向排列传感器编号。
 * 默认：index 0 是最左，index 7 是最右。
 * 如果你发现左边压线却右侧数据变化，把这里改 1。
 */
#define DRV_GRAY_4051_INDEX_REVERSE       0U

typedef struct {
    uint16_t raw[DRV_GRAY_4051_CHANNEL_NUM];
    uint16_t filt[DRV_GRAY_4051_CHANNEL_NUM];
    uint8_t  scan_index;
    uint8_t  valid;
} Drv_Gray4051_Info_t;

void Drv_Gray4051_Init(void);
void Drv_Gray4051_Update(void);

uint16_t Drv_Gray4051_GetRaw(uint8_t index);
uint16_t Drv_Gray4051_GetFilt(uint8_t index);
BSP_Status_t Drv_Gray4051_GetRawArray(uint16_t *out_buf, uint8_t max_count);
BSP_Status_t Drv_Gray4051_GetFiltArray(uint16_t *out_buf, uint8_t max_count);
BSP_Status_t Drv_Gray4051_GetInfo(Drv_Gray4051_Info_t *info);
uint8_t Drv_Gray4051_IsValid(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_GRAY_4051_H */
