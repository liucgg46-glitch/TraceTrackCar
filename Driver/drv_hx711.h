#ifndef __DRV_HX711_H
#define __DRV_HX711_H

#include "bsp_common.h"
#include "bsp_gpio.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * HX711 24 位称重 ADC 驱动
 * ============================================================================
 *
 * 默认使用通道 A、128 倍增益和模块板载采样率配置。驱动只在 DOUT 拉低时读取
 * 一帧数据，不等待转换完成；因此 Drv_HX711_Update() 可由 1 ms 任务安全推进。
 *
 * 克重换算关系：
 *   pressure_g = (filtered_counts - tare_offset_counts) / scale_counts_per_g
 *
 * 不同称重传感器、供电和机械安装的灵敏度不同，不能使用通用固定比例。本项目
 * 已按 160.1 g 实测值固化比例；每次上电保持称重台空载，驱动会稳定后自动去皮。
 */

#define DRV_HX711_ENABLE                         1U
#define DRV_HX711_DOUT_GPIO                      BSP_GPIO_HX711_DOUT
#define DRV_HX711_PD_SCK_GPIO                    BSP_GPIO_HX711_PD_SCK

/* 10 SPS 模块建议使用 8 点均值；切到 80 SPS 后可按响应需求减小。 */
#define DRV_HX711_AVERAGE_SAMPLES                8U

/*
 * 上电自动去皮：连续收集一组稳定的空载原始值作为零点。
 * 32 个样本在 10 SPS 模式下约需 3.2 秒；窗口跨度过大时重新开始，避免碰动称重台
 * 时误记零点。自动去皮完成前克重接口返回 BSP_ERROR。
 */
#define DRV_HX711_AUTO_TARE_ENABLE               1U
#define DRV_HX711_AUTO_TARE_SAMPLE_COUNT        32U
#define DRV_HX711_AUTO_TARE_MAX_SPAN_COUNTS    500L

/* 超过该时间没有新转换时判定离线。10 SPS 正常转换周期约为 100 ms。 */
#define DRV_HX711_OFFLINE_TIMEOUT_MS             500U

/* 标定时原始变化量过小通常表示未放砝码或传感器接线异常。 */
#define DRV_HX711_MIN_CALIBRATION_DELTA_COUNTS   100L

/*
 * 实测标定：空载均值约 -540045.3，160.1 g 负载均值约 -428626.8，
 * scale = (-428626.8 - -540045.3) / 160.1 = 695.93 counts/g。
 */
#define DRV_HX711_DEFAULT_SCALE_COUNTS_PER_G   695.93f

typedef enum {
    DRV_HX711_CHANNEL_A_GAIN_128 = 1,
    DRV_HX711_CHANNEL_B_GAIN_32  = 2,
    DRV_HX711_CHANNEL_A_GAIN_64  = 3
} Drv_HX711_Gain_t;

typedef struct {
    int32_t raw_counts;
    int32_t filtered_counts;
    int32_t tare_offset_counts;
    float scale_counts_per_g;
    float pressure_g;
    uint32_t timestamp_ms;
    uint32_t sample_count;
    uint32_t error_count;
    uint32_t timeout_count;
    uint8_t initialized;
    uint8_t online;
    uint8_t data_valid;
    uint8_t calibrated;
    uint8_t tare_ready;
    uint8_t powered_down;
    Drv_HX711_Gain_t gain;
} Drv_HX711_Info_t;

void Drv_HX711_Init(void);
BSP_Status_t Drv_HX711_Update(void);

BSP_Status_t Drv_HX711_GetRaw(int32_t *raw_counts);
BSP_Status_t Drv_HX711_GetFiltered(int32_t *filtered_counts);
BSP_Status_t Drv_HX711_GetGram(float *pressure_g);
BSP_Status_t Drv_HX711_GetInfo(Drv_HX711_Info_t *info);

BSP_Status_t Drv_HX711_Tare(void);
BSP_Status_t Drv_HX711_CalibrateKnownWeight(float known_weight_g);
BSP_Status_t Drv_HX711_SetScale(float counts_per_g);
BSP_Status_t Drv_HX711_SetGain(Drv_HX711_Gain_t gain);

void Drv_HX711_PowerDown(void);
void Drv_HX711_PowerUp(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_HX711_H */
