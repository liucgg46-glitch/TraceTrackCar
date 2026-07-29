#ifndef __DRV_GRAY_YAHBOOM_UART_H
#define __DRV_GRAY_YAHBOOM_UART_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_GRAY_YAHBOOM_CHANNEL_NUM             8U
#define DRV_GRAY_YAHBOOM_INPUT_MAX               4096U
#define DRV_GRAY_YAHBOOM_OUTPUT_MAX              4095U

/* 模块正面左到右为 X8 到 X1；工程统一约定 index 0 为最左侧。 */
#define YAHBOOM_GRAY_REVERSE_ORDER               1U
/* Yahboom 串口原始模拟量实测为白底低、黑线高；驱动反相后保持工程约定：黑线数值低。 */
#define YAHBOOM_GRAY_BLACK_IS_HIGH               1U
/* 资料建议上电后等待更久；实车调试默认3秒，若冷启动漂移再调大。 */
#define YAHBOOM_GRAY_WARMUP_MS                   3000UL
#define YAHBOOM_GRAY_FRAME_TIMEOUT_MS            100UL
#define YAHBOOM_GRAY_COMMAND_RETRY_MS            500UL
#define YAHBOOM_GRAY_UPDATE_PERIOD_MS            5UL
#define YAHBOOM_GRAY_MAX_BYTES_PER_UPDATE        96U
#define YAHBOOM_GRAY_FRAME_BUFFER_SIZE           96U
#define YAHBOOM_GRAY_FILTER_SHIFT                0U

typedef struct {
    uint16_t raw[DRV_GRAY_YAHBOOM_CHANNEL_NUM];
    uint16_t filt[DRV_GRAY_YAHBOOM_CHANNEL_NUM];

    uint8_t online;
    uint8_t valid;
    uint8_t initialized;
    uint8_t command_pending;
    uint8_t command_sent;

    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint32_t rx_overflow_count;
    uint32_t parse_error_count;
    uint32_t uart_error_count;
    uint32_t command_count;
    uint32_t last_frame_tick;
    uint32_t frame_age_ms;
} Drv_GrayYahboom_Info_t;

void Drv_GrayYahboom_Init(void);
BSP_Status_t Drv_GrayYahboom_Update(void);

uint16_t Drv_GrayYahboom_GetRaw(uint8_t index);
uint16_t Drv_GrayYahboom_GetFilt(uint8_t index);
BSP_Status_t Drv_GrayYahboom_GetRawArray(uint16_t *out_buf, uint8_t max_count);
BSP_Status_t Drv_GrayYahboom_GetFiltArray(uint16_t *out_buf, uint8_t max_count);
BSP_Status_t Drv_GrayYahboom_GetInfo(Drv_GrayYahboom_Info_t *info);
uint8_t Drv_GrayYahboom_IsOnline(void);

/*
 * 严格解析一帧完整模拟量数据。frame 必须包含 '$' 和结尾 '#';
 * 该接口无硬件副作用，供驱动内部和协议单元验证复用。
 */
BSP_Status_t Drv_GrayYahboom_ParseAnalogFrame(const uint8_t *frame,
                                              uint16_t length,
                                              uint16_t values[DRV_GRAY_YAHBOOM_CHANNEL_NUM]);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_GRAY_YAHBOOM_UART_H */
