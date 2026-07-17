#ifndef __DRV_E220_H
#define __DRV_E220_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * USART1 使用的 E220 透明传输适配层。
 *
 * UART 电气参数仍在 BSP/bsp_uart.h 中配置：
 *   USART1 TX 使用 PA9，RX 使用 PA10，115200、8N1、无硬件流控。
 * E220 AUX 使用 BSP_GPIO_E220_AUX（PE6），按普通 GPIO 输入轮询。
 */
#ifndef DRV_E220_TX_QUEUE_DEPTH
#define DRV_E220_TX_QUEUE_DEPTH          16U
#endif

#ifndef DRV_E220_TX_FRAME_MAX_SIZE
#define DRV_E220_TX_FRAME_MAX_SIZE       512U
#endif

typedef struct {
    uint16_t queued_frames;
    uint32_t deferred_frames;
    uint32_t retried_frames;
    uint32_t queue_full_frames;
    uint32_t oversize_frames;
} Drv_E220_TxStats_t;

void Drv_E220_Init(void);
uint8_t Drv_E220_IsReady(void);
void Drv_E220_Task(void);
BSP_Status_t Drv_E220_GetTxStats(Drv_E220_TxStats_t *stats);
void Drv_E220_ClearTxStats(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_E220_H */
