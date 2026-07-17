#include "drv_e220.h"

#include "bsp_gpio.h"
#include "bsp_uart.h"

#if VEHICLE_UART1_E220_ENABLE
typedef struct {
    uint16_t len;
    uint8_t data[DRV_E220_TX_FRAME_MAX_SIZE];
} Drv_E220_TxFrame_t;

static Drv_E220_TxFrame_t s_tx_queue[DRV_E220_TX_QUEUE_DEPTH];
static volatile uint16_t s_tx_head;
static volatile uint16_t s_tx_tail;
static volatile uint16_t s_tx_count;
static volatile uint32_t s_tx_deferred_frames;
static volatile uint32_t s_tx_retried_frames;
static volatile uint32_t s_tx_queue_full_frames;
static volatile uint32_t s_tx_oversize_frames;

#if (DRV_E220_TX_QUEUE_DEPTH == 0U)
#error "DRV_E220_TX_QUEUE_DEPTH must be greater than zero"
#endif

#if (DRV_E220_TX_FRAME_MAX_SIZE > UART_TX_BUF_SIZE)
#error "DRV_E220_TX_FRAME_MAX_SIZE must not exceed UART_TX_BUF_SIZE"
#endif

static void Drv_E220_ResetQueue(void)
{
    uint32_t primask = BSP_EnterCritical();

    s_tx_head = 0U;
    s_tx_tail = 0U;
    s_tx_count = 0U;
    s_tx_deferred_frames = 0U;
    s_tx_retried_frames = 0U;
    s_tx_queue_full_frames = 0U;
    s_tx_oversize_frames = 0U;

    BSP_ExitCritical(primask);
}

static uint8_t Drv_E220_TxReadyGuard(void)
{
    /*
     * 软件队列中仍有待发帧时，新帧不能插到旧帧之前，
     * 否则日志行的先后顺序会被打乱。
     */
    if (s_tx_count > 0U) {
        return 0U;
    }

    if (BSP_GPIO_Read(BSP_GPIO_E220_AUX) == 0U) {
        return 0U;
    }

    return (BSP_UART_IsTxBusy(UART_PORT1) == 0U) ? 1U : 0U;
}

static BSP_Status_t Drv_E220_DeferFrame(const uint8_t *data, uint16_t len)
{
    Drv_E220_TxFrame_t *frame;
    uint16_t i;
    uint32_t primask;

    if (data == 0 || len == 0U) {
        return BSP_PARAM;
    }

    if (len > DRV_E220_TX_FRAME_MAX_SIZE) {
        s_tx_oversize_frames++;
        return BSP_PARAM;
    }

    primask = BSP_EnterCritical();
    if (s_tx_count >= DRV_E220_TX_QUEUE_DEPTH) {
        s_tx_queue_full_frames++;
        BSP_ExitCritical(primask);
        return BSP_BUSY;
    }

    frame = &s_tx_queue[s_tx_head];
    frame->len = len;
    for (i = 0U; i < len; i++) {
        frame->data[i] = data[i];
    }

    s_tx_head = (uint16_t)((s_tx_head + 1U) % DRV_E220_TX_QUEUE_DEPTH);
    s_tx_count++;
    s_tx_deferred_frames++;
    BSP_ExitCritical(primask);

    return BSP_OK;
}
#endif

void Drv_E220_Init(void)
{
#if VEHICLE_UART1_E220_ENABLE
    Drv_E220_ResetQueue();
    BSP_UART_SetTxReadyGuard(UART_PORT1, Drv_E220_TxReadyGuard);
    BSP_UART_SetTxDeferredHandler(UART_PORT1, Drv_E220_DeferFrame);
#else
    BSP_UART_SetTxReadyGuard(UART_PORT1, 0);
    BSP_UART_SetTxDeferredHandler(UART_PORT1, 0);
#endif
}

uint8_t Drv_E220_IsReady(void)
{
#if VEHICLE_UART1_E220_ENABLE
    return Drv_E220_TxReadyGuard();
#else
    return 1U;
#endif
}

void Drv_E220_Task(void)
{
#if VEHICLE_UART1_E220_ENABLE
    Drv_E220_TxFrame_t *frame;
    BSP_Status_t status;
    uint32_t primask;

    if (s_tx_count == 0U || BSP_GPIO_Read(BSP_GPIO_E220_AUX) == 0U ||
        BSP_UART_IsTxBusy(UART_PORT1) != 0U) {
        return;
    }

    frame = &s_tx_queue[s_tx_tail];
    status = BSP_UART_WriteFrameNow(UART_PORT1, frame->data, frame->len);
    if (status != BSP_OK) {
        return;
    }

    primask = BSP_EnterCritical();
    s_tx_tail = (uint16_t)((s_tx_tail + 1U) % DRV_E220_TX_QUEUE_DEPTH);
    s_tx_count--;
    s_tx_retried_frames++;
    BSP_ExitCritical(primask);
#endif
}

BSP_Status_t Drv_E220_GetTxStats(Drv_E220_TxStats_t *stats)
{
    uint32_t primask;

    if (stats == 0) {
        return BSP_PARAM;
    }

    primask = BSP_EnterCritical();
#if VEHICLE_UART1_E220_ENABLE
    stats->queued_frames = s_tx_count;
    stats->deferred_frames = s_tx_deferred_frames;
    stats->retried_frames = s_tx_retried_frames;
    stats->queue_full_frames = s_tx_queue_full_frames;
    stats->oversize_frames = s_tx_oversize_frames;
#else
    stats->queued_frames = 0U;
    stats->deferred_frames = 0U;
    stats->retried_frames = 0U;
    stats->queue_full_frames = 0U;
    stats->oversize_frames = 0U;
#endif
    BSP_ExitCritical(primask);

    return BSP_OK;
}

void Drv_E220_ClearTxStats(void)
{
#if VEHICLE_UART1_E220_ENABLE
    uint32_t primask = BSP_EnterCritical();

    s_tx_deferred_frames = 0U;
    s_tx_retried_frames = 0U;
    s_tx_queue_full_frames = 0U;
    s_tx_oversize_frames = 0U;

    BSP_ExitCritical(primask);
#endif
}
