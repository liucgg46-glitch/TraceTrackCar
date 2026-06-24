#include "bsp_uart.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

/*
 * 通用 UART/USART 非阻塞驱动实现。
 *
 * BSP 边界：本文件只负责“字节流怎么通过 UART 总线收发”。
 * 它不解析协议帧、不判断帧头帧尾、不处理 MAVLink/K210/遥控器协议。
 * 这些协议逻辑应放在上层设备/协议文件中，例如 k210_uart.c、mavlink_port.c。
 *
 * 本文件不写死任何可迁移的硬件资源，所有 USART、GPIO、DMA Stream、IRQn 都来自
 * bsp_uart.h 的配置宏。移植到新项目时，只需要修改 bsp_uart.h；本文件保持不动。
 */

typedef struct {
    USART_TypeDef       *periph;
    BSP_ClockCmdFn_t     periph_clock_fn;
    uint32_t             periph_clock_mask;
    uint32_t             dma_rcc_mask;

    uint32_t             baudrate;
    uint16_t             word_length;
    uint16_t             stop_bits;
    uint16_t             parity;
    uint16_t             flow_control;

    GPIO_TypeDef        *tx_port;
    uint16_t             tx_pin;
    uint8_t              tx_pinsrc;
    GPIO_TypeDef        *rx_port;
    uint16_t             rx_pin;
    uint8_t              rx_pinsrc;
    uint8_t              af;

    DMA_Stream_TypeDef  *dma_rx_stream;
    DMA_Stream_TypeDef  *dma_tx_stream;
    uint32_t             dma_channel;
    uint32_t             dma_rx_flags_all;
    uint32_t             dma_tx_flags_all;
    uint32_t             dma_tx_it_tc;
    uint32_t             dma_tx_it_te;
    uint32_t             dma_tx_it_dme;
    uint32_t             dma_tx_it_fe;

    IRQn_Type            usart_irqn;
    IRQn_Type            dma_tx_irqn;
} UART_Cfg_t;

static const UART_Cfg_t s_uart_cfg[UART_PORT_COUNT] = {
#if UART_PORT1_ENABLE
    [UART_PORT1] = {
        .periph            = UART_PORT1_PERIPH,
        .periph_clock_fn   = UART_PORT1_PERIPH_CLOCK_FN,
        .periph_clock_mask = UART_PORT1_PERIPH_CLOCK_MASK,
        .dma_rcc_mask      = UART_PORT1_DMA_RCC_MASK,
        .baudrate          = UART_PORT1_BAUDRATE,
        .word_length       = UART_PORT1_WORD_LENGTH,
        .stop_bits         = UART_PORT1_STOP_BITS,
        .parity            = UART_PORT1_PARITY,
        .flow_control      = UART_PORT1_FLOW_CONTROL,
        .tx_port           = UART_PORT1_TX_GPIO_PORT,
        .tx_pin            = UART_PORT1_TX_PIN,
        .tx_pinsrc         = UART_PORT1_TX_PINSRC,
        .rx_port           = UART_PORT1_RX_GPIO_PORT,
        .rx_pin            = UART_PORT1_RX_PIN,
        .rx_pinsrc         = UART_PORT1_RX_PINSRC,
        .af                = UART_PORT1_AF,
        .dma_rx_stream     = UART_PORT1_DMA_RX_STREAM,
        .dma_tx_stream     = UART_PORT1_DMA_TX_STREAM,
        .dma_channel       = UART_PORT1_DMA_CHANNEL,
        .dma_rx_flags_all  = BSP_DMA_FLAGS_ALL(UART_PORT1_DMA_RX_STREAM_NUM),
        .dma_tx_flags_all  = BSP_DMA_FLAGS_ALL(UART_PORT1_DMA_TX_STREAM_NUM),
        .dma_tx_it_tc      = BSP_DMA_IT_TC(UART_PORT1_DMA_TX_STREAM_NUM),
        .dma_tx_it_te      = BSP_DMA_IT_TE(UART_PORT1_DMA_TX_STREAM_NUM),
        .dma_tx_it_dme     = BSP_DMA_IT_DME(UART_PORT1_DMA_TX_STREAM_NUM),
        .dma_tx_it_fe      = BSP_DMA_IT_FE(UART_PORT1_DMA_TX_STREAM_NUM),
        .usart_irqn        = UART_PORT1_IRQn,
        .dma_tx_irqn       = UART_PORT1_DMA_TX_IRQn,
    },
#endif
#if UART_PORT2_ENABLE
    [UART_PORT2] = {
        .periph            = UART_PORT2_PERIPH,
        .periph_clock_fn   = UART_PORT2_PERIPH_CLOCK_FN,
        .periph_clock_mask = UART_PORT2_PERIPH_CLOCK_MASK,
        .dma_rcc_mask      = UART_PORT2_DMA_RCC_MASK,
        .baudrate          = UART_PORT2_BAUDRATE,
        .word_length       = UART_PORT2_WORD_LENGTH,
        .stop_bits         = UART_PORT2_STOP_BITS,
        .parity            = UART_PORT2_PARITY,
        .flow_control      = UART_PORT2_FLOW_CONTROL,
        .tx_port           = UART_PORT2_TX_GPIO_PORT,
        .tx_pin            = UART_PORT2_TX_PIN,
        .tx_pinsrc         = UART_PORT2_TX_PINSRC,
        .rx_port           = UART_PORT2_RX_GPIO_PORT,
        .rx_pin            = UART_PORT2_RX_PIN,
        .rx_pinsrc         = UART_PORT2_RX_PINSRC,
        .af                = UART_PORT2_AF,
        .dma_rx_stream     = UART_PORT2_DMA_RX_STREAM,
        .dma_tx_stream     = UART_PORT2_DMA_TX_STREAM,
        .dma_channel       = UART_PORT2_DMA_CHANNEL,
        .dma_rx_flags_all  = BSP_DMA_FLAGS_ALL(UART_PORT2_DMA_RX_STREAM_NUM),
        .dma_tx_flags_all  = BSP_DMA_FLAGS_ALL(UART_PORT2_DMA_TX_STREAM_NUM),
        .dma_tx_it_tc      = BSP_DMA_IT_TC(UART_PORT2_DMA_TX_STREAM_NUM),
        .dma_tx_it_te      = BSP_DMA_IT_TE(UART_PORT2_DMA_TX_STREAM_NUM),
        .dma_tx_it_dme     = BSP_DMA_IT_DME(UART_PORT2_DMA_TX_STREAM_NUM),
        .dma_tx_it_fe      = BSP_DMA_IT_FE(UART_PORT2_DMA_TX_STREAM_NUM),
        .usart_irqn        = UART_PORT2_IRQn,
        .dma_tx_irqn       = UART_PORT2_DMA_TX_IRQn,
    },
#endif
#if UART_PORT3_ENABLE
    [UART_PORT3] = {
        .periph            = UART_PORT3_PERIPH,
        .periph_clock_fn   = UART_PORT3_PERIPH_CLOCK_FN,
        .periph_clock_mask = UART_PORT3_PERIPH_CLOCK_MASK,
        .dma_rcc_mask      = UART_PORT3_DMA_RCC_MASK,
        .baudrate          = UART_PORT3_BAUDRATE,
        .word_length       = UART_PORT3_WORD_LENGTH,
        .stop_bits         = UART_PORT3_STOP_BITS,
        .parity            = UART_PORT3_PARITY,
        .flow_control      = UART_PORT3_FLOW_CONTROL,
        .tx_port           = UART_PORT3_TX_GPIO_PORT,
        .tx_pin            = UART_PORT3_TX_PIN,
        .tx_pinsrc         = UART_PORT3_TX_PINSRC,
        .rx_port           = UART_PORT3_RX_GPIO_PORT,
        .rx_pin            = UART_PORT3_RX_PIN,
        .rx_pinsrc         = UART_PORT3_RX_PINSRC,
        .af                = UART_PORT3_AF,
        .dma_rx_stream     = UART_PORT3_DMA_RX_STREAM,
        .dma_tx_stream     = UART_PORT3_DMA_TX_STREAM,
        .dma_channel       = UART_PORT3_DMA_CHANNEL,
        .dma_rx_flags_all  = BSP_DMA_FLAGS_ALL(UART_PORT3_DMA_RX_STREAM_NUM),
        .dma_tx_flags_all  = BSP_DMA_FLAGS_ALL(UART_PORT3_DMA_TX_STREAM_NUM),
        .dma_tx_it_tc      = BSP_DMA_IT_TC(UART_PORT3_DMA_TX_STREAM_NUM),
        .dma_tx_it_te      = BSP_DMA_IT_TE(UART_PORT3_DMA_TX_STREAM_NUM),
        .dma_tx_it_dme     = BSP_DMA_IT_DME(UART_PORT3_DMA_TX_STREAM_NUM),
        .dma_tx_it_fe      = BSP_DMA_IT_FE(UART_PORT3_DMA_TX_STREAM_NUM),
        .usart_irqn        = UART_PORT3_IRQn,
        .dma_tx_irqn       = UART_PORT3_DMA_TX_IRQn,
    },
#endif
#if UART_PORT4_ENABLE
    [UART_PORT4] = {
        .periph            = UART_PORT4_PERIPH,
        .periph_clock_fn   = UART_PORT4_PERIPH_CLOCK_FN,
        .periph_clock_mask = UART_PORT4_PERIPH_CLOCK_MASK,
        .dma_rcc_mask      = UART_PORT4_DMA_RCC_MASK,
        .baudrate          = UART_PORT4_BAUDRATE,
        .word_length       = UART_PORT4_WORD_LENGTH,
        .stop_bits         = UART_PORT4_STOP_BITS,
        .parity            = UART_PORT4_PARITY,
        .flow_control      = UART_PORT4_FLOW_CONTROL,
        .tx_port           = UART_PORT4_TX_GPIO_PORT,
        .tx_pin            = UART_PORT4_TX_PIN,
        .tx_pinsrc         = UART_PORT4_TX_PINSRC,
        .rx_port           = UART_PORT4_RX_GPIO_PORT,
        .rx_pin            = UART_PORT4_RX_PIN,
        .rx_pinsrc         = UART_PORT4_RX_PINSRC,
        .af                = UART_PORT4_AF,
        .dma_rx_stream     = UART_PORT4_DMA_RX_STREAM,
        .dma_tx_stream     = UART_PORT4_DMA_TX_STREAM,
        .dma_channel       = UART_PORT4_DMA_CHANNEL,
        .dma_rx_flags_all  = BSP_DMA_FLAGS_ALL(UART_PORT4_DMA_RX_STREAM_NUM),
        .dma_tx_flags_all  = BSP_DMA_FLAGS_ALL(UART_PORT4_DMA_TX_STREAM_NUM),
        .dma_tx_it_tc      = BSP_DMA_IT_TC(UART_PORT4_DMA_TX_STREAM_NUM),
        .dma_tx_it_te      = BSP_DMA_IT_TE(UART_PORT4_DMA_TX_STREAM_NUM),
        .dma_tx_it_dme     = BSP_DMA_IT_DME(UART_PORT4_DMA_TX_STREAM_NUM),
        .dma_tx_it_fe      = BSP_DMA_IT_FE(UART_PORT4_DMA_TX_STREAM_NUM),
        .usart_irqn        = UART_PORT4_IRQn,
        .dma_tx_irqn       = UART_PORT4_DMA_TX_IRQn,
    },
#endif
#if UART_PORT5_ENABLE
    [UART_PORT5] = {
        .periph            = UART_PORT5_PERIPH,
        .periph_clock_fn   = UART_PORT5_PERIPH_CLOCK_FN,
        .periph_clock_mask = UART_PORT5_PERIPH_CLOCK_MASK,
        .dma_rcc_mask      = UART_PORT5_DMA_RCC_MASK,
        .baudrate          = UART_PORT5_BAUDRATE,
        .word_length       = UART_PORT5_WORD_LENGTH,
        .stop_bits         = UART_PORT5_STOP_BITS,
        .parity            = UART_PORT5_PARITY,
        .flow_control      = UART_PORT5_FLOW_CONTROL,
        .tx_port           = UART_PORT5_TX_GPIO_PORT,
        .tx_pin            = UART_PORT5_TX_PIN,
        .tx_pinsrc         = UART_PORT5_TX_PINSRC,
        .rx_port           = UART_PORT5_RX_GPIO_PORT,
        .rx_pin            = UART_PORT5_RX_PIN,
        .rx_pinsrc         = UART_PORT5_RX_PINSRC,
        .af                = UART_PORT5_AF,
        .dma_rx_stream     = UART_PORT5_DMA_RX_STREAM,
        .dma_tx_stream     = UART_PORT5_DMA_TX_STREAM,
        .dma_channel       = UART_PORT5_DMA_CHANNEL,
        .dma_rx_flags_all  = BSP_DMA_FLAGS_ALL(UART_PORT5_DMA_RX_STREAM_NUM),
        .dma_tx_flags_all  = BSP_DMA_FLAGS_ALL(UART_PORT5_DMA_TX_STREAM_NUM),
        .dma_tx_it_tc      = BSP_DMA_IT_TC(UART_PORT5_DMA_TX_STREAM_NUM),
        .dma_tx_it_te      = BSP_DMA_IT_TE(UART_PORT5_DMA_TX_STREAM_NUM),
        .dma_tx_it_dme     = BSP_DMA_IT_DME(UART_PORT5_DMA_TX_STREAM_NUM),
        .dma_tx_it_fe      = BSP_DMA_IT_FE(UART_PORT5_DMA_TX_STREAM_NUM),
        .usart_irqn        = UART_PORT5_IRQn,
        .dma_tx_irqn       = UART_PORT5_DMA_TX_IRQn,
    },
#endif
#if UART_PORT6_ENABLE
    [UART_PORT6] = {
        .periph            = UART_PORT6_PERIPH,
        .periph_clock_fn   = UART_PORT6_PERIPH_CLOCK_FN,
        .periph_clock_mask = UART_PORT6_PERIPH_CLOCK_MASK,
        .dma_rcc_mask      = UART_PORT6_DMA_RCC_MASK,
        .baudrate          = UART_PORT6_BAUDRATE,
        .word_length       = UART_PORT6_WORD_LENGTH,
        .stop_bits         = UART_PORT6_STOP_BITS,
        .parity            = UART_PORT6_PARITY,
        .flow_control      = UART_PORT6_FLOW_CONTROL,
        .tx_port           = UART_PORT6_TX_GPIO_PORT,
        .tx_pin            = UART_PORT6_TX_PIN,
        .tx_pinsrc         = UART_PORT6_TX_PINSRC,
        .rx_port           = UART_PORT6_RX_GPIO_PORT,
        .rx_pin            = UART_PORT6_RX_PIN,
        .rx_pinsrc         = UART_PORT6_RX_PINSRC,
        .af                = UART_PORT6_AF,
        .dma_rx_stream     = UART_PORT6_DMA_RX_STREAM,
        .dma_tx_stream     = UART_PORT6_DMA_TX_STREAM,
        .dma_channel       = UART_PORT6_DMA_CHANNEL,
        .dma_rx_flags_all  = BSP_DMA_FLAGS_ALL(UART_PORT6_DMA_RX_STREAM_NUM),
        .dma_tx_flags_all  = BSP_DMA_FLAGS_ALL(UART_PORT6_DMA_TX_STREAM_NUM),
        .dma_tx_it_tc      = BSP_DMA_IT_TC(UART_PORT6_DMA_TX_STREAM_NUM),
        .dma_tx_it_te      = BSP_DMA_IT_TE(UART_PORT6_DMA_TX_STREAM_NUM),
        .dma_tx_it_dme     = BSP_DMA_IT_DME(UART_PORT6_DMA_TX_STREAM_NUM),
        .dma_tx_it_fe      = BSP_DMA_IT_FE(UART_PORT6_DMA_TX_STREAM_NUM),
        .usart_irqn        = UART_PORT6_IRQn,
        .dma_tx_irqn       = UART_PORT6_DMA_TX_IRQn,
    },
#endif
};

typedef struct {
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint16_t rx_count;
    volatile uint16_t rx_overflow;
    volatile uint16_t rx_dma_pos;

    volatile uint16_t tx_head;
    volatile uint16_t tx_tail;
    volatile uint16_t tx_count;
    volatile uint16_t tx_drop;
    volatile uint16_t tx_dma_len;
    volatile uint8_t  tx_busy;
    volatile uint8_t  initialized;
} UART_Runtime_t;

static volatile UART_Runtime_t s_uart_rt[UART_PORT_COUNT];
static uint8_t s_uart_rx_ring[UART_PORT_COUNT][UART_RX_BUF_SIZE];
static uint8_t s_uart_tx_ring[UART_PORT_COUNT][UART_TX_BUF_SIZE];
static uint8_t s_uart_dma_rx_buf[UART_PORT_COUNT][UART_DMA_RX_BUF_SIZE];

static void UART_DrainRxDMA(UART_Port_t port);
static void UART_TxStart(UART_Port_t port);

static uint16_t UART_TxFreeUnlocked(UART_Port_t port)
{
    return (uint16_t)(UART_TX_BUF_SIZE - s_uart_rt[port].tx_count);
}

static void UART_PushRxByte(UART_Port_t port, uint8_t data)
{
    volatile UART_Runtime_t *rt = &s_uart_rt[port];

    if (rt->rx_count < UART_RX_BUF_SIZE) {
        s_uart_rx_ring[port][rt->rx_head] = data;
        rt->rx_head = (uint16_t)((rt->rx_head + 1U) % UART_RX_BUF_SIZE);
        rt->rx_count++;
    } else {
        rt->rx_overflow++;
    }
}

static void UART_DrainRxDMA(UART_Port_t port)
{
    const UART_Cfg_t *cfg = &s_uart_cfg[port];
    volatile UART_Runtime_t *rt = &s_uart_rt[port];
    uint16_t pos;

    pos = (uint16_t)(UART_DMA_RX_BUF_SIZE - DMA_GetCurrDataCounter(cfg->dma_rx_stream));
    if (pos >= UART_DMA_RX_BUF_SIZE) {
        pos = 0;
    }

    while (rt->rx_dma_pos != pos) {
        UART_PushRxByte(port, s_uart_dma_rx_buf[port][rt->rx_dma_pos]);
        rt->rx_dma_pos = (uint16_t)((rt->rx_dma_pos + 1U) % UART_DMA_RX_BUF_SIZE);
    }
}

static void UART_TxStart(UART_Port_t port)
{
    const UART_Cfg_t *cfg = &s_uart_cfg[port];
    volatile UART_Runtime_t *rt = &s_uart_rt[port];
    uint16_t len;

    if (rt->tx_busy || rt->tx_count == 0U) {
        return;
    }

    len = rt->tx_count;
    if ((rt->tx_tail + len) > UART_TX_BUF_SIZE) {
        len = (uint16_t)(UART_TX_BUF_SIZE - rt->tx_tail);
    }

    rt->tx_dma_len = len;
    rt->tx_busy = 1U;

    DMA_Cmd(cfg->dma_tx_stream, DISABLE);
    if (!BSP_DMA_WaitDisable(cfg->dma_tx_stream, UART_DMA_WAIT_TIMEOUT)) {
        rt->tx_busy = 0U;
        return;
    }

    DMA_ClearFlag(cfg->dma_tx_stream, cfg->dma_tx_flags_all);
    cfg->dma_tx_stream->M0AR = (uint32_t)&s_uart_tx_ring[port][rt->tx_tail];
    cfg->dma_tx_stream->NDTR = len;
    DMA_Cmd(cfg->dma_tx_stream, ENABLE);
}

static void UART_TxFinish(UART_Port_t port)
{
    const UART_Cfg_t *cfg = &s_uart_cfg[port];
    volatile UART_Runtime_t *rt = &s_uart_rt[port];
    uint16_t sent = rt->tx_dma_len;

    DMA_Cmd(cfg->dma_tx_stream, DISABLE);
    (void)BSP_DMA_WaitDisable(cfg->dma_tx_stream, UART_DMA_WAIT_TIMEOUT);

    if (sent > rt->tx_count) {
        sent = rt->tx_count;
    }

    rt->tx_tail = (uint16_t)((rt->tx_tail + sent) % UART_TX_BUF_SIZE);
    rt->tx_count = (uint16_t)(rt->tx_count - sent);
    rt->tx_dma_len = 0U;
    rt->tx_busy = 0U;

    if (rt->tx_count > 0U) {
        UART_TxStart(port);
    }
}

void BSP_UART_Init(UART_Port_t port)
{
    const UART_Cfg_t *cfg;
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    DMA_InitTypeDef dma;
    NVIC_InitTypeDef nvic;
    uint32_t tmp;

    if (port >= UART_PORT_COUNT) {
        return;
    }

    cfg = &s_uart_cfg[port];

    BSP_GPIO_ClockEnable(cfg->tx_port);
    BSP_GPIO_ClockEnable(cfg->rx_port);
    cfg->periph_clock_fn(cfg->periph_clock_mask, ENABLE);
    RCC_AHB1PeriphClockCmd(cfg->dma_rcc_mask, ENABLE);

    GPIO_PinAFConfig(cfg->tx_port, cfg->tx_pinsrc, cfg->af);
    GPIO_PinAFConfig(cfg->rx_port, cfg->rx_pinsrc, cfg->af);

    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin   = cfg->tx_pin;
    GPIO_Init(cfg->tx_port, &gpio);
    gpio.GPIO_Pin   = cfg->rx_pin;
    GPIO_Init(cfg->rx_port, &gpio);

    USART_DeInit(cfg->periph);
    usart.USART_BaudRate            = cfg->baudrate;
    usart.USART_WordLength          = cfg->word_length;
    usart.USART_StopBits            = cfg->stop_bits;
    usart.USART_Parity              = cfg->parity;
    usart.USART_HardwareFlowControl = cfg->flow_control;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(cfg->periph, &usart);

    DMA_Cmd(cfg->dma_rx_stream, DISABLE);
    DMA_Cmd(cfg->dma_tx_stream, DISABLE);
    (void)BSP_DMA_WaitDisable(cfg->dma_rx_stream, UART_DMA_WAIT_TIMEOUT);
    (void)BSP_DMA_WaitDisable(cfg->dma_tx_stream, UART_DMA_WAIT_TIMEOUT);

    DMA_DeInit(cfg->dma_rx_stream);
    DMA_StructInit(&dma);
    dma.DMA_Channel            = cfg->dma_channel;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&cfg->periph->DR;
    dma.DMA_Memory0BaseAddr    = (uint32_t)s_uart_dma_rx_buf[port];
    dma.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize         = UART_DMA_RX_BUF_SIZE;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode               = DMA_Mode_Circular;
    dma.DMA_Priority           = DMA_Priority_High;
    dma.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(cfg->dma_rx_stream, &dma);

    DMA_DeInit(cfg->dma_tx_stream);
    DMA_StructInit(&dma);
    dma.DMA_Channel            = cfg->dma_channel;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&cfg->periph->DR;
    dma.DMA_Memory0BaseAddr    = (uint32_t)s_uart_tx_ring[port];
    dma.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    dma.DMA_BufferSize         = 1;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode               = DMA_Mode_Normal;
    dma.DMA_Priority           = DMA_Priority_High;
    dma.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(cfg->dma_tx_stream, &dma);

    DMA_ClearFlag(cfg->dma_rx_stream, cfg->dma_rx_flags_all);
    DMA_ClearFlag(cfg->dma_tx_stream, cfg->dma_tx_flags_all);
    /*
     * TX DMA 使用 Direct Mode（FIFO disabled）。
     * 不开启 FE 中断：在 STM32F4 上 FIFO disabled 时 FEIF 可能出现干扰，
     * 如果把 FE 当作发送完成/错误处理，可能导致只发出前 1~2 字节就丢弃整包。
     * 因此 TX 只开启 TC/TE/DME；FE 标志在启动前统一清除即可。
     */
    DMA_ITConfig(cfg->dma_tx_stream, DMA_IT_TC | DMA_IT_TE | DMA_IT_DME, ENABLE);

    nvic.NVIC_IRQChannel = cfg->usart_irqn;
    nvic.NVIC_IRQChannelPreemptionPriority = UART_IRQ_PREEMPT_PRIO;
    nvic.NVIC_IRQChannelSubPriority = UART_IRQ_SUB_PRIO;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = cfg->dma_tx_irqn;
    nvic.NVIC_IRQChannelPreemptionPriority = UART_DMA_TX_PREEMPT_PRIO;
    nvic.NVIC_IRQChannelSubPriority = UART_DMA_TX_SUB_PRIO;
    NVIC_Init(&nvic);

    s_uart_rt[port].rx_head = 0;
    s_uart_rt[port].rx_tail = 0;
    s_uart_rt[port].rx_count = 0;
    s_uart_rt[port].rx_overflow = 0;
    s_uart_rt[port].rx_dma_pos = 0;
    s_uart_rt[port].tx_head = 0;
    s_uart_rt[port].tx_tail = 0;
    s_uart_rt[port].tx_count = 0;
    s_uart_rt[port].tx_drop = 0;
    s_uart_rt[port].tx_dma_len = 0;
    s_uart_rt[port].tx_busy = 0;
    s_uart_rt[port].initialized = 1U;

    USART_DMACmd(cfg->periph, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(cfg->periph, USART_DMAReq_Tx, ENABLE);
    USART_ITConfig(cfg->periph, USART_IT_IDLE, ENABLE);

    DMA_Cmd(cfg->dma_rx_stream, ENABLE);
    USART_Cmd(cfg->periph, ENABLE);

    tmp = cfg->periph->SR;
    tmp = cfg->periph->DR;
    (void)tmp;
}

void BSP_UART_InitAll(void)
{
    UART_Port_t port;

    for (port = (UART_Port_t)0; port < UART_PORT_COUNT; port = (UART_Port_t)(port + 1)) {
        BSP_UART_Init(port);
    }
}

uint16_t BSP_UART_Write(UART_Port_t port, const uint8_t *data, uint16_t len)
{
    volatile UART_Runtime_t *rt;
    uint16_t written = 0;
    uint8_t need_start = 0;
    uint32_t primask;

    if (port >= UART_PORT_COUNT || data == 0 || len == 0U) {
        return 0;
    }

    rt = &s_uart_rt[port];

    primask = BSP_EnterCritical();

    while (written < len && UART_TxFreeUnlocked(port) > 0U) {
        s_uart_tx_ring[port][rt->tx_head] = data[written];
        rt->tx_head = (uint16_t)((rt->tx_head + 1U) % UART_TX_BUF_SIZE);
        rt->tx_count++;
        written++;
    }

    if (written < len) {
        rt->tx_drop = (uint16_t)(rt->tx_drop + (len - written));
    }

    if (!rt->tx_busy && rt->tx_count > 0U) {
        need_start = 1U;
    }

    BSP_ExitCritical(primask);

    if (need_start) {
        UART_TxStart(port);
    }

    return written;
}

BSP_Status_t BSP_UART_WriteFrame(UART_Port_t port, const uint8_t *data, uint16_t len)
{
    volatile UART_Runtime_t *rt;
    uint16_t i;
    uint8_t need_start = 0U;
    uint32_t primask;

    if (port >= UART_PORT_COUNT || data == 0 || len == 0U) {
        return BSP_PARAM;
    }

    rt = &s_uart_rt[port];

    /*
     * 整包发送的关键点：检查剩余空间和写入数据必须在同一个临界区内完成。
     * 否则中断可能在“刚检查完空间”之后改变 tx_count，导致协议帧被拆半。
     */
    primask = BSP_EnterCritical();

    if (UART_TxFreeUnlocked(port) < len) {
        BSP_ExitCritical(primask);
        return BSP_BUSY;
    }

    for (i = 0U; i < len; i++) {
        s_uart_tx_ring[port][rt->tx_head] = data[i];
        rt->tx_head = (uint16_t)((rt->tx_head + 1U) % UART_TX_BUF_SIZE);
        rt->tx_count++;
    }

    if (!rt->tx_busy && rt->tx_count > 0U) {
        need_start = 1U;
    }

    BSP_ExitCritical(primask);

    if (need_start) {
        UART_TxStart(port);
    }

    return BSP_OK;
}

BSP_Status_t BSP_UART_SendData_NonBlocking(UART_Port_t port, const uint8_t *data, uint16_t len)
{
    /*
     * 兼容旧名字，但语义改成“整包非阻塞发送”。
     * 返回 BSP_BUSY 时不会写入半包，避免上层协议帧被截断。
     */
    return BSP_UART_WriteFrame(port, data, len);
}

uint16_t BSP_UART_Read(UART_Port_t port, uint8_t *buf, uint16_t len)
{
    uint16_t read = 0;

    if (port >= UART_PORT_COUNT || buf == 0 || len == 0U) {
        return 0;
    }

    while (read < len && BSP_UART_GetChar(port, &buf[read])) {
        read++;
    }

    return read;
}

uint8_t BSP_UART_GetChar(UART_Port_t port, uint8_t *ch)
{
    volatile UART_Runtime_t *rt;
    uint8_t ok = 0;
    uint32_t primask;

    if (port >= UART_PORT_COUNT || ch == 0) {
        return 0U;
    }

    rt = &s_uart_rt[port];

    primask = BSP_EnterCritical();
    if (rt->rx_count > 0U) {
        *ch = s_uart_rx_ring[port][rt->rx_tail];
        rt->rx_tail = (uint16_t)((rt->rx_tail + 1U) % UART_RX_BUF_SIZE);
        rt->rx_count--;
        ok = 1U;
    }
    BSP_ExitCritical(primask);

    return ok;
}

uint16_t BSP_UART_Available(UART_Port_t port)
{
    uint16_t count;
    uint32_t primask;

    if (port >= UART_PORT_COUNT) {
        return 0;
    }

    primask = BSP_EnterCritical();
    count = s_uart_rt[port].rx_count;
    BSP_ExitCritical(primask);

    return count;
}

uint8_t BSP_UART_IsTxBusy(UART_Port_t port)
{
    if (port >= UART_PORT_COUNT) {
        return 1U;
    }
    return (uint8_t)(s_uart_rt[port].tx_busy || s_uart_rt[port].tx_count > 0U);
}

uint16_t BSP_UART_TxFree(UART_Port_t port)
{
    uint16_t free_count;
    uint32_t primask;

    if (port >= UART_PORT_COUNT) {
        return 0;
    }

    primask = BSP_EnterCritical();
    free_count = UART_TxFreeUnlocked(port);
    BSP_ExitCritical(primask);

    return free_count;
}

void BSP_UART_FlushRx(UART_Port_t port)
{
    volatile UART_Runtime_t *rt;
    uint32_t primask;

    if (port >= UART_PORT_COUNT) {
        return;
    }

    rt = &s_uart_rt[port];
    primask = BSP_EnterCritical();
    rt->rx_head = 0;
    rt->rx_tail = 0;
    rt->rx_count = 0;
    rt->rx_overflow = 0;
    rt->rx_dma_pos = (uint16_t)(UART_DMA_RX_BUF_SIZE - DMA_GetCurrDataCounter(s_uart_cfg[port].dma_rx_stream));
    if (rt->rx_dma_pos >= UART_DMA_RX_BUF_SIZE) {
        rt->rx_dma_pos = 0;
    }
    BSP_ExitCritical(primask);
}

void BSP_UART_FlushTx(UART_Port_t port)
{
    const UART_Cfg_t *cfg;
    volatile UART_Runtime_t *rt;
    uint32_t primask;

    if (port >= UART_PORT_COUNT) {
        return;
    }

    cfg = &s_uart_cfg[port];
    rt = &s_uart_rt[port];

    DMA_Cmd(cfg->dma_tx_stream, DISABLE);
    (void)BSP_DMA_WaitDisable(cfg->dma_tx_stream, UART_DMA_WAIT_TIMEOUT);
    DMA_ClearFlag(cfg->dma_tx_stream, cfg->dma_tx_flags_all);

    primask = BSP_EnterCritical();
    rt->tx_head = 0;
    rt->tx_tail = 0;
    rt->tx_count = 0;
    rt->tx_drop = 0;
    rt->tx_dma_len = 0;
    rt->tx_busy = 0;
    BSP_ExitCritical(primask);
}


BSP_Status_t BSP_UART_GetStats(UART_Port_t port, UART_Stats_t *stats)
{
    uint32_t primask;

    if (port >= UART_PORT_COUNT || stats == 0) {
        return BSP_PARAM;
    }

    primask = BSP_EnterCritical();
    stats->rx_overflow = s_uart_rt[port].rx_overflow;
    stats->tx_drop     = s_uart_rt[port].tx_drop;
    stats->rx_count    = s_uart_rt[port].rx_count;
    stats->tx_count    = s_uart_rt[port].tx_count;
    BSP_ExitCritical(primask);

    return BSP_OK;
}

void BSP_UART_ClearStats(UART_Port_t port)
{
    uint32_t primask;

    if (port >= UART_PORT_COUNT) {
        return;
    }

    primask = BSP_EnterCritical();
    s_uart_rt[port].rx_overflow = 0U;
    s_uart_rt[port].tx_drop = 0U;
    BSP_ExitCritical(primask);
}

void BSP_UART_Task(UART_Port_t port)
{
    uint32_t primask;

    if (port >= UART_PORT_COUNT) {
        return;
    }

    primask = BSP_EnterCritical();
    UART_DrainRxDMA(port);
    BSP_ExitCritical(primask);
}

void BSP_UART_TaskAll(void)
{
    UART_Port_t port;

    for (port = (UART_Port_t)0; port < UART_PORT_COUNT; port = (UART_Port_t)(port + 1)) {
        BSP_UART_Task(port);
    }
}

void BSP_UART_USART_ISR(UART_Port_t port)
{
    const UART_Cfg_t *cfg;
    uint32_t sr;
    uint32_t tmp;

    if (port >= UART_PORT_COUNT) {
        return;
    }

    cfg = &s_uart_cfg[port];
    sr = cfg->periph->SR;

    if ((sr & (USART_SR_IDLE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U) {
        tmp = cfg->periph->DR;
        (void)tmp;
        UART_DrainRxDMA(port);
    }
}

void BSP_UART_DMA_TX_ISR(UART_Port_t port)
{
    const UART_Cfg_t *cfg;

    if (port >= UART_PORT_COUNT) {
        return;
    }

    cfg = &s_uart_cfg[port];

    /*
     * 先处理真正的 DMA 错误：TE/DME。
     * 注意：不要把 FE 当作发送完成处理。TX 使用 FIFO disabled 时，FEIF 可能干扰，
     * 如果在 FE 分支调用 UART_TxFinish，会出现只收到 "U" / "UA" 这种残包。
     */
    if (DMA_GetITStatus(cfg->dma_tx_stream, cfg->dma_tx_it_te)  != RESET ||
        DMA_GetITStatus(cfg->dma_tx_stream, cfg->dma_tx_it_dme) != RESET) {
        DMA_ClearITPendingBit(cfg->dma_tx_stream,
                              cfg->dma_tx_it_te | cfg->dma_tx_it_dme | cfg->dma_tx_it_fe);
        UART_TxFinish(port);
        return;
    }

    /* 如果 FE 标志意外置位，只清掉，不结束本次发送。 */
    if (DMA_GetITStatus(cfg->dma_tx_stream, cfg->dma_tx_it_fe) != RESET) {
        DMA_ClearITPendingBit(cfg->dma_tx_stream, cfg->dma_tx_it_fe);
    }

    if (DMA_GetITStatus(cfg->dma_tx_stream, cfg->dma_tx_it_tc) != RESET) {
        DMA_ClearITPendingBit(cfg->dma_tx_stream, cfg->dma_tx_it_tc);
        UART_TxFinish(port);
    }
}

/* ---------------------------------------------------------------------------
 * 实际中断入口。启用哪个 UART_PORTx，就在本文件内生成对应 IRQHandler。
 * 若工程模板 stm32f4xx_it.c 中已有同名空函数，请删除那些空函数，避免重复定义。
 * ------------------------------------------------------------------------- */
#if UART_PORT1_ENABLE
void UART_PORT1_IRQ_HANDLER(void)
{
    BSP_UART_USART_ISR(UART_PORT1);
}

void UART_PORT1_DMA_TX_IRQ_HANDLER(void)
{
    BSP_UART_DMA_TX_ISR(UART_PORT1);
}
#endif

#if UART_PORT2_ENABLE
void UART_PORT2_IRQ_HANDLER(void)
{
    BSP_UART_USART_ISR(UART_PORT2);
}

void UART_PORT2_DMA_TX_IRQ_HANDLER(void)
{
    BSP_UART_DMA_TX_ISR(UART_PORT2);
}
#endif

#if UART_PORT3_ENABLE
void UART_PORT3_IRQ_HANDLER(void)
{
    BSP_UART_USART_ISR(UART_PORT3);
}

void UART_PORT3_DMA_TX_IRQ_HANDLER(void)
{
    BSP_UART_DMA_TX_ISR(UART_PORT3);
}
#endif

#if UART_PORT4_ENABLE
void UART_PORT4_IRQ_HANDLER(void)
{
    BSP_UART_USART_ISR(UART_PORT4);
}

void UART_PORT4_DMA_TX_IRQ_HANDLER(void)
{
    BSP_UART_DMA_TX_ISR(UART_PORT4);
}
#endif

#if UART_PORT5_ENABLE
void UART_PORT5_IRQ_HANDLER(void)
{
    BSP_UART_USART_ISR(UART_PORT5);
}

void UART_PORT5_DMA_TX_IRQ_HANDLER(void)
{
    BSP_UART_DMA_TX_ISR(UART_PORT5);
}
#endif

#if UART_PORT6_ENABLE
void UART_PORT6_IRQ_HANDLER(void)
{
    BSP_UART_USART_ISR(UART_PORT6);
}

void UART_PORT6_DMA_TX_IRQ_HANDLER(void)
{
    BSP_UART_DMA_TX_ISR(UART_PORT6);
}
#endif
