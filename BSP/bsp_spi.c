#include "bsp_spi.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"

/*
 * 本文件不出现任何具体的外设/引脚名字（SPI1、PA5...），全部通过
 * bsp_spi.h 里的配置宏 + SPI_Bus_t 枚举下标驱动。新增/切换通道只需要改
 * bsp_spi.h，这个文件不需要任何修改。
 *
 * BSP 边界：本文件只负责 SPI 总线字节收发，不管理具体设备的寄存器、命令、
 * CS/NSS 引脚。CS 应放在设备层控制，例如 pmw3901.c 在每次寄存器读写前后
 * 自己拉低/拉高片选。
 */

/* ===========================================================================
 * 每路通道的静态配置表：编译期由 bsp_spi.h 里的 #define 生成。
 * =========================================================================== */
typedef struct {
    SPI_TypeDef          *periph;
    BSP_ClockCmdFn_t       periph_clock_fn;
    uint32_t               periph_clock_mask;
    uint32_t               dma_rcc_mask;

    GPIO_TypeDef          *sck_port;
    uint16_t                sck_pin;
    uint8_t                 sck_pinsrc;
    GPIO_TypeDef          *miso_port;
    uint16_t                miso_pin;
    uint8_t                 miso_pinsrc;
    GPIO_TypeDef          *mosi_port;
    uint16_t                mosi_pin;
    uint8_t                 mosi_pinsrc;
    uint8_t                 af;

    uint16_t                cpol;
    uint16_t                cpha;
    uint16_t                baud_prescaler;

    DMA_Stream_TypeDef     *dma_rx_stream;
    DMA_Stream_TypeDef     *dma_tx_stream;
    uint32_t                dma_channel;

    uint32_t                dma_rx_flags_all;
    uint32_t                dma_rx_it_tc;
    uint32_t                dma_rx_it_te;
    uint32_t                dma_rx_it_dme;
    uint32_t                dma_rx_it_fe;

    uint32_t                dma_tx_flags_all;
    uint32_t                dma_tx_it_te;
    uint32_t                dma_tx_it_dme;
    uint32_t                dma_tx_it_fe;

    IRQn_Type                dma_rx_irqn;
    IRQn_Type                dma_tx_irqn;
} SPI_Cfg_t;

static const SPI_Cfg_t s_spi_cfg[SPI_BUS_COUNT] = {
#if SPI_BUS1_ENABLE
    [SPI_BUS1] = {
        .periph             = SPI_BUS1_PERIPH,
        .periph_clock_fn    = SPI_BUS1_PERIPH_CLOCK_FN,
        .periph_clock_mask  = SPI_BUS1_PERIPH_CLOCK_MASK,
        .dma_rcc_mask       = SPI_BUS1_DMA_RCC_MASK,
        .sck_port           = SPI_BUS1_SCK_GPIO_PORT,
        .sck_pin            = SPI_BUS1_SCK_PIN,
        .sck_pinsrc         = SPI_BUS1_SCK_PINSRC,
        .miso_port          = SPI_BUS1_MISO_GPIO_PORT,
        .miso_pin           = SPI_BUS1_MISO_PIN,
        .miso_pinsrc        = SPI_BUS1_MISO_PINSRC,
        .mosi_port          = SPI_BUS1_MOSI_GPIO_PORT,
        .mosi_pin           = SPI_BUS1_MOSI_PIN,
        .mosi_pinsrc        = SPI_BUS1_MOSI_PINSRC,
        .af                 = SPI_BUS1_AF,
        .cpol               = SPI_BUS1_CPOL,
        .cpha               = SPI_BUS1_CPHA,
        .baud_prescaler     = SPI_BUS1_BAUD_PRESCALER,
        .dma_rx_stream      = SPI_BUS1_DMA_RX_STREAM,
        .dma_tx_stream      = SPI_BUS1_DMA_TX_STREAM,
        .dma_channel        = SPI_BUS1_DMA_CHANNEL,
        .dma_rx_flags_all   = BSP_DMA_FLAGS_ALL(SPI_BUS1_DMA_RX_STREAM_NUM),
        .dma_rx_it_tc       = BSP_DMA_IT_TC(SPI_BUS1_DMA_RX_STREAM_NUM),
        .dma_rx_it_te       = BSP_DMA_IT_TE(SPI_BUS1_DMA_RX_STREAM_NUM),
        .dma_rx_it_dme      = BSP_DMA_IT_DME(SPI_BUS1_DMA_RX_STREAM_NUM),
        .dma_rx_it_fe       = BSP_DMA_IT_FE(SPI_BUS1_DMA_RX_STREAM_NUM),
        .dma_tx_flags_all   = BSP_DMA_FLAGS_ALL(SPI_BUS1_DMA_TX_STREAM_NUM),
        .dma_tx_it_te       = BSP_DMA_IT_TE(SPI_BUS1_DMA_TX_STREAM_NUM),
        .dma_tx_it_dme      = BSP_DMA_IT_DME(SPI_BUS1_DMA_TX_STREAM_NUM),
        .dma_tx_it_fe       = BSP_DMA_IT_FE(SPI_BUS1_DMA_TX_STREAM_NUM),
        .dma_rx_irqn        = SPI_BUS1_DMA_RX_IRQn,
        .dma_tx_irqn        = SPI_BUS1_DMA_TX_IRQn,
    },
#endif
#if SPI_BUS2_ENABLE
    [SPI_BUS2] = {
        .periph             = SPI_BUS2_PERIPH,
        .periph_clock_fn    = SPI_BUS2_PERIPH_CLOCK_FN,
        .periph_clock_mask  = SPI_BUS2_PERIPH_CLOCK_MASK,
        .dma_rcc_mask       = SPI_BUS2_DMA_RCC_MASK,
        .sck_port           = SPI_BUS2_SCK_GPIO_PORT,
        .sck_pin            = SPI_BUS2_SCK_PIN,
        .sck_pinsrc         = SPI_BUS2_SCK_PINSRC,
        .miso_port          = SPI_BUS2_MISO_GPIO_PORT,
        .miso_pin           = SPI_BUS2_MISO_PIN,
        .miso_pinsrc        = SPI_BUS2_MISO_PINSRC,
        .mosi_port          = SPI_BUS2_MOSI_GPIO_PORT,
        .mosi_pin           = SPI_BUS2_MOSI_PIN,
        .mosi_pinsrc        = SPI_BUS2_MOSI_PINSRC,
        .af                 = SPI_BUS2_AF,
        .cpol               = SPI_BUS2_CPOL,
        .cpha               = SPI_BUS2_CPHA,
        .baud_prescaler     = SPI_BUS2_BAUD_PRESCALER,
        .dma_rx_stream      = SPI_BUS2_DMA_RX_STREAM,
        .dma_tx_stream      = SPI_BUS2_DMA_TX_STREAM,
        .dma_channel        = SPI_BUS2_DMA_CHANNEL,
        .dma_rx_flags_all   = BSP_DMA_FLAGS_ALL(SPI_BUS2_DMA_RX_STREAM_NUM),
        .dma_rx_it_tc       = BSP_DMA_IT_TC(SPI_BUS2_DMA_RX_STREAM_NUM),
        .dma_rx_it_te       = BSP_DMA_IT_TE(SPI_BUS2_DMA_RX_STREAM_NUM),
        .dma_rx_it_dme      = BSP_DMA_IT_DME(SPI_BUS2_DMA_RX_STREAM_NUM),
        .dma_rx_it_fe       = BSP_DMA_IT_FE(SPI_BUS2_DMA_RX_STREAM_NUM),
        .dma_tx_flags_all   = BSP_DMA_FLAGS_ALL(SPI_BUS2_DMA_TX_STREAM_NUM),
        .dma_tx_it_te       = BSP_DMA_IT_TE(SPI_BUS2_DMA_TX_STREAM_NUM),
        .dma_tx_it_dme      = BSP_DMA_IT_DME(SPI_BUS2_DMA_TX_STREAM_NUM),
        .dma_tx_it_fe       = BSP_DMA_IT_FE(SPI_BUS2_DMA_TX_STREAM_NUM),
        .dma_rx_irqn        = SPI_BUS2_DMA_RX_IRQn,
        .dma_tx_irqn        = SPI_BUS2_DMA_TX_IRQn,
    },
#endif
#if SPI_BUS3_ENABLE
    [SPI_BUS3] = {
        .periph             = SPI_BUS3_PERIPH,
        .periph_clock_fn    = SPI_BUS3_PERIPH_CLOCK_FN,
        .periph_clock_mask  = SPI_BUS3_PERIPH_CLOCK_MASK,
        .dma_rcc_mask       = SPI_BUS3_DMA_RCC_MASK,
        .sck_port           = SPI_BUS3_SCK_GPIO_PORT,
        .sck_pin            = SPI_BUS3_SCK_PIN,
        .sck_pinsrc         = SPI_BUS3_SCK_PINSRC,
        .miso_port          = SPI_BUS3_MISO_GPIO_PORT,
        .miso_pin           = SPI_BUS3_MISO_PIN,
        .miso_pinsrc        = SPI_BUS3_MISO_PINSRC,
        .mosi_port          = SPI_BUS3_MOSI_GPIO_PORT,
        .mosi_pin           = SPI_BUS3_MOSI_PIN,
        .mosi_pinsrc        = SPI_BUS3_MOSI_PINSRC,
        .af                 = SPI_BUS3_AF,
        .cpol               = SPI_BUS3_CPOL,
        .cpha               = SPI_BUS3_CPHA,
        .baud_prescaler     = SPI_BUS3_BAUD_PRESCALER,
        .dma_rx_stream      = SPI_BUS3_DMA_RX_STREAM,
        .dma_tx_stream      = SPI_BUS3_DMA_TX_STREAM,
        .dma_channel        = SPI_BUS3_DMA_CHANNEL,
        .dma_rx_flags_all   = BSP_DMA_FLAGS_ALL(SPI_BUS3_DMA_RX_STREAM_NUM),
        .dma_rx_it_tc       = BSP_DMA_IT_TC(SPI_BUS3_DMA_RX_STREAM_NUM),
        .dma_rx_it_te       = BSP_DMA_IT_TE(SPI_BUS3_DMA_RX_STREAM_NUM),
        .dma_rx_it_dme      = BSP_DMA_IT_DME(SPI_BUS3_DMA_RX_STREAM_NUM),
        .dma_rx_it_fe       = BSP_DMA_IT_FE(SPI_BUS3_DMA_RX_STREAM_NUM),
        .dma_tx_flags_all   = BSP_DMA_FLAGS_ALL(SPI_BUS3_DMA_TX_STREAM_NUM),
        .dma_tx_it_te       = BSP_DMA_IT_TE(SPI_BUS3_DMA_TX_STREAM_NUM),
        .dma_tx_it_dme      = BSP_DMA_IT_DME(SPI_BUS3_DMA_TX_STREAM_NUM),
        .dma_tx_it_fe       = BSP_DMA_IT_FE(SPI_BUS3_DMA_TX_STREAM_NUM),
        .dma_rx_irqn        = SPI_BUS3_DMA_RX_IRQn,
        .dma_tx_irqn        = SPI_BUS3_DMA_TX_IRQn,
    },
#endif
};

/* ===========================================================================
 * 每路通道的运行时状态。
 * =========================================================================== */
typedef struct {
    volatile uint8_t busy;
    volatile uint8_t cb_called;
    SPI_Callback_t   done_cb;
    void            *done_ctx;
    uint32_t         start_tick;
} SPI_Runtime_t;

static volatile SPI_Runtime_t s_spi_rt[SPI_BUS_COUNT];

/* ==================== 内部工具函数 ==================== */
static void SPI_FinishFromISR(SPI_Bus_t bus, BSP_Status_t status)
{
    const SPI_Cfg_t *cfg = &s_spi_cfg[bus];
    volatile SPI_Runtime_t *rt = &s_spi_rt[bus];
    SPI_Callback_t cb;
    void *ctx;

    if (rt->cb_called) {
        return;
    }
    rt->cb_called = 1;

    SPI_I2S_DMACmd(cfg->periph, SPI_I2S_DMAReq_Rx, DISABLE);
    SPI_I2S_DMACmd(cfg->periph, SPI_I2S_DMAReq_Tx, DISABLE);

    DMA_Cmd(cfg->dma_rx_stream, DISABLE);
    DMA_Cmd(cfg->dma_tx_stream, DISABLE);

    cb  = rt->done_cb;
    ctx = rt->done_ctx;

    rt->done_cb  = 0;
    rt->done_ctx = 0;
    rt->busy     = 0;

    if (cb != 0) {
        cb(bus, ctx, status);
    }
}

/* ==================== 初始化 ==================== */
void BSP_SPI_Init(SPI_Bus_t bus)
{
    const SPI_Cfg_t *cfg;
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    if (bus >= SPI_BUS_COUNT) return;
    cfg = &s_spi_cfg[bus];

    BSP_GPIO_ClockEnable(cfg->sck_port);
    BSP_GPIO_ClockEnable(cfg->miso_port);
    BSP_GPIO_ClockEnable(cfg->mosi_port);
    cfg->periph_clock_fn(cfg->periph_clock_mask, ENABLE);
    RCC_AHB1PeriphClockCmd(cfg->dma_rcc_mask, ENABLE);

    GPIO_PinAFConfig(cfg->sck_port,  cfg->sck_pinsrc,  cfg->af);
    GPIO_PinAFConfig(cfg->miso_port, cfg->miso_pinsrc, cfg->af);
    GPIO_PinAFConfig(cfg->mosi_port, cfg->mosi_pinsrc, cfg->af);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = cfg->sck_pin;
    GPIO_Init(cfg->sck_port, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = cfg->miso_pin;
    GPIO_Init(cfg->miso_port, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = cfg->mosi_pin;
    GPIO_Init(cfg->mosi_port, &GPIO_InitStructure);

    SPI_I2S_DeInit(cfg->periph);
    SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL              = cfg->cpol;
    SPI_InitStructure.SPI_CPHA              = cfg->cpha;
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = cfg->baud_prescaler;
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial     = 7;
    SPI_Init(cfg->periph, &SPI_InitStructure);

    SPI_NSSInternalSoftwareConfig(cfg->periph, SPI_NSSInternalSoft_Set);

    /* ---- DMA 模板初始化：先建好通道，真正传输时再重设地址/长度并使能 ---- */
    DMA_DeInit(cfg->dma_rx_stream);
    (void)BSP_DMA_WaitDisable(cfg->dma_rx_stream, SPI_BLOCK_TIMEOUT);

    DMA_InitStructure.DMA_Channel            = cfg->dma_channel;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&cfg->periph->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = 0;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = 1;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(cfg->dma_rx_stream, &DMA_InitStructure);
    DMA_ITConfig(cfg->dma_rx_stream, DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);

    DMA_DeInit(cfg->dma_tx_stream);
    (void)BSP_DMA_WaitDisable(cfg->dma_tx_stream, SPI_BLOCK_TIMEOUT);

    DMA_InitStructure.DMA_Channel            = cfg->dma_channel;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&cfg->periph->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = 0;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize         = 1;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(cfg->dma_tx_stream, &DMA_InitStructure);
    DMA_ITConfig(cfg->dma_tx_stream, DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = cfg->dma_rx_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = cfg->dma_tx_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    SPI_Cmd(cfg->periph, ENABLE);

    s_spi_rt[bus].busy      = 0;
    s_spi_rt[bus].cb_called = 0;
    s_spi_rt[bus].done_cb   = 0;
    s_spi_rt[bus].done_ctx  = 0;
}

void BSP_SPI_InitAll(void)
{
    for (SPI_Bus_t bus = (SPI_Bus_t)0; bus < SPI_BUS_COUNT; bus = (SPI_Bus_t)(bus + 1)) {
        BSP_SPI_Init(bus);
    }
}

/* ==================== 阻塞 API ==================== */
uint8_t BSP_SPI_TransferByte(SPI_Bus_t bus, uint8_t tx, uint8_t *rx)
{
    SPI_TypeDef *periph;
    uint32_t timeout;

    if (bus >= SPI_BUS_COUNT || rx == 0) {
        return 0;
    }
    periph = s_spi_cfg[bus].periph;

    timeout = SPI_BLOCK_TIMEOUT;
    while (SPI_I2S_GetFlagStatus(periph, SPI_I2S_FLAG_TXE) == RESET) {
        if (--timeout == 0U) {
            return 0;
        }
    }

    SPI_I2S_SendData(periph, tx);

    timeout = SPI_BLOCK_TIMEOUT;
    while (SPI_I2S_GetFlagStatus(periph, SPI_I2S_FLAG_RXNE) == RESET) {
        if (--timeout == 0U) {
            return 0;
        }
    }

    *rx = (uint8_t)SPI_I2S_ReceiveData(periph);
    return 1;
}


BSP_Status_t BSP_SPI_Transfer(SPI_Bus_t bus,
                              const uint8_t *tx_buf,
                              uint8_t *rx_buf,
                              uint16_t len,
                              uint8_t dummy_tx)
{
    uint16_t i;
    uint8_t rx_dummy;
    uint8_t tx;

    if (bus >= SPI_BUS_COUNT || len == 0U) {
        return BSP_PARAM;
    }

    for (i = 0U; i < len; i++) {
        tx = (tx_buf != 0) ? tx_buf[i] : dummy_tx;

        if (!BSP_SPI_TransferByte(bus, tx, &rx_dummy)) {
            return BSP_TIMEOUT;
        }

        if (rx_buf != 0) {
            rx_buf[i] = rx_dummy;
        }
    }

    return BSP_OK;
}

uint8_t BSP_SPI_ReadWriteByte(SPI_Bus_t bus, uint8_t data)
{
    uint8_t rx = 0xFF;
    (void)BSP_SPI_TransferByte(bus, data, &rx);
    return rx;
}

/* ==================== 非阻塞 API ==================== */
BSP_Status_t BSP_SPI_TransferAsync_DMA(SPI_Bus_t bus,
                                        uint8_t *tx_buf,
                                        uint8_t *rx_buf,
                                        uint16_t len,
                                        SPI_Callback_t cb,
                                        void *ctx)
{
    const SPI_Cfg_t *cfg;
    volatile SPI_Runtime_t *rt;
    uint8_t dummy;
    DMA_InitTypeDef DMA_InitStructure;
    uint32_t primask;

    if (bus >= SPI_BUS_COUNT) return BSP_PARAM;
    if (len == 0 || tx_buf == 0 || rx_buf == 0) return BSP_PARAM;

    cfg = &s_spi_cfg[bus];
    rt  = &s_spi_rt[bus];

    primask = BSP_EnterCritical();

    if (rt->busy) {
        BSP_ExitCritical(primask);
        return BSP_BUSY;
    }

    rt->busy       = 1;
    rt->cb_called  = 0;
    rt->done_cb    = cb;
    rt->done_ctx   = ctx;
    rt->start_tick = BSP_GET_TICK();

    BSP_ExitCritical(primask);

    SPI_I2S_DMACmd(cfg->periph, SPI_I2S_DMAReq_Rx, DISABLE);
    SPI_I2S_DMACmd(cfg->periph, SPI_I2S_DMAReq_Tx, DISABLE);

    DMA_Cmd(cfg->dma_rx_stream, DISABLE);
    DMA_Cmd(cfg->dma_tx_stream, DISABLE);

    if (!BSP_DMA_WaitDisable(cfg->dma_rx_stream, SPI_BLOCK_TIMEOUT) ||
        !BSP_DMA_WaitDisable(cfg->dma_tx_stream, SPI_BLOCK_TIMEOUT)) {
        SPI_FinishFromISR(bus, BSP_TIMEOUT);
        return BSP_TIMEOUT;
    }

    /* 传输开始前把可能残留的接收数据冲掉，避免脏数据污染下一次传输 */
    while (SPI_I2S_GetFlagStatus(cfg->periph, SPI_I2S_FLAG_RXNE) == SET) {
        dummy = (uint8_t)SPI_I2S_ReceiveData(cfg->periph);
        (void)dummy;
    }

    DMA_ClearFlag(cfg->dma_rx_stream, cfg->dma_rx_flags_all);
    DMA_ClearFlag(cfg->dma_tx_stream, cfg->dma_tx_flags_all);

    DMA_DeInit(cfg->dma_rx_stream);
    if (!BSP_DMA_WaitDisable(cfg->dma_rx_stream, SPI_BLOCK_TIMEOUT)) {
        SPI_FinishFromISR(bus, BSP_TIMEOUT);
        return BSP_TIMEOUT;
    }

    DMA_InitStructure.DMA_Channel            = cfg->dma_channel;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&cfg->periph->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)rx_buf;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = len;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(cfg->dma_rx_stream, &DMA_InitStructure);
    DMA_ITConfig(cfg->dma_rx_stream, DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);

    DMA_DeInit(cfg->dma_tx_stream);
    if (!BSP_DMA_WaitDisable(cfg->dma_tx_stream, SPI_BLOCK_TIMEOUT)) {
        SPI_FinishFromISR(bus, BSP_TIMEOUT);
        return BSP_TIMEOUT;
    }

    DMA_InitStructure.DMA_Channel            = cfg->dma_channel;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&cfg->periph->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)tx_buf;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize         = len;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(cfg->dma_tx_stream, &DMA_InitStructure);
    DMA_ITConfig(cfg->dma_tx_stream, DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);

    DMA_Cmd(cfg->dma_rx_stream, ENABLE);
    DMA_Cmd(cfg->dma_tx_stream, ENABLE);

    SPI_I2S_DMACmd(cfg->periph, SPI_I2S_DMAReq_Rx, ENABLE);
    SPI_I2S_DMACmd(cfg->periph, SPI_I2S_DMAReq_Tx, ENABLE);

    return BSP_OK;
}

uint8_t BSP_SPI_IsBusy(SPI_Bus_t bus)
{
    if (bus >= SPI_BUS_COUNT) return 1U;    /* 非法通道号，保守地当成"忙" */
    return s_spi_rt[bus].busy;
}

void BSP_SPI_Task(SPI_Bus_t bus)
{
    volatile SPI_Runtime_t *rt;

    if (bus >= SPI_BUS_COUNT) return;
    rt = &s_spi_rt[bus];

    if (!rt->busy) {
        return;
    }

    if ((BSP_GET_TICK() - rt->start_tick) > SPI_ASYNC_TIMEOUT_MS) {
        SPI_FinishFromISR(bus, BSP_TIMEOUT);
    }
}

void BSP_SPI_TaskAll(void)
{
    for (SPI_Bus_t bus = (SPI_Bus_t)0; bus < SPI_BUS_COUNT; bus = (SPI_Bus_t)(bus + 1)) {
        BSP_SPI_Task(bus);
    }
}

/* ==================== ISR ==================== */
void BSP_SPI_DMA_RX_ISR(SPI_Bus_t bus)
{
    const SPI_Cfg_t *cfg;

    if (bus >= SPI_BUS_COUNT) return;
    cfg = &s_spi_cfg[bus];

    if (DMA_GetITStatus(cfg->dma_rx_stream, cfg->dma_rx_it_te)  != RESET ||
        DMA_GetITStatus(cfg->dma_rx_stream, cfg->dma_rx_it_dme) != RESET ||
        DMA_GetITStatus(cfg->dma_rx_stream, cfg->dma_rx_it_fe)  != RESET)
    {
        DMA_ClearITPendingBit(cfg->dma_rx_stream,
                              cfg->dma_rx_it_te | cfg->dma_rx_it_dme | cfg->dma_rx_it_fe);
        SPI_FinishFromISR(bus, BSP_ERROR);
        return;
    }

    if (DMA_GetITStatus(cfg->dma_rx_stream, cfg->dma_rx_it_tc) != RESET)
    {
        DMA_ClearITPendingBit(cfg->dma_rx_stream, cfg->dma_rx_it_tc);

        uint32_t timeout = SPI_BLOCK_TIMEOUT;
        while (SPI_I2S_GetFlagStatus(cfg->periph, SPI_I2S_FLAG_BSY) == SET) {
            if (--timeout == 0U) {
                SPI_FinishFromISR(bus, BSP_TIMEOUT);
                return;
            }
        }

        SPI_FinishFromISR(bus, BSP_OK);
    }
}

void BSP_SPI_DMA_TX_ISR(SPI_Bus_t bus)
{
    const SPI_Cfg_t *cfg;

    if (bus >= SPI_BUS_COUNT) return;
    cfg = &s_spi_cfg[bus];

    if (DMA_GetITStatus(cfg->dma_tx_stream, cfg->dma_tx_it_te)  != RESET ||
        DMA_GetITStatus(cfg->dma_tx_stream, cfg->dma_tx_it_dme) != RESET ||
        DMA_GetITStatus(cfg->dma_tx_stream, cfg->dma_tx_it_fe)  != RESET)
    {
        DMA_ClearITPendingBit(cfg->dma_tx_stream,
                              cfg->dma_tx_it_te | cfg->dma_tx_it_dme | cfg->dma_tx_it_fe);
        SPI_FinishFromISR(bus, BSP_ERROR);
    }
}

/* ---------------------------------------------------------------------------
 * 实际中断入口。启用哪个 SPI_BUSx，就在本文件内生成对应 IRQHandler。
 * 若工程模板 stm32f4xx_it.c 中已有同名空函数，请删除那些空函数，避免重复定义。
 * ------------------------------------------------------------------------- */
#if SPI_BUS1_ENABLE
void SPI_BUS1_DMA_RX_IRQ_HANDLER(void)
{
    BSP_SPI_DMA_RX_ISR(SPI_BUS1);
}

void SPI_BUS1_DMA_TX_IRQ_HANDLER(void)
{
    BSP_SPI_DMA_TX_ISR(SPI_BUS1);
}
#endif

#if SPI_BUS2_ENABLE
void SPI_BUS2_DMA_RX_IRQ_HANDLER(void)
{
    BSP_SPI_DMA_RX_ISR(SPI_BUS2);
}

void SPI_BUS2_DMA_TX_IRQ_HANDLER(void)
{
    BSP_SPI_DMA_TX_ISR(SPI_BUS2);
}
#endif

#if SPI_BUS3_ENABLE
void SPI_BUS3_DMA_RX_IRQ_HANDLER(void)
{
    BSP_SPI_DMA_RX_ISR(SPI_BUS3);
}

void SPI_BUS3_DMA_TX_IRQ_HANDLER(void)
{
    BSP_SPI_DMA_TX_ISR(SPI_BUS3);
}
#endif
