#include "bsp_i2c.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"
#include <string.h>

/*
 * 本文件不出现任何具体的外设/引脚名字（I2C1、PB8...），全部通过
 * bsp_i2c.h 里的配置宏 + I2C_Bus_t 枚举下标驱动。新增/切换通道只需要改
 * bsp_i2c.h，这个文件不需要任何修改。
 *
 * BSP 边界：本文件只负责 I2C 总线事务。
 * - BSP_I2C_MasterWrite/Read/WriteRead：纯总线 API，不解释数据内容；
 * - 设备寄存器地址、命令格式、WHO_AM_I 等逻辑由设备层自己拼包。
 */

/* ===========================================================================
 * 每路通道的静态配置表：编译期由 bsp_i2c.h 里的 #define 生成。
 * =========================================================================== */
typedef struct {
    I2C_TypeDef         *periph;
    BSP_ClockCmdFn_t      periph_clock_fn;
    uint32_t              periph_clock_mask;
    uint32_t              dma_rcc_mask;
    uint32_t              clock_hz;

    GPIO_TypeDef         *scl_port;
    uint16_t              scl_pin;
    uint8_t               scl_pinsrc;
    GPIO_TypeDef         *sda_port;
    uint16_t              sda_pin;
    uint8_t               sda_pinsrc;
    uint8_t               af;

    DMA_Stream_TypeDef   *dma_rx_stream;
    DMA_Stream_TypeDef   *dma_tx_stream;
    uint32_t              dma_channel;

    uint32_t              dma_rx_flags_all;
    uint32_t              dma_rx_it_tc;
    uint32_t              dma_rx_it_te;
    uint32_t              dma_rx_it_dme;
    uint32_t              dma_rx_it_fe;

    uint32_t              dma_tx_flags_all;
    uint32_t              dma_tx_it_tc;
    uint32_t              dma_tx_it_te;
    uint32_t              dma_tx_it_dme;
    uint32_t              dma_tx_it_fe;

    IRQn_Type             ev_irqn;
    IRQn_Type             er_irqn;
    IRQn_Type             dma_rx_irqn;
    IRQn_Type             dma_tx_irqn;
} I2C_Cfg_t;

static const I2C_Cfg_t s_i2c_cfg[I2C_BUS_COUNT] = {
#if I2C_BUS1_ENABLE
    [I2C_BUS1] = {
        .periph             = I2C_BUS1_PERIPH,
        .periph_clock_fn    = I2C_BUS1_PERIPH_CLOCK_FN,
        .periph_clock_mask  = I2C_BUS1_PERIPH_CLOCK_MASK,
        .dma_rcc_mask       = I2C_BUS1_DMA_RCC_MASK,
        .clock_hz           = I2C_BUS1_CLOCK_HZ,
        .scl_port           = I2C_BUS1_SCL_GPIO_PORT,
        .scl_pin            = I2C_BUS1_SCL_PIN,
        .scl_pinsrc         = I2C_BUS1_SCL_PINSRC,
        .sda_port           = I2C_BUS1_SDA_GPIO_PORT,
        .sda_pin            = I2C_BUS1_SDA_PIN,
        .sda_pinsrc         = I2C_BUS1_SDA_PINSRC,
        .af                 = I2C_BUS1_AF,
        .dma_rx_stream      = I2C_BUS1_DMA_RX_STREAM,
        .dma_tx_stream      = I2C_BUS1_DMA_TX_STREAM,
        .dma_channel        = I2C_BUS1_DMA_CHANNEL,
        .dma_rx_flags_all   = BSP_DMA_FLAGS_ALL(I2C_BUS1_DMA_RX_STREAM_NUM),
        .dma_rx_it_tc       = BSP_DMA_IT_TC(I2C_BUS1_DMA_RX_STREAM_NUM),
        .dma_rx_it_te       = BSP_DMA_IT_TE(I2C_BUS1_DMA_RX_STREAM_NUM),
        .dma_rx_it_dme      = BSP_DMA_IT_DME(I2C_BUS1_DMA_RX_STREAM_NUM),
        .dma_rx_it_fe       = BSP_DMA_IT_FE(I2C_BUS1_DMA_RX_STREAM_NUM),
        .dma_tx_flags_all   = BSP_DMA_FLAGS_ALL(I2C_BUS1_DMA_TX_STREAM_NUM),
        .dma_tx_it_tc       = BSP_DMA_IT_TC(I2C_BUS1_DMA_TX_STREAM_NUM),
        .dma_tx_it_te       = BSP_DMA_IT_TE(I2C_BUS1_DMA_TX_STREAM_NUM),
        .dma_tx_it_dme      = BSP_DMA_IT_DME(I2C_BUS1_DMA_TX_STREAM_NUM),
        .dma_tx_it_fe       = BSP_DMA_IT_FE(I2C_BUS1_DMA_TX_STREAM_NUM),
        .ev_irqn            = I2C_BUS1_EV_IRQn,
        .er_irqn            = I2C_BUS1_ER_IRQn,
        .dma_rx_irqn        = I2C_BUS1_DMA_RX_IRQn,
        .dma_tx_irqn        = I2C_BUS1_DMA_TX_IRQn,
    },
#endif
#if I2C_BUS2_ENABLE
    [I2C_BUS2] = {
        .periph             = I2C_BUS2_PERIPH,
        .periph_clock_fn    = I2C_BUS2_PERIPH_CLOCK_FN,
        .periph_clock_mask  = I2C_BUS2_PERIPH_CLOCK_MASK,
        .dma_rcc_mask       = I2C_BUS2_DMA_RCC_MASK,
        .clock_hz           = I2C_BUS2_CLOCK_HZ,
        .scl_port           = I2C_BUS2_SCL_GPIO_PORT,
        .scl_pin            = I2C_BUS2_SCL_PIN,
        .scl_pinsrc         = I2C_BUS2_SCL_PINSRC,
        .sda_port           = I2C_BUS2_SDA_GPIO_PORT,
        .sda_pin            = I2C_BUS2_SDA_PIN,
        .sda_pinsrc         = I2C_BUS2_SDA_PINSRC,
        .af                 = I2C_BUS2_AF,
        .dma_rx_stream      = I2C_BUS2_DMA_RX_STREAM,
        .dma_tx_stream      = I2C_BUS2_DMA_TX_STREAM,
        .dma_channel        = I2C_BUS2_DMA_CHANNEL,
        .dma_rx_flags_all   = BSP_DMA_FLAGS_ALL(I2C_BUS2_DMA_RX_STREAM_NUM),
        .dma_rx_it_tc       = BSP_DMA_IT_TC(I2C_BUS2_DMA_RX_STREAM_NUM),
        .dma_rx_it_te       = BSP_DMA_IT_TE(I2C_BUS2_DMA_RX_STREAM_NUM),
        .dma_rx_it_dme      = BSP_DMA_IT_DME(I2C_BUS2_DMA_RX_STREAM_NUM),
        .dma_rx_it_fe       = BSP_DMA_IT_FE(I2C_BUS2_DMA_RX_STREAM_NUM),
        .dma_tx_flags_all   = BSP_DMA_FLAGS_ALL(I2C_BUS2_DMA_TX_STREAM_NUM),
        .dma_tx_it_tc       = BSP_DMA_IT_TC(I2C_BUS2_DMA_TX_STREAM_NUM),
        .dma_tx_it_te       = BSP_DMA_IT_TE(I2C_BUS2_DMA_TX_STREAM_NUM),
        .dma_tx_it_dme      = BSP_DMA_IT_DME(I2C_BUS2_DMA_TX_STREAM_NUM),
        .dma_tx_it_fe       = BSP_DMA_IT_FE(I2C_BUS2_DMA_TX_STREAM_NUM),
        .ev_irqn            = I2C_BUS2_EV_IRQn,
        .er_irqn            = I2C_BUS2_ER_IRQn,
        .dma_rx_irqn        = I2C_BUS2_DMA_RX_IRQn,
        .dma_tx_irqn        = I2C_BUS2_DMA_TX_IRQn,
    },
#endif
#if I2C_BUS3_ENABLE
    [I2C_BUS3] = {
        .periph             = I2C_BUS3_PERIPH,
        .periph_clock_fn    = I2C_BUS3_PERIPH_CLOCK_FN,
        .periph_clock_mask  = I2C_BUS3_PERIPH_CLOCK_MASK,
        .dma_rcc_mask       = I2C_BUS3_DMA_RCC_MASK,
        .clock_hz           = I2C_BUS3_CLOCK_HZ,
        .scl_port           = I2C_BUS3_SCL_GPIO_PORT,
        .scl_pin            = I2C_BUS3_SCL_PIN,
        .scl_pinsrc         = I2C_BUS3_SCL_PINSRC,
        .sda_port           = I2C_BUS3_SDA_GPIO_PORT,
        .sda_pin            = I2C_BUS3_SDA_PIN,
        .sda_pinsrc         = I2C_BUS3_SDA_PINSRC,
        .af                 = I2C_BUS3_AF,
        .dma_rx_stream      = I2C_BUS3_DMA_RX_STREAM,
        .dma_tx_stream      = I2C_BUS3_DMA_TX_STREAM,
        .dma_channel        = I2C_BUS3_DMA_CHANNEL,
        .dma_rx_flags_all   = BSP_DMA_FLAGS_ALL(I2C_BUS3_DMA_RX_STREAM_NUM),
        .dma_rx_it_tc       = BSP_DMA_IT_TC(I2C_BUS3_DMA_RX_STREAM_NUM),
        .dma_rx_it_te       = BSP_DMA_IT_TE(I2C_BUS3_DMA_RX_STREAM_NUM),
        .dma_rx_it_dme      = BSP_DMA_IT_DME(I2C_BUS3_DMA_RX_STREAM_NUM),
        .dma_rx_it_fe       = BSP_DMA_IT_FE(I2C_BUS3_DMA_RX_STREAM_NUM),
        .dma_tx_flags_all   = BSP_DMA_FLAGS_ALL(I2C_BUS3_DMA_TX_STREAM_NUM),
        .dma_tx_it_tc       = BSP_DMA_IT_TC(I2C_BUS3_DMA_TX_STREAM_NUM),
        .dma_tx_it_te       = BSP_DMA_IT_TE(I2C_BUS3_DMA_TX_STREAM_NUM),
        .dma_tx_it_dme      = BSP_DMA_IT_DME(I2C_BUS3_DMA_TX_STREAM_NUM),
        .dma_tx_it_fe       = BSP_DMA_IT_FE(I2C_BUS3_DMA_TX_STREAM_NUM),
        .ev_irqn            = I2C_BUS3_EV_IRQn,
        .er_irqn            = I2C_BUS3_ER_IRQn,
        .dma_rx_irqn        = I2C_BUS3_DMA_RX_IRQn,
        .dma_tx_irqn        = I2C_BUS3_DMA_TX_IRQn,
    },
#endif
};

/* ===========================================================================
 * 每路通道的运行时状态：原来的单个全局结构体，现在按通道下标存放一份。
 * =========================================================================== */
typedef enum {
    I2C_SM_IDLE = 0,
    I2C_SM_START_W,
    I2C_SM_ADDR_W,
    I2C_SM_TX_DMA,
    I2C_SM_TX_WAIT_BTF,
    I2C_SM_START_R,
    I2C_SM_ADDR_R,
    I2C_SM_RX_ONE,
    I2C_SM_RX_DMA
} I2C_State_t;

typedef struct {
    volatile I2C_State_t state;

    uint8_t  dev_addr;          /* 7-bit 地址，调用者传 0x68 这种地址，不要左移 */

    /*
     * 纯总线事务描述：
     *   写：      tx_buf/tx_len 有效，rx_len = 0
     *   读：      tx_len = 0，rx_buf/rx_len 有效
     *   写后读：  先发送 tx_buf，再 repeated START 读 rx_buf
     * BSP 层不解释 tx_buf[0] 是否为寄存器地址，这由设备层决定。
     */
    uint8_t *tx_buf;
    uint16_t tx_len;
    uint8_t *rx_buf;
    uint16_t rx_len;
    uint8_t  need_read;

    uint32_t start_tick;
    I2C_Callback_t callback;
} I2C_Runtime_t;

static volatile I2C_Runtime_t s_i2c_rt[I2C_BUS_COUNT];
static uint8_t  s_i2c_tx_copy[I2C_BUS_COUNT][I2C_TX_COPY_BUF_LEN];
static uint32_t s_i2c_hw_busy_start[I2C_BUS_COUNT];

/* ==================== 内部工具函数 ==================== */
static uint8_t I2C_WaitEvent(I2C_TypeDef *periph, uint32_t event)
{
    uint32_t timeout = I2C_BLOCK_TIMEOUT;
    while (!I2C_CheckEvent(periph, event)) {
        if (--timeout == 0) return 0;
    }
    return 1;
}

static uint8_t I2C_WaitFlagSet(I2C_TypeDef *periph, uint32_t flag)
{
    uint32_t timeout = I2C_BLOCK_TIMEOUT;
    while (I2C_GetFlagStatus(periph, flag) == RESET) {
        if (--timeout == 0U) return 0U;
    }
    return 1U;
}

static uint8_t I2C_WaitBusFree(I2C_TypeDef *periph)
{
    uint32_t timeout = I2C_BLOCK_TIMEOUT;
    while ((periph->SR2 & I2C_SR2_BUSY) != 0U) {
        if (--timeout == 0U) return 0U;
    }
    return 1U;
}

static void I2C_DMA_ClearFlags(I2C_Bus_t bus)
{
    const I2C_Cfg_t *cfg = &s_i2c_cfg[bus];

    DMA_ClearFlag(cfg->dma_rx_stream, cfg->dma_rx_flags_all);
    DMA_ClearFlag(cfg->dma_tx_stream, cfg->dma_tx_flags_all);
}

static void I2C_StopAndResetPeripheral(I2C_Bus_t bus)
{
    const I2C_Cfg_t *cfg = &s_i2c_cfg[bus];

    I2C_GenerateSTOP(cfg->periph, ENABLE);
    I2C_DMACmd(cfg->periph, DISABLE);
    I2C_DMALastTransferCmd(cfg->periph, DISABLE);
    I2C_AcknowledgeConfig(cfg->periph, ENABLE);

    I2C_ITConfig(cfg->periph, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);

    DMA_Cmd(cfg->dma_rx_stream, DISABLE);
    DMA_Cmd(cfg->dma_tx_stream, DISABLE);
    (void)BSP_DMA_WaitDisable(cfg->dma_rx_stream, 10000);
    (void)BSP_DMA_WaitDisable(cfg->dma_tx_stream, 10000);
    I2C_DMA_ClearFlags(bus);

    I2C_ClearITPendingBit(cfg->periph, I2C_IT_BERR | I2C_IT_ARLO |
                                        I2C_IT_AF   | I2C_IT_OVR);

    /* STM32F4 硬件 I2C 偶尔 BUSY 锁死，错误/超时时复位一下外设更稳 */
    I2C_SoftwareResetCmd(cfg->periph, ENABLE);
    for (volatile uint32_t i = 0; i < 1000; i++) { ; }
    I2C_SoftwareResetCmd(cfg->periph, DISABLE);
    I2C_Cmd(cfg->periph, ENABLE);
    I2C_AcknowledgeConfig(cfg->periph, ENABLE);
}

static void I2C_Finish(I2C_Bus_t bus, int result)
{
    const I2C_Cfg_t *cfg = &s_i2c_cfg[bus];
    volatile I2C_Runtime_t *rt = &s_i2c_rt[bus];
    I2C_Callback_t cb = rt->callback;

    I2C_ITConfig(cfg->periph, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
    I2C_DMACmd(cfg->periph, DISABLE);
    I2C_DMALastTransferCmd(cfg->periph, DISABLE);
    I2C_AcknowledgeConfig(cfg->periph, ENABLE);

    rt->state = I2C_SM_IDLE;
    rt->callback = 0;

    if (cb) {
        cb(bus, result);
    }
}

static void I2C_Abort(I2C_Bus_t bus, int result)
{
    volatile I2C_Runtime_t *rt = &s_i2c_rt[bus];
    I2C_Callback_t cb = rt->callback;

    I2C_StopAndResetPeripheral(bus);
    rt->state = I2C_SM_IDLE;
    rt->callback = 0;

    if (cb) {
        cb(bus, result);
    }
}

static uint8_t I2C_ConfigTxDMA(I2C_Bus_t bus, uint8_t *buf, uint16_t len)
{
    const I2C_Cfg_t *cfg = &s_i2c_cfg[bus];
    DMA_InitTypeDef dma;

    DMA_Cmd(cfg->dma_tx_stream, DISABLE);
    if (!BSP_DMA_WaitDisable(cfg->dma_tx_stream, 10000)) return 0;

    DMA_DeInit(cfg->dma_tx_stream);
    DMA_StructInit(&dma);
    dma.DMA_Channel            = cfg->dma_channel;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&cfg->periph->DR;
    dma.DMA_Memory0BaseAddr    = (uint32_t)buf;
    dma.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    dma.DMA_BufferSize         = len;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode               = DMA_Mode_Normal;
    dma.DMA_Priority           = DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_Init(cfg->dma_tx_stream, &dma);

    DMA_ClearFlag(cfg->dma_tx_stream, cfg->dma_tx_flags_all);
    DMA_ITConfig(cfg->dma_tx_stream, DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);
    return 1;
}

static uint8_t I2C_ConfigRxDMA(I2C_Bus_t bus, uint8_t *buf, uint16_t len)
{
    const I2C_Cfg_t *cfg = &s_i2c_cfg[bus];
    DMA_InitTypeDef dma;

    DMA_Cmd(cfg->dma_rx_stream, DISABLE);
    if (!BSP_DMA_WaitDisable(cfg->dma_rx_stream, 10000)) return 0;

    DMA_DeInit(cfg->dma_rx_stream);
    DMA_StructInit(&dma);
    dma.DMA_Channel            = cfg->dma_channel;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&cfg->periph->DR;
    dma.DMA_Memory0BaseAddr    = (uint32_t)buf;
    dma.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize         = len;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode               = DMA_Mode_Normal;
    dma.DMA_Priority           = DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_Init(cfg->dma_rx_stream, &dma);

    DMA_ClearFlag(cfg->dma_rx_stream, cfg->dma_rx_flags_all);
    DMA_ITConfig(cfg->dma_rx_stream, DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);
    return 1;
}

static BSP_Status_t I2C_BeginAsync(I2C_Bus_t bus, uint8_t dev_addr,
                                    uint8_t *tx_buf, uint16_t tx_len,
                                    uint8_t *rx_buf, uint16_t rx_len,
                                    I2C_Callback_t callback)
{
    const I2C_Cfg_t *cfg = &s_i2c_cfg[bus];
    volatile I2C_Runtime_t *rt = &s_i2c_rt[bus];
    uint32_t primask;

    if ((tx_len == 0U) && (rx_len == 0U)) return BSP_PARAM;
    if ((tx_len > 0U && tx_buf == 0) || (rx_len > 0U && rx_buf == 0)) return BSP_PARAM;

    primask = BSP_EnterCritical();

    if (rt->state != I2C_SM_IDLE) {
        BSP_ExitCritical(primask);
        return BSP_BUSY;
    }

    if ((cfg->periph->SR2 & I2C_SR2_BUSY) != 0U) {
        BSP_ExitCritical(primask);
        return BSP_BUSY;
    }

    rt->dev_addr   = dev_addr;
    rt->tx_buf     = tx_buf;
    rt->tx_len     = tx_len;
    rt->rx_buf     = rx_buf;
    rt->rx_len     = rx_len;
    rt->need_read  = (rx_len > 0U) ? 1U : 0U;
    rt->callback   = callback;
    rt->start_tick = BSP_GET_TICK();
    rt->state      = (tx_len > 0U) ? I2C_SM_START_W : I2C_SM_START_R;

    I2C_AcknowledgeConfig(cfg->periph, ENABLE);
    I2C_ITConfig(cfg->periph, I2C_IT_ERR, ENABLE);
    I2C_ITConfig(cfg->periph, I2C_IT_EVT, ENABLE);
    I2C_GenerateSTART(cfg->periph, ENABLE);

    BSP_ExitCritical(primask);
    return BSP_OK;
}

static void I2C_ClearErrorAndStop_Blocking(I2C_TypeDef *periph)
{
    I2C_GenerateSTOP(periph, ENABLE);
    I2C_ClearITPendingBit(periph, I2C_IT_BERR | I2C_IT_ARLO | I2C_IT_AF | I2C_IT_OVR);
    I2C_AcknowledgeConfig(periph, ENABLE);
}

/*
 * 探测 7-bit 从机地址是否应答。
 *
 * 说明：
 *   1. 只用于 BSP_I2C_ScanBus() 的阻塞调试扫描；
 *   2. 发送 START + 地址写方向，不发送实际数据；
 *   3. 收到 ADDR 表示从机 ACK，收到 AF 表示 NACK；
 *   4. 每次探测后都会发送 STOP，恢复 ACK。
 */
static uint8_t I2C_BlockingProbeAddress(I2C_TypeDef *periph, uint8_t dev_addr)
{
    volatile uint32_t tmp;
    uint32_t timeout;

    I2C_ClearITPendingBit(periph, I2C_IT_BERR | I2C_IT_ARLO | I2C_IT_AF | I2C_IT_OVR);

    I2C_GenerateSTART(periph, ENABLE);
    if (!I2C_WaitFlagSet(periph, I2C_FLAG_SB)) {
        I2C_ClearErrorAndStop_Blocking(periph);
        return 0U;
    }

    I2C_Send7bitAddress(periph, dev_addr << 1, I2C_Direction_Transmitter);

    timeout = I2C_BLOCK_TIMEOUT;
    while (timeout-- > 0U) {
        uint16_t sr1 = periph->SR1;

        if ((sr1 & I2C_SR1_ADDR) != 0U) {
            tmp = periph->SR1;
            tmp = periph->SR2;
            (void)tmp;

            I2C_GenerateSTOP(periph, ENABLE);
            I2C_AcknowledgeConfig(periph, ENABLE);
            return 1U;
        }

        if ((sr1 & I2C_SR1_AF) != 0U) {
            I2C_ClearFlag(periph, I2C_FLAG_AF);
            I2C_GenerateSTOP(periph, ENABLE);
            I2C_AcknowledgeConfig(periph, ENABLE);
            return 0U;
        }

        if ((sr1 & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR)) != 0U) {
            I2C_ClearErrorAndStop_Blocking(periph);
            return 0U;
        }
    }

    I2C_ClearErrorAndStop_Blocking(periph);
    return 0U;
}

/* ==================== 初始化 ==================== */
void BSP_I2C_Init(I2C_Bus_t bus)
{
    const I2C_Cfg_t *cfg;
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    if (bus >= I2C_BUS_COUNT) return;
    cfg = &s_i2c_cfg[bus];

    BSP_GPIO_ClockEnable(cfg->scl_port);
    BSP_GPIO_ClockEnable(cfg->sda_port);
    cfg->periph_clock_fn(cfg->periph_clock_mask, ENABLE);
    RCC_AHB1PeriphClockCmd(cfg->dma_rcc_mask, ENABLE);

    GPIO_PinAFConfig(cfg->scl_port, cfg->scl_pinsrc, cfg->af);
    GPIO_PinAFConfig(cfg->sda_port, cfg->sda_pinsrc, cfg->af);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = cfg->scl_pin;
    GPIO_Init(cfg->scl_port, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = cfg->sda_pin;
    GPIO_Init(cfg->sda_port, &GPIO_InitStructure);

    I2C_DeInit(cfg->periph);
    I2C_InitStructure.I2C_ClockSpeed          = cfg->clock_hz;
    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1         = 0x00;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(cfg->periph, &I2C_InitStructure);

    I2C_Cmd(cfg->periph, ENABLE);
    I2C_AcknowledgeConfig(cfg->periph, ENABLE);

    /* 先关 DMA，真正传输时再配置并打开 */
    DMA_Cmd(cfg->dma_rx_stream, DISABLE);
    DMA_Cmd(cfg->dma_tx_stream, DISABLE);
    I2C_DMA_ClearFlags(bus);

    /* I2C 事件/错误中断 */
    NVIC_InitStructure.NVIC_IRQChannel = cfg->ev_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = cfg->er_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = cfg->dma_rx_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = cfg->dma_tx_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    I2C_ITConfig(cfg->periph, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);

    s_i2c_rt[bus].state = I2C_SM_IDLE;
    s_i2c_rt[bus].callback = 0;
    s_i2c_hw_busy_start[bus] = 0;

    for (volatile uint32_t i = 0; i < 10000; i++) { ; }
}

void BSP_I2C_InitAll(void)
{
    for (I2C_Bus_t bus = (I2C_Bus_t)0; bus < I2C_BUS_COUNT; bus = (I2C_Bus_t)(bus + 1)) {
        BSP_I2C_Init(bus);
    }
}

/* ==================== 阻塞 API ==================== */
static uint8_t I2C_BlockingBeginWrite(I2C_TypeDef *periph, uint8_t dev_addr)
{
    I2C_GenerateSTART(periph, ENABLE);
    if (!I2C_WaitEvent(periph, I2C_EVENT_MASTER_MODE_SELECT)) return 0U;

    I2C_Send7bitAddress(periph, dev_addr << 1, I2C_Direction_Transmitter);
    if (!I2C_WaitEvent(periph, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return 0U;

    return 1U;
}

static uint8_t I2C_BlockingSendByte(I2C_TypeDef *periph, uint8_t data)
{
    I2C_SendData(periph, data);
    return I2C_WaitEvent(periph, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
}

static uint8_t I2C_BlockingReadAfterStart(I2C_TypeDef *periph, uint8_t dev_addr,
                                          uint8_t *buf, uint16_t len)
{
    volatile uint32_t tmp;
    uint16_t remaining = len;

    if (buf == 0 || len == 0U) return 0U;

    I2C_GenerateSTART(periph, ENABLE);
    if (!I2C_WaitFlagSet(periph, I2C_FLAG_SB)) return 0U;

    I2C_Send7bitAddress(periph, dev_addr << 1, I2C_Direction_Receiver);

    if (len == 1U) {
        /* 单字节读：必须在清 ADDR 前关闭 ACK，清 ADDR 后马上 STOP。 */
        if (!I2C_WaitFlagSet(periph, I2C_FLAG_ADDR)) return 0U;
        I2C_AcknowledgeConfig(periph, DISABLE);
        tmp = periph->SR1;
        tmp = periph->SR2;
        (void)tmp;
        I2C_GenerateSTOP(periph, ENABLE);

        if (!I2C_WaitFlagSet(periph, I2C_FLAG_RXNE)) return 0U;
        buf[0] = (uint8_t)I2C_ReceiveData(periph);
    } else if (len == 2U) {
        /* 双字节读：使用 POS，让最后两个字节一起 NACK 后 STOP。 */
        if (!I2C_WaitFlagSet(periph, I2C_FLAG_ADDR)) return 0U;
        I2C_NACKPositionConfig(periph, I2C_NACKPosition_Next);
        I2C_AcknowledgeConfig(periph, DISABLE);
        tmp = periph->SR1;
        tmp = periph->SR2;
        (void)tmp;

        if (!I2C_WaitFlagSet(periph, I2C_FLAG_BTF)) return 0U;
        I2C_GenerateSTOP(periph, ENABLE);
        buf[0] = (uint8_t)I2C_ReceiveData(periph);
        buf[1] = (uint8_t)I2C_ReceiveData(periph);
        I2C_NACKPositionConfig(periph, I2C_NACKPosition_Current);
    } else {
        /* 多字节读：前 N-3 字节 ACK，最后 3 字节按 STM32F4 推荐时序处理。 */
        I2C_AcknowledgeConfig(periph, ENABLE);
        if (!I2C_WaitFlagSet(periph, I2C_FLAG_ADDR)) return 0U;
        tmp = periph->SR1;
        tmp = periph->SR2;
        (void)tmp;

        while (remaining > 3U) {
            if (!I2C_WaitFlagSet(periph, I2C_FLAG_RXNE)) return 0U;
            *buf++ = (uint8_t)I2C_ReceiveData(periph);
            remaining--;
        }

        if (!I2C_WaitFlagSet(periph, I2C_FLAG_BTF)) return 0U;
        I2C_AcknowledgeConfig(periph, DISABLE);

        *buf++ = (uint8_t)I2C_ReceiveData(periph);
        remaining--;
        I2C_GenerateSTOP(periph, ENABLE);

        *buf++ = (uint8_t)I2C_ReceiveData(periph);
        remaining--;

        if (!I2C_WaitFlagSet(periph, I2C_FLAG_RXNE)) return 0U;
        *buf++ = (uint8_t)I2C_ReceiveData(periph);
        remaining--;
        (void)remaining;
    }

    I2C_AcknowledgeConfig(periph, ENABLE);
    I2C_NACKPositionConfig(periph, I2C_NACKPosition_Current);
    return 1U;
}

BSP_Status_t BSP_I2C_MasterWrite(I2C_Bus_t bus, uint8_t dev_addr, const uint8_t *data, uint16_t len)
{
    I2C_TypeDef *periph;
    uint16_t i;

    if (bus >= I2C_BUS_COUNT || data == 0 || len == 0U) return BSP_PARAM;
    if (BSP_I2C_IsBusy(bus)) return BSP_BUSY;

    periph = s_i2c_cfg[bus].periph;
    if (!I2C_WaitBusFree(periph)) return BSP_BUSY;

    if (!I2C_BlockingBeginWrite(periph, dev_addr)) goto error;

    for (i = 0U; i < len; i++) {
        if (!I2C_BlockingSendByte(periph, data[i])) goto error;
    }

    I2C_GenerateSTOP(periph, ENABLE);
    return BSP_OK;

error:
    I2C_ClearErrorAndStop_Blocking(periph);
    return BSP_ERROR;
}

BSP_Status_t BSP_I2C_MasterRead(I2C_Bus_t bus, uint8_t dev_addr, uint8_t *buf, uint16_t len)
{
    I2C_TypeDef *periph;

    if (bus >= I2C_BUS_COUNT || buf == 0) return BSP_PARAM;
    if (len == 0U) return BSP_OK;
    if (BSP_I2C_IsBusy(bus)) return BSP_BUSY;

    periph = s_i2c_cfg[bus].periph;
    if (!I2C_WaitBusFree(periph)) return BSP_BUSY;

    if (!I2C_BlockingReadAfterStart(periph, dev_addr, buf, len)) goto error;
    return BSP_OK;

error:
    I2C_ClearErrorAndStop_Blocking(periph);
    I2C_AcknowledgeConfig(periph, ENABLE);
    I2C_NACKPositionConfig(periph, I2C_NACKPosition_Current);
    return BSP_ERROR;
}

BSP_Status_t BSP_I2C_MasterWriteRead(I2C_Bus_t bus, uint8_t dev_addr,
                                     const uint8_t *tx_data, uint16_t tx_len,
                                     uint8_t *rx_buf, uint16_t rx_len)
{
    I2C_TypeDef *periph;
    uint16_t i;

    if (bus >= I2C_BUS_COUNT) return BSP_PARAM;
    if ((tx_len == 0U) && (rx_len == 0U)) return BSP_PARAM;
    if (tx_len > 0U && tx_data == 0) return BSP_PARAM;
    if (rx_len > 0U && rx_buf == 0) return BSP_PARAM;
    if (BSP_I2C_IsBusy(bus)) return BSP_BUSY;

    if (tx_len == 0U) {
        return BSP_I2C_MasterRead(bus, dev_addr, rx_buf, rx_len);
    }
    if (rx_len == 0U) {
        return BSP_I2C_MasterWrite(bus, dev_addr, tx_data, tx_len);
    }

    periph = s_i2c_cfg[bus].periph;
    if (!I2C_WaitBusFree(periph)) return BSP_BUSY;

    if (!I2C_BlockingBeginWrite(periph, dev_addr)) goto error;

    for (i = 0U; i < tx_len; i++) {
        if (!I2C_BlockingSendByte(periph, tx_data[i])) goto error;
    }

    /* 不发 STOP，直接 repeated START 进入读阶段。 */
    if (!I2C_BlockingReadAfterStart(periph, dev_addr, rx_buf, rx_len)) goto error;
    return BSP_OK;

error:
    I2C_ClearErrorAndStop_Blocking(periph);
    I2C_AcknowledgeConfig(periph, ENABLE);
    I2C_NACKPositionConfig(periph, I2C_NACKPosition_Current);
    return BSP_ERROR;
}

BSP_Status_t BSP_I2C_ScanBus(I2C_Bus_t bus, uint8_t *out_addr_list, uint8_t max_count, uint8_t *out_found_count)
{
    I2C_TypeDef *periph;
    uint8_t addr;
    uint8_t found = 0U;

    if (bus >= I2C_BUS_COUNT || out_found_count == 0) return BSP_PARAM;
    if (max_count > 0U && out_addr_list == 0) return BSP_PARAM;
    if (BSP_I2C_IsBusy(bus)) return BSP_BUSY;

    periph = s_i2c_cfg[bus].periph;
    if (!I2C_WaitBusFree(periph)) return BSP_BUSY;

    /* 扫描标准 7-bit 可用地址范围：0x08~0x77。
     * 0x00~0x07 和 0x78~0x7F 为保留地址，不扫描。
     */
    for (addr = 0x08U; addr <= 0x77U; addr++) {
        if (I2C_BlockingProbeAddress(periph, addr)) {
            if (found < max_count) {
                out_addr_list[found] = addr;
            }

            if (found < 0xFFU) {
                found++;
            }
        }
    }

    *out_found_count = found;
    I2C_AcknowledgeConfig(periph, ENABLE);
    I2C_NACKPositionConfig(periph, I2C_NACKPosition_Current);
    return BSP_OK;
}

/* ==================== 非阻塞 API ==================== */
BSP_Status_t BSP_I2C_MasterWrite_DMA_Async(I2C_Bus_t bus, uint8_t dev_addr,
                                           const uint8_t *tx_data, uint16_t tx_len,
                                           I2C_Callback_t callback)
{
    if (bus >= I2C_BUS_COUNT) return BSP_PARAM;
    if (tx_data == 0 || tx_len == 0U) return BSP_PARAM;
    if (tx_len > I2C_TX_COPY_BUF_LEN) return BSP_PARAM;
    if (BSP_I2C_IsBusy(bus)) return BSP_BUSY;

    /* 异步写会复制 TX 数据，所以调用后 tx_data 可以立即释放/复用。 */
    memcpy(s_i2c_tx_copy[bus], tx_data, tx_len);
    return I2C_BeginAsync(bus, dev_addr,
                          s_i2c_tx_copy[bus], tx_len,
                          0, 0U,
                          callback);
}

BSP_Status_t BSP_I2C_MasterRead_DMA_Async(I2C_Bus_t bus, uint8_t dev_addr,
                                          uint8_t *rx_buf, uint16_t rx_len,
                                          I2C_Callback_t callback)
{
    if (bus >= I2C_BUS_COUNT) return BSP_PARAM;
    if (rx_buf == 0 || rx_len == 0U) return BSP_PARAM;
    if (BSP_I2C_IsBusy(bus)) return BSP_BUSY;

    /* 异步读不会复制 RX 缓冲区，rx_buf 必须保持有效直到 callback 被调用。 */
    return I2C_BeginAsync(bus, dev_addr,
                          0, 0U,
                          rx_buf, rx_len,
                          callback);
}

BSP_Status_t BSP_I2C_MasterWriteRead_DMA_Async(I2C_Bus_t bus, uint8_t dev_addr,
                                               const uint8_t *tx_data, uint16_t tx_len,
                                               uint8_t *rx_buf, uint16_t rx_len,
                                               I2C_Callback_t callback)
{
    if (bus >= I2C_BUS_COUNT) return BSP_PARAM;
    if (tx_data == 0 || tx_len == 0U || rx_buf == 0 || rx_len == 0U) return BSP_PARAM;
    if (tx_len > I2C_TX_COPY_BUF_LEN) return BSP_PARAM;
    if (BSP_I2C_IsBusy(bus)) return BSP_BUSY;

    memcpy(s_i2c_tx_copy[bus], tx_data, tx_len);
    return I2C_BeginAsync(bus, dev_addr,
                          s_i2c_tx_copy[bus], tx_len,
                          rx_buf, rx_len,
                          callback);
}

uint8_t BSP_I2C_IsBusy(I2C_Bus_t bus)
{
    if (bus >= I2C_BUS_COUNT) return 1U;    /* 非法通道号，保守地当成"忙" */
    return (s_i2c_rt[bus].state != I2C_SM_IDLE);
}

void BSP_I2C_Task(I2C_Bus_t bus)
{
    volatile I2C_Runtime_t *rt;
    const I2C_Cfg_t *cfg;

    if (bus >= I2C_BUS_COUNT) return;
    rt  = &s_i2c_rt[bus];
    cfg = &s_i2c_cfg[bus];

    if (rt->state != I2C_SM_IDLE) {
        s_i2c_hw_busy_start[bus] = 0;

        if ((BSP_GET_TICK() - rt->start_tick) > I2C_ASYNC_TIMEOUT_MS) {
            I2C_Abort(bus, -2);
        }

        return;
    }

    /*
     * 软件状态机已经空闲，但硬件 I2C 仍然 BUSY：
     * 说明可能是 STOP 未释放、从机拉低 SDA/SCL，或者 STM32F4 I2C BUSY 位卡死。
     * 不要让 APP 永远跳过，而是在这里自动复位一次 I2C 外设。
     */
    if ((cfg->periph->SR2 & I2C_SR2_BUSY) != 0) {
        if (s_i2c_hw_busy_start[bus] == 0) {
            s_i2c_hw_busy_start[bus] = BSP_GET_TICK();
        } else if ((BSP_GET_TICK() - s_i2c_hw_busy_start[bus]) > 5U) {
            I2C_StopAndResetPeripheral(bus);
            s_i2c_hw_busy_start[bus] = 0;
        }
    } else {
        s_i2c_hw_busy_start[bus] = 0;
    }
}

void BSP_I2C_TaskAll(void)
{
    for (I2C_Bus_t bus = (I2C_Bus_t)0; bus < I2C_BUS_COUNT; bus = (I2C_Bus_t)(bus + 1)) {
        BSP_I2C_Task(bus);
    }
}

/* ==================== ISR 状态机 ==================== */
void BSP_I2C_EV_ISR(I2C_Bus_t bus)
{
    volatile I2C_Runtime_t *rt;
    const I2C_Cfg_t *cfg;
    uint32_t sr1;

    if (bus >= I2C_BUS_COUNT) return;
    rt  = &s_i2c_rt[bus];
    cfg = &s_i2c_cfg[bus];
    sr1 = cfg->periph->SR1;

    switch (rt->state) {
    case I2C_SM_START_W:
        if (sr1 & I2C_SR1_SB) {
            I2C_Send7bitAddress(cfg->periph, rt->dev_addr << 1, I2C_Direction_Transmitter);
            rt->state = I2C_SM_ADDR_W;
        }
        break;

    case I2C_SM_ADDR_W:
        if (sr1 & I2C_SR1_ADDR) {
            (void)cfg->periph->SR1;
            (void)cfg->periph->SR2;

            if (rt->tx_len == 0U) {
                if (rt->need_read) {
                    I2C_GenerateSTART(cfg->periph, ENABLE);
                    rt->state = I2C_SM_START_R;
                } else {
                    I2C_GenerateSTOP(cfg->periph, ENABLE);
                    I2C_Finish(bus, 0);
                }
                return;
            }

            if (!I2C_ConfigTxDMA(bus, rt->tx_buf, rt->tx_len)) {
                I2C_Abort(bus, -1);
                return;
            }

            rt->state = I2C_SM_TX_DMA;
            I2C_ITConfig(cfg->periph, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
            I2C_DMACmd(cfg->periph, ENABLE);
            DMA_Cmd(cfg->dma_tx_stream, ENABLE);
        }
        break;

    case I2C_SM_START_R:
        if (sr1 & I2C_SR1_SB) {
            I2C_Send7bitAddress(cfg->periph, rt->dev_addr << 1, I2C_Direction_Receiver);
            rt->state = I2C_SM_ADDR_R;
        }
        break;

    case I2C_SM_ADDR_R:
        if (sr1 & I2C_SR1_ADDR) {
            if (rt->rx_len == 1U) {
                /* 单字节读：ADDR 清除前关闭 ACK，ADDR 清除后立即 STOP，再等 RXNE 读 DR。 */
                I2C_AcknowledgeConfig(cfg->periph, DISABLE);
                (void)cfg->periph->SR1;
                (void)cfg->periph->SR2;
                I2C_GenerateSTOP(cfg->periph, ENABLE);
                rt->state = I2C_SM_RX_ONE;
                I2C_ITConfig(cfg->periph, I2C_IT_BUF, ENABLE);
            } else {
                if (!I2C_ConfigRxDMA(bus, rt->rx_buf, rt->rx_len)) {
                    I2C_Abort(bus, -1);
                    return;
                }

                rt->state = I2C_SM_RX_DMA;
                I2C_DMALastTransferCmd(cfg->periph, ENABLE);
                I2C_DMACmd(cfg->periph, ENABLE);
                DMA_Cmd(cfg->dma_rx_stream, ENABLE);

                /* 清 ADDR 后从机才会真正开始送数据，此时 DMA 已经准备好。 */
                (void)cfg->periph->SR1;
                (void)cfg->periph->SR2;
                I2C_ITConfig(cfg->periph, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
            }
        }
        break;

    case I2C_SM_RX_ONE:
        if (sr1 & I2C_SR1_RXNE) {
            rt->rx_buf[0] = (uint8_t)I2C_ReceiveData(cfg->periph);
            I2C_Finish(bus, 0);
        }
        break;

    case I2C_SM_TX_WAIT_BTF:
        if (sr1 & I2C_SR1_BTF) {
            if (rt->need_read) {
                I2C_GenerateSTART(cfg->periph, ENABLE);
                rt->state = I2C_SM_START_R;
            } else {
                I2C_GenerateSTOP(cfg->periph, ENABLE);
                I2C_Finish(bus, 0);
            }
        }
        break;

    default:
        break;
    }
}

void BSP_I2C_ER_ISR(I2C_Bus_t bus)
{
    const I2C_Cfg_t *cfg;
    uint32_t sr1;

    if (bus >= I2C_BUS_COUNT) return;
    cfg = &s_i2c_cfg[bus];
    sr1 = cfg->periph->SR1;

    if (sr1 & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR)) {
        I2C_ClearITPendingBit(cfg->periph, I2C_IT_BERR | I2C_IT_ARLO |
                                            I2C_IT_AF   | I2C_IT_OVR);
        I2C_Abort(bus, -1);
    }
}

void BSP_I2C_DMA_RX_ISR(I2C_Bus_t bus)
{
    const I2C_Cfg_t *cfg;

    if (bus >= I2C_BUS_COUNT) return;
    cfg = &s_i2c_cfg[bus];

    if (DMA_GetITStatus(cfg->dma_rx_stream, cfg->dma_rx_it_te)  != RESET ||
        DMA_GetITStatus(cfg->dma_rx_stream, cfg->dma_rx_it_dme) != RESET ||
        DMA_GetITStatus(cfg->dma_rx_stream, cfg->dma_rx_it_fe)  != RESET) {

        DMA_ClearITPendingBit(cfg->dma_rx_stream,
                              cfg->dma_rx_it_te | cfg->dma_rx_it_dme | cfg->dma_rx_it_fe);
        I2C_Abort(bus, -1);
        return;
    }

    if (DMA_GetITStatus(cfg->dma_rx_stream, cfg->dma_rx_it_tc) != RESET) {
        DMA_ClearITPendingBit(cfg->dma_rx_stream, cfg->dma_rx_it_tc);
        DMA_Cmd(cfg->dma_rx_stream, DISABLE);
        I2C_DMACmd(cfg->periph, DISABLE);
        I2C_DMALastTransferCmd(cfg->periph, DISABLE);
        I2C_GenerateSTOP(cfg->periph, ENABLE);
        I2C_Finish(bus, 0);
    }
}

void BSP_I2C_DMA_TX_ISR(I2C_Bus_t bus)
{
    volatile I2C_Runtime_t *rt;
    const I2C_Cfg_t *cfg;

    if (bus >= I2C_BUS_COUNT) return;
    rt  = &s_i2c_rt[bus];
    cfg = &s_i2c_cfg[bus];

    if (DMA_GetITStatus(cfg->dma_tx_stream, cfg->dma_tx_it_te)  != RESET ||
        DMA_GetITStatus(cfg->dma_tx_stream, cfg->dma_tx_it_dme) != RESET ||
        DMA_GetITStatus(cfg->dma_tx_stream, cfg->dma_tx_it_fe)  != RESET) {

        DMA_ClearITPendingBit(cfg->dma_tx_stream,
                              cfg->dma_tx_it_te | cfg->dma_tx_it_dme | cfg->dma_tx_it_fe);
        I2C_Abort(bus, -1);
        return;
    }

    if (DMA_GetITStatus(cfg->dma_tx_stream, cfg->dma_tx_it_tc) != RESET) {
        DMA_ClearITPendingBit(cfg->dma_tx_stream, cfg->dma_tx_it_tc);
        DMA_Cmd(cfg->dma_tx_stream, DISABLE);
        I2C_DMACmd(cfg->periph, DISABLE);

        /* DMA 完成只代表数据进了 I2C->DR，不代表最后一个字节已经上总线。
         * 这里转到 BTF 状态，由 I2C 事件中断确认真正发送完成后再 STOP。
         */
        rt->state = I2C_SM_TX_WAIT_BTF;
        I2C_ITConfig(cfg->periph, I2C_IT_EVT, ENABLE);
    }
}

/* ---------------------------------------------------------------------------
 * 实际中断入口。启用哪个 I2C_BUSx，就在本文件内生成对应 IRQHandler。
 * 若工程模板 stm32f4xx_it.c 中已有同名空函数，请删除那些空函数，避免重复定义。
 * ------------------------------------------------------------------------- */
#if I2C_BUS1_ENABLE
void I2C_BUS1_EV_IRQ_HANDLER(void)
{
    BSP_I2C_EV_ISR(I2C_BUS1);
}

void I2C_BUS1_ER_IRQ_HANDLER(void)
{
    BSP_I2C_ER_ISR(I2C_BUS1);
}

void I2C_BUS1_DMA_RX_IRQ_HANDLER(void)
{
    BSP_I2C_DMA_RX_ISR(I2C_BUS1);
}

void I2C_BUS1_DMA_TX_IRQ_HANDLER(void)
{
    BSP_I2C_DMA_TX_ISR(I2C_BUS1);
}
#endif

#if I2C_BUS2_ENABLE
void I2C_BUS2_EV_IRQ_HANDLER(void)
{
    BSP_I2C_EV_ISR(I2C_BUS2);
}

void I2C_BUS2_ER_IRQ_HANDLER(void)
{
    BSP_I2C_ER_ISR(I2C_BUS2);
}

void I2C_BUS2_DMA_RX_IRQ_HANDLER(void)
{
    BSP_I2C_DMA_RX_ISR(I2C_BUS2);
}

void I2C_BUS2_DMA_TX_IRQ_HANDLER(void)
{
    BSP_I2C_DMA_TX_ISR(I2C_BUS2);
}
#endif

#if I2C_BUS3_ENABLE
void I2C_BUS3_EV_IRQ_HANDLER(void)
{
    BSP_I2C_EV_ISR(I2C_BUS3);
}

void I2C_BUS3_ER_IRQ_HANDLER(void)
{
    BSP_I2C_ER_ISR(I2C_BUS3);
}

void I2C_BUS3_DMA_RX_IRQ_HANDLER(void)
{
    BSP_I2C_DMA_RX_ISR(I2C_BUS3);
}

void I2C_BUS3_DMA_TX_IRQ_HANDLER(void)
{
    BSP_I2C_DMA_TX_ISR(I2C_BUS3);
}
#endif
