#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include "bsp_common.h"
#include "stm32f4xx_spi.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 通用 SPI 总线驱动（多实例）
 * ============================================================================
 * 总体约定见 bsp_common.h 顶部说明。本文件只负责"配置"，所有逻辑都在
 * bsp_spi.c 里按下面的枚举下标驱动，移植/换通道时 bsp_spi.c 不需要改一行。
 *
 * 用法：
 *   1. BSP_SPI_Init(SPI_BUS1) 之后即可用 BSP_SPI_TransferByte 阻塞收发，
 *      或者 BSP_SPI_TransferAsync_DMA 非阻塞收发。
 *   2. 片选（CS/NSS）引脚不属于本驱动管理范围：一条 SPI 总线上可能挂多个
 *      设备，各自的 CS 引脚由设备层代码自行控制（在调用收发函数前后拉低/
 *      拉高），总线驱动只负责 SCK/MISO/MOSI 时序本身。
 *   3. 非阻塞 API 注意事项：
 *        - tx_buf/rx_buf 必须在 DMA 完成前保持有效；
 *        - cb 在中断上下文里被调用，只适合置标志，不要做阻塞或耗时的事
 *          （不要在里面 printf）。
 *   4. 建议每 1~5ms 调一次 BSP_SPI_Task(bus)（或 BSP_SPI_TaskAll()），用来
 *      处理非阻塞传输的软件看门狗超时（DMA 异常卡死时强制恢复）。
 * ============================================================================
 */

/* ---------------------------------------------------------------------------
 * 全局调优参数（所有通道共用）
 * ------------------------------------------------------------------------- */
#define SPI_BLOCK_TIMEOUT          100000UL  /* 阻塞收发单字节超时（循环计数） */
#define SPI_ASYNC_TIMEOUT_MS       50U       /* 非阻塞传输软件看门狗超时（毫秒） */

/* ===========================================================================
 * 通道配置区：每一路 SPI 总线一个配置块。
 * 新增/切换通道时，只需要改这里，bsp_spi.c 不用动。
 * =========================================================================== */

/*
 * ----- SPI_BUS1：当前硬件在用的这一路（PMW3901，SPI Mode3） -----
 * PA5 -> SCK, PA6 -> MISO, PA7 -> MOSI
 */
#define SPI_BUS1_ENABLE             0
#define SPI_BUS1_PERIPH             SPI1
#define SPI_BUS1_PERIPH_CLOCK_FN    RCC_APB2PeriphClockCmd  /* SPI1 挂在 APB2 上 */
#define SPI_BUS1_PERIPH_CLOCK_MASK  RCC_APB2Periph_SPI1
#define SPI_BUS1_DMA_RCC_MASK       RCC_AHB1Periph_DMA2     /* DMA2_Stream0/3 都在 DMA2 上 */

#define SPI_BUS1_SCK_GPIO_PORT      GPIOA
#define SPI_BUS1_SCK_PIN            GPIO_Pin_5
#define SPI_BUS1_SCK_PINSRC         GPIO_PinSource5
#define SPI_BUS1_MISO_GPIO_PORT     GPIOA
#define SPI_BUS1_MISO_PIN           GPIO_Pin_6
#define SPI_BUS1_MISO_PINSRC        GPIO_PinSource6
#define SPI_BUS1_MOSI_GPIO_PORT     GPIOA
#define SPI_BUS1_MOSI_PIN           GPIO_Pin_7
#define SPI_BUS1_MOSI_PINSRC        GPIO_PinSource7
#define SPI_BUS1_AF                 GPIO_AF_SPI1

#define SPI_BUS1_CPOL               SPI_CPOL_High   /* PMW3901 用 SPI Mode 3：CPOL=1, CPHA=1 */
#define SPI_BUS1_CPHA               SPI_CPHA_2Edge

/*
 * 调试阶段先用 /128，稳定后可改 /64。
 * SPI1 在 APB2=84MHz 时：/128 = 656 kHz，/64 = 1.3125 MHz。
 * 实际频率取决于工程时钟树配置，换板子/换主频时请重新核对。
 */
#define SPI_BUS1_BAUD_PRESCALER     SPI_BaudRatePrescaler_128

#define SPI_BUS1_DMA_RX_STREAM      DMA2_Stream0
#define SPI_BUS1_DMA_RX_STREAM_NUM  0
#define SPI_BUS1_DMA_TX_STREAM      DMA2_Stream3
#define SPI_BUS1_DMA_TX_STREAM_NUM  3
#define SPI_BUS1_DMA_CHANNEL        DMA_Channel_3
#define SPI_BUS1_DMA_RX_IRQn        DMA2_Stream0_IRQn
#define SPI_BUS1_DMA_TX_IRQn        DMA2_Stream3_IRQn
#define SPI_BUS1_DMA_RX_IRQ_HANDLER DMA2_Stream0_IRQHandler
#define SPI_BUS1_DMA_TX_IRQ_HANDLER DMA2_Stream3_IRQHandler

/*
 * ----- SPI_BUS2 / SPI_BUS3：示例模板，默认关闭 -----
 * 启用方法：把 _ENABLE 改成 1，按下面的样子核对/填写各字段。
 *
 * 注意：DMA_RX/TX_STREAM、DMA_CHANNEL 是按常见的 SPI2/SPI3 资源占用整理的
 * 参考值，启用前务必对照《STM32F4xx 参考手册》DMA1/DMA2 请求映射表核实，
 * 避免和项目里其它外设抢用同一个 DMA Stream。
 */
#define SPI_BUS2_ENABLE             1
#define SPI_BUS2_PERIPH             SPI2
#define SPI_BUS2_PERIPH_CLOCK_FN    RCC_APB1PeriphClockCmd  /* SPI2 挂在 APB1 上 */
#define SPI_BUS2_PERIPH_CLOCK_MASK  RCC_APB1Periph_SPI2
#define SPI_BUS2_DMA_RCC_MASK       RCC_AHB1Periph_DMA1

#define SPI_BUS2_SCK_GPIO_PORT      GPIOB
#define SPI_BUS2_SCK_PIN            GPIO_Pin_13
#define SPI_BUS2_SCK_PINSRC         GPIO_PinSource13
#define SPI_BUS2_MISO_GPIO_PORT     GPIOB
#define SPI_BUS2_MISO_PIN           GPIO_Pin_14
#define SPI_BUS2_MISO_PINSRC        GPIO_PinSource14
#define SPI_BUS2_MOSI_GPIO_PORT     GPIOB
#define SPI_BUS2_MOSI_PIN           GPIO_Pin_15
#define SPI_BUS2_MOSI_PINSRC        GPIO_PinSource15
#define SPI_BUS2_AF                 GPIO_AF_SPI2

#define SPI_BUS2_CPOL               SPI_CPOL_Low    /* 默认 Mode 0，按实际设备调整 */
#define SPI_BUS2_CPHA               SPI_CPHA_1Edge
#define SPI_BUS2_BAUD_PRESCALER     SPI_BaudRatePrescaler_16

#define SPI_BUS2_DMA_RX_STREAM      DMA1_Stream3    /* 参考值，启用前请核实 */
#define SPI_BUS2_DMA_RX_STREAM_NUM  3
#define SPI_BUS2_DMA_TX_STREAM      DMA1_Stream4    /* 参考值，启用前请核实 */
#define SPI_BUS2_DMA_TX_STREAM_NUM  4
#define SPI_BUS2_DMA_CHANNEL        DMA_Channel_0   /* 参考值，启用前请核实 */
#define SPI_BUS2_DMA_RX_IRQn        DMA1_Stream3_IRQn
#define SPI_BUS2_DMA_TX_IRQn        DMA1_Stream4_IRQn
#define SPI_BUS2_DMA_RX_IRQ_HANDLER DMA1_Stream3_IRQHandler
#define SPI_BUS2_DMA_TX_IRQ_HANDLER DMA1_Stream4_IRQHandler

#define SPI_BUS3_ENABLE             0
#define SPI_BUS3_PERIPH             SPI3
#define SPI_BUS3_PERIPH_CLOCK_FN    RCC_APB1PeriphClockCmd  /* SPI3 也挂在 APB1 上 */
#define SPI_BUS3_PERIPH_CLOCK_MASK  RCC_APB1Periph_SPI3
#define SPI_BUS3_DMA_RCC_MASK       RCC_AHB1Periph_DMA1

#define SPI_BUS3_SCK_GPIO_PORT      GPIOC
#define SPI_BUS3_SCK_PIN            GPIO_Pin_10
#define SPI_BUS3_SCK_PINSRC         GPIO_PinSource10
#define SPI_BUS3_MISO_GPIO_PORT     GPIOC
#define SPI_BUS3_MISO_PIN           GPIO_Pin_11
#define SPI_BUS3_MISO_PINSRC        GPIO_PinSource11
#define SPI_BUS3_MOSI_GPIO_PORT     GPIOC
#define SPI_BUS3_MOSI_PIN           GPIO_Pin_12
#define SPI_BUS3_MOSI_PINSRC        GPIO_PinSource12
#define SPI_BUS3_AF                 GPIO_AF_SPI3

#define SPI_BUS3_CPOL               SPI_CPOL_High
#define SPI_BUS3_CPHA               SPI_CPHA_2Edge
#define SPI_BUS3_BAUD_PRESCALER     SPI_BaudRatePrescaler_128

#define SPI_BUS3_DMA_RX_STREAM      DMA1_Stream0    /* 参考值，启用前请核实，注意与 BUS2 是否冲突 */
#define SPI_BUS3_DMA_RX_STREAM_NUM  0
#define SPI_BUS3_DMA_TX_STREAM      DMA1_Stream5    /* 参考值，启用前请核实，注意与 BUS2 是否冲突 */
#define SPI_BUS3_DMA_TX_STREAM_NUM  5
#define SPI_BUS3_DMA_CHANNEL        DMA_Channel_0   /* 参考值，启用前请核实 */
#define SPI_BUS3_DMA_RX_IRQn        DMA1_Stream0_IRQn
#define SPI_BUS3_DMA_TX_IRQn        DMA1_Stream5_IRQn
#define SPI_BUS3_DMA_RX_IRQ_HANDLER DMA1_Stream0_IRQHandler
#define SPI_BUS3_DMA_TX_IRQ_HANDLER DMA1_Stream5_IRQHandler

/* ===========================================================================
 * 通道枚举：只有 _ENABLE 为 1 的通道才会出现，不占用额外 RAM/ROM。
 * 公共 API 的第一个参数都是这个枚举，用来选择操作哪一路总线。
 * =========================================================================== */
typedef enum {
#if SPI_BUS1_ENABLE
    SPI_BUS1,
#endif
#if SPI_BUS2_ENABLE
    SPI_BUS2,
#endif
#if SPI_BUS3_ENABLE
    SPI_BUS3,
#endif
    SPI_BUS_COUNT
} SPI_Bus_t;

/*
 * status（见 BSP_Status_t）：
 *   BSP_OK      = 成功
 *   BSP_ERROR   = DMA/SPI 错误
 *   BSP_TIMEOUT = 软件看门狗超时
 */
typedef void (*SPI_Callback_t)(SPI_Bus_t bus, void *ctx, BSP_Status_t status);

/* ===========================================================================
 * 公共 API：统一加 BSP_ 前缀，避免和 StdPeriph 库自己的 SPI_Init() 等函数
 * 重名。第一个参数都是 SPI_Bus_t，用来选择具体物理总线。
 * =========================================================================== */

void BSP_SPI_Init(SPI_Bus_t bus);
void BSP_SPI_InitAll(void);

/* 阻塞收发 1 字节：成功返回 1，失败（超时）返回 0 */
uint8_t BSP_SPI_TransferByte(SPI_Bus_t bus, uint8_t tx, uint8_t *rx);

/*
 * 阻塞收发 len 字节。
 * SPI 是全双工：每发 1 字节都会同时收 1 字节。
 * tx_buf == 0 时发送 dummy 字节；rx_buf == 0 时丢弃收到的数据。
 * CS/NSS 仍然由设备层在调用前后自行拉低/拉高。
 */
BSP_Status_t BSP_SPI_Transfer(SPI_Bus_t bus,
                              const uint8_t *tx_buf,
                              uint8_t *rx_buf,
                              uint16_t len,
                              uint8_t dummy_tx);

/* 兼容旧接口形态的单字节收发：失败时返回 0xFF */
uint8_t BSP_SPI_ReadWriteByte(SPI_Bus_t bus, uint8_t data);

/*
 * DMA 非阻塞收发：
 * 返回值：
 *   BSP_OK    = 成功启动，完成后进 cb
 *   BSP_BUSY  = 该通道正在传输，本次未启动
 *   BSP_PARAM = 参数错误 / 通道号越界
 *
 * 注意：
 *   - tx_buf/rx_buf 必须在 DMA 完成前保持有效；
 *   - cb 在 DMA 中断上下文中调用，只能置标志，不要 printf。
 */
BSP_Status_t BSP_SPI_TransferAsync_DMA(SPI_Bus_t bus,
                                        uint8_t *tx_buf,
                                        uint8_t *rx_buf,
                                        uint16_t len,
                                        SPI_Callback_t cb,
                                        void *ctx);

uint8_t BSP_SPI_IsBusy(SPI_Bus_t bus);

/* 建议 1ms~5ms 调一次，处理非阻塞传输的软件看门狗超时 */
void BSP_SPI_Task(SPI_Bus_t bus);
void BSP_SPI_TaskAll(void);

/* ---------------------------------------------------------------------------
 * ISR 入口。
 *
 * bsp_spi.c 已经按 SPI_BUSx_ENABLE 自动生成对应 DMA Stream IRQHandler。
 * 工程的 stm32f4xx_it.c 不需要再转调；若模板里已有同名空函数，请删除空函数，
 * 避免重复定义。
 *
 * 下面这些 BSP_SPI_xxx_ISR 仍然保留为内部公共入口，便于必要时手动复用。
 * ------------------------------------------------------------------------- */
void BSP_SPI_DMA_RX_ISR(SPI_Bus_t bus);
void BSP_SPI_DMA_TX_ISR(SPI_Bus_t bus);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SPI_H */
