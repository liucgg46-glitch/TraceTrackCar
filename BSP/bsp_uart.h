#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "bsp_common.h"
#include "stm32f4xx_usart.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 通用 UART/USART 非阻塞驱动（多实例）
 * ============================================================================
 * 特性：
 *   - 支持 USART1/USART2/USART3/UART4/UART5/USART6，按需打开；
 *   - RX 使用 DMA Circular + IDLE 中断，收到一帧或空闲时自动搬入软件环形队列；
 *   - TX 使用软件环形队列 + DMA 分段发送，调用发送函数后立即返回；
 *   - 迁移项目时只改本文件的配置区，bsp_uart.c 不需要改动；
 *   - USART IRQHandler 和 TX DMA IRQHandler 已在 bsp_uart.c 内生成，不需要在
 *     stm32f4xx_it.c 里再转调。
 *
 * 使用建议：
 *   1. 初始化：BSP_UART_Init(UART_PORT1) 或 BSP_UART_InitAll();
 *   2. 发送：
 *        - BSP_UART_Write(...)：流式写入，缓冲区满时允许部分写入；
 *        - BSP_UART_WriteFrame(...) / BSP_UART_SendData_NonBlocking(...)：整包写入，
 *          缓冲区不足时完全不写，适合协议帧。
 *   3. 接收：BSP_UART_Available(...) 查询，再用 BSP_UART_GetChar/BSP_UART_Read 读取；
 *   4. 若存在连续无空闲的高速数据流，可在主循环 1~5ms 调用 BSP_UART_TaskAll()
 *      主动搬运 DMA 新数据，IDLE 中断仍然保留。
 */

/* ---------------------------------------------------------------------------
 * 全局调优参数
 * ------------------------------------------------------------------------- */
#define UART_RX_BUF_SIZE             256U
#define UART_TX_BUF_SIZE             512U
#define UART_DMA_RX_BUF_SIZE         256U
#define UART_DMA_WAIT_TIMEOUT        100000UL

#define UART_IRQ_PREEMPT_PRIO        1U
#define UART_IRQ_SUB_PRIO            0U
#define UART_DMA_TX_PREEMPT_PRIO     1U
#define UART_DMA_TX_SUB_PRIO         1U

/* ---------------------------------------------------------------------------
 * UART_PORT1: USART1, PA9(TX), PA10(RX), DMA2 Stream7(TX), DMA2 Stream2(RX)
 * ------------------------------------------------------------------------- */
#define UART_PORT1_ENABLE             1
#define UART_PORT1_PERIPH             USART1
#define UART_PORT1_PERIPH_CLOCK_FN    RCC_APB2PeriphClockCmd
#define UART_PORT1_PERIPH_CLOCK_MASK  RCC_APB2Periph_USART1
#define UART_PORT1_DMA_RCC_MASK       RCC_AHB1Periph_DMA2
#define UART_PORT1_BAUDRATE           115200UL
#define UART_PORT1_WORD_LENGTH        USART_WordLength_8b
#define UART_PORT1_STOP_BITS          USART_StopBits_1
#define UART_PORT1_PARITY             USART_Parity_No
#define UART_PORT1_FLOW_CONTROL       USART_HardwareFlowControl_None
#define UART_PORT1_TX_GPIO_PORT       GPIOA
#define UART_PORT1_TX_PIN             GPIO_Pin_9
#define UART_PORT1_TX_PINSRC          GPIO_PinSource9
#define UART_PORT1_RX_GPIO_PORT       GPIOA
#define UART_PORT1_RX_PIN             GPIO_Pin_10
#define UART_PORT1_RX_PINSRC          GPIO_PinSource10
#define UART_PORT1_AF                 GPIO_AF_USART1
#define UART_PORT1_DMA_RX_STREAM      DMA2_Stream2
#define UART_PORT1_DMA_RX_STREAM_NUM  2
#define UART_PORT1_DMA_TX_STREAM      DMA2_Stream7
#define UART_PORT1_DMA_TX_STREAM_NUM  7
#define UART_PORT1_DMA_CHANNEL        DMA_Channel_4
#define UART_PORT1_IRQn               USART1_IRQn
#define UART_PORT1_DMA_TX_IRQn        DMA2_Stream7_IRQn
#define UART_PORT1_IRQ_HANDLER        USART1_IRQHandler
#define UART_PORT1_DMA_TX_IRQ_HANDLER DMA2_Stream7_IRQHandler

/* ---------------------------------------------------------------------------
 * UART_PORT2: USART2
 *
 * PA2 -> USART2_TX
 * PA3 -> USART2_RX
 *
 * DMA:
 * USART2_RX -> DMA1 Stream5 Channel4
 * USART2_TX -> DMA1 Stream6 Channel4
 * ------------------------------------------------------------------------- */
#define UART_PORT2_ENABLE                1

#define UART_PORT2_PERIPH                USART2
#define UART_PORT2_PERIPH_CLOCK_FN       RCC_APB1PeriphClockCmd
#define UART_PORT2_PERIPH_CLOCK_MASK     RCC_APB1Periph_USART2

#define UART_PORT2_DMA_RCC_MASK          RCC_AHB1Periph_DMA1

#define UART_PORT2_BAUDRATE              115200UL
#define UART_PORT2_WORD_LENGTH           USART_WordLength_8b
#define UART_PORT2_STOP_BITS             USART_StopBits_1
#define UART_PORT2_PARITY                USART_Parity_No
#define UART_PORT2_FLOW_CONTROL          USART_HardwareFlowControl_None

/* USART2发送引脚：PA2 */
#define UART_PORT2_TX_GPIO_PORT          GPIOA
#define UART_PORT2_TX_PIN                GPIO_Pin_2
#define UART_PORT2_TX_PINSRC             GPIO_PinSource2

/* USART2接收引脚：PA3 */
#define UART_PORT2_RX_GPIO_PORT          GPIOA
#define UART_PORT2_RX_PIN                GPIO_Pin_3
#define UART_PORT2_RX_PINSRC             GPIO_PinSource3

#define UART_PORT2_AF                    GPIO_AF_USART2

/* USART2_RX：DMA1 Stream5 Channel4 */
#define UART_PORT2_DMA_RX_STREAM         DMA1_Stream5
#define UART_PORT2_DMA_RX_STREAM_NUM     5

/* USART2_TX：DMA1 Stream6 Channel4 */
#define UART_PORT2_DMA_TX_STREAM         DMA1_Stream6
#define UART_PORT2_DMA_TX_STREAM_NUM     6

#define UART_PORT2_DMA_CHANNEL           DMA_Channel_4

#define UART_PORT2_IRQn                  USART2_IRQn
#define UART_PORT2_DMA_TX_IRQn           DMA1_Stream6_IRQn

#define UART_PORT2_IRQ_HANDLER           USART2_IRQHandler
#define UART_PORT2_DMA_TX_IRQ_HANDLER    DMA1_Stream6_IRQHandler

/* ---------------------------------------------------------------------------
 * 下面是常用备用通道模板，默认关闭。启用前请按芯片手册核对引脚复用、
 * DMA Stream/Channel 是否与工程中其他外设冲突。
 * ------------------------------------------------------------------------- */
#define UART_PORT3_ENABLE             0
#define UART_PORT3_PERIPH             USART3
#define UART_PORT3_PERIPH_CLOCK_FN    RCC_APB1PeriphClockCmd
#define UART_PORT3_PERIPH_CLOCK_MASK  RCC_APB1Periph_USART3
#define UART_PORT3_DMA_RCC_MASK       RCC_AHB1Periph_DMA1
#define UART_PORT3_BAUDRATE           115200UL
#define UART_PORT3_WORD_LENGTH        USART_WordLength_8b
#define UART_PORT3_STOP_BITS          USART_StopBits_1
#define UART_PORT3_PARITY             USART_Parity_No
#define UART_PORT3_FLOW_CONTROL       USART_HardwareFlowControl_None
#define UART_PORT3_TX_GPIO_PORT       GPIOB
#define UART_PORT3_TX_PIN             GPIO_Pin_10
#define UART_PORT3_TX_PINSRC          GPIO_PinSource10
#define UART_PORT3_RX_GPIO_PORT       GPIOB
#define UART_PORT3_RX_PIN             GPIO_Pin_11
#define UART_PORT3_RX_PINSRC          GPIO_PinSource11
#define UART_PORT3_AF                 GPIO_AF_USART3
#define UART_PORT3_DMA_RX_STREAM      DMA1_Stream1
#define UART_PORT3_DMA_RX_STREAM_NUM  1
#define UART_PORT3_DMA_TX_STREAM      DMA1_Stream3
#define UART_PORT3_DMA_TX_STREAM_NUM  3
#define UART_PORT3_DMA_CHANNEL        DMA_Channel_4
#define UART_PORT3_IRQn               USART3_IRQn
#define UART_PORT3_DMA_TX_IRQn        DMA1_Stream3_IRQn
#define UART_PORT3_IRQ_HANDLER        USART3_IRQHandler
#define UART_PORT3_DMA_TX_IRQ_HANDLER DMA1_Stream3_IRQHandler

#define UART_PORT4_ENABLE             0
#define UART_PORT4_PERIPH             UART4
#define UART_PORT4_PERIPH_CLOCK_FN    RCC_APB1PeriphClockCmd
#define UART_PORT4_PERIPH_CLOCK_MASK  RCC_APB1Periph_UART4
#define UART_PORT4_DMA_RCC_MASK       RCC_AHB1Periph_DMA1
#define UART_PORT4_BAUDRATE           115200UL
#define UART_PORT4_WORD_LENGTH        USART_WordLength_8b
#define UART_PORT4_STOP_BITS          USART_StopBits_1
#define UART_PORT4_PARITY             USART_Parity_No
#define UART_PORT4_FLOW_CONTROL       USART_HardwareFlowControl_None
#define UART_PORT4_TX_GPIO_PORT       GPIOA
#define UART_PORT4_TX_PIN             GPIO_Pin_0
#define UART_PORT4_TX_PINSRC          GPIO_PinSource0
#define UART_PORT4_RX_GPIO_PORT       GPIOA
#define UART_PORT4_RX_PIN             GPIO_Pin_1
#define UART_PORT4_RX_PINSRC          GPIO_PinSource1
#define UART_PORT4_AF                 GPIO_AF_UART4
#define UART_PORT4_DMA_RX_STREAM      DMA1_Stream2
#define UART_PORT4_DMA_RX_STREAM_NUM  2
#define UART_PORT4_DMA_TX_STREAM      DMA1_Stream4
#define UART_PORT4_DMA_TX_STREAM_NUM  4
#define UART_PORT4_DMA_CHANNEL        DMA_Channel_4
#define UART_PORT4_IRQn               UART4_IRQn
#define UART_PORT4_DMA_TX_IRQn        DMA1_Stream4_IRQn
#define UART_PORT4_IRQ_HANDLER        UART4_IRQHandler
#define UART_PORT4_DMA_TX_IRQ_HANDLER DMA1_Stream4_IRQHandler

#define UART_PORT5_ENABLE             0
#define UART_PORT5_PERIPH             UART5
#define UART_PORT5_PERIPH_CLOCK_FN    RCC_APB1PeriphClockCmd
#define UART_PORT5_PERIPH_CLOCK_MASK  RCC_APB1Periph_UART5
#define UART_PORT5_DMA_RCC_MASK       RCC_AHB1Periph_DMA1
#define UART_PORT5_BAUDRATE           115200UL
#define UART_PORT5_WORD_LENGTH        USART_WordLength_8b
#define UART_PORT5_STOP_BITS          USART_StopBits_1
#define UART_PORT5_PARITY             USART_Parity_No
#define UART_PORT5_FLOW_CONTROL       USART_HardwareFlowControl_None
#define UART_PORT5_TX_GPIO_PORT       GPIOC
#define UART_PORT5_TX_PIN             GPIO_Pin_12
#define UART_PORT5_TX_PINSRC          GPIO_PinSource12
#define UART_PORT5_RX_GPIO_PORT       GPIOD
#define UART_PORT5_RX_PIN             GPIO_Pin_2
#define UART_PORT5_RX_PINSRC          GPIO_PinSource2
#define UART_PORT5_AF                 GPIO_AF_UART5
#define UART_PORT5_DMA_RX_STREAM      DMA1_Stream0
#define UART_PORT5_DMA_RX_STREAM_NUM  0
#define UART_PORT5_DMA_TX_STREAM      DMA1_Stream7
#define UART_PORT5_DMA_TX_STREAM_NUM  7
#define UART_PORT5_DMA_CHANNEL        DMA_Channel_4
#define UART_PORT5_IRQn               UART5_IRQn
#define UART_PORT5_DMA_TX_IRQn        DMA1_Stream7_IRQn
#define UART_PORT5_IRQ_HANDLER        UART5_IRQHandler
#define UART_PORT5_DMA_TX_IRQ_HANDLER DMA1_Stream7_IRQHandler

#define UART_PORT6_ENABLE             0
#define UART_PORT6_PERIPH             USART6
#define UART_PORT6_PERIPH_CLOCK_FN    RCC_APB2PeriphClockCmd
#define UART_PORT6_PERIPH_CLOCK_MASK  RCC_APB2Periph_USART6
#define UART_PORT6_DMA_RCC_MASK       RCC_AHB1Periph_DMA2
#define UART_PORT6_BAUDRATE           115200UL
#define UART_PORT6_WORD_LENGTH        USART_WordLength_8b
#define UART_PORT6_STOP_BITS          USART_StopBits_1
#define UART_PORT6_PARITY             USART_Parity_No
#define UART_PORT6_FLOW_CONTROL       USART_HardwareFlowControl_None
#define UART_PORT6_TX_GPIO_PORT       GPIOC
#define UART_PORT6_TX_PIN             GPIO_Pin_6
#define UART_PORT6_TX_PINSRC          GPIO_PinSource6
#define UART_PORT6_RX_GPIO_PORT       GPIOC
#define UART_PORT6_RX_PIN             GPIO_Pin_7
#define UART_PORT6_RX_PINSRC          GPIO_PinSource7
#define UART_PORT6_AF                 GPIO_AF_USART6
#define UART_PORT6_DMA_RX_STREAM      DMA2_Stream1
#define UART_PORT6_DMA_RX_STREAM_NUM  1
#define UART_PORT6_DMA_TX_STREAM      DMA2_Stream6
#define UART_PORT6_DMA_TX_STREAM_NUM  6
#define UART_PORT6_DMA_CHANNEL        DMA_Channel_5
#define UART_PORT6_IRQn               USART6_IRQn
#define UART_PORT6_DMA_TX_IRQn        DMA2_Stream6_IRQn
#define UART_PORT6_IRQ_HANDLER        USART6_IRQHandler
#define UART_PORT6_DMA_TX_IRQ_HANDLER DMA2_Stream6_IRQHandler

typedef enum {
#if UART_PORT1_ENABLE
    UART_PORT1,
#endif
#if UART_PORT2_ENABLE
    UART_PORT2,
#endif
#if UART_PORT3_ENABLE
    UART_PORT3,
#endif
#if UART_PORT4_ENABLE
    UART_PORT4,
#endif
#if UART_PORT5_ENABLE
    UART_PORT5,
#endif
#if UART_PORT6_ENABLE
    UART_PORT6,
#endif
    UART_PORT_COUNT
} UART_Port_t;

/*
 * 上层只使用语义端口名，具体硬件实例由BSP配置。
 * STM32原工程中调试口与E220共用UART_PORT1，是否连接E220由链路模式决定。
 */
#define UART_PORT_DEBUG  UART_PORT1
#define UART_PORT_E220   UART_PORT1
#define UART_PORT_K210   UART_PORT2
#define DEBUG_UART_PORT  UART_PORT_DEBUG

/*
 * UART 运行统计。
 * - rx_overflow：RX 软件环形缓冲区满时丢掉的字节数；
 * - tx_drop：调用 BSP_UART_Write() 流式写入时，因为 TX 环形缓冲区满而未写入的字节数；
 * - rx_count/tx_count：当前 RX/TX 软件环形缓冲区内等待处理的字节数。
 */
typedef struct {
    uint16_t rx_overflow;
    uint16_t tx_drop;
    uint16_t rx_count;
    uint16_t tx_count;
} UART_Stats_t;

/*
 * 可选的逐端口发送就绪检查。
 * 返回 1 允许新的写入；返回 0 时，非阻塞写入报告忙。
 * 传入空函数指针时保持原有 UART 行为。
 */
typedef uint8_t (*BSP_UART_TxReadyFn_t)(void);

/*
 * 可选的完整帧延后处理函数。
 * 仅当 BSP_UART_WriteFrame() 当前无法接收整帧时调用。
 * 回调函数复制并保存完整帧后才能返回 BSP_OK。
 */
typedef BSP_Status_t (*BSP_UART_TxDeferredFn_t)(const uint8_t *data, uint16_t len);

void BSP_UART_Init(UART_Port_t port);
void BSP_UART_InitAll(void);
void BSP_UART_SetTxReadyGuard(UART_Port_t port, BSP_UART_TxReadyFn_t guard);
void BSP_UART_SetTxDeferredHandler(UART_Port_t port, BSP_UART_TxDeferredFn_t handler);

/*
 * 流式写入：尽可能把 data 写入 TX 环形缓冲区，返回实际写入字节数。
 * 如果缓冲区空间不足，可能只写入一部分；未写入的字节会计入 tx_drop。
 * 适合 printf/日志流，不建议直接用于必须完整发送的协议帧。
 */
uint16_t BSP_UART_Write(UART_Port_t port, const uint8_t *data, uint16_t len);

/*
 * 整包写入：只有当 TX 环形缓冲区剩余空间 >= len 时才写入。
 * 返回 BSP_BUSY 时，本次不会写入任何字节，适合 MAVLink、自定义帧等协议包。
 */
BSP_Status_t BSP_UART_WriteFrame(UART_Port_t port, const uint8_t *data, uint16_t len);

/*
 * 驱动层重试专用：直接尝试写入 UART TX 环形缓冲区，不调用就绪保护和延后处理器。
 * 仍保持整帧原子语义；空间不足时返回 BSP_BUSY，且不写入任何字节。
 */
BSP_Status_t BSP_UART_WriteFrameNow(UART_Port_t port, const uint8_t *data, uint16_t len);

/* 兼容旧接口：现在等价于 BSP_UART_WriteFrame()，避免协议帧被部分写入。 */
BSP_Status_t BSP_UART_SendData_NonBlocking(UART_Port_t port, const uint8_t *data, uint16_t len);

uint16_t BSP_UART_Read(UART_Port_t port, uint8_t *buf, uint16_t len);
uint8_t BSP_UART_GetChar(UART_Port_t port, uint8_t *ch);
uint16_t BSP_UART_Available(UART_Port_t port);

uint8_t BSP_UART_IsTxBusy(UART_Port_t port);
uint16_t BSP_UART_TxFree(UART_Port_t port);
void BSP_UART_FlushRx(UART_Port_t port);
void BSP_UART_FlushTx(UART_Port_t port);

/* 读取/清除运行统计，用于调试丢包、串口带宽不足、主循环读取不及时等问题。 */
BSP_Status_t BSP_UART_GetStats(UART_Port_t port, UART_Stats_t *stats);
void BSP_UART_ClearStats(UART_Port_t port);

void BSP_UART_Task(UART_Port_t port);
void BSP_UART_TaskAll(void);

void BSP_UART_USART_ISR(UART_Port_t port);
void BSP_UART_DMA_TX_ISR(UART_Port_t port);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_H */
