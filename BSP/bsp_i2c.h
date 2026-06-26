#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include "bsp_common.h"
#include "stm32f4xx_i2c.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 通用 I2C 总线驱动（多实例）
 * ============================================================================
 * 总体约定见 bsp_common.h 顶部说明。本文件只负责"配置"，所有逻辑都在
 * bsp_i2c.c 里按下面的枚举下标驱动，移植/换通道时 bsp_i2c.c 不需要改一行。
 *
 * 用法：
 *   1. BSP_I2C_Init(I2C_BUS1) 之后，使用 BSP_I2C_MasterWrite /
 *      BSP_I2C_MasterRead / BSP_I2C_MasterWriteRead 这组纯总线 API。
 *      设备寄存器地址、命令格式、WHO_AM_I 等逻辑应放在 mpu9250.c、bmp280.c
 *      这类设备层文件中。BSP 层不再提供旧工程的寄存器便捷接口。
 *   2. 非阻塞 API 注意事项：
 *        - 读操作的 rx_buf 必须保持有效，直到 callback 被调用；
 *        - 写操作的 tx_data 会被复制到驱动内部缓冲区（最大长度见
 *          I2C_TX_COPY_BUF_LEN），调用后原数组可以立刻释放；
 *        - callback 在中断上下文里被调用，只适合置标志/拷数据，不要做
 *          阻塞或耗时的事（不要在里面 printf）。
 *   3. 建议每 1~5ms 调一次 BSP_I2C_Task(bus)（或 BSP_I2C_TaskAll()），用来处理
 *      异步传输超时、以及"软件状态机已空闲但硬件 BUSY 卡死"的自动恢复。
 * ============================================================================
 */

/* ---------------------------------------------------------------------------
 * 全局调优参数（所有通道共用）
 * ------------------------------------------------------------------------- */
#define I2C_BLOCK_TIMEOUT          3000UL  /* 阻塞 API 单次等待超时（循环计数） */
#define I2C_ASYNC_TIMEOUT_MS       20U     /* 非阻塞传输软件看门狗超时（毫秒） */
#define I2C_TX_COPY_BUF_LEN        32U     /* 每个通道内部 TX 拷贝缓冲区大小 */
#define I2C_ASYNC_TX_IRQ_MAX_LEN   I2C_TX_COPY_BUF_LEN

/* ===========================================================================
 * 通道配置区：每一路 I2C 总线一个配置块。
 * 新增/切换通道时，只需要改这里，bsp_i2c.c 不用动。
 * =========================================================================== */

/* ----- I2C_BUS1：当前硬件在用的这一路（保持与现有工程一致） ----- */
#define I2C_BUS1_ENABLE             1
#define I2C_BUS1_PERIPH             I2C1
#define I2C_BUS1_PERIPH_CLOCK_FN    RCC_APB1PeriphClockCmd  /* I2C1 挂在 APB1 上 */
#define I2C_BUS1_PERIPH_CLOCK_MASK  RCC_APB1Periph_I2C1
#define I2C_BUS1_DMA_RCC_MASK       RCC_AHB1Periph_DMA1     /* DMA1_Stream0/7 都在 DMA1 上 */
#define I2C_BUS1_CLOCK_HZ           100000UL            /* 先 100kHz，稳定后可改 400kHz */
#define I2C_BUS1_SCL_GPIO_PORT      GPIOB
#define I2C_BUS1_SCL_PIN            GPIO_Pin_8
#define I2C_BUS1_SCL_PINSRC         GPIO_PinSource8
#define I2C_BUS1_SDA_GPIO_PORT      GPIOB
#define I2C_BUS1_SDA_PIN            GPIO_Pin_9
#define I2C_BUS1_SDA_PINSRC         GPIO_PinSource9
#define I2C_BUS1_AF                 GPIO_AF_I2C1
#define I2C_BUS1_DMA_RX_STREAM      DMA1_Stream0
#define I2C_BUS1_DMA_RX_STREAM_NUM  0               /* DMA1_Stream0 -> 0，用于拼接标志位常量 */
#define I2C_BUS1_DMA_TX_STREAM      DMA1_Stream7
#define I2C_BUS1_DMA_TX_STREAM_NUM  7               /* DMA1_Stream7 -> 7 */
#define I2C_BUS1_DMA_CHANNEL        DMA_Channel_1
#define I2C_BUS1_EV_IRQn            I2C1_EV_IRQn
#define I2C_BUS1_ER_IRQn            I2C1_ER_IRQn
#define I2C_BUS1_DMA_RX_IRQn        DMA1_Stream0_IRQn
#define I2C_BUS1_DMA_TX_IRQn        DMA1_Stream7_IRQn
#define I2C_BUS1_EV_IRQ_HANDLER     I2C1_EV_IRQHandler
#define I2C_BUS1_ER_IRQ_HANDLER     I2C1_ER_IRQHandler
#define I2C_BUS1_DMA_RX_IRQ_HANDLER DMA1_Stream0_IRQHandler
#define I2C_BUS1_DMA_TX_IRQ_HANDLER DMA1_Stream7_IRQHandler

/*
 * ----- I2C_BUS2 / I2C_BUS3：示例模板，默认关闭 -----
 * 启用方法：把 _ENABLE 改成 1，按下面的样子核对/填写各字段。
 *
 * 注意：BUS2、BUS3 的 DMA_RX/TX_STREAM、DMA_CHANNEL 是按常见的 I2C2/I2C3
 * 资源占用整理的参考值，不同 STM32F4 型号、不同工程里其它外设占用 DMA 的
 * 情况都可能不一样。如果要同时启用 BUS2 和 BUS3（或者它们与项目里其它
 * DMA 用户冲突），启用前务必对照《STM32F4xx 参考手册》里 DMA1/DMA2 请求
 * 映射表逐项核实 Stream/Channel，避免多个外设抢用同一个 DMA Stream。
 */
#define I2C_BUS2_ENABLE             0
#define I2C_BUS2_PERIPH             I2C2
#define I2C_BUS2_PERIPH_CLOCK_FN    RCC_APB1PeriphClockCmd  /* I2C2 也挂在 APB1 上 */
#define I2C_BUS2_PERIPH_CLOCK_MASK  RCC_APB1Periph_I2C2
#define I2C_BUS2_DMA_RCC_MASK       RCC_AHB1Periph_DMA1
#define I2C_BUS2_CLOCK_HZ           100000UL
#define I2C_BUS2_SCL_GPIO_PORT      GPIOB
#define I2C_BUS2_SCL_PIN            GPIO_Pin_10
#define I2C_BUS2_SCL_PINSRC         GPIO_PinSource10
#define I2C_BUS2_SDA_GPIO_PORT      GPIOB
#define I2C_BUS2_SDA_PIN            GPIO_Pin_11
#define I2C_BUS2_SDA_PINSRC         GPIO_PinSource11
#define I2C_BUS2_AF                 GPIO_AF_I2C2
#define I2C_BUS2_DMA_RX_STREAM      DMA1_Stream2    /* 参考值，启用前请核实 */
#define I2C_BUS2_DMA_RX_STREAM_NUM  2               /* 必须和上面 DMA_RX_STREAM 的编号一致 */
#define I2C_BUS2_DMA_TX_STREAM      DMA1_Stream4    /* 参考值，启用前请核实 */
#define I2C_BUS2_DMA_TX_STREAM_NUM  4               /* 必须和上面 DMA_TX_STREAM 的编号一致 */
#define I2C_BUS2_DMA_CHANNEL        DMA_Channel_7   /* 参考值，启用前请核实 */
#define I2C_BUS2_EV_IRQn            I2C2_EV_IRQn
#define I2C_BUS2_ER_IRQn            I2C2_ER_IRQn
#define I2C_BUS2_DMA_RX_IRQn        DMA1_Stream2_IRQn
#define I2C_BUS2_DMA_TX_IRQn        DMA1_Stream4_IRQn
#define I2C_BUS2_EV_IRQ_HANDLER     I2C2_EV_IRQHandler
#define I2C_BUS2_ER_IRQ_HANDLER     I2C2_ER_IRQHandler
#define I2C_BUS2_DMA_RX_IRQ_HANDLER DMA1_Stream2_IRQHandler
#define I2C_BUS2_DMA_TX_IRQ_HANDLER DMA1_Stream4_IRQHandler

#define I2C_BUS3_ENABLE             0
#define I2C_BUS3_PERIPH             I2C3
#define I2C_BUS3_PERIPH_CLOCK_FN    RCC_APB1PeriphClockCmd  /* I2C3 也挂在 APB1 上 */
#define I2C_BUS3_PERIPH_CLOCK_MASK  RCC_APB1Periph_I2C3
#define I2C_BUS3_DMA_RCC_MASK       RCC_AHB1Periph_DMA1
#define I2C_BUS3_CLOCK_HZ           100000UL
#define I2C_BUS3_SCL_GPIO_PORT      GPIOA            /* I2C3 的 SCL/SDA 不在同一个端口，分开配置 */
#define I2C_BUS3_SCL_PIN            GPIO_Pin_8
#define I2C_BUS3_SCL_PINSRC         GPIO_PinSource8
#define I2C_BUS3_SDA_GPIO_PORT      GPIOC
#define I2C_BUS3_SDA_PIN            GPIO_Pin_9
#define I2C_BUS3_SDA_PINSRC         GPIO_PinSource9
#define I2C_BUS3_AF                 GPIO_AF_I2C3
#define I2C_BUS3_DMA_RX_STREAM      DMA1_Stream2    /* 参考值，启用前请核实，注意与 BUS2 是否冲突 */
#define I2C_BUS3_DMA_RX_STREAM_NUM  2               /* 必须和上面 DMA_RX_STREAM 的编号一致 */
#define I2C_BUS3_DMA_TX_STREAM      DMA1_Stream4    /* 参考值，启用前请核实，注意与 BUS2 是否冲突 */
#define I2C_BUS3_DMA_TX_STREAM_NUM  4               /* 必须和上面 DMA_TX_STREAM 的编号一致 */
#define I2C_BUS3_DMA_CHANNEL        DMA_Channel_3   /* 参考值，启用前请核实 */
#define I2C_BUS3_EV_IRQn            I2C3_EV_IRQn
#define I2C_BUS3_ER_IRQn            I2C3_ER_IRQn
#define I2C_BUS3_DMA_RX_IRQn        DMA1_Stream2_IRQn
#define I2C_BUS3_DMA_TX_IRQn        DMA1_Stream4_IRQn
#define I2C_BUS3_EV_IRQ_HANDLER     I2C3_EV_IRQHandler
#define I2C_BUS3_ER_IRQ_HANDLER     I2C3_ER_IRQHandler
#define I2C_BUS3_DMA_RX_IRQ_HANDLER DMA1_Stream2_IRQHandler
#define I2C_BUS3_DMA_TX_IRQ_HANDLER DMA1_Stream4_IRQHandler

/* ===========================================================================
 * 通道枚举：只有 _ENABLE 为 1 的通道才会出现，不占用额外 RAM/ROM。
 * 公共 API 的第一个参数都是这个枚举，用来选择操作哪一路总线。
 * =========================================================================== */
typedef enum {
#if I2C_BUS1_ENABLE
    I2C_BUS1,
#endif
#if I2C_BUS2_ENABLE
    I2C_BUS2,
#endif
#if I2C_BUS3_ENABLE
    I2C_BUS3,
#endif
    I2C_BUS_COUNT       /* 已启用的通道总数，同时充当"非法通道号"的上界 */
} I2C_Bus_t;

/*
 * result（由 bsp_i2c.c 内部状态机产生，会通过 callback 回传）：
 *   0  = 成功
 *  -1  = I2C/DMA 错误（NACK、仲裁丢失、总线错误等）
 *  -2  = 软件超时
 */
typedef void (*I2C_Callback_t)(I2C_Bus_t bus, int result);

typedef struct {
    uint8_t state;
    uint8_t dev_addr;
    uint16_t tx_len;
    uint16_t tx_pos;
    uint16_t rx_len;
    uint8_t need_read;
    uint16_t sr1;
    uint16_t sr2;
    uint32_t start_tick;
} BSP_I2C_Debug_t;

/* ===========================================================================
 * 公共 API：统一加 BSP_ 前缀，避免和 StdPeriph 库自己的 I2C_Init() 等函数
 * 重名（StdPeriph 占用了 "I2C_*" 这个命名空间）。第一个参数都是 I2C_Bus_t，
 * 用来选择具体物理总线。
 * =========================================================================== */

/* ---- 初始化 ---- */
void BSP_I2C_Init(I2C_Bus_t bus);
void BSP_I2C_InitAll(void);                 /* 依次初始化所有已启用的通道 */

/*
 * ---- 纯总线阻塞 API ----
 * dev_addr 一律传 7-bit 地址，例如 MPU6050/MPU9250 常见地址传 0x68，不要传 0xD0。
 * BSP 层只负责 I2C 总线事务，不解释 data/tx_data 里的内容。
 */
BSP_Status_t BSP_I2C_MasterWrite(I2C_Bus_t bus, uint8_t dev_addr, const uint8_t *data, uint16_t len);
BSP_Status_t BSP_I2C_MasterRead(I2C_Bus_t bus, uint8_t dev_addr, uint8_t *buf, uint16_t len);
BSP_Status_t BSP_I2C_MasterWriteRead(I2C_Bus_t bus, uint8_t dev_addr,
                                     const uint8_t *tx_data, uint16_t tx_len,
                                     uint8_t *rx_buf, uint16_t rx_len);

/*
 * 扫描总线上的从机地址（阻塞，仅用于调试/上电自检）。
 * 把找到的 7bit 地址依次写入 out_addr_list，最多写 max_count 个，
 * 实际找到的数量通过 out_found_count 返回。驱动本身不做任何打印/输出，
 * 需要看结果的话由调用者自己决定怎么打印（比如喂给 BSP_UART_WriteFrame）。
 */
BSP_Status_t BSP_I2C_ScanBus(I2C_Bus_t bus, uint8_t *out_addr_list, uint8_t max_count, uint8_t *out_found_count);

/*
 * 非阻塞 DMA + 状态机 API。
 * 返回值：
 *   BSP_OK      = 已经成功启动，完成后进 callback
 *   BSP_BUSY    = 该通道状态机忙，本次没有启动
 *   BSP_PARAM   = 参数错误 / 总线忙 / 通道号越界，本次没有启动
 */
/* 纯总线 DMA 异步 API：BSP 层不解释 tx_data/rx_buf 内容。 */
BSP_Status_t BSP_I2C_MasterWrite_DMA_Async(I2C_Bus_t bus, uint8_t dev_addr,
                                           const uint8_t *tx_data, uint16_t tx_len,
                                           I2C_Callback_t callback);
BSP_Status_t BSP_I2C_MasterRead_DMA_Async(I2C_Bus_t bus, uint8_t dev_addr,
                                          uint8_t *rx_buf, uint16_t rx_len,
                                          I2C_Callback_t callback);
BSP_Status_t BSP_I2C_MasterWriteRead_DMA_Async(I2C_Bus_t bus, uint8_t dev_addr,
                                               const uint8_t *tx_data, uint16_t tx_len,
                                               uint8_t *rx_buf, uint16_t rx_len,
                                               I2C_Callback_t callback);

uint8_t BSP_I2C_IsBusy(I2C_Bus_t bus);
BSP_Status_t BSP_I2C_GetDebug(I2C_Bus_t bus, BSP_I2C_Debug_t *debug);

/* 建议 1ms~5ms 调一次，处理异步超时保护和 BUSY 卡死自恢复 */
void BSP_I2C_Task(I2C_Bus_t bus);
void BSP_I2C_TaskAll(void);

/* ---------------------------------------------------------------------------
 * ISR 入口。
 *
 * bsp_i2c.c 已经按 I2C_BUSx_ENABLE 自动生成 I2Cx_EV_IRQHandler、
 * I2Cx_ER_IRQHandler 以及对应 DMA Stream IRQHandler。工程的 stm32f4xx_it.c
 * 不需要再转调；若模板里已有同名空函数，请删除空函数，避免重复定义。
 *
 * 下面这些 BSP_I2C_xxx_ISR 仍然保留为内部公共入口，便于必要时手动复用。
 * ------------------------------------------------------------------------- */
void BSP_I2C_EV_ISR(I2C_Bus_t bus);
void BSP_I2C_ER_ISR(I2C_Bus_t bus);
void BSP_I2C_DMA_RX_ISR(I2C_Bus_t bus);    /* 对应该通道配置的 DMA_RX_STREAM */
void BSP_I2C_DMA_TX_ISR(I2C_Bus_t bus);    /* 对应该通道配置的 DMA_TX_STREAM */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_I2C_H */
