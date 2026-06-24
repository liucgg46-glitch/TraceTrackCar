#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "bsp_common.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 通用正交编码器 BSP（支持 1~4 路）
 * ============================================================================
 * 定位：
 *   只负责 TIM 编码器模式计数、周期测速，不负责轮径换算、左右轮平均、
 *   四轮映射、底盘运动学和速度闭环。
 *
 * 分层建议：
 *   bsp_encoder：只读 CH1~CH4 的 TIM 编码器计数；
 *   drv_encoder：把 CH1~CH4 映射成 FL/FR/RL/RR，统一方向，换算 mm/s；
 *   odometer：根据左右轮速度/位移估计里程和角度；
 *   chassis：速度闭环和差速分配。
 *
 * 移植方法：
 *   只改本文件配置区：TIM、GPIO、AF、是否反向、是否启用。
 *   bsp_encoder.c 不需要改。
 *
 * 使用约定：
 *   1. 建议每 10ms 调一次 BSP_Encoder_UpdateAll()；
 *   2. 前进时如果某一路速度为负，优先改 BSP_ENCODER_CHx_REVERSE；
 *   3. 不要在 chassis / pid / motion 里到处对编码器取反；
 *   4. 计数器按 16bit 差值处理，Update 周期内增量不要超过 32767；
 *   5. 默认 4 路只是模板，实际比赛车必须按自己的接线核对引脚复用。
 *
 * 当前电赛小车推荐引脚规划 v1.4：
 *   CH1：TIM1 PE9 / PE11
 *   CH2：TIM2 PA0 / PA1
 *   CH3：TIM3 PA6 / PA7
 *   CH4：TIM4 PB6 / PB7
 *
 * 这样分配的目的：
 *   - 不占用 PA15 / PB3 / PB4，尽量不影响 SWD/JTAG/SWO 调试脚；
 *   - TIM8 整个留给 4 路电机 PWM：PC6/PC7/PC8/PC9；
 *   - SPI 改用 SPI2：PB13/PB14/PB15；
 *   - I2C 保留 I2C1：PB8/PB9；
 *   - 每一路编码器独占一个 TIM 的 CH1/CH2。
 *
 * 注意：
 *   - 不要把同一个物理引脚同时分给 PWM、SPI、I2C、UART、ADC；
 *   - 如果某一路暂时不用，把 BSP_ENCODER_CHx_ENABLE 改成 0；
 *   - 当前 CH3 使用 PA6/PA7，因此 SPI1 的 PA5/PA6/PA7 不再作为推荐 SPI。
 */

#define BSP_ENCODER_UPDATE_PERIOD_MS       10U

/* ============================ CH1：TIM1 PE9/PE11 ============================ */
#define BSP_ENCODER_CH1_ENABLE             1
#define BSP_ENCODER_CH1_TIM                TIM1
#define BSP_ENCODER_CH1_TIM_CLOCK_FN       RCC_APB2PeriphClockCmd
#define BSP_ENCODER_CH1_TIM_CLOCK_MASK     RCC_APB2Periph_TIM1
#define BSP_ENCODER_CH1_GPIO_PORT_A        GPIOE
#define BSP_ENCODER_CH1_GPIO_PIN_A         GPIO_Pin_9
#define BSP_ENCODER_CH1_GPIO_PINSRC_A      GPIO_PinSource9
#define BSP_ENCODER_CH1_GPIO_PORT_B        GPIOE
#define BSP_ENCODER_CH1_GPIO_PIN_B         GPIO_Pin_11
#define BSP_ENCODER_CH1_GPIO_PINSRC_B      GPIO_PinSource11
#define BSP_ENCODER_CH1_GPIO_AF            GPIO_AF_TIM1
#define BSP_ENCODER_CH1_PERIOD             0xFFFFU
#define BSP_ENCODER_CH1_REVERSE            0

/* ============================ CH2：TIM2 PA0/PA1 ============================ */
#define BSP_ENCODER_CH2_ENABLE             1
#define BSP_ENCODER_CH2_TIM                TIM2
#define BSP_ENCODER_CH2_TIM_CLOCK_FN       RCC_APB1PeriphClockCmd
#define BSP_ENCODER_CH2_TIM_CLOCK_MASK     RCC_APB1Periph_TIM2
#define BSP_ENCODER_CH2_GPIO_PORT_A        GPIOA
#define BSP_ENCODER_CH2_GPIO_PIN_A         GPIO_Pin_0
#define BSP_ENCODER_CH2_GPIO_PINSRC_A      GPIO_PinSource0
#define BSP_ENCODER_CH2_GPIO_PORT_B        GPIOA
#define BSP_ENCODER_CH2_GPIO_PIN_B         GPIO_Pin_1
#define BSP_ENCODER_CH2_GPIO_PINSRC_B      GPIO_PinSource1
#define BSP_ENCODER_CH2_GPIO_AF            GPIO_AF_TIM2
#define BSP_ENCODER_CH2_PERIOD             0xFFFFU
#define BSP_ENCODER_CH2_REVERSE            0

/* ============================ CH3：TIM3 PA6/PA7 ============================ */
#define BSP_ENCODER_CH3_ENABLE             1
#define BSP_ENCODER_CH3_TIM                TIM3
#define BSP_ENCODER_CH3_TIM_CLOCK_FN       RCC_APB1PeriphClockCmd
#define BSP_ENCODER_CH3_TIM_CLOCK_MASK     RCC_APB1Periph_TIM3
#define BSP_ENCODER_CH3_GPIO_PORT_A        GPIOA
#define BSP_ENCODER_CH3_GPIO_PIN_A         GPIO_Pin_6
#define BSP_ENCODER_CH3_GPIO_PINSRC_A      GPIO_PinSource6
#define BSP_ENCODER_CH3_GPIO_PORT_B        GPIOA
#define BSP_ENCODER_CH3_GPIO_PIN_B         GPIO_Pin_7
#define BSP_ENCODER_CH3_GPIO_PINSRC_B      GPIO_PinSource7
#define BSP_ENCODER_CH3_GPIO_AF            GPIO_AF_TIM3
#define BSP_ENCODER_CH3_PERIOD             0xFFFFU
#define BSP_ENCODER_CH3_REVERSE            0

/* ============================ CH4：TIM4 PB6/PB7 ============================ */
#define BSP_ENCODER_CH4_ENABLE             1
#define BSP_ENCODER_CH4_TIM                TIM4
#define BSP_ENCODER_CH4_TIM_CLOCK_FN       RCC_APB1PeriphClockCmd
#define BSP_ENCODER_CH4_TIM_CLOCK_MASK     RCC_APB1Periph_TIM4
#define BSP_ENCODER_CH4_GPIO_PORT_A        GPIOB
#define BSP_ENCODER_CH4_GPIO_PIN_A         GPIO_Pin_6
#define BSP_ENCODER_CH4_GPIO_PINSRC_A      GPIO_PinSource6
#define BSP_ENCODER_CH4_GPIO_PORT_B        GPIOB
#define BSP_ENCODER_CH4_GPIO_PIN_B         GPIO_Pin_7
#define BSP_ENCODER_CH4_GPIO_PINSRC_B      GPIO_PinSource7
#define BSP_ENCODER_CH4_GPIO_AF            GPIO_AF_TIM4
#define BSP_ENCODER_CH4_PERIOD             0xFFFFU
#define BSP_ENCODER_CH4_REVERSE            0

/*
 * 编码器通道枚举。
 * BSP 层仍然使用 CH1~CH4，不直接写 FL/FR/RL/RR，避免 BSP 绑定具体车体结构。
 * 轮子映射建议在 drv_encoder.h 中完成，例如：
 *   #define DRV_ENCODER_FL_BSP_ID   BSP_ENCODER_CH1
 *   #define DRV_ENCODER_FR_BSP_ID   BSP_ENCODER_CH2
 *   #define DRV_ENCODER_RL_BSP_ID   BSP_ENCODER_CH3
 *   #define DRV_ENCODER_RR_BSP_ID   BSP_ENCODER_CH4
 */
typedef enum {
#if BSP_ENCODER_CH1_ENABLE
    BSP_ENCODER_CH1,
#endif
#if BSP_ENCODER_CH2_ENABLE
    BSP_ENCODER_CH2,
#endif
#if BSP_ENCODER_CH3_ENABLE
    BSP_ENCODER_CH3,
#endif
#if BSP_ENCODER_CH4_ENABLE
    BSP_ENCODER_CH4,
#endif
    BSP_ENCODER_COUNT
} BSP_Encoder_Id_t;

typedef struct {
    int16_t  delta_count;        /* 最近一次 Update 的增量，单位：count */
    int32_t  total_count;        /* 累计增量，Clear 后重新累计，单位：count */
    int32_t  speed_cps;          /* counts per second，单位：count/s */
    uint32_t update_time_ms;     /* 最近一次更新时刻 */
} BSP_Encoder_Info_t;

void         BSP_Encoder_Init(BSP_Encoder_Id_t id);
void         BSP_Encoder_InitAll(void);
void         BSP_Encoder_Update(BSP_Encoder_Id_t id);
void         BSP_Encoder_UpdateAll(void);
int16_t      BSP_Encoder_GetDelta(BSP_Encoder_Id_t id);
int32_t      BSP_Encoder_GetSpeedCps(BSP_Encoder_Id_t id);
int32_t      BSP_Encoder_GetTotal(BSP_Encoder_Id_t id);
void         BSP_Encoder_ClearTotal(BSP_Encoder_Id_t id);
void         BSP_Encoder_ClearAllTotal(void);
BSP_Status_t BSP_Encoder_GetInfo(BSP_Encoder_Id_t id, BSP_Encoder_Info_t *info);

void Test_Encoder_Log(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ENCODER_H */
