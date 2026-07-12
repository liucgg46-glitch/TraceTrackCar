#ifndef __DRV_VL53L1X_H
#define __DRV_VL53L1X_H

#include "bsp_common.h"
#include "bsp_i2c.h"
#include "bsp_gpio.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * VL53L1X ToF 激光测距 Driver 配置区
 * ============================================================================
 *
 * 硬件连接（默认单传感器、软件轮询模式）：
 *   VL53L1X SDA   -> I2C1 SDA（本工程默认 PB9）
 *   VL53L1X SCL   -> I2C1 SCL（本工程默认 PB8）
 *   VL53L1X GND   -> STM32 GND
 *   VL53L1X VIN   -> 按所购模块要求供电
 *   VL53L1X XSHUT -> 默认不由 MCU 控制；模块必须已上拉为高电平
 *   VL53L1X GPIO1 -> 默认不用，可悬空
 *
 * 地址说明：
 *   ST ULD API 使用 8-bit 地址，默认 0x52；
 *   本工程 BSP 使用 7-bit 地址，平台层会自动把 0x52 转换为 0x29。
 *
 * 调度说明：
 *   Drv_VL53L1X_Update() 建议每 1 ms 调用一次。
 *   它不会等待测量完成，而是通过状态机分步检查数据、读取结果、清中断。
 *   官方 VL53L1X_SensorInit() 本身为同步 API，只在启动/重连阶段执行一次。
 */

/* ------------------------------ 总开关 ----------------------------------- */
#define DRV_VL53L1X_ENABLE                     1U

/* ------------------------------ I2C 配置 --------------------------------- */
#define DRV_VL53L1X_I2C_BUS                    I2C_BUS1
#define DRV_VL53L1X_I2C_ADDR_8BIT              0x52U

/* VL53L1X 芯片识别字：0x010F=0xEA，0x0110=0xCC，因此 16 位 ID 为 0xEACC。 */
#define DRV_VL53L1X_EXPECTED_SENSOR_ID          0xEACCU
#define DRV_VL53L1X_PLATFORM_MAX_TRANSFER      256U

/* ------------------------------ 测距模式 --------------------------------- */
#define DRV_VL53L1X_DISTANCE_MODE_SHORT        1U
#define DRV_VL53L1X_DISTANCE_MODE_LONG         2U

/*
 * SHORT：抗环境光能力更好，典型最大距离约 1.3 m。
 * LONG ：暗环境下可达到约 4 m，但强环境光下量程下降更明显。
 */
#define DRV_VL53L1X_DISTANCE_MODE              DRV_VL53L1X_DISTANCE_MODE_LONG

/*
 * Timing Budget 可选值：15、20、33、50、100、200、500 ms。
 * 15 ms 只允许 SHORT 模式；LONG 模式最小建议 33 ms。
 * 时间越长，测量稳定性和量程通常越好，但更新率降低、功耗增加。
 */
#define DRV_VL53L1X_TIMING_BUDGET_MS           50U

/*
 * 两次测量的启动间隔。必须 >= Timing Budget，否则实际间隔可能翻倍。
 * 小车避障默认 60 ms，约 16.7 Hz。
 */
#define DRV_VL53L1X_INTER_MEASUREMENT_MS       60U

/* 软件轮询数据就绪寄存器的最短间隔。无需等于测量周期。 */
#define DRV_VL53L1X_READY_POLL_PERIOD_MS       5U

/* GPIO1 中断极性配置。当前未接 GPIO1，仅影响内部数据就绪逻辑。 */
#define DRV_VL53L1X_INTERRUPT_ACTIVE_HIGH      0U

/* ------------------------------ 数据有效范围 ----------------------------- */
#define DRV_VL53L1X_MIN_VALID_DISTANCE_MM      20U
#define DRV_VL53L1X_MAX_VALID_DISTANCE_MM      4000U
#define DRV_VL53L1X_VALID_RANGE_STATUS         0U

/* ------------------------------ 距离滤波 --------------------------------- */
/*
 * 1：输出 IIR 滤波后的 distance_mm；0：distance_mm 直接等于 raw_distance_mm。
 * FILTER_DIV 越大越平滑，但响应越慢。推荐 2~8。
 */
#define DRV_VL53L1X_FILTER_ENABLE              1U
#define DRV_VL53L1X_FILTER_DIV                 4U

/* ------------------------------ 可选标定值 ------------------------------- */
/* Offset 单位 mm；没有做过专门标定时保持关闭。 */
#define DRV_VL53L1X_APPLY_OFFSET_ENABLE        0U
#define DRV_VL53L1X_OFFSET_MM                  0

/* Xtalk 单位 cps；安装盖板并完成串扰标定后再开启。 */
#define DRV_VL53L1X_APPLY_XTALK_ENABLE         0U
#define DRV_VL53L1X_XTALK_CPS                  0U

/* ------------------------------ 启动与恢复 ------------------------------- */
#define DRV_VL53L1X_POWER_ON_WAIT_MS           3U
#define DRV_VL53L1X_BOOT_POLL_PERIOD_MS        2U
#define DRV_VL53L1X_BOOT_TIMEOUT_MS            100U
#define DRV_VL53L1X_REINIT_DELAY_MS            500U
#define DRV_VL53L1X_RUNTIME_RETRY_DELAY_MS     5U
#define DRV_VL53L1X_MAX_CONSECUTIVE_ERRORS     3U
#define DRV_VL53L1X_ONLINE_TIMEOUT_MS          500U

/*
 * XSHUT 默认关闭：适合常见带稳压/上拉的 VL53L1X 模块。
 * 如需 MCU 控制 XSHUT：
 *   1. 在 BSP/bsp_gpio.h 增加一个空闲输出通道并定义设备别名；
 *   2. 把 USE_XSHUT 改成 1；
 *   3. 将 XSHUT_GPIO 改成该别名。
 */
#define DRV_VL53L1X_USE_XSHUT                  0U
#if DRV_VL53L1X_USE_XSHUT
#define DRV_VL53L1X_XSHUT_GPIO                 BSP_GPIO_CH1 /* 启用前必须改成专用空闲脚 */
#define DRV_VL53L1X_XSHUT_LOW_MS               2U
#endif

/* ------------------------------ 编译期检查 ------------------------------- */
#if ((DRV_VL53L1X_DISTANCE_MODE != DRV_VL53L1X_DISTANCE_MODE_SHORT) && \
     (DRV_VL53L1X_DISTANCE_MODE != DRV_VL53L1X_DISTANCE_MODE_LONG))
#error "DRV_VL53L1X_DISTANCE_MODE must be SHORT(1) or LONG(2)"
#endif

#if !((DRV_VL53L1X_TIMING_BUDGET_MS == 15U)  || \
      (DRV_VL53L1X_TIMING_BUDGET_MS == 20U)  || \
      (DRV_VL53L1X_TIMING_BUDGET_MS == 33U)  || \
      (DRV_VL53L1X_TIMING_BUDGET_MS == 50U)  || \
      (DRV_VL53L1X_TIMING_BUDGET_MS == 100U) || \
      (DRV_VL53L1X_TIMING_BUDGET_MS == 200U) || \
      (DRV_VL53L1X_TIMING_BUDGET_MS == 500U))
#error "Unsupported VL53L1X timing budget"
#endif

#if ((DRV_VL53L1X_DISTANCE_MODE == DRV_VL53L1X_DISTANCE_MODE_LONG) && \
     (DRV_VL53L1X_TIMING_BUDGET_MS < 33U))
#error "LONG mode requires timing budget >= 33 ms"
#endif

#if (DRV_VL53L1X_INTER_MEASUREMENT_MS < DRV_VL53L1X_TIMING_BUDGET_MS)
#error "VL53L1X inter-measurement period must be >= timing budget"
#endif

#if (DRV_VL53L1X_FILTER_DIV == 0U)
#error "DRV_VL53L1X_FILTER_DIV must be >= 1"
#endif

#if (DRV_VL53L1X_MAX_CONSECUTIVE_ERRORS == 0U)
#error "DRV_VL53L1X_MAX_CONSECUTIVE_ERRORS must be >= 1"
#endif

/* ------------------------------ 状态与数据 ------------------------------- */
typedef enum {
    DRV_VL53L1X_STATE_DISABLED = 0,
    DRV_VL53L1X_STATE_XSHUT_LOW,
    DRV_VL53L1X_STATE_POWER_ON_WAIT,
    DRV_VL53L1X_STATE_BOOT_CHECK,
    DRV_VL53L1X_STATE_SENSOR_INIT,
    DRV_VL53L1X_STATE_SET_DISTANCE_MODE,
    DRV_VL53L1X_STATE_SET_TIMING_BUDGET,
    DRV_VL53L1X_STATE_SET_INTER_MEASUREMENT,
    DRV_VL53L1X_STATE_SET_INTERRUPT_POLARITY,
    DRV_VL53L1X_STATE_SET_OFFSET,
    DRV_VL53L1X_STATE_SET_XTALK,
    DRV_VL53L1X_STATE_START_RANGING,
    DRV_VL53L1X_STATE_CHECK_DATA_READY,
    DRV_VL53L1X_STATE_READ_RESULT,
    DRV_VL53L1X_STATE_CLEAR_INTERRUPT,
    DRV_VL53L1X_STATE_ERROR_WAIT,
    DRV_VL53L1X_STATE_STOPPED
} Drv_VL53L1X_State_t;

typedef struct {
    uint8_t enabled;
    uint8_t initialized;
    uint8_t ranging;
    uint8_t online;
    uint8_t data_valid;
    uint8_t new_data;
    uint8_t data_ready;
    uint8_t range_status;
    uint8_t boot_state;
    uint8_t last_api_status;
    uint8_t consecutive_error_count;
    Drv_VL53L1X_State_t state;

    uint16_t sensor_id;
    uint16_t raw_distance_mm;
    uint16_t distance_mm;
    uint16_t ambient_kcps;
    uint16_t signal_per_spad_kcps;
    uint16_t spad_count;

    uint32_t measurement_count;
    uint32_t valid_count;
    uint32_t error_count;
    uint32_t busy_skip_count;
    uint32_t reinit_count;
    uint32_t last_sample_ms;
    uint32_t last_valid_ms;
} Drv_VL53L1X_Info_t;

void Drv_VL53L1X_Init(void);
BSP_Status_t Drv_VL53L1X_Update(void);
void Drv_VL53L1X_RequestReinit(void);
BSP_Status_t Drv_VL53L1X_Stop(void);

uint8_t Drv_VL53L1X_IsOnline(void);
uint8_t Drv_VL53L1X_HasNewData(void);
void Drv_VL53L1X_ClearNewData(void);
BSP_Status_t Drv_VL53L1X_GetDistanceMm(uint16_t *distance_mm);
BSP_Status_t Drv_VL53L1X_GetInfo(Drv_VL53L1X_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_VL53L1X_H */
