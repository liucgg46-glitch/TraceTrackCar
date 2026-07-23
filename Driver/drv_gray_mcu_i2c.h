#ifndef __DRV_GRAY_MCU_I2C_H
#define __DRV_GRAY_MCU_I2C_H

#include "bsp_common.h"
#include "bsp_i2c.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 感为 8 路灰度传感器 I2C 驱动。
 *
 * 关键协议：
 *   0xAA：ping，返回 0x66；
 *   0xB0：连续通道模拟量；
 *   0xCE：传输通道使能；
 *   0xCF：归一化使能（固件 V3.6 及以上）；
 *   0xC1：固件版本。
 *
 * 连续采样采用手册 7.9 的“方法 2”：
 *   1. 初始化时只发送一次 0xB0，并产生 STOP；
 *   2. 运行中反复执行纯读事务，每次读取 8 字节。
 *
 * 这样不会在每一帧都重复发送 0xB0，也不会每一帧都使用 repeated START，
 * 更适合与 OLED 等设备共用一条 I2C 总线。
 */

#define DRV_GRAY_MCU_CHANNEL_NUM                 8U

#define DRV_GRAY_MCU_I2C_BUS                     I2C_BUS1

/*
 * 地址：无 AD 跳线时通常为 0x4C。
 * 启动阶段可扫描 0x4C~0x4F；一旦成功，运行阶段锁定该地址，
 * 单次通讯错误不会再跳到 0x4D/0x4E/0x4F。
 */
#define DRV_GRAY_MCU_ADDR_AD1                    0U
#define DRV_GRAY_MCU_ADDR_AD0                    0U
#define DRV_GRAY_MCU_DEFAULT_ADDR_7BIT           \
    ((uint8_t)(0x4CU | ((DRV_GRAY_MCU_ADDR_AD1 & 0x01U) << 1U) | \
                       (DRV_GRAY_MCU_ADDR_AD0 & 0x01U)))
#define DRV_GRAY_MCU_AUTO_ADDR_SCAN              1U
#define DRV_GRAY_MCU_UNLOCK_AFTER_PING_FAILS     20U

#define DRV_GRAY_MCU_CMD_PING                    0xAAU
#define DRV_GRAY_MCU_PING_OK                     0x66U
#define DRV_GRAY_MCU_CMD_ANALOG_ALL              0xB0U
#define DRV_GRAY_MCU_CMD_DIGITAL                 0xDDU
#define DRV_GRAY_MCU_CMD_CHANNEL_ENABLE          0xCEU
#define DRV_GRAY_MCU_CMD_NORMALIZE               0xCFU
#define DRV_GRAY_MCU_CMD_ERROR                   0xDEU
#define DRV_GRAY_MCU_CMD_REBOOT                  0xC0U
#define DRV_GRAY_MCU_CMD_FIRMWARE                0xC1U

#define DRV_GRAY_MCU_CHANNEL_ENABLE_MASK         0xFFU
#define DRV_GRAY_MCU_NORMALIZE_MASK              0x00U

#define DRV_GRAY_MCU_FIRST_PING_DELAY_MS         5U
#define DRV_GRAY_MCU_PING_RETRY_MS               50U
#define DRV_GRAY_MCU_UPDATE_PERIOD_MS            10U
#define DRV_GRAY_MCU_RUNTIME_RETRY_MS             5U
#define DRV_GRAY_MCU_INIT_RETRY_MS               20U
#define DRV_GRAY_MCU_ERROR_BACKOFF_MS            100U
#define DRV_GRAY_MCU_REBOOT_WAIT_MS              20U
#define DRV_GRAY_MCU_RUNTIME_FAIL_LIMIT           5U
#define DRV_GRAY_MCU_STALE_TIMEOUT_MS            100U

#define DRV_GRAY_MCU_SCALE_TO_12BIT              1U
#define DRV_GRAY_MCU_FILTER_SHIFT                0U
#define DRV_GRAY_MCU_INDEX_REVERSE               1U

#if DRV_GRAY_MCU_SCALE_TO_12BIT
#define DRV_GRAY_MCU_DISABLED_VALUE              4095U
#else
#define DRV_GRAY_MCU_DISABLED_VALUE              255U
#endif

typedef enum {
    DRV_GRAY_MCU_PHASE_IDLE = 0,
    DRV_GRAY_MCU_PHASE_PING,
    DRV_GRAY_MCU_PHASE_FIRMWARE,
    DRV_GRAY_MCU_PHASE_CHANNEL_ENABLE,
    DRV_GRAY_MCU_PHASE_NORMALIZE,
    DRV_GRAY_MCU_PHASE_ANALOG_SELECT,
    DRV_GRAY_MCU_PHASE_ANALOG,
    DRV_GRAY_MCU_PHASE_DIGITAL,
    DRV_GRAY_MCU_PHASE_ERROR,
    DRV_GRAY_MCU_PHASE_REBOOT
} Drv_GrayMcu_Phase_t;

typedef struct {
    uint16_t raw[DRV_GRAY_MCU_CHANNEL_NUM];
    uint16_t filt[DRV_GRAY_MCU_CHANNEL_NUM];
    uint8_t  rx[DRV_GRAY_MCU_CHANNEL_NUM];

    uint8_t  digital_data;
    uint8_t  digital_mask;
    uint8_t  error_flags;
    uint8_t  firmware_version;
    uint8_t  channel_enable_mask;
    uint8_t  normalize_mask;

    uint8_t  online;
    uint8_t  valid;
    uint8_t  initialized;
    uint8_t  active_addr;

    uint8_t  current_phase;
    uint8_t  last_phase;
    uint8_t  done_phase;
    uint8_t  last_op;
    uint8_t  last_reg;
    uint8_t  last_rx_len;
    uint8_t  ping_value;

    uint8_t  scan_count;
    uint8_t  scan_mask;
    uint8_t  address_locked;
    uint8_t  consecutive_error_count;

    uint32_t update_count;
    uint32_t error_count;
    uint32_t ping_ok_count;
    uint32_t ping_error_count;
    uint32_t last_update_ms;

    BSP_Status_t last_status;
    int          last_i2c_result;
} Drv_GrayMcu_Info_t;

void Drv_GrayMcu_Init(void);
BSP_Status_t Drv_GrayMcu_Update(void);

uint16_t Drv_GrayMcu_GetRaw(uint8_t index);
uint16_t Drv_GrayMcu_GetFilt(uint8_t index);
BSP_Status_t Drv_GrayMcu_GetRawArray(uint16_t *out_buf, uint8_t max_count);
BSP_Status_t Drv_GrayMcu_GetFiltArray(uint16_t *out_buf, uint8_t max_count);
BSP_Status_t Drv_GrayMcu_GetInfo(Drv_GrayMcu_Info_t *info);
uint8_t Drv_GrayMcu_IsOnline(void);
uint8_t Drv_GrayMcu_IsBusy(void);
BSP_Status_t Drv_GrayMcu_GetRx8(uint8_t *out_buf, uint8_t max_count);

BSP_Status_t Drv_GrayMcu_Ping(void);
BSP_Status_t Drv_GrayMcu_ReadDigital(uint8_t *digital_data);
BSP_Status_t Drv_GrayMcu_ReadErrorFlags(uint8_t *error_flags);
BSP_Status_t Drv_GrayMcu_ReadFirmware(uint8_t *firmware_version);
BSP_Status_t Drv_GrayMcu_SetChannelEnable(uint8_t enable_mask);
BSP_Status_t Drv_GrayMcu_SetFlatEnable(uint8_t enable_mask);
BSP_Status_t Drv_GrayMcu_Reboot(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_GRAY_MCU_I2C_H */
