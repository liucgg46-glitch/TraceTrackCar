#ifndef __DRV_ICM20948_H
#define __DRV_ICM20948_H

#include "bsp_common.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * ICM-20948 九轴 IMU 驱动配置区
 * ============================================================================
 *
 * 硬件接口：
 *   SPI2_SCK  -> PB13
 *   SPI2_MISO -> PB14（ICM-20948 SDO）
 *   SPI2_MOSI -> PB15（ICM-20948 SDI）
 *   CS        -> PE5（由 BSP_GPIO_ICM20948_CS 管理）
 *
 * SPI2 与 TFT LCD 共用 SCK/MISO/MOSI，但两者使用独立 CS。驱动在每次访问前
 * 检查 BSP_SPI_IsBusy()，LCD 正在 DMA 传输时本轮自动跳过，不会抢占总线。
 *
 * 本驱动提供：
 *   - 三轴加速度、三轴角速度、三轴磁场和芯片温度；
 *   - 原始 ADC 数据、物理量数据、滤波数据；
 *   - 芯片硬件 DLPF；
 *   - 三点中值去毛刺 + 一阶低通滤波；
 *   - 上电静止陀螺仪零偏标定；
 *   - 磁力计 AK09916 自动初始化和连续读取；
 *   - 错误计数、离线判定和自动重新初始化。
 *
 * 注意：本文件只输出传感器数据，不做姿态角融合。Mahony/EKF 等姿态算法应放在
 * Algorithm 层，读取本驱动输出的 filtered 数据。
 */

/* ============================== 总开关与硬件 ============================== */
#define DRV_ICM20948_ENABLE                    1U
#define DRV_ICM20948_SPI_BUS                   SPI_BUS2
#define DRV_ICM20948_CS_GPIO                   BSP_GPIO_ICM20948_CS

/* ICM-20948 主芯片固定标识。 */
#define DRV_ICM20948_WHO_AM_I_EXPECTED          0xEAU

/*
 * AK09916 身份判断：
 *   - 官方 eMD 驱动读取 WIA1（0x00），期望 0x48；
 *   - ICM-20948 v1.5 数据手册明确列出 WIA2（0x01），固定为 0x09。
 * 为兼容不同批次，只要其中一个标识正确就允许继续初始化，同时保留两个值用于诊断。
 */
#define DRV_ICM20948_MAG_WIA1_EXPECTED          0x48U
#define DRV_ICM20948_MAG_WIA2_EXPECTED          0x09U

/* ============================== 采样配置 ================================== */
/*
 * 加速度计与陀螺仪低噪声模式输出频率：
 *   ODR = 1125 Hz / (1 + DIV)
 * 默认 DIV=10，对应约 102.27 Hz。
 */
#define DRV_ICM20948_GYRO_SAMPLE_DIV            10U
#define DRV_ICM20948_ACCEL_SAMPLE_DIV           10U

/* 驱动检查新数据的周期。建议不大于实际采样周期。 */
#define DRV_ICM20948_UPDATE_PERIOD_MS            5U

/*
 * 满量程选择：
 *   加速度：0=±2g，1=±4g，2=±8g，3=±16g
 *   陀螺仪：0=±250dps，1=±500dps，2=±1000dps，3=±2000dps
 */
#define DRV_ICM20948_ACCEL_FS_SEL                1U
#define DRV_ICM20948_GYRO_FS_SEL                 1U

/*
 * 芯片内部数字低通滤波器（DLPF）。
 * ENABLE=1 时，CFG=3 对应：
 *   陀螺仪约 51.2 Hz 带宽；
 *   加速度计约 50.4 Hz 带宽。
 */
#define DRV_ICM20948_HW_DLPF_ENABLE              1U
#define DRV_ICM20948_GYRO_DLPF_CFG               3U
#define DRV_ICM20948_ACCEL_DLPF_CFG              3U

/* AK09916 单次测量模式；运行时由 SLV1 每个内部 I2C 周期重新触发。 */
#define DRV_ICM20948_MAG_ENABLE                  1U

/*
 * 0：磁力计初始化失败时，保留六轴加速度/陀螺仪采样，mag_valid 保持 0；
 * 1：磁力计初始化失败时，整个 IMU 进入错误重试。
 * 调试阶段建议为 0，便于区分“主 IMU 正常、内部磁力计异常”。
 */
#define DRV_ICM20948_MAG_REQUIRED                0U

#define DRV_ICM20948_MAG_MODE                    0x01U

/*
 * 磁力计初始化访问方式：
 *   1：使用 I2C_SLV0 读取 + I2C_SLV1 写入（推荐）。
 *      该方式与后续连续磁力计采样使用同一套内部 I2C 调度器，可绕开
 *      某些模块上 I2C_SLV4 的 EN 位始终不自动清零、DONE 始终为 0 的问题。
 *   0：使用 I2C_SLV4 单字节事务，保留用于对比诊断。
 */
#define DRV_ICM20948_MAG_INIT_USE_SLV0_SLV1      1U

/*
 * SLV0/SLV1 由内部采样时钟触发，不是写寄存器后立即完成。
 * 官方 eMD 一次性辅助事务等待 60 ms；这里采用相同等待时间，兼容启动初期时钟尚未稳定的情况。
 */
#define DRV_ICM20948_MAG_SLV01_WAIT_MS           60U

/* ICM-20948 内部 I2C Master 时钟和外部传感器 ODR 配置。 */
#define DRV_ICM20948_I2C_MST_CTRL_VALUE          0x17U
#define DRV_ICM20948_I2C_MST_ODR_CONFIG          0x04U

/* ============================== 时间与错误恢复 ============================ */
/* 裸芯片上电后最长约 100 ms 才保证可读写，模块也按此等待。 */
#define DRV_ICM20948_POWER_ON_WAIT_MS           100U
#define DRV_ICM20948_RESET_WAIT_MS              100U
#define DRV_ICM20948_WAKE_WAIT_MS                50U
#define DRV_ICM20948_MAG_RESET_WAIT_MS           10U

/*
 * 内部 I2C Master 使能/复位后留出稳定时间。AK09916 初始化失败时，
 * 驱动会复位内部 I2C Master 并重试，而不是立即让整个 IMU 离线。
 */
#define DRV_ICM20948_I2C_MASTER_START_WAIT_MS    10U
#define DRV_ICM20948_I2C_MASTER_RESET_WAIT_MS    10U
#define DRV_ICM20948_AUX_TRANSACTION_TIMEOUT_MS 100U
#define DRV_ICM20948_MAG_INIT_MAX_RETRIES        10U
#define DRV_ICM20948_ERROR_RETRY_MS             500U
#define DRV_ICM20948_ONLINE_TIMEOUT_MS          200U
#define DRV_ICM20948_MAX_CONSECUTIVE_ERRORS       3U

/* ============================== 自动零偏标定 ============================== */
/*
 * 上电后保持模块静止，驱动自动累计陀螺仪零偏。
 * 约 102 Hz 采样时，200 个稳定样本约需 2 秒。
 */
#define DRV_ICM20948_GYRO_CAL_ENABLE              1U
#define DRV_ICM20948_GYRO_CAL_SAMPLE_COUNT      200U
#define DRV_ICM20948_GYRO_CAL_MIN_SAMPLES       DRV_ICM20948_GYRO_CAL_SAMPLE_COUNT
#define DRV_ICM20948_GYRO_CAL_TIMEOUT_MS        6000U
#define DRV_ICM20948_GYRO_CAL_MAX_DPS             6.0f
#define DRV_ICM20948_GYRO_CAL_ACCEL_MIN_G         0.80f
#define DRV_ICM20948_GYRO_CAL_ACCEL_MAX_G         1.20f

/* ============================== 软件滤波 ================================== */
/*
 * 先使用最近 3 个样本中值去除单点尖峰，再使用一阶低通：
 *   y = y + alpha * (x - y)
 * alpha 越小越平滑但响应越慢；越大响应越快但噪声更明显。
 */
#define DRV_ICM20948_SW_FILTER_ENABLE             1U
#define DRV_ICM20948_MEDIAN3_ENABLE               1U
#define DRV_ICM20948_ACCEL_FILTER_ALPHA           0.25f
#define DRV_ICM20948_GYRO_FILTER_ALPHA            0.35f
#define DRV_ICM20948_MAG_FILTER_ALPHA             0.20f
#define DRV_ICM20948_TEMP_FILTER_ALPHA            0.10f

/* ============================== 轴映射 ==================================== */
/*
 * SRC 取值：0=X，1=Y，2=Z；SIGN 取值：+1.0f 或 -1.0f。
 * 默认保持芯片坐标系。模块安装方向改变时，只改这里，不改 .c 文件。
 */
#define DRV_ICM20948_AXIS_X_SOURCE                0U
#define DRV_ICM20948_AXIS_Y_SOURCE                1U
#define DRV_ICM20948_AXIS_Z_SOURCE                2U
#define DRV_ICM20948_AXIS_X_SIGN                  1.0f
#define DRV_ICM20948_AXIS_Y_SIGN                  1.0f
#define DRV_ICM20948_AXIS_Z_SIGN                  1.0f

/*
 * AK09916 在 ICM-20948 封装内部相对 Accel/Gyro 的固定安装矩阵。
 * InvenSense 官方 eMD 的 AK09916 二级矩阵为 diag(+1, -1, -1)。
 * 必须先完成此对齐，再应用上面的整板安装方向映射。
 */
#define DRV_ICM20948_MAG_TO_IMU_X_SIGN            1.0f
#define DRV_ICM20948_MAG_TO_IMU_Y_SIGN           -1.0f
#define DRV_ICM20948_MAG_TO_IMU_Z_SIGN           -1.0f

/* ============================== 静态校准参数 ============================== */
/*
 * 计算顺序：
 *   corrected = (mapped - offset) * scale
 *
 * 陀螺仪还会额外减去上电自动标定得到的零偏。
 * 磁力计正式用于航向前，建议完成硬铁偏置和软铁比例标定，再填写这些参数。
 */
#define DRV_ICM20948_ACCEL_OFFSET_X_G             0.0f
#define DRV_ICM20948_ACCEL_OFFSET_Y_G             0.0f
#define DRV_ICM20948_ACCEL_OFFSET_Z_G             0.0f
#define DRV_ICM20948_ACCEL_SCALE_X                1.0f
#define DRV_ICM20948_ACCEL_SCALE_Y                1.0f
#define DRV_ICM20948_ACCEL_SCALE_Z                1.0f

#define DRV_ICM20948_GYRO_OFFSET_X_DPS            0.0f
#define DRV_ICM20948_GYRO_OFFSET_Y_DPS            0.0f
#define DRV_ICM20948_GYRO_OFFSET_Z_DPS            0.0f
#define DRV_ICM20948_GYRO_SCALE_X                 1.0f
#define DRV_ICM20948_GYRO_SCALE_Y                 1.0f
#define DRV_ICM20948_GYRO_SCALE_Z                 1.0f

#define DRV_ICM20948_MAG_OFFSET_X_UT              0.0f
#define DRV_ICM20948_MAG_OFFSET_Y_UT              0.0f
#define DRV_ICM20948_MAG_OFFSET_Z_UT              0.0f
#define DRV_ICM20948_MAG_SCALE_X                  1.0f
#define DRV_ICM20948_MAG_SCALE_Y                  1.0f
#define DRV_ICM20948_MAG_SCALE_Z                  1.0f

/* 温度换算：T = (raw - offset) / 333.87 + 21°C。 */
#define DRV_ICM20948_TEMP_ROOM_OFFSET_LSB         0.0f
#define DRV_ICM20948_TEMP_SENSITIVITY_LSB_C     333.87f
#define DRV_ICM20948_TEMP_ROOM_C                 21.0f

/* ============================== 数据类型 ================================== */
typedef struct {
    float x;
    float y;
    float z;
} Drv_ICM20948_Vector3f_t;

typedef struct {
    int16_t accel[3];
    int16_t gyro[3];
    int16_t mag[3];
    int16_t temperature;
} Drv_ICM20948_Raw_t;

typedef struct {
    Drv_ICM20948_Raw_t raw;

    /* 已完成量程换算、轴映射和静态校准，但未经过软件低通。 */
    Drv_ICM20948_Vector3f_t accel_g;
    Drv_ICM20948_Vector3f_t gyro_dps;
    Drv_ICM20948_Vector3f_t mag_uT;
    float temperature_c;

    /* 推荐给上层算法使用的滤波结果。 */
    Drv_ICM20948_Vector3f_t accel_filtered_g;
    Drv_ICM20948_Vector3f_t gyro_filtered_dps;
    Drv_ICM20948_Vector3f_t mag_filtered_uT;
    float temperature_filtered_c;

    uint32_t timestamp_ms;
    uint8_t accel_gyro_valid;
    uint8_t mag_valid;
    uint8_t mag_updated;
    uint8_t new_data;
} Drv_ICM20948_Data_t;

typedef enum {
    DRV_ICM20948_STATE_DISABLED = 0,
    DRV_ICM20948_STATE_POWER_WAIT,
    DRV_ICM20948_STATE_WHO_AM_I,
    DRV_ICM20948_STATE_RESET,
    DRV_ICM20948_STATE_RESET_WAIT,
    DRV_ICM20948_STATE_CONFIG_POWER,
    DRV_ICM20948_STATE_WAKE_WAIT,
    DRV_ICM20948_STATE_CONFIG_ACCEL_GYRO,
    DRV_ICM20948_STATE_CONFIG_I2C_MASTER,
    DRV_ICM20948_STATE_I2C_MASTER_START_WAIT,
    DRV_ICM20948_STATE_MAG_RESET_START,
    DRV_ICM20948_STATE_MAG_RESET_WAIT,
    DRV_ICM20948_STATE_MAG_RESET_DELAY,
    DRV_ICM20948_STATE_MAG_WIA1_START,
    DRV_ICM20948_STATE_MAG_WIA1_WAIT,
    DRV_ICM20948_STATE_MAG_WIA2_START,
    DRV_ICM20948_STATE_MAG_WIA2_WAIT,
    DRV_ICM20948_STATE_MAG_MODE_START,
    DRV_ICM20948_STATE_MAG_MODE_WAIT,
    DRV_ICM20948_STATE_CONFIG_MAG_STREAM,
    DRV_ICM20948_STATE_MAG_RECOVERY_RESET,
    DRV_ICM20948_STATE_MAG_RECOVERY_RESET_WAIT,
    DRV_ICM20948_STATE_MAG_RECOVERY_ENABLE,
    DRV_ICM20948_STATE_MAG_RECOVERY_ENABLE_WAIT,
    DRV_ICM20948_STATE_GYRO_CALIBRATION,
    DRV_ICM20948_STATE_RUN,
    DRV_ICM20948_STATE_ERROR_WAIT
} Drv_ICM20948_State_t;

typedef struct {
    uint8_t enabled;
    uint8_t online;
    uint8_t initialized;
    uint8_t running;
    uint8_t calibrating;
    uint8_t data_valid;
    uint8_t mag_valid;
    uint8_t who_am_i;
    uint8_t mag_wia1;
    uint8_t mag_wia2;
    uint8_t mag_st1;
    uint8_t mag_st2;

    /* 最近一次内部 I2C Master 状态寄存器和本轮磁力计恢复次数。 */
    uint8_t last_i2c_mst_status;
    uint8_t mag_retry_count;

    /* 关键寄存器回读值，用于判断 SPI 写操作是否真正生效。 */
    uint8_t user_ctrl_readback;
    uint8_t lp_config_readback;
    uint8_t i2c_mst_ctrl_readback;
    uint8_t slv4_addr_readback;
    uint8_t slv4_ctrl_readback;

    /*
     * 磁力计初始化诊断：
     *   mag_init_method = 1：SLV4；2：SLV0/SLV1。
     *   slv0/slv1_* 为内部辅助 I2C 配置寄存器回读值。
     */
    uint8_t mag_init_method;
    uint8_t slv0_addr_readback;
    uint8_t slv0_ctrl_readback;
    uint8_t slv1_addr_readback;
    uint8_t slv1_ctrl_readback;

    uint8_t consecutive_errors;
    BSP_Status_t last_status;
    Drv_ICM20948_State_t state;

    uint32_t sample_count;
    uint32_t valid_count;
    uint32_t mag_valid_count;
    uint32_t mag_not_ready_count;
    uint32_t mag_overflow_count;
    uint32_t mag_nack_count;
    uint32_t error_count;
    uint32_t busy_skip_count;
    uint32_t reinit_count;

    uint16_t gyro_cal_samples;
    Drv_ICM20948_Vector3f_t gyro_bias_dps;
    float gyro_cal_last_max_abs_dps;
    float gyro_cal_last_accel_norm_g;
    uint32_t gyro_cal_reject_count;
} Drv_ICM20948_Info_t;

/* ============================== 公共接口 ================================== */
void Drv_ICM20948_Init(void);
BSP_Status_t Drv_ICM20948_Update(void);

/* 获取当前完整缓存。只有返回 BSP_OK 时，加速度和陀螺仪数据才可用于控制。 */
BSP_Status_t Drv_ICM20948_GetData(Drv_ICM20948_Data_t *data);
BSP_Status_t Drv_ICM20948_GetInfo(Drv_ICM20948_Info_t *info);

uint8_t Drv_ICM20948_IsOnline(void);
uint8_t Drv_ICM20948_HasNewData(void);
void Drv_ICM20948_ClearNewData(void);

/* 请求在后续 Update() 中重新初始化，不在当前调用点直接访问 SPI。 */
void Drv_ICM20948_RequestReinit(void);

/* 重新开始静止陀螺仪零偏标定。调用后应保持模块静止。 */
void Drv_ICM20948_StartGyroCalibration(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_ICM20948_H */
