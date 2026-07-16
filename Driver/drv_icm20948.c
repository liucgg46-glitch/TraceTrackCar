#include "drv_icm20948.h"

#include "bsp_systick.h"
#include <math.h>
#include <string.h>

/* ============================== 寄存器定义 ================================ */
#define ICM20948_REG_BANK_SEL                 0x7FU

/* User Bank 0 */
#define ICM20948_B0_WHO_AM_I                  0x00U
#define ICM20948_B0_USER_CTRL                 0x03U
#define ICM20948_B0_LP_CONFIG                 0x05U
#define ICM20948_B0_PWR_MGMT_1                0x06U
#define ICM20948_B0_PWR_MGMT_2                0x07U
#define ICM20948_B0_I2C_MST_STATUS            0x17U
#define ICM20948_B0_INT_STATUS_1              0x1AU
#define ICM20948_B0_ACCEL_XOUT_H              0x2DU
#define ICM20948_B0_EXT_SLV_SENS_DATA_00      0x3BU

/* User Bank 2 */
#define ICM20948_B2_GYRO_SMPLRT_DIV           0x00U
#define ICM20948_B2_GYRO_CONFIG_1             0x01U
#define ICM20948_B2_ACCEL_SMPLRT_DIV_1        0x10U
#define ICM20948_B2_ACCEL_SMPLRT_DIV_2        0x11U
#define ICM20948_B2_ACCEL_CONFIG              0x14U

/* User Bank 3 */
#define ICM20948_B3_I2C_MST_ODR_CONFIG        0x00U
#define ICM20948_B3_I2C_MST_CTRL              0x01U
#define ICM20948_B3_I2C_SLV0_ADDR             0x03U
#define ICM20948_B3_I2C_SLV0_REG              0x04U
#define ICM20948_B3_I2C_SLV0_CTRL             0x05U
#define ICM20948_B3_I2C_SLV0_DO               0x06U
#define ICM20948_B3_I2C_SLV1_ADDR             0x07U
#define ICM20948_B3_I2C_SLV1_REG              0x08U
#define ICM20948_B3_I2C_SLV1_CTRL             0x09U
#define ICM20948_B3_I2C_SLV1_DO               0x0AU
#define ICM20948_B3_I2C_SLV4_ADDR             0x13U
#define ICM20948_B3_I2C_SLV4_REG              0x14U
#define ICM20948_B3_I2C_SLV4_CTRL             0x15U
#define ICM20948_B3_I2C_SLV4_DO               0x16U
#define ICM20948_B3_I2C_SLV4_DI               0x17U

/* AK09916 */
#define AK09916_I2C_ADDR                       0x0CU
#define AK09916_REG_WIA1                       0x00U
#define AK09916_REG_WIA2                       0x01U
#define AK09916_REG_ST1                        0x10U
#define AK09916_REG_CNTL2                      0x31U
#define AK09916_REG_CNTL3                      0x32U

/* 位定义 */
#define ICM20948_SPI_READ_BIT                  0x80U
#define ICM20948_USER_CTRL_I2C_MST_EN          0x20U
#define ICM20948_USER_CTRL_I2C_IF_DIS          0x10U
#define ICM20948_USER_CTRL_I2C_MST_RST         0x02U
#define ICM20948_LP_CONFIG_I2C_MST_CYCLE       0x40U
#define ICM20948_PWR_DEVICE_RESET              0x80U
#define ICM20948_DATA_READY_BIT                0x01U
#define ICM20948_I2C_SLV4_DONE                 0x40U
#define ICM20948_I2C_SLV4_NACK                 0x10U
#define ICM20948_I2C_SLV0_NACK                 0x01U
#define ICM20948_I2C_SLV1_NACK                 0x02U
#define ICM20948_I2C_SLV_READ                  0x80U
#define ICM20948_I2C_SLV_ENABLE                0x80U

#define ICM20948_BURST_DATA_LEN                23U
#define ICM20948_INVALID_BANK                  0xFFU
#if (DRV_ICM20948_ACCEL_FS_SEL > 3U)
#error "DRV_ICM20948_ACCEL_FS_SEL must be 0..3"
#endif
#if (DRV_ICM20948_GYRO_FS_SEL > 3U)
#error "DRV_ICM20948_GYRO_FS_SEL must be 0..3"
#endif
#if (DRV_ICM20948_ACCEL_DLPF_CFG > 7U)
#error "DRV_ICM20948_ACCEL_DLPF_CFG must be 0..7"
#endif
#if (DRV_ICM20948_GYRO_DLPF_CFG > 7U)
#error "DRV_ICM20948_GYRO_DLPF_CFG must be 0..7"
#endif
#if (DRV_ICM20948_MAG_INIT_USE_SLV0_SLV1 > 1U)
#error "DRV_ICM20948_MAG_INIT_USE_SLV0_SLV1 must be 0 or 1"
#endif
#if (DRV_ICM20948_MAG_SLV01_WAIT_MS < 10U)
#error "DRV_ICM20948_MAG_SLV01_WAIT_MS should be at least 10 ms"
#endif
#if (DRV_ICM20948_AXIS_X_SOURCE > 2U) || \
    (DRV_ICM20948_AXIS_Y_SOURCE > 2U) || \
    (DRV_ICM20948_AXIS_Z_SOURCE > 2U)
#error "ICM20948 axis source must be 0, 1 or 2"
#endif

/* ============================== 内部数据 ================================== */
typedef struct {
    float history[3][3];
    float output[3];
    uint8_t index;
    uint8_t count;
    uint8_t initialized;
} ICM20948_VectorFilter_t;

typedef struct {
    float output;
    uint8_t initialized;
} ICM20948_ScalarFilter_t;

static Drv_ICM20948_Data_t s_data;
static Drv_ICM20948_Info_t s_info;

static uint8_t s_current_bank = ICM20948_INVALID_BANK;
static uint32_t s_state_start_ms = 0U;
static uint32_t s_last_poll_ms = 0U;
static uint32_t s_last_sample_ms = 0U;
static uint8_t s_reinit_requested = 0U;
static uint8_t s_mag_stream_enabled = 0U;

static ICM20948_VectorFilter_t s_accel_filter;
static ICM20948_VectorFilter_t s_gyro_filter;
static ICM20948_VectorFilter_t s_mag_filter;
static ICM20948_ScalarFilter_t s_temp_filter;

static float s_gyro_cal_sum[3];
static uint32_t s_gyro_cal_start_ms = 0U;

/* ============================== 通用工具 ================================== */
static float ICM20948_AbsF(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t ICM20948_IsMagIdentityValid(void)
{
    return ((s_info.mag_wia1 == DRV_ICM20948_MAG_WIA1_EXPECTED) ||
            (s_info.mag_wia2 == DRV_ICM20948_MAG_WIA2_EXPECTED)) ? 1U : 0U;
}

static float ICM20948_Median3(float a, float b, float c)
{
    float temp;

    if (a > b) {
        temp = a;
        a = b;
        b = temp;
    }
    if (b > c) {
        temp = b;
        b = c;
        c = temp;
    }
    if (a > b) {
        temp = a;
        a = b;
        b = temp;
    }
    return b;
}

static void ICM20948_ResetFilters(void)
{
    memset(&s_accel_filter, 0, sizeof(s_accel_filter));
    memset(&s_gyro_filter, 0, sizeof(s_gyro_filter));
    memset(&s_mag_filter, 0, sizeof(s_mag_filter));
    memset(&s_temp_filter, 0, sizeof(s_temp_filter));
}

static void ICM20948_FilterVector(ICM20948_VectorFilter_t *filter,
                                  const float input[3],
                                  float alpha,
                                  float output[3])
{
    uint8_t axis;
    float sample;

    if ((filter == 0) || (input == 0) || (output == 0)) {
        return;
    }

#if (DRV_ICM20948_SW_FILTER_ENABLE == 0U)
    for (axis = 0U; axis < 3U; axis++) {
        output[axis] = input[axis];
    }
    (void)alpha;
    return;
#else
    for (axis = 0U; axis < 3U; axis++) {
        filter->history[axis][filter->index] = input[axis];

#if (DRV_ICM20948_MEDIAN3_ENABLE != 0U)
        if (filter->count >= 3U) {
            sample = ICM20948_Median3(filter->history[axis][0],
                                      filter->history[axis][1],
                                      filter->history[axis][2]);
        } else {
            sample = input[axis];
        }
#else
        sample = input[axis];
#endif

        if (filter->initialized == 0U) {
            filter->output[axis] = sample;
        } else {
            filter->output[axis] += alpha * (sample - filter->output[axis]);
        }
        output[axis] = filter->output[axis];
    }

    filter->index++;
    if (filter->index >= 3U) {
        filter->index = 0U;
    }
    if (filter->count < 3U) {
        filter->count++;
    }
    filter->initialized = 1U;
#endif
}

static float ICM20948_FilterScalar(ICM20948_ScalarFilter_t *filter,
                                   float input,
                                   float alpha)
{
#if (DRV_ICM20948_SW_FILTER_ENABLE == 0U)
    (void)filter;
    (void)alpha;
    return input;
#else
    if (filter->initialized == 0U) {
        filter->output = input;
        filter->initialized = 1U;
    } else {
        filter->output += alpha * (input - filter->output);
    }
    return filter->output;
#endif
}

static float ICM20948_GetAccelSensitivity(void)
{
    static const float sensitivity[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
    return sensitivity[DRV_ICM20948_ACCEL_FS_SEL];
}

static float ICM20948_GetGyroSensitivity(void)
{
    static const float sensitivity[4] = {131.0f, 65.5f, 32.8f, 16.4f};
    return sensitivity[DRV_ICM20948_GYRO_FS_SEL];
}

static void ICM20948_MapAxes(const float source[3], float mapped[3])
{
    mapped[0] = source[DRV_ICM20948_AXIS_X_SOURCE] * DRV_ICM20948_AXIS_X_SIGN;
    mapped[1] = source[DRV_ICM20948_AXIS_Y_SOURCE] * DRV_ICM20948_AXIS_Y_SIGN;
    mapped[2] = source[DRV_ICM20948_AXIS_Z_SOURCE] * DRV_ICM20948_AXIS_Z_SIGN;
}

static void ICM20948_MapMagAxes(const float source[3], float mapped[3])
{
    float aligned[3];

    /* AK09916 -> ICM20948 Accel/Gyro 坐标，再统一应用模块安装方向映射。 */
    aligned[0] = source[0] * DRV_ICM20948_MAG_TO_IMU_X_SIGN;
    aligned[1] = source[1] * DRV_ICM20948_MAG_TO_IMU_Y_SIGN;
    aligned[2] = source[2] * DRV_ICM20948_MAG_TO_IMU_Z_SIGN;
    ICM20948_MapAxes(aligned, mapped);
}

/* ============================== SPI 基础访问 =============================== */
static BSP_Status_t ICM20948_SPIRead(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t rx;
    BSP_Status_t status;

    if ((data == 0) || (len == 0U)) {
        return BSP_PARAM;
    }
    if (BSP_SPI_IsBusy(DRV_ICM20948_SPI_BUS) != 0U) {
        s_info.busy_skip_count++;
        return BSP_BUSY;
    }

    BSP_GPIO_Write(DRV_ICM20948_CS_GPIO, 0U);

    if (BSP_SPI_TransferByte(DRV_ICM20948_SPI_BUS,
                             (uint8_t)(reg | ICM20948_SPI_READ_BIT),
                             &rx) == 0U) {
        BSP_GPIO_Write(DRV_ICM20948_CS_GPIO, 1U);
        return BSP_TIMEOUT;
    }

    status = BSP_SPI_Transfer(DRV_ICM20948_SPI_BUS, 0, data, len, 0xFFU);
    if (status == BSP_OK) {
        status = BSP_SPI_WaitIdle(DRV_ICM20948_SPI_BUS);
    }
    BSP_GPIO_Write(DRV_ICM20948_CS_GPIO, 1U);
    return status;
}

static BSP_Status_t ICM20948_SPIWrite(uint8_t reg, const uint8_t *data, uint16_t len)
{
    uint8_t rx;
    BSP_Status_t status;

    if ((data == 0) || (len == 0U)) {
        return BSP_PARAM;
    }
    if (BSP_SPI_IsBusy(DRV_ICM20948_SPI_BUS) != 0U) {
        s_info.busy_skip_count++;
        return BSP_BUSY;
    }

    BSP_GPIO_Write(DRV_ICM20948_CS_GPIO, 0U);

    if (BSP_SPI_TransferByte(DRV_ICM20948_SPI_BUS,
                             (uint8_t)(reg & 0x7FU),
                             &rx) == 0U) {
        BSP_GPIO_Write(DRV_ICM20948_CS_GPIO, 1U);
        return BSP_TIMEOUT;
    }

    status = BSP_SPI_Transfer(DRV_ICM20948_SPI_BUS, data, 0, len, 0xFFU);
    if (status == BSP_OK) {
        status = BSP_SPI_WaitIdle(DRV_ICM20948_SPI_BUS);
    }
    BSP_GPIO_Write(DRV_ICM20948_CS_GPIO, 1U);
    return status;
}

static BSP_Status_t ICM20948_SelectBank(uint8_t bank)
{
    uint8_t value;
    BSP_Status_t status;

    if (bank > 3U) {
        return BSP_PARAM;
    }
    if (s_current_bank == bank) {
        return BSP_OK;
    }

    value = (uint8_t)(bank << 4);
    status = ICM20948_SPIWrite(ICM20948_REG_BANK_SEL, &value, 1U);
    if (status == BSP_OK) {
        s_current_bank = bank;
    }
    return status;
}

static BSP_Status_t ICM20948_ReadRegister(uint8_t bank, uint8_t reg, uint8_t *value)
{
    BSP_Status_t status;

    status = ICM20948_SelectBank(bank);
    if (status != BSP_OK) {
        return status;
    }
    return ICM20948_SPIRead(reg, value, 1U);
}

static BSP_Status_t ICM20948_ReadRegisters(uint8_t bank,
                                           uint8_t reg,
                                           uint8_t *data,
                                           uint16_t len)
{
    BSP_Status_t status;

    status = ICM20948_SelectBank(bank);
    if (status != BSP_OK) {
        return status;
    }
    return ICM20948_SPIRead(reg, data, len);
}

static BSP_Status_t ICM20948_WriteRegister(uint8_t bank, uint8_t reg, uint8_t value)
{
    BSP_Status_t status;

    status = ICM20948_SelectBank(bank);
    if (status != BSP_OK) {
        return status;
    }
    return ICM20948_SPIWrite(reg, &value, 1U);
}

/* ============================== 错误与状态 ================================ */
static void ICM20948_SetState(Drv_ICM20948_State_t state)
{
    s_info.state = state;
    s_state_start_ms = BSP_GET_TICK();
}

static void ICM20948_EnterError(BSP_Status_t status)
{
    s_info.last_status = status;
    s_info.error_count++;
    s_info.consecutive_errors++;
    s_info.online = 0U;
    s_info.initialized = 0U;
    s_info.running = 0U;
    s_info.calibrating = 0U;
    s_info.data_valid = 0U;
    s_data.accel_gyro_valid = 0U;
    ICM20948_SetState(DRV_ICM20948_STATE_ERROR_WAIT);
}

static BSP_Status_t ICM20948_HandleInitStatus(BSP_Status_t status)
{
    if (status == BSP_BUSY) {
        return BSP_BUSY;
    }
    if (status != BSP_OK) {
        ICM20948_EnterError(status);
        return status;
    }

    s_info.last_status = BSP_OK;
    s_info.consecutive_errors = 0U;
    return BSP_OK;
}

static BSP_Status_t ICM20948_HandleRunStatus(BSP_Status_t status)
{
    if (status == BSP_BUSY) {
        return BSP_BUSY;
    }
    if (status != BSP_OK) {
        s_info.last_status = status;
        s_info.error_count++;
        s_info.consecutive_errors++;
        if (s_info.consecutive_errors >= DRV_ICM20948_MAX_CONSECUTIVE_ERRORS) {
            ICM20948_EnterError(status);
        }
        return status;
    }

    s_info.last_status = BSP_OK;
    s_info.consecutive_errors = 0U;
    return BSP_OK;
}

/* ============================== AK09916 访问 =============================== */
/*
 * SLV0/SLV1 初始化路径
 * --------------------
 * I2C_SLV4 是一次性事务通道；部分模块/芯片组合会出现 SLV4_CTRL.EN 始终
 * 保持 1、I2C_MST_STATUS.SLV4_DONE 始终为 0。为避免该通道阻塞磁力计，
 * 默认改用与连续采样相同的 SLV0/SLV1 周期调度器：
 *   - SLV0：从 AK09916 连续读取；
 *   - SLV1：向 AK09916 写复位或工作模式命令。
 */

static BSP_Status_t ICM20948_AuxSlv0Disable(void)
{
    BSP_Status_t status;

    status = ICM20948_WriteRegister(3U, ICM20948_B3_I2C_SLV0_CTRL, 0x00U);
    if (status != BSP_OK) return status;
    return ICM20948_ReadRegister(3U,
                                 ICM20948_B3_I2C_SLV0_CTRL,
                                 &s_info.slv0_ctrl_readback);
}

static BSP_Status_t ICM20948_AuxSlv1Disable(void)
{
    BSP_Status_t status;

    status = ICM20948_WriteRegister(3U, ICM20948_B3_I2C_SLV1_CTRL, 0x00U);
    if (status != BSP_OK) return status;
    return ICM20948_ReadRegister(3U,
                                 ICM20948_B3_I2C_SLV1_CTRL,
                                 &s_info.slv1_ctrl_readback);
}

static BSP_Status_t ICM20948_AuxSlv0StartRead(uint8_t reg, uint8_t len)
{
    BSP_Status_t status;

    if ((len == 0U) || (len > 15U)) {
        return BSP_PARAM;
    }

    /* 先关闭旧任务，避免修改地址/寄存器期间仍在执行上一轮访问。 */
    status = ICM20948_WriteRegister(3U, ICM20948_B3_I2C_SLV0_CTRL, 0x00U);
    if (status != BSP_OK) return status;

    status = ICM20948_WriteRegister(3U,
                                    ICM20948_B3_I2C_SLV0_ADDR,
                                    (uint8_t)(ICM20948_I2C_SLV_READ |
                                              AK09916_I2C_ADDR));
    if (status != BSP_OK) return status;
    status = ICM20948_WriteRegister(3U, ICM20948_B3_I2C_SLV0_REG, reg);
    if (status != BSP_OK) return status;
    status = ICM20948_WriteRegister(3U,
                                    ICM20948_B3_I2C_SLV0_CTRL,
                                    (uint8_t)(ICM20948_I2C_SLV_ENABLE | len));
    if (status != BSP_OK) return status;

    status = ICM20948_ReadRegister(3U,
                                   ICM20948_B3_I2C_SLV0_ADDR,
                                   &s_info.slv0_addr_readback);
    if (status != BSP_OK) return status;
    return ICM20948_ReadRegister(3U,
                                 ICM20948_B3_I2C_SLV0_CTRL,
                                 &s_info.slv0_ctrl_readback);
}

static BSP_Status_t ICM20948_AuxSlv1StartWrite(uint8_t reg, uint8_t value)
{
    BSP_Status_t status;

    /* SLV1_DO 必须在使能 SLV1 前写入。 */
    status = ICM20948_WriteRegister(3U, ICM20948_B3_I2C_SLV1_CTRL, 0x00U);
    if (status != BSP_OK) return status;
    status = ICM20948_WriteRegister(3U, ICM20948_B3_I2C_SLV1_DO, value);
    if (status != BSP_OK) return status;
    status = ICM20948_WriteRegister(3U,
                                    ICM20948_B3_I2C_SLV1_ADDR,
                                    AK09916_I2C_ADDR);
    if (status != BSP_OK) return status;
    status = ICM20948_WriteRegister(3U, ICM20948_B3_I2C_SLV1_REG, reg);
    if (status != BSP_OK) return status;
    status = ICM20948_WriteRegister(3U,
                                    ICM20948_B3_I2C_SLV1_CTRL,
                                    (uint8_t)(ICM20948_I2C_SLV_ENABLE | 1U));
    if (status != BSP_OK) return status;

    status = ICM20948_ReadRegister(3U,
                                   ICM20948_B3_I2C_SLV1_ADDR,
                                   &s_info.slv1_addr_readback);
    if (status != BSP_OK) return status;
    return ICM20948_ReadRegister(3U,
                                 ICM20948_B3_I2C_SLV1_CTRL,
                                 &s_info.slv1_ctrl_readback);
}

static BSP_Status_t ICM20948_AuxSlv01ReadStatus(uint8_t nack_mask)
{
    BSP_Status_t status;
    uint8_t value;

    status = ICM20948_ReadRegister(0U, ICM20948_B0_I2C_MST_STATUS, &value);
    if (status != BSP_OK) {
        return status;
    }

    s_info.last_i2c_mst_status = value;
    if ((value & nack_mask) != 0U) {
        return BSP_ERROR;
    }
    return BSP_OK;
}

static BSP_Status_t ICM20948_AuxSlv0ReadData(uint8_t *data, uint8_t len)
{
    if ((data == 0) || (len == 0U)) {
        return BSP_PARAM;
    }

    return ICM20948_ReadRegisters(0U,
                                  ICM20948_B0_EXT_SLV_SENS_DATA_00,
                                  data,
                                  len);
}

/* SLV4 对比诊断路径，宏关闭 SLV0/SLV1 初始化时使用。 */
static BSP_Status_t ICM20948_AuxStartWrite(uint8_t reg, uint8_t value)
{
    BSP_Status_t status;

    status = ICM20948_WriteRegister(3U, ICM20948_B3_I2C_SLV4_ADDR, AK09916_I2C_ADDR);
    if (status != BSP_OK) return status;
    status = ICM20948_WriteRegister(3U, ICM20948_B3_I2C_SLV4_REG, reg);
    if (status != BSP_OK) return status;
    status = ICM20948_WriteRegister(3U, ICM20948_B3_I2C_SLV4_DO, value);
    if (status != BSP_OK) return status;
    status = ICM20948_WriteRegister(3U, ICM20948_B3_I2C_SLV4_CTRL, ICM20948_I2C_SLV_ENABLE);
    if (status != BSP_OK) return status;

    /* 回读地址和控制寄存器，确认 Bank3 写入没有被提前拉高 CS 截断。 */
    status = ICM20948_ReadRegister(3U, ICM20948_B3_I2C_SLV4_ADDR, &s_info.slv4_addr_readback);
    if (status != BSP_OK) return status;
    return ICM20948_ReadRegister(3U, ICM20948_B3_I2C_SLV4_CTRL, &s_info.slv4_ctrl_readback);
}

static BSP_Status_t ICM20948_AuxStartRead(uint8_t reg)
{
    BSP_Status_t status;

    status = ICM20948_WriteRegister(3U,
                                    ICM20948_B3_I2C_SLV4_ADDR,
                                    (uint8_t)(ICM20948_I2C_SLV_READ | AK09916_I2C_ADDR));
    if (status != BSP_OK) return status;
    status = ICM20948_WriteRegister(3U, ICM20948_B3_I2C_SLV4_REG, reg);
    if (status != BSP_OK) return status;
    status = ICM20948_WriteRegister(3U, ICM20948_B3_I2C_SLV4_CTRL, ICM20948_I2C_SLV_ENABLE);
    if (status != BSP_OK) return status;

    status = ICM20948_ReadRegister(3U, ICM20948_B3_I2C_SLV4_ADDR, &s_info.slv4_addr_readback);
    if (status != BSP_OK) return status;
    return ICM20948_ReadRegister(3U, ICM20948_B3_I2C_SLV4_CTRL, &s_info.slv4_ctrl_readback);
}

static BSP_Status_t ICM20948_AuxCheckDone(uint8_t *done)
{
    uint8_t status_reg;
    BSP_Status_t status;

    if (done == 0) {
        return BSP_PARAM;
    }
    *done = 0U;

    status = ICM20948_ReadRegister(0U, ICM20948_B0_I2C_MST_STATUS, &status_reg);
    if (status != BSP_OK) {
        return status;
    }

    /* 保存原始状态，方便串口区分“无完成标志”和“NACK”。 */
    s_info.last_i2c_mst_status = status_reg;

    if ((status_reg & ICM20948_I2C_SLV4_NACK) != 0U) {
        return BSP_ERROR;
    }
    if ((status_reg & ICM20948_I2C_SLV4_DONE) != 0U) {
        *done = 1U;
    }
    return BSP_OK;
}

static BSP_Status_t ICM20948_AuxReadData(uint8_t *value)
{
    return ICM20948_ReadRegister(3U, ICM20948_B3_I2C_SLV4_DI, value);
}

static BSP_Status_t ICM20948_CheckAuxTimeout(void)
{
    if ((BSP_GET_TICK() - s_state_start_ms) > DRV_ICM20948_AUX_TRANSACTION_TIMEOUT_MS) {
        return BSP_TIMEOUT;
    }
    return BSP_OK;
}

/* ============================== 数据处理 ================================== */
static int16_t ICM20948_MakeI16BE(uint8_t high, uint8_t low)
{
    return (int16_t)((((uint16_t)high) << 8) | (uint16_t)low);
}

static int16_t ICM20948_MakeI16LE(uint8_t low, uint8_t high)
{
    return (int16_t)((((uint16_t)high) << 8) | (uint16_t)low);
}

static void ICM20948_ProcessSample(const uint8_t buffer[ICM20948_BURST_DATA_LEN])
{
    float accel_chip[3];
    float gyro_chip[3];
    float mag_chip[3];
    float accel_mapped[3];
    float gyro_mapped[3];
    float mag_mapped[3];
    float accel_corrected[3];
    float gyro_corrected[3];
    float mag_corrected[3];
    float accel_sensitivity;
    float gyro_sensitivity;
    uint8_t mag_new;
    uint8_t mag_overflow;

    s_data.raw.accel[0] = ICM20948_MakeI16BE(buffer[0], buffer[1]);
    s_data.raw.accel[1] = ICM20948_MakeI16BE(buffer[2], buffer[3]);
    s_data.raw.accel[2] = ICM20948_MakeI16BE(buffer[4], buffer[5]);
    s_data.raw.gyro[0] = ICM20948_MakeI16BE(buffer[6], buffer[7]);
    s_data.raw.gyro[1] = ICM20948_MakeI16BE(buffer[8], buffer[9]);
    s_data.raw.gyro[2] = ICM20948_MakeI16BE(buffer[10], buffer[11]);
    s_data.raw.temperature = ICM20948_MakeI16BE(buffer[12], buffer[13]);

    accel_sensitivity = ICM20948_GetAccelSensitivity();
    gyro_sensitivity = ICM20948_GetGyroSensitivity();

    accel_chip[0] = (float)s_data.raw.accel[0] / accel_sensitivity;
    accel_chip[1] = (float)s_data.raw.accel[1] / accel_sensitivity;
    accel_chip[2] = (float)s_data.raw.accel[2] / accel_sensitivity;

    gyro_chip[0] = (float)s_data.raw.gyro[0] / gyro_sensitivity;
    gyro_chip[1] = (float)s_data.raw.gyro[1] / gyro_sensitivity;
    gyro_chip[2] = (float)s_data.raw.gyro[2] / gyro_sensitivity;

    ICM20948_MapAxes(accel_chip, accel_mapped);
    ICM20948_MapAxes(gyro_chip, gyro_mapped);

    accel_corrected[0] = (accel_mapped[0] - DRV_ICM20948_ACCEL_OFFSET_X_G) * DRV_ICM20948_ACCEL_SCALE_X;
    accel_corrected[1] = (accel_mapped[1] - DRV_ICM20948_ACCEL_OFFSET_Y_G) * DRV_ICM20948_ACCEL_SCALE_Y;
    accel_corrected[2] = (accel_mapped[2] - DRV_ICM20948_ACCEL_OFFSET_Z_G) * DRV_ICM20948_ACCEL_SCALE_Z;

    gyro_corrected[0] = (gyro_mapped[0] - DRV_ICM20948_GYRO_OFFSET_X_DPS - s_info.gyro_bias_dps.x) * DRV_ICM20948_GYRO_SCALE_X;
    gyro_corrected[1] = (gyro_mapped[1] - DRV_ICM20948_GYRO_OFFSET_Y_DPS - s_info.gyro_bias_dps.y) * DRV_ICM20948_GYRO_SCALE_Y;
    gyro_corrected[2] = (gyro_mapped[2] - DRV_ICM20948_GYRO_OFFSET_Z_DPS - s_info.gyro_bias_dps.z) * DRV_ICM20948_GYRO_SCALE_Z;

    s_data.accel_g.x = accel_corrected[0];
    s_data.accel_g.y = accel_corrected[1];
    s_data.accel_g.z = accel_corrected[2];
    s_data.gyro_dps.x = gyro_corrected[0];
    s_data.gyro_dps.y = gyro_corrected[1];
    s_data.gyro_dps.z = gyro_corrected[2];

    ICM20948_FilterVector(&s_accel_filter,
                          accel_corrected,
                          DRV_ICM20948_ACCEL_FILTER_ALPHA,
                          &s_data.accel_filtered_g.x);
    ICM20948_FilterVector(&s_gyro_filter,
                          gyro_corrected,
                          DRV_ICM20948_GYRO_FILTER_ALPHA,
                          &s_data.gyro_filtered_dps.x);

    s_data.temperature_c = (((float)s_data.raw.temperature - DRV_ICM20948_TEMP_ROOM_OFFSET_LSB) /
                            DRV_ICM20948_TEMP_SENSITIVITY_LSB_C) + DRV_ICM20948_TEMP_ROOM_C;
    s_data.temperature_filtered_c = ICM20948_FilterScalar(&s_temp_filter,
                                                           s_data.temperature_c,
                                                           DRV_ICM20948_TEMP_FILTER_ALPHA);

    /* 外部传感器数据布局：ST1, HXL, HXH, HYL, HYH, HZL, HZH, TMPS, ST2。 */
    s_data.mag_updated = 0U;
    mag_new = 0U;
    mag_overflow = 0U;
    if (s_mag_stream_enabled != 0U) {
        s_info.mag_st1 = buffer[14];
        s_info.mag_st2 = buffer[22];
        mag_new = (uint8_t)(s_info.mag_st1 & 0x01U);
        mag_overflow = (uint8_t)(s_info.mag_st2 & 0x08U);

        if (mag_new == 0U) {
            s_info.mag_not_ready_count++;
        } else if (mag_overflow != 0U) {
            s_info.mag_overflow_count++;
        }
    }

    if ((s_mag_stream_enabled != 0U) &&
        (mag_new != 0U) && (mag_overflow == 0U)) {
        s_data.raw.mag[0] = ICM20948_MakeI16LE(buffer[15], buffer[16]);
        s_data.raw.mag[1] = ICM20948_MakeI16LE(buffer[17], buffer[18]);
        s_data.raw.mag[2] = ICM20948_MakeI16LE(buffer[19], buffer[20]);

        mag_chip[0] = (float)s_data.raw.mag[0] * 0.15f;
        mag_chip[1] = (float)s_data.raw.mag[1] * 0.15f;
        mag_chip[2] = (float)s_data.raw.mag[2] * 0.15f;
        ICM20948_MapMagAxes(mag_chip, mag_mapped);

        mag_corrected[0] = (mag_mapped[0] - DRV_ICM20948_MAG_OFFSET_X_UT) * DRV_ICM20948_MAG_SCALE_X;
        mag_corrected[1] = (mag_mapped[1] - DRV_ICM20948_MAG_OFFSET_Y_UT) * DRV_ICM20948_MAG_SCALE_Y;
        mag_corrected[2] = (mag_mapped[2] - DRV_ICM20948_MAG_OFFSET_Z_UT) * DRV_ICM20948_MAG_SCALE_Z;

        s_data.mag_uT.x = mag_corrected[0];
        s_data.mag_uT.y = mag_corrected[1];
        s_data.mag_uT.z = mag_corrected[2];
        ICM20948_FilterVector(&s_mag_filter,
                              mag_corrected,
                              DRV_ICM20948_MAG_FILTER_ALPHA,
                              &s_data.mag_filtered_uT.x);

        s_data.mag_valid = 1U;
        s_data.mag_updated = 1U;
        s_info.mag_valid = 1U;
        s_info.mag_valid_count++;
    }

    s_data.timestamp_ms = BSP_GET_TICK();
    s_data.new_data = 1U;
    s_info.sample_count++;
    s_last_sample_ms = s_data.timestamp_ms;
    s_info.online = 1U;
}

static BSP_Status_t ICM20948_ReadFreshSample(void)
{
    uint8_t data_ready;
    uint8_t aux_status;
    uint8_t buffer[ICM20948_BURST_DATA_LEN];
    uint16_t read_len;
    BSP_Status_t status;

    memset(buffer, 0, sizeof(buffer));
    read_len = (s_mag_stream_enabled != 0U) ? ICM20948_BURST_DATA_LEN : 14U;

    if (s_mag_stream_enabled != 0U) {
        status = ICM20948_ReadRegister(0U, ICM20948_B0_I2C_MST_STATUS, &aux_status);
        if (status != BSP_OK) {
            return status;
        }
        s_info.last_i2c_mst_status = aux_status;
        if ((aux_status & (ICM20948_I2C_SLV0_NACK | ICM20948_I2C_SLV1_NACK)) != 0U) {
            s_info.mag_nack_count++;
            s_info.mag_valid = 0U;
            s_data.mag_valid = 0U;
        }
    }

    status = ICM20948_ReadRegister(0U, ICM20948_B0_INT_STATUS_1, &data_ready);
    if (status != BSP_OK) {
        return status;
    }
    if ((data_ready & ICM20948_DATA_READY_BIT) == 0U) {
        return BSP_BUSY;
    }

    status = ICM20948_ReadRegisters(0U,
                                    ICM20948_B0_ACCEL_XOUT_H,
                                    buffer,
                                    read_len);
    if (status != BSP_OK) {
        return status;
    }

    ICM20948_ProcessSample(buffer);
    return BSP_OK;
}

static uint8_t ICM20948_IsCalibrationSampleStable(void)
{
    float gyro_abs_x;
    float gyro_abs_y;
    float gyro_abs_z;
    float gyro_abs_max;
    float accel_norm_sq;
    float accel_min_sq;
    float accel_max_sq;

    gyro_abs_x = ICM20948_AbsF(s_data.gyro_dps.x + s_info.gyro_bias_dps.x);
    gyro_abs_y = ICM20948_AbsF(s_data.gyro_dps.y + s_info.gyro_bias_dps.y);
    gyro_abs_z = ICM20948_AbsF(s_data.gyro_dps.z + s_info.gyro_bias_dps.z);
    gyro_abs_max = gyro_abs_x;
    if (gyro_abs_y > gyro_abs_max) gyro_abs_max = gyro_abs_y;
    if (gyro_abs_z > gyro_abs_max) gyro_abs_max = gyro_abs_z;

    accel_norm_sq = s_data.accel_g.x * s_data.accel_g.x +
                    s_data.accel_g.y * s_data.accel_g.y +
                    s_data.accel_g.z * s_data.accel_g.z;
    accel_min_sq = DRV_ICM20948_GYRO_CAL_ACCEL_MIN_G * DRV_ICM20948_GYRO_CAL_ACCEL_MIN_G;
    accel_max_sq = DRV_ICM20948_GYRO_CAL_ACCEL_MAX_G * DRV_ICM20948_GYRO_CAL_ACCEL_MAX_G;

    s_info.gyro_cal_last_max_abs_dps = gyro_abs_max;
    s_info.gyro_cal_last_accel_norm_g = sqrtf(accel_norm_sq);

    if (gyro_abs_max > DRV_ICM20948_GYRO_CAL_MAX_DPS) {
        return 0U;
    }

    return ((accel_norm_sq >= accel_min_sq) && (accel_norm_sq <= accel_max_sq)) ? 1U : 0U;
}

static void ICM20948_FinishGyroCalibration(void)
{
    /* Never accept a partial or timed-out calibration result. */
    if (s_info.gyro_cal_samples < DRV_ICM20948_GYRO_CAL_MIN_SAMPLES) {
        memset(s_gyro_cal_sum, 0, sizeof(s_gyro_cal_sum));
        s_info.gyro_cal_samples = 0U;
        s_gyro_cal_start_ms = BSP_GET_TICK();
        return;
    }

    s_info.gyro_bias_dps.x = s_gyro_cal_sum[0] / (float)s_info.gyro_cal_samples;
    s_info.gyro_bias_dps.y = s_gyro_cal_sum[1] / (float)s_info.gyro_cal_samples;
    s_info.gyro_bias_dps.z = s_gyro_cal_sum[2] / (float)s_info.gyro_cal_samples;

    s_info.calibrating = 0U;
    s_info.initialized = 1U;
    s_info.running = 1U;
    s_info.data_valid = 0U;
    s_data.accel_gyro_valid = 0U;
    ICM20948_ResetFilters();
    ICM20948_SetState(DRV_ICM20948_STATE_RUN);
}

/* ============================== 配置步骤 ================================== */
static BSP_Status_t ICM20948_ConfigPower(void)
{
    BSP_Status_t status;

    /* SPI 模式下关闭主 I2C 从接口，随后唤醒芯片并开启全部 Accel/Gyro 轴。 */
    status = ICM20948_WriteRegister(0U,
                                    ICM20948_B0_USER_CTRL,
                                    ICM20948_USER_CTRL_I2C_IF_DIS);
    if (status != BSP_OK) return status;
    status = ICM20948_WriteRegister(0U, ICM20948_B0_PWR_MGMT_1, 0x01U);
    if (status != BSP_OK) return status;
    status = ICM20948_WriteRegister(0U, ICM20948_B0_PWR_MGMT_2, 0x00U);
    if (status != BSP_OK) return status;

    /*
     * I2C_MST_CYCLE 必须保持置位，内部辅助 I2C 才会按采样时钟执行
     * SLV0/SLV1 事务。官方 eMD 的低噪声模式同样固定写入该位。
     * 原先写 0x00 会造成地址/控制寄存器回读正确，但 AUX 总线上没有
     * 实际事务，因此 WIA、ST1、ST2 始终读到 0 且不会产生 NACK。
     */
    status = ICM20948_WriteRegister(0U,
                                    ICM20948_B0_LP_CONFIG,
                                    ICM20948_LP_CONFIG_I2C_MST_CYCLE);
    if (status != BSP_OK) return status;

    status = ICM20948_ReadRegister(0U,
                                   ICM20948_B0_LP_CONFIG,
                                   &s_info.lp_config_readback);
    if (status != BSP_OK) return status;

    if ((s_info.lp_config_readback & ICM20948_LP_CONFIG_I2C_MST_CYCLE) == 0U) {
        return BSP_ERROR;
    }
    return BSP_OK;
}

static BSP_Status_t ICM20948_ConfigAccelGyro(void)
{
    BSP_Status_t status;
    uint8_t gyro_config;
    uint8_t accel_config;
    uint16_t accel_div;

    gyro_config = (uint8_t)((DRV_ICM20948_GYRO_DLPF_CFG << 3) |
                            (DRV_ICM20948_GYRO_FS_SEL << 1));
    accel_config = (uint8_t)((DRV_ICM20948_ACCEL_DLPF_CFG << 3) |
                             (DRV_ICM20948_ACCEL_FS_SEL << 1));
#if (DRV_ICM20948_HW_DLPF_ENABLE != 0U)
    gyro_config |= 0x01U;
    accel_config |= 0x01U;
#endif

    status = ICM20948_WriteRegister(2U,
                                    ICM20948_B2_GYRO_SMPLRT_DIV,
                                    (uint8_t)DRV_ICM20948_GYRO_SAMPLE_DIV);
    if (status != BSP_OK) return status;
    status = ICM20948_WriteRegister(2U, ICM20948_B2_GYRO_CONFIG_1, gyro_config);
    if (status != BSP_OK) return status;

    accel_div = (uint16_t)DRV_ICM20948_ACCEL_SAMPLE_DIV;
    status = ICM20948_WriteRegister(2U,
                                    ICM20948_B2_ACCEL_SMPLRT_DIV_1,
                                    (uint8_t)((accel_div >> 8) & 0x0FU));
    if (status != BSP_OK) return status;
    status = ICM20948_WriteRegister(2U,
                                    ICM20948_B2_ACCEL_SMPLRT_DIV_2,
                                    (uint8_t)(accel_div & 0xFFU));
    if (status != BSP_OK) return status;
    return ICM20948_WriteRegister(2U, ICM20948_B2_ACCEL_CONFIG, accel_config);
}

static BSP_Status_t ICM20948_EnableInternalI2CMaster(void)
{
    BSP_Status_t status;

    /* 重新写入时钟配置，兼容内部 Master 复位后的寄存器状态。 */
    status = ICM20948_WriteRegister(3U,
                                    ICM20948_B3_I2C_MST_ODR_CONFIG,
                                    DRV_ICM20948_I2C_MST_ODR_CONFIG);
    if (status != BSP_OK) return status;
    status = ICM20948_WriteRegister(3U,
                                    ICM20948_B3_I2C_MST_CTRL,
                                    DRV_ICM20948_I2C_MST_CTRL_VALUE);
    if (status != BSP_OK) return status;

    status = ICM20948_WriteRegister(0U,
                                    ICM20948_B0_USER_CTRL,
                                    (uint8_t)(ICM20948_USER_CTRL_I2C_IF_DIS |
                                              ICM20948_USER_CTRL_I2C_MST_EN));
    if (status != BSP_OK) return status;

    status = ICM20948_ReadRegister(0U, ICM20948_B0_USER_CTRL, &s_info.user_ctrl_readback);
    if (status != BSP_OK) return status;
    status = ICM20948_ReadRegister(3U, ICM20948_B3_I2C_MST_CTRL, &s_info.i2c_mst_ctrl_readback);
    if (status != BSP_OK) return status;

    if ((s_info.user_ctrl_readback &
         (ICM20948_USER_CTRL_I2C_IF_DIS | ICM20948_USER_CTRL_I2C_MST_EN)) !=
        (ICM20948_USER_CTRL_I2C_IF_DIS | ICM20948_USER_CTRL_I2C_MST_EN)) {
        return BSP_ERROR;
    }
    if ((s_info.i2c_mst_ctrl_readback & 0x1FU) !=
        (DRV_ICM20948_I2C_MST_CTRL_VALUE & 0x1FU)) {
        return BSP_ERROR;
    }

    return BSP_OK;
}

static BSP_Status_t ICM20948_ResetInternalI2CMaster(void)
{
    /*
     * 这里只在 SLV4 事务超时、NACK 或磁力计 ID 异常时调用。
     * I2C_MST_RST 会自动清零；同时暂时关闭 I2C_MST_EN，随后由状态机重启。
     */
    return ICM20948_WriteRegister(0U,
                                  ICM20948_B0_USER_CTRL,
                                  (uint8_t)(ICM20948_USER_CTRL_I2C_IF_DIS |
                                            ICM20948_USER_CTRL_I2C_MST_RST));
}

static BSP_Status_t ICM20948_BeginMagRecovery(BSP_Status_t reason)
{
    s_info.last_status = reason;
    s_info.mag_valid = 0U;
    s_info.mag_st1 = 0U;
    s_info.mag_st2 = 0U;
    s_data.mag_valid = 0U;
    s_data.mag_updated = 0U;

    if (s_info.mag_retry_count < 0xFFU) {
        s_info.mag_retry_count++;
    }

    if (s_info.mag_retry_count >= DRV_ICM20948_MAG_INIT_MAX_RETRIES) {
#if (DRV_ICM20948_MAG_REQUIRED != 0U)
        ICM20948_EnterError(reason);
        return reason;
#else
        /*
         * 磁力计异常不能阻塞加速度计和陀螺仪。继续进入六轴采样，
         * 上层通过 mag_valid=0 明确知道磁场数据不可用。
         */
        s_mag_stream_enabled = 0U;
        s_info.online = 1U;
        s_info.mag_valid = 0U;
        s_data.mag_valid = 0U;
#if (DRV_ICM20948_GYRO_CAL_ENABLE != 0U)
        Drv_ICM20948_StartGyroCalibration();
#else
        s_info.initialized = 1U;
        s_info.running = 1U;
        ICM20948_SetState(DRV_ICM20948_STATE_RUN);
#endif
        return BSP_OK;
#endif
    }

    ICM20948_SetState(DRV_ICM20948_STATE_MAG_RECOVERY_RESET);
    return BSP_BUSY;
}

static BSP_Status_t ICM20948_HandleMagInitStatus(BSP_Status_t status)
{
    if (status == BSP_BUSY) {
        return BSP_BUSY;
    }
    if (status != BSP_OK) {
        return ICM20948_BeginMagRecovery(status);
    }

    s_info.last_status = BSP_OK;
    return BSP_OK;
}

static BSP_Status_t ICM20948_ConfigMagStream(void)
{
    BSP_Status_t status;

    /*
     * 与官方 eMD 的非 DMP 路径一致：
     *   SLV0 每周期读取上一帧 ST1..ST2 共 9 字节；
     *   SLV1 每周期向 CNTL2 写 0x01，触发下一次单次测量。
     * 这样不依赖连续模式在不同模块批次上的保持状态。
     */
    status = ICM20948_AuxSlv0StartRead(AK09916_REG_ST1, 9U);
    if (status != BSP_OK) return status;

    return ICM20948_AuxSlv1StartWrite(AK09916_REG_CNTL2,
                                      DRV_ICM20948_MAG_MODE);
}

/* ============================== 公共接口 ================================== */
void Drv_ICM20948_Init(void)
{
    memset(&s_data, 0, sizeof(s_data));
    memset(&s_info, 0, sizeof(s_info));
    memset(s_gyro_cal_sum, 0, sizeof(s_gyro_cal_sum));
    ICM20948_ResetFilters();

    s_info.enabled = (DRV_ICM20948_ENABLE != 0U) ? 1U : 0U;
    s_info.last_status = BSP_OK;
    s_info.last_i2c_mst_status = 0U;
    s_info.mag_retry_count = 0U;
    s_info.user_ctrl_readback = 0U;
    s_info.lp_config_readback = 0U;
    s_info.i2c_mst_ctrl_readback = 0U;
    s_info.slv4_addr_readback = 0U;
    s_info.slv4_ctrl_readback = 0U;
#if (DRV_ICM20948_MAG_INIT_USE_SLV0_SLV1 != 0U)
    s_info.mag_init_method = 2U;
#else
    s_info.mag_init_method = 1U;
#endif
    s_info.slv0_addr_readback = 0U;
    s_info.slv0_ctrl_readback = 0U;
    s_info.slv1_addr_readback = 0U;
    s_info.slv1_ctrl_readback = 0U;
    s_mag_stream_enabled = 0U;
    s_current_bank = ICM20948_INVALID_BANK;
    s_reinit_requested = 0U;
    s_last_poll_ms = BSP_GET_TICK();
    s_last_sample_ms = s_last_poll_ms;

    /* CS 上电必须保持高电平，防止与 LCD 共享 SPI2 时误选中。 */
    BSP_GPIO_Write(DRV_ICM20948_CS_GPIO, 1U);

#if (DRV_ICM20948_ENABLE != 0U)
    ICM20948_SetState(DRV_ICM20948_STATE_POWER_WAIT);
#else
    ICM20948_SetState(DRV_ICM20948_STATE_DISABLED);
#endif
}

void Drv_ICM20948_StartGyroCalibration(void)
{
#if (DRV_ICM20948_ENABLE != 0U)
    memset(s_gyro_cal_sum, 0, sizeof(s_gyro_cal_sum));
    s_info.gyro_cal_samples = 0U;
    s_info.gyro_cal_last_max_abs_dps = 0.0f;
    s_info.gyro_cal_last_accel_norm_g = 0.0f;
    s_info.gyro_cal_reject_count = 0U;
    s_info.gyro_bias_dps.x = 0.0f;
    s_info.gyro_bias_dps.y = 0.0f;
    s_info.gyro_bias_dps.z = 0.0f;
    s_info.initialized = 0U;
    s_info.running = 0U;
    s_info.calibrating = 1U;
    s_info.data_valid = 0U;
    s_data.accel_gyro_valid = 0U;
    s_gyro_cal_start_ms = BSP_GET_TICK();
    ICM20948_ResetFilters();
    ICM20948_SetState(DRV_ICM20948_STATE_GYRO_CALIBRATION);
#endif
}

BSP_Status_t Drv_ICM20948_Update(void)
{
#if (DRV_ICM20948_ENABLE == 0U)
    return BSP_ERROR;
#else
    BSP_Status_t status;
#if (DRV_ICM20948_MAG_INIT_USE_SLV0_SLV1 == 0U)
    uint8_t done;
#endif
    uint32_t now;

    now = BSP_GET_TICK();

    if (s_reinit_requested != 0U) {
        s_reinit_requested = 0U;
        s_info.reinit_count++;
        s_info.online = 0U;
        s_info.initialized = 0U;
        s_info.running = 0U;
        s_info.data_valid = 0U;
        s_info.mag_valid = 0U;
        s_info.mag_retry_count = 0U;
        s_info.last_i2c_mst_status = 0U;
        s_info.user_ctrl_readback = 0U;
        s_info.lp_config_readback = 0U;
        s_info.i2c_mst_ctrl_readback = 0U;
        s_info.mag_wia1 = 0U;
        s_info.mag_wia2 = 0U;
        s_info.mag_st1 = 0U;
        s_info.mag_st2 = 0U;
        s_info.slv4_addr_readback = 0U;
        s_info.slv4_ctrl_readback = 0U;
#if (DRV_ICM20948_MAG_INIT_USE_SLV0_SLV1 != 0U)
        s_info.mag_init_method = 2U;
#else
        s_info.mag_init_method = 1U;
#endif
        s_info.slv0_addr_readback = 0U;
        s_info.slv0_ctrl_readback = 0U;
        s_info.slv1_addr_readback = 0U;
        s_info.slv1_ctrl_readback = 0U;
        s_mag_stream_enabled = 0U;
        s_data.accel_gyro_valid = 0U;
        s_data.mag_valid = 0U;
        s_data.mag_updated = 0U;
        s_current_bank = ICM20948_INVALID_BANK;
        ICM20948_SetState(DRV_ICM20948_STATE_POWER_WAIT);
    }

    if ((s_info.initialized != 0U) &&
        ((now - s_last_sample_ms) > DRV_ICM20948_ONLINE_TIMEOUT_MS)) {
        s_info.online = 0U;
        s_info.data_valid = 0U;
        s_data.accel_gyro_valid = 0U;
        ICM20948_EnterError(BSP_TIMEOUT);
        return BSP_TIMEOUT;
    }

    switch (s_info.state) {
        case DRV_ICM20948_STATE_POWER_WAIT:
            if ((now - s_state_start_ms) < DRV_ICM20948_POWER_ON_WAIT_MS) {
                return BSP_BUSY;
            }
            ICM20948_SetState(DRV_ICM20948_STATE_WHO_AM_I);
            return BSP_OK;

        case DRV_ICM20948_STATE_WHO_AM_I:
            status = ICM20948_ReadRegister(0U, ICM20948_B0_WHO_AM_I, &s_info.who_am_i);
            if (ICM20948_HandleInitStatus(status) != BSP_OK) return status;
            if (s_info.who_am_i != DRV_ICM20948_WHO_AM_I_EXPECTED) {
                ICM20948_EnterError(BSP_ERROR);
                return BSP_ERROR;
            }
            ICM20948_SetState(DRV_ICM20948_STATE_RESET);
            return BSP_OK;

        case DRV_ICM20948_STATE_RESET:
            status = ICM20948_WriteRegister(0U, ICM20948_B0_PWR_MGMT_1, ICM20948_PWR_DEVICE_RESET);
            if (ICM20948_HandleInitStatus(status) != BSP_OK) return status;
            s_current_bank = ICM20948_INVALID_BANK;
            ICM20948_SetState(DRV_ICM20948_STATE_RESET_WAIT);
            return BSP_OK;

        case DRV_ICM20948_STATE_RESET_WAIT:
            if ((now - s_state_start_ms) < DRV_ICM20948_RESET_WAIT_MS) {
                return BSP_BUSY;
            }
            ICM20948_SetState(DRV_ICM20948_STATE_CONFIG_POWER);
            return BSP_OK;

        case DRV_ICM20948_STATE_CONFIG_POWER:
            status = ICM20948_ConfigPower();
            if (ICM20948_HandleInitStatus(status) != BSP_OK) return status;
            ICM20948_SetState(DRV_ICM20948_STATE_WAKE_WAIT);
            return BSP_OK;

        case DRV_ICM20948_STATE_WAKE_WAIT:
            if ((now - s_state_start_ms) < DRV_ICM20948_WAKE_WAIT_MS) {
                return BSP_BUSY;
            }
            ICM20948_SetState(DRV_ICM20948_STATE_CONFIG_ACCEL_GYRO);
            return BSP_OK;

        case DRV_ICM20948_STATE_CONFIG_ACCEL_GYRO:
            status = ICM20948_ConfigAccelGyro();
            if (ICM20948_HandleInitStatus(status) != BSP_OK) return status;
            ICM20948_SetState(DRV_ICM20948_STATE_CONFIG_I2C_MASTER);
            return BSP_OK;

        case DRV_ICM20948_STATE_CONFIG_I2C_MASTER:
#if (DRV_ICM20948_MAG_ENABLE != 0U)
            status = ICM20948_EnableInternalI2CMaster();
            if (ICM20948_HandleInitStatus(status) != BSP_OK) return status;
            ICM20948_SetState(DRV_ICM20948_STATE_I2C_MASTER_START_WAIT);
#else
            s_info.online = 1U;
            s_mag_stream_enabled = 0U;
#if (DRV_ICM20948_GYRO_CAL_ENABLE != 0U)
            Drv_ICM20948_StartGyroCalibration();
#else
            s_info.initialized = 1U;
            s_info.running = 1U;
            ICM20948_SetState(DRV_ICM20948_STATE_RUN);
#endif
#endif
            return BSP_OK;

        case DRV_ICM20948_STATE_I2C_MASTER_START_WAIT:
            if ((now - s_state_start_ms) < DRV_ICM20948_I2C_MASTER_START_WAIT_MS) {
                return BSP_BUSY;
            }
            ICM20948_SetState(DRV_ICM20948_STATE_MAG_RESET_START);
            return BSP_OK;

        case DRV_ICM20948_STATE_MAG_RESET_START:
#if (DRV_ICM20948_MAG_INIT_USE_SLV0_SLV1 != 0U)
            /*
             * 通过 SLV1 周期写通道发送 AK09916 软件复位命令。
             * SLV1 会随内部采样时钟执行，因此进入 WAIT 后再等待至少两个周期。
             */
            status = ICM20948_AuxSlv1StartWrite(AK09916_REG_CNTL3, 0x01U);
#else
            status = ICM20948_AuxStartWrite(AK09916_REG_CNTL3, 0x01U);
#endif
            status = ICM20948_HandleMagInitStatus(status);
            if (status != BSP_OK) return status;
            ICM20948_SetState(DRV_ICM20948_STATE_MAG_RESET_WAIT);
            return BSP_OK;

        case DRV_ICM20948_STATE_MAG_RESET_WAIT:
#if (DRV_ICM20948_MAG_INIT_USE_SLV0_SLV1 != 0U)
            if ((now - s_state_start_ms) < DRV_ICM20948_MAG_SLV01_WAIT_MS) {
                return BSP_BUSY;
            }
            status = ICM20948_AuxSlv01ReadStatus(ICM20948_I2C_SLV1_NACK);
            if (status != BSP_OK) {
                return ICM20948_BeginMagRecovery(status);
            }
            status = ICM20948_AuxSlv1Disable();
            if (status != BSP_OK) {
                return ICM20948_HandleMagInitStatus(status);
            }
            ICM20948_SetState(DRV_ICM20948_STATE_MAG_RESET_DELAY);
            return BSP_OK;
#else
            status = ICM20948_AuxCheckDone(&done);
            if (status != BSP_OK) {
                return ICM20948_HandleMagInitStatus(status);
            }
            if (done == 0U) {
                status = ICM20948_CheckAuxTimeout();
                if (status != BSP_OK) {
                    return ICM20948_BeginMagRecovery(status);
                }
                return BSP_BUSY;
            }
            ICM20948_SetState(DRV_ICM20948_STATE_MAG_RESET_DELAY);
            return BSP_OK;
#endif

        case DRV_ICM20948_STATE_MAG_RESET_DELAY:
            if ((now - s_state_start_ms) < DRV_ICM20948_MAG_RESET_WAIT_MS) {
                return BSP_BUSY;
            }
            ICM20948_SetState(DRV_ICM20948_STATE_MAG_WIA1_START);
            return BSP_OK;

        case DRV_ICM20948_STATE_MAG_WIA1_START:
#if (DRV_ICM20948_MAG_INIT_USE_SLV0_SLV1 != 0U)
            /* WIA1/WIA2 地址连续，一次事务同时读取，便于兼容官方驱动和数据手册。 */
            status = ICM20948_AuxSlv0StartRead(AK09916_REG_WIA1, 2U);
#else
            status = ICM20948_AuxStartRead(AK09916_REG_WIA1);
#endif
            status = ICM20948_HandleMagInitStatus(status);
            if (status != BSP_OK) return status;
            ICM20948_SetState(DRV_ICM20948_STATE_MAG_WIA1_WAIT);
            return BSP_OK;

        case DRV_ICM20948_STATE_MAG_WIA1_WAIT:
#if (DRV_ICM20948_MAG_INIT_USE_SLV0_SLV1 != 0U)
        {
            uint8_t mag_id[2];

            if ((now - s_state_start_ms) < DRV_ICM20948_MAG_SLV01_WAIT_MS) {
                return BSP_BUSY;
            }

            status = ICM20948_AuxSlv01ReadStatus(ICM20948_I2C_SLV0_NACK);
            if (status != BSP_OK) {
                return ICM20948_BeginMagRecovery(status);
            }

            status = ICM20948_AuxSlv0ReadData(mag_id, 2U);
            if (status != BSP_OK) {
                return ICM20948_HandleMagInitStatus(status);
            }

            s_info.mag_wia1 = mag_id[0];
            s_info.mag_wia2 = mag_id[1];

            status = ICM20948_AuxSlv0Disable();
            if (status != BSP_OK) {
                return ICM20948_HandleMagInitStatus(status);
            }

            if (ICM20948_IsMagIdentityValid() == 0U) {
                return ICM20948_BeginMagRecovery(BSP_ERROR);
            }

            ICM20948_SetState(DRV_ICM20948_STATE_MAG_MODE_START);
            return BSP_OK;
        }
#else
            status = ICM20948_AuxCheckDone(&done);
            if (status != BSP_OK) {
                return ICM20948_HandleMagInitStatus(status);
            }
            if (done == 0U) {
                status = ICM20948_CheckAuxTimeout();
                if (status != BSP_OK) {
                    return ICM20948_BeginMagRecovery(status);
                }
                return BSP_BUSY;
            }
            status = ICM20948_AuxReadData(&s_info.mag_wia1);
            if (status != BSP_OK) {
                return ICM20948_HandleMagInitStatus(status);
            }
            ICM20948_SetState(DRV_ICM20948_STATE_MAG_WIA2_START);
            return BSP_OK;
#endif

        case DRV_ICM20948_STATE_MAG_WIA2_START:
#if (DRV_ICM20948_MAG_INIT_USE_SLV0_SLV1 != 0U)
            ICM20948_SetState(DRV_ICM20948_STATE_MAG_MODE_START);
            return BSP_OK;
#else
            status = ICM20948_AuxStartRead(AK09916_REG_WIA2);
            status = ICM20948_HandleMagInitStatus(status);
            if (status != BSP_OK) return status;
            ICM20948_SetState(DRV_ICM20948_STATE_MAG_WIA2_WAIT);
            return BSP_OK;
#endif

        case DRV_ICM20948_STATE_MAG_WIA2_WAIT:
#if (DRV_ICM20948_MAG_INIT_USE_SLV0_SLV1 != 0U)
            ICM20948_SetState(DRV_ICM20948_STATE_MAG_MODE_START);
            return BSP_OK;
#else
            status = ICM20948_AuxCheckDone(&done);
            if (status != BSP_OK) {
                return ICM20948_HandleMagInitStatus(status);
            }
            if (done == 0U) {
                status = ICM20948_CheckAuxTimeout();
                if (status != BSP_OK) {
                    return ICM20948_BeginMagRecovery(status);
                }
                return BSP_BUSY;
            }
            status = ICM20948_AuxReadData(&s_info.mag_wia2);
            if (status != BSP_OK) {
                return ICM20948_HandleMagInitStatus(status);
            }
            if (ICM20948_IsMagIdentityValid() == 0U) {
                return ICM20948_BeginMagRecovery(BSP_ERROR);
            }
            ICM20948_SetState(DRV_ICM20948_STATE_MAG_MODE_START);
            return BSP_OK;
#endif

        case DRV_ICM20948_STATE_MAG_MODE_START:
#if (DRV_ICM20948_MAG_INIT_USE_SLV0_SLV1 != 0U)
            status = ICM20948_AuxSlv1StartWrite(AK09916_REG_CNTL2,
                                                DRV_ICM20948_MAG_MODE);
#else
            status = ICM20948_AuxStartWrite(AK09916_REG_CNTL2,
                                            DRV_ICM20948_MAG_MODE);
#endif
            status = ICM20948_HandleMagInitStatus(status);
            if (status != BSP_OK) return status;
            ICM20948_SetState(DRV_ICM20948_STATE_MAG_MODE_WAIT);
            return BSP_OK;

        case DRV_ICM20948_STATE_MAG_MODE_WAIT:
#if (DRV_ICM20948_MAG_INIT_USE_SLV0_SLV1 != 0U)
            if ((now - s_state_start_ms) < DRV_ICM20948_MAG_SLV01_WAIT_MS) {
                return BSP_BUSY;
            }
            status = ICM20948_AuxSlv01ReadStatus(ICM20948_I2C_SLV1_NACK);
            if (status != BSP_OK) {
                return ICM20948_BeginMagRecovery(status);
            }
            status = ICM20948_AuxSlv1Disable();
            if (status != BSP_OK) {
                return ICM20948_HandleMagInitStatus(status);
            }
            ICM20948_SetState(DRV_ICM20948_STATE_CONFIG_MAG_STREAM);
            return BSP_OK;
#else
            status = ICM20948_AuxCheckDone(&done);
            if (status != BSP_OK) {
                return ICM20948_HandleMagInitStatus(status);
            }
            if (done == 0U) {
                status = ICM20948_CheckAuxTimeout();
                if (status != BSP_OK) {
                    return ICM20948_BeginMagRecovery(status);
                }
                return BSP_BUSY;
            }
            ICM20948_SetState(DRV_ICM20948_STATE_CONFIG_MAG_STREAM);
            return BSP_OK;
#endif

        case DRV_ICM20948_STATE_CONFIG_MAG_STREAM:
            status = ICM20948_ConfigMagStream();
            if (status != BSP_OK) {
                return ICM20948_HandleMagInitStatus(status);
            }
            s_mag_stream_enabled = 1U;
            s_info.online = 1U;
#if (DRV_ICM20948_GYRO_CAL_ENABLE != 0U)
            Drv_ICM20948_StartGyroCalibration();
#else
            s_info.initialized = 1U;
            s_info.running = 1U;
            ICM20948_SetState(DRV_ICM20948_STATE_RUN);
#endif
            return BSP_OK;

        case DRV_ICM20948_STATE_MAG_RECOVERY_RESET:
            status = ICM20948_ResetInternalI2CMaster();
            if (status == BSP_BUSY) return BSP_BUSY;
            if (status != BSP_OK) {
                ICM20948_EnterError(status);
                return status;
            }
            ICM20948_SetState(DRV_ICM20948_STATE_MAG_RECOVERY_RESET_WAIT);
            return BSP_OK;

        case DRV_ICM20948_STATE_MAG_RECOVERY_RESET_WAIT:
            if ((now - s_state_start_ms) < DRV_ICM20948_I2C_MASTER_RESET_WAIT_MS) {
                return BSP_BUSY;
            }
            ICM20948_SetState(DRV_ICM20948_STATE_MAG_RECOVERY_ENABLE);
            return BSP_OK;

        case DRV_ICM20948_STATE_MAG_RECOVERY_ENABLE:
            status = ICM20948_EnableInternalI2CMaster();
            if (status == BSP_BUSY) return BSP_BUSY;
            if (status != BSP_OK) {
                ICM20948_EnterError(status);
                return status;
            }
            ICM20948_SetState(DRV_ICM20948_STATE_MAG_RECOVERY_ENABLE_WAIT);
            return BSP_OK;

        case DRV_ICM20948_STATE_MAG_RECOVERY_ENABLE_WAIT:
            if ((now - s_state_start_ms) < DRV_ICM20948_I2C_MASTER_START_WAIT_MS) {
                return BSP_BUSY;
            }
            /* 与 SparkFun 的恢复思路一致：Master 恢复后直接重读 WIA。 */
            ICM20948_SetState(DRV_ICM20948_STATE_MAG_WIA1_START);
            return BSP_OK;

        case DRV_ICM20948_STATE_GYRO_CALIBRATION:
            if ((now - s_last_poll_ms) < DRV_ICM20948_UPDATE_PERIOD_MS) {
                return BSP_BUSY;
            }
            s_last_poll_ms = now;
            status = ICM20948_ReadFreshSample();
            if (status == BSP_BUSY) {
                if ((now - s_gyro_cal_start_ms) >= DRV_ICM20948_GYRO_CAL_TIMEOUT_MS) {
                    memset(s_gyro_cal_sum, 0, sizeof(s_gyro_cal_sum));
                    s_info.gyro_cal_samples = 0U;
                    s_gyro_cal_start_ms = now;
                }
                return BSP_BUSY;
            }
            if (ICM20948_HandleRunStatus(status) != BSP_OK) return status;

            if (ICM20948_IsCalibrationSampleStable() != 0U) {
                /* 当前 gyro_dps 尚未减去运行时零偏，因此可直接累计。 */
                s_gyro_cal_sum[0] += s_data.gyro_dps.x;
                s_gyro_cal_sum[1] += s_data.gyro_dps.y;
                s_gyro_cal_sum[2] += s_data.gyro_dps.z;
                s_info.gyro_cal_samples++;
            } else {
                s_info.gyro_cal_reject_count++;
            }

            if (s_info.gyro_cal_samples >= DRV_ICM20948_GYRO_CAL_SAMPLE_COUNT) {
                ICM20948_FinishGyroCalibration();
            } else if ((now - s_gyro_cal_start_ms) >= DRV_ICM20948_GYRO_CAL_TIMEOUT_MS) {
                memset(s_gyro_cal_sum, 0, sizeof(s_gyro_cal_sum));
                s_info.gyro_cal_samples = 0U;
                s_gyro_cal_start_ms = now;
            }
            return BSP_OK;

        case DRV_ICM20948_STATE_RUN:
            if ((now - s_last_poll_ms) < DRV_ICM20948_UPDATE_PERIOD_MS) {
                return BSP_BUSY;
            }
            s_last_poll_ms = now;
            status = ICM20948_ReadFreshSample();
            if (status == BSP_BUSY) {
                return BSP_BUSY;
            }
            if (ICM20948_HandleRunStatus(status) != BSP_OK) return status;

            s_data.accel_gyro_valid = 1U;
            s_info.data_valid = 1U;
            s_info.valid_count++;
            return BSP_OK;

        case DRV_ICM20948_STATE_ERROR_WAIT:
            if ((now - s_state_start_ms) < DRV_ICM20948_ERROR_RETRY_MS) {
                return BSP_BUSY;
            }
            s_info.reinit_count++;
            s_info.mag_retry_count = 0U;
            s_info.last_i2c_mst_status = 0U;
            s_current_bank = ICM20948_INVALID_BANK;
            ICM20948_SetState(DRV_ICM20948_STATE_POWER_WAIT);
            return BSP_OK;

        case DRV_ICM20948_STATE_DISABLED:
        default:
            return BSP_ERROR;
    }
#endif
}

BSP_Status_t Drv_ICM20948_GetData(Drv_ICM20948_Data_t *data)
{
    uint32_t primask;

    if (data == 0) {
        return BSP_PARAM;
    }
    if ((s_info.online == 0U) || (s_info.data_valid == 0U) ||
        (s_data.accel_gyro_valid == 0U)) {
        return BSP_ERROR;
    }

    primask = BSP_EnterCritical();
    memcpy(data, &s_data, sizeof(*data));
    BSP_ExitCritical(primask);
    return BSP_OK;
}

BSP_Status_t Drv_ICM20948_GetInfo(Drv_ICM20948_Info_t *info)
{
    uint32_t primask;

    if (info == 0) {
        return BSP_PARAM;
    }
    primask = BSP_EnterCritical();
    memcpy(info, &s_info, sizeof(*info));
    BSP_ExitCritical(primask);
    return BSP_OK;
}

uint8_t Drv_ICM20948_IsOnline(void)
{
    return s_info.online;
}

uint8_t Drv_ICM20948_HasNewData(void)
{
    return s_data.new_data;
}

void Drv_ICM20948_ClearNewData(void)
{
    s_data.new_data = 0U;
}

void Drv_ICM20948_RequestReinit(void)
{
    s_reinit_requested = 1U;
}
