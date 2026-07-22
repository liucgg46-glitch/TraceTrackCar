#include "drv_gray_mcu_i2c.h"
#include <string.h>

typedef enum {
    GRAY_MCU_STATE_IDLE = 0,
    GRAY_MCU_STATE_BUSY,
    GRAY_MCU_STATE_DONE
} GrayMcu_State_t;

typedef enum {
    GRAY_MCU_OP_NONE = 0,
    GRAY_MCU_OP_PING,
    GRAY_MCU_OP_FIRMWARE,
    GRAY_MCU_OP_SET_CHANNEL,
    GRAY_MCU_OP_SET_NORMALIZE,
    GRAY_MCU_OP_ANALOG_SELECT,
    GRAY_MCU_OP_ANALOG_READ,
    GRAY_MCU_OP_DIGITAL,
    GRAY_MCU_OP_ERROR,
    GRAY_MCU_OP_REBOOT
} GrayMcu_Op_t;

typedef enum {
    GRAY_MCU_INIT_PING = 0,
    GRAY_MCU_INIT_FIRMWARE,
    GRAY_MCU_INIT_SET_CHANNEL,
    GRAY_MCU_INIT_SET_NORMALIZE,
    GRAY_MCU_INIT_SELECT_ANALOG,
    GRAY_MCU_INIT_RUN
} GrayMcu_InitStep_t;

static Drv_GrayMcu_Info_t s_gray;
static uint8_t s_tx_buf[2];
static uint8_t s_rx_buf[DRV_GRAY_MCU_CHANNEL_NUM];
static uint8_t s_rx_len;
static uint8_t s_write_value;

static volatile GrayMcu_State_t s_state = GRAY_MCU_STATE_IDLE;
static volatile int s_async_result;
static GrayMcu_Op_t s_op = GRAY_MCU_OP_NONE;
static GrayMcu_InitStep_t s_init_step = GRAY_MCU_INIT_PING;
static uint32_t s_next_action_ms;
static uint8_t s_runtime_fail_count;
static uint8_t s_locked_ping_fail_count;

#if DRV_GRAY_MCU_AUTO_ADDR_SCAN
static const uint8_t s_addr_list[] = { 0x4CU, 0x4DU, 0x4EU, 0x4FU };
static uint8_t s_addr_index;
#endif

static uint8_t GrayMcu_TimeReached(uint32_t now, uint32_t target)
{
    return ((int32_t)(now - target) >= 0) ? 1U : 0U;
}

static uint8_t GrayMcu_MapIndex(uint8_t channel_index)
{
#if DRV_GRAY_MCU_INDEX_REVERSE
    return (uint8_t)(DRV_GRAY_MCU_CHANNEL_NUM - 1U - channel_index);
#else
    return channel_index;
#endif
}

static uint8_t GrayMcu_PopCount8(uint8_t value)
{
    uint8_t count = 0U;

    while (value != 0U) {
        count = (uint8_t)(count + (value & 0x01U));
        value >>= 1U;
    }

    return count;
}

static uint16_t GrayMcu_ExpandAnalog(uint8_t value)
{
#if DRV_GRAY_MCU_SCALE_TO_12BIT
    return (uint16_t)(((uint16_t)value << 4U) | ((uint16_t)value >> 4U));
#else
    return (uint16_t)value;
#endif
}

static uint16_t GrayMcu_Filter(uint16_t old_value, uint16_t new_value)
{
#if DRV_GRAY_MCU_FILTER_SHIFT == 0
    (void)old_value;
    return new_value;
#else
    int32_t diff = (int32_t)new_value - (int32_t)old_value;
    return (uint16_t)((int32_t)old_value + (diff >> DRV_GRAY_MCU_FILTER_SHIFT));
#endif
}

static uint8_t GrayMcu_NormalizeSupported(uint8_t firmware)
{
    uint8_t major = (uint8_t)(firmware >> 4U);
    uint8_t minor = (uint8_t)(firmware & 0x0FU);

    if (major > 3U) {
        return 1U;
    }

    return (uint8_t)((major == 3U) && (minor >= 6U));
}

static BSP_Status_t GrayMcu_ResultToStatus(int result)
{
    if (result == 0) {
        return BSP_OK;
    }
    if (result == -2) {
        return BSP_TIMEOUT;
    }
    return BSP_ERROR;
}

static void GrayMcu_Callback(I2C_Bus_t bus, int result)
{
    (void)bus;
    s_async_result = result;
    s_state = GRAY_MCU_STATE_DONE;
}

static void GrayMcu_SetIdle(void)
{
    s_state = GRAY_MCU_STATE_IDLE;
    s_op = GRAY_MCU_OP_NONE;
    s_gray.current_phase = DRV_GRAY_MCU_PHASE_IDLE;
}

static void GrayMcu_RecordStartFailure(BSP_Status_t status,
                                       Drv_GrayMcu_Phase_t phase)
{
    s_gray.last_status = status;
    s_gray.last_i2c_result = -1;
    s_gray.last_phase = (uint8_t)phase;
    s_gray.done_phase = (uint8_t)phase;

    /* BSP_BUSY 只表示总线正在被 OLED 等设备使用，不算传感器错误。 */
    if (status != BSP_BUSY) {
        s_gray.error_count++;
        if (s_gray.consecutive_error_count < 0xFFU) {
            s_gray.consecutive_error_count++;
        }
        s_next_action_ms = BSP_GET_TICK() + DRV_GRAY_MCU_INIT_RETRY_MS;
    }
}

static BSP_Status_t GrayMcu_StartCommandRead(GrayMcu_Op_t op,
                                             Drv_GrayMcu_Phase_t phase,
                                             uint8_t command,
                                             uint8_t rx_len)
{
    BSP_Status_t ret;

    if ((rx_len == 0U) || (rx_len > DRV_GRAY_MCU_CHANNEL_NUM)) {
        return BSP_PARAM;
    }
    if (s_state != GRAY_MCU_STATE_IDLE) {
        return BSP_BUSY;
    }
    if (BSP_I2C_IsBusy(DRV_GRAY_MCU_I2C_BUS) != 0U) {
        return BSP_BUSY;
    }

    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    s_tx_buf[0] = command;
    s_rx_len = rx_len;
    s_write_value = 0U;
    s_op = op;
    s_async_result = 0;
    s_state = GRAY_MCU_STATE_BUSY;

    s_gray.current_phase = (uint8_t)phase;
    s_gray.last_phase = (uint8_t)phase;
    s_gray.done_phase = DRV_GRAY_MCU_PHASE_IDLE;
    s_gray.last_op = (uint8_t)op;
    s_gray.last_reg = command;
    s_gray.last_rx_len = rx_len;
    s_gray.last_status = BSP_BUSY;

    ret = BSP_I2C_MasterWriteRead_DMA_Async(DRV_GRAY_MCU_I2C_BUS,
                                            s_gray.active_addr,
                                            s_tx_buf,
                                            1U,
                                            s_rx_buf,
                                            rx_len,
                                            GrayMcu_Callback);
    if (ret != BSP_OK) {
        GrayMcu_SetIdle();
        GrayMcu_RecordStartFailure(ret, phase);
        return ret;
    }

    return BSP_BUSY;
}

static BSP_Status_t GrayMcu_StartReadOnly(GrayMcu_Op_t op,
                                          Drv_GrayMcu_Phase_t phase,
                                          uint8_t diagnostic_command,
                                          uint8_t rx_len)
{
    BSP_Status_t ret;

    if ((rx_len == 0U) || (rx_len > DRV_GRAY_MCU_CHANNEL_NUM)) {
        return BSP_PARAM;
    }
    if (s_state != GRAY_MCU_STATE_IDLE) {
        return BSP_BUSY;
    }
    if (BSP_I2C_IsBusy(DRV_GRAY_MCU_I2C_BUS) != 0U) {
        return BSP_BUSY;
    }

    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    s_rx_len = rx_len;
    s_write_value = 0U;
    s_op = op;
    s_async_result = 0;
    s_state = GRAY_MCU_STATE_BUSY;

    s_gray.current_phase = (uint8_t)phase;
    s_gray.last_phase = (uint8_t)phase;
    s_gray.done_phase = DRV_GRAY_MCU_PHASE_IDLE;
    s_gray.last_op = (uint8_t)op;
    s_gray.last_reg = diagnostic_command;
    s_gray.last_rx_len = rx_len;
    s_gray.last_status = BSP_BUSY;

    ret = BSP_I2C_MasterRead_DMA_Async(DRV_GRAY_MCU_I2C_BUS,
                                       s_gray.active_addr,
                                       s_rx_buf,
                                       rx_len,
                                       GrayMcu_Callback);
    if (ret != BSP_OK) {
        GrayMcu_SetIdle();
        GrayMcu_RecordStartFailure(ret, phase);
        return ret;
    }

    return BSP_BUSY;
}

static BSP_Status_t GrayMcu_StartWrite2(GrayMcu_Op_t op,
                                        Drv_GrayMcu_Phase_t phase,
                                        uint8_t command,
                                        uint8_t value)
{
    BSP_Status_t ret;

    if (s_state != GRAY_MCU_STATE_IDLE) {
        return BSP_BUSY;
    }
    if (BSP_I2C_IsBusy(DRV_GRAY_MCU_I2C_BUS) != 0U) {
        return BSP_BUSY;
    }

    s_tx_buf[0] = command;
    s_tx_buf[1] = value;
    s_rx_len = 0U;
    s_write_value = value;
    s_op = op;
    s_async_result = 0;
    s_state = GRAY_MCU_STATE_BUSY;

    s_gray.current_phase = (uint8_t)phase;
    s_gray.last_phase = (uint8_t)phase;
    s_gray.done_phase = DRV_GRAY_MCU_PHASE_IDLE;
    s_gray.last_op = (uint8_t)op;
    s_gray.last_reg = command;
    s_gray.last_rx_len = 0U;
    s_gray.last_status = BSP_BUSY;

    ret = BSP_I2C_MasterWrite_DMA_Async(DRV_GRAY_MCU_I2C_BUS,
                                        s_gray.active_addr,
                                        s_tx_buf,
                                        2U,
                                        GrayMcu_Callback);
    if (ret != BSP_OK) {
        GrayMcu_SetIdle();
        GrayMcu_RecordStartFailure(ret, phase);
        return ret;
    }

    return BSP_BUSY;
}

static BSP_Status_t GrayMcu_StartWrite1(GrayMcu_Op_t op,
                                        Drv_GrayMcu_Phase_t phase,
                                        uint8_t command)
{
    BSP_Status_t ret;

    if (s_state != GRAY_MCU_STATE_IDLE) {
        return BSP_BUSY;
    }
    if (BSP_I2C_IsBusy(DRV_GRAY_MCU_I2C_BUS) != 0U) {
        return BSP_BUSY;
    }

    s_tx_buf[0] = command;
    s_rx_len = 0U;
    s_write_value = 0U;
    s_op = op;
    s_async_result = 0;
    s_state = GRAY_MCU_STATE_BUSY;

    s_gray.current_phase = (uint8_t)phase;
    s_gray.last_phase = (uint8_t)phase;
    s_gray.done_phase = DRV_GRAY_MCU_PHASE_IDLE;
    s_gray.last_op = (uint8_t)op;
    s_gray.last_reg = command;
    s_gray.last_rx_len = 0U;
    s_gray.last_status = BSP_BUSY;

    ret = BSP_I2C_MasterWrite_DMA_Async(DRV_GRAY_MCU_I2C_BUS,
                                        s_gray.active_addr,
                                        s_tx_buf,
                                        1U,
                                        GrayMcu_Callback);
    if (ret != BSP_OK) {
        GrayMcu_SetIdle();
        GrayMcu_RecordStartFailure(ret, phase);
        return ret;
    }

    return BSP_BUSY;
}

static void GrayMcu_FillDisabledChannels(void)
{
    uint8_t ch;
    uint8_t dst;

    for (ch = 0U; ch < DRV_GRAY_MCU_CHANNEL_NUM; ch++) {
        if ((s_gray.channel_enable_mask & (uint8_t)(1U << ch)) == 0U) {
            dst = GrayMcu_MapIndex(ch);
            s_gray.rx[dst] = 0xFFU;
            s_gray.raw[dst] = DRV_GRAY_MCU_DISABLED_VALUE;
            s_gray.filt[dst] = DRV_GRAY_MCU_DISABLED_VALUE;
        }
    }
}

static void GrayMcu_ParseAnalog(void)
{
    uint8_t ch;
    uint8_t dst;
    uint8_t rx_index = 0U;
    uint16_t raw;

    GrayMcu_FillDisabledChannels();

    for (ch = 0U; ch < DRV_GRAY_MCU_CHANNEL_NUM; ch++) {
        if ((s_gray.channel_enable_mask & (uint8_t)(1U << ch)) == 0U) {
            continue;
        }
        if (rx_index >= s_rx_len) {
            break;
        }

        dst = GrayMcu_MapIndex(ch);
        s_gray.rx[dst] = s_rx_buf[rx_index++];
        raw = GrayMcu_ExpandAnalog(s_gray.rx[dst]);
        s_gray.raw[dst] = raw;

        if (s_gray.valid == 0U) {
            s_gray.filt[dst] = raw;
        } else {
            s_gray.filt[dst] = GrayMcu_Filter(s_gray.filt[dst], raw);
        }
    }

    s_runtime_fail_count = 0U;
    s_gray.consecutive_error_count = 0U;
    s_gray.valid = 1U;
    s_gray.online = 1U;
    s_gray.initialized = 1U;
    s_gray.update_count++;
    s_gray.last_update_ms = BSP_GET_TICK();
}

static void GrayMcu_ParseDigital(uint8_t data)
{
    uint8_t ch;
    uint8_t dst;
    uint8_t bit_value;

    s_gray.digital_data = data;
    s_gray.digital_mask = 0U;

    for (ch = 0U; ch < DRV_GRAY_MCU_CHANNEL_NUM; ch++) {
        bit_value = (uint8_t)((data >> ch) & 0x01U);
        dst = GrayMcu_MapIndex(ch);
        if (bit_value == 0U) {
            s_gray.digital_mask |= (uint8_t)(1U << dst);
        }
    }
}

#if DRV_GRAY_MCU_AUTO_ADDR_SCAN
static void GrayMcu_SelectInitialAddress(void)
{
    uint8_t i;

    s_addr_index = 0U;
    for (i = 0U; i < (uint8_t)(sizeof(s_addr_list) / sizeof(s_addr_list[0])); i++) {
        if (s_addr_list[i] == DRV_GRAY_MCU_DEFAULT_ADDR_7BIT) {
            s_addr_index = i;
            break;
        }
    }

    s_gray.active_addr = s_addr_list[s_addr_index];
    s_gray.scan_count = (uint8_t)(sizeof(s_addr_list) / sizeof(s_addr_list[0]));
    s_gray.scan_mask = (uint8_t)(1U << s_addr_index);
}

static void GrayMcu_AdvanceAddress(uint32_t now)
{
    uint8_t count = (uint8_t)(sizeof(s_addr_list) / sizeof(s_addr_list[0]));

    if (s_gray.address_locked != 0U) {
        s_next_action_ms = now + DRV_GRAY_MCU_PING_RETRY_MS;
        return;
    }

    s_addr_index++;
    if (s_addr_index >= count) {
        s_addr_index = 0U;
        s_next_action_ms = now + DRV_GRAY_MCU_ERROR_BACKOFF_MS;
    } else {
        s_next_action_ms = now + DRV_GRAY_MCU_PING_RETRY_MS;
    }

    s_gray.active_addr = s_addr_list[s_addr_index];
    s_gray.scan_mask = (uint8_t)(1U << s_addr_index);
}
#else
static void GrayMcu_SelectInitialAddress(void)
{
    s_gray.active_addr = DRV_GRAY_MCU_DEFAULT_ADDR_7BIT;
    s_gray.scan_count = 1U;
    s_gray.scan_mask = 1U;
}

static void GrayMcu_AdvanceAddress(uint32_t now)
{
    s_next_action_ms = now + DRV_GRAY_MCU_PING_RETRY_MS;
}
#endif

static void GrayMcu_HandlePingFailure(uint32_t now)
{
    s_gray.ping_error_count++;
    s_init_step = GRAY_MCU_INIT_PING;

    if (s_gray.address_locked != 0U) {
        if (s_locked_ping_fail_count < 0xFFU) {
            s_locked_ping_fail_count++;
        }

        if (s_locked_ping_fail_count >= DRV_GRAY_MCU_UNLOCK_AFTER_PING_FAILS) {
            s_gray.address_locked = 0U;
            s_locked_ping_fail_count = 0U;
        }
    }

    GrayMcu_AdvanceAddress(now);
}

static void GrayMcu_HandleFailure(uint32_t now, GrayMcu_Op_t op)
{
    s_gray.last_status = GrayMcu_ResultToStatus(s_async_result);
    s_gray.last_i2c_result = s_async_result;
    s_gray.error_count++;

    if (s_gray.consecutive_error_count < 0xFFU) {
        s_gray.consecutive_error_count++;
    }

    if (op == GRAY_MCU_OP_PING) {
        s_gray.online = 0U;
        s_gray.initialized = 0U;
        GrayMcu_HandlePingFailure(now);
        return;
    }

    if ((op == GRAY_MCU_OP_ANALOG_READ) ||
        (op == GRAY_MCU_OP_ANALOG_SELECT)) {
        if (s_runtime_fail_count < 0xFFU) {
            s_runtime_fail_count++;
        }

        /*
         * 发生一次读错误时不丢弃已经确认的 0x4C 地址，也不去扫描其他地址。
         * 重新发送一次 0xB0，再继续纯读；连续失败达到阈值后才重新 ping。
         */
        if (s_runtime_fail_count < DRV_GRAY_MCU_RUNTIME_FAIL_LIMIT) {
            s_init_step = GRAY_MCU_INIT_SELECT_ANALOG;
            s_next_action_ms = now + DRV_GRAY_MCU_RUNTIME_RETRY_MS;
        } else {
            s_gray.online = 0U;
            s_gray.initialized = 0U;
            s_init_step = GRAY_MCU_INIT_PING;
            s_next_action_ms = now + DRV_GRAY_MCU_ERROR_BACKOFF_MS;
        }
        return;
    }

    /* 初始化中的固件/寄存器事务失败：保留当前步骤，在同一地址重试。 */
    s_next_action_ms = now + DRV_GRAY_MCU_INIT_RETRY_MS;
}

static BSP_Status_t GrayMcu_HandleDone(void)
{
    GrayMcu_Op_t op;
    uint32_t now;

    if (s_state != GRAY_MCU_STATE_DONE) {
        return BSP_BUSY;
    }

    now = BSP_GET_TICK();
    op = s_op;
    s_gray.done_phase = s_gray.current_phase;
    s_gray.last_phase = s_gray.current_phase;
    s_gray.last_op = (uint8_t)op;
    s_gray.last_i2c_result = s_async_result;

    GrayMcu_SetIdle();

    if (s_async_result != 0) {
        GrayMcu_HandleFailure(now, op);
        return s_gray.last_status;
    }

    s_gray.last_status = BSP_OK;
    s_gray.consecutive_error_count = 0U;

    switch (op) {
    case GRAY_MCU_OP_PING:
        s_gray.ping_value = s_rx_buf[0];
        if (s_rx_buf[0] != DRV_GRAY_MCU_PING_OK) {
            s_gray.error_count++;
            s_gray.online = 0U;
            s_gray.initialized = 0U;
            GrayMcu_HandlePingFailure(now);
            return BSP_ERROR;
        }

        s_gray.online = 1U;
        s_gray.address_locked = 1U;
        s_locked_ping_fail_count = 0U;
        s_gray.ping_ok_count++;
        if (s_init_step == GRAY_MCU_INIT_PING) {
            s_init_step = GRAY_MCU_INIT_FIRMWARE;
        }
        s_next_action_ms = now;
        break;

    case GRAY_MCU_OP_FIRMWARE:
        s_gray.firmware_version = s_rx_buf[0];
        s_gray.online = 1U;
        if (s_init_step == GRAY_MCU_INIT_FIRMWARE) {
            s_init_step = GRAY_MCU_INIT_SET_CHANNEL;
        } else if (s_init_step == GRAY_MCU_INIT_RUN) {
            s_init_step = GRAY_MCU_INIT_SELECT_ANALOG;
        }
        s_next_action_ms = now;
        break;

    case GRAY_MCU_OP_SET_CHANNEL:
        s_gray.channel_enable_mask = s_write_value;
        s_gray.online = 1U;
        if (s_init_step == GRAY_MCU_INIT_SET_CHANNEL) {
            s_init_step = GRAY_MCU_INIT_SET_NORMALIZE;
        } else {
            s_init_step = GRAY_MCU_INIT_SELECT_ANALOG;
        }
        s_next_action_ms = now;
        break;

    case GRAY_MCU_OP_SET_NORMALIZE:
        s_gray.normalize_mask = s_write_value;
        s_gray.online = 1U;
        if (s_init_step == GRAY_MCU_INIT_SET_NORMALIZE) {
            s_init_step = GRAY_MCU_INIT_SELECT_ANALOG;
        } else {
            s_init_step = GRAY_MCU_INIT_SELECT_ANALOG;
        }
        s_next_action_ms = now;
        break;

    case GRAY_MCU_OP_ANALOG_SELECT:
        s_gray.online = 1U;
        s_gray.initialized = 1U;
        s_init_step = GRAY_MCU_INIT_RUN;
        s_next_action_ms = now;
        break;

    case GRAY_MCU_OP_ANALOG_READ:
        GrayMcu_ParseAnalog();
        s_init_step = GRAY_MCU_INIT_RUN;
        s_next_action_ms = now + DRV_GRAY_MCU_UPDATE_PERIOD_MS;
        break;

    case GRAY_MCU_OP_DIGITAL:
        GrayMcu_ParseDigital(s_rx_buf[0]);
        s_gray.online = 1U;
        s_init_step = GRAY_MCU_INIT_SELECT_ANALOG;
        s_next_action_ms = now;
        break;

    case GRAY_MCU_OP_ERROR:
        s_gray.error_flags = s_rx_buf[0];
        s_gray.online = 1U;
        s_init_step = GRAY_MCU_INIT_SELECT_ANALOG;
        s_next_action_ms = now;
        break;

    case GRAY_MCU_OP_REBOOT:
        s_gray.online = 0U;
        s_gray.valid = 0U;
        s_gray.initialized = 0U;
        s_gray.firmware_version = 0U;
        s_gray.normalize_mask = 0U;
        s_gray.channel_enable_mask = DRV_GRAY_MCU_CHANNEL_ENABLE_MASK;
        s_gray.address_locked = 0U;
        s_runtime_fail_count = 0U;
        s_locked_ping_fail_count = 0U;
        s_init_step = GRAY_MCU_INIT_PING;
        s_next_action_ms = now + DRV_GRAY_MCU_REBOOT_WAIT_MS;
        break;

    default:
        s_next_action_ms = now + DRV_GRAY_MCU_UPDATE_PERIOD_MS;
        break;
    }

    return BSP_OK;
}

void Drv_GrayMcu_Init(void)
{
    uint8_t i;

    memset(&s_gray, 0, sizeof(s_gray));
    memset(s_tx_buf, 0, sizeof(s_tx_buf));
    memset(s_rx_buf, 0, sizeof(s_rx_buf));

    for (i = 0U; i < DRV_GRAY_MCU_CHANNEL_NUM; i++) {
        s_gray.raw[i] = DRV_GRAY_MCU_DISABLED_VALUE;
        s_gray.filt[i] = DRV_GRAY_MCU_DISABLED_VALUE;
        s_gray.rx[i] = 0xFFU;
    }

    GrayMcu_SelectInitialAddress();

    s_gray.channel_enable_mask = DRV_GRAY_MCU_CHANNEL_ENABLE_MASK;
    s_gray.normalize_mask = 0U;
    s_gray.current_phase = DRV_GRAY_MCU_PHASE_IDLE;
    s_gray.last_phase = DRV_GRAY_MCU_PHASE_IDLE;
    s_gray.done_phase = DRV_GRAY_MCU_PHASE_IDLE;
    s_gray.last_op = GRAY_MCU_OP_NONE;
    s_gray.last_status = BSP_BUSY;

    s_state = GRAY_MCU_STATE_IDLE;
    s_op = GRAY_MCU_OP_NONE;
    s_async_result = 0;
    s_rx_len = 0U;
    s_write_value = 0U;
    s_runtime_fail_count = 0U;
    s_locked_ping_fail_count = 0U;
    s_init_step = GRAY_MCU_INIT_PING;
    s_next_action_ms = BSP_GET_TICK() + DRV_GRAY_MCU_FIRST_PING_DELAY_MS;
}

BSP_Status_t Drv_GrayMcu_Update(void)
{
    uint32_t now;
    uint8_t read_len;

    if (s_state == GRAY_MCU_STATE_DONE) {
        return GrayMcu_HandleDone();
    }

    if (s_state != GRAY_MCU_STATE_IDLE) {
        return BSP_BUSY;
    }

    now = BSP_GET_TICK();
    if (GrayMcu_TimeReached(now, s_next_action_ms) == 0U) {
        return BSP_BUSY;
    }

    switch (s_init_step) {
    case GRAY_MCU_INIT_PING:
        return GrayMcu_StartCommandRead(GRAY_MCU_OP_PING,
                                        DRV_GRAY_MCU_PHASE_PING,
                                        DRV_GRAY_MCU_CMD_PING,
                                        1U);

    case GRAY_MCU_INIT_FIRMWARE:
        return GrayMcu_StartCommandRead(GRAY_MCU_OP_FIRMWARE,
                                        DRV_GRAY_MCU_PHASE_FIRMWARE,
                                        DRV_GRAY_MCU_CMD_FIRMWARE,
                                        1U);

    case GRAY_MCU_INIT_SET_CHANNEL:
        return GrayMcu_StartWrite2(GRAY_MCU_OP_SET_CHANNEL,
                                   DRV_GRAY_MCU_PHASE_CHANNEL_ENABLE,
                                   DRV_GRAY_MCU_CMD_CHANNEL_ENABLE,
                                   DRV_GRAY_MCU_CHANNEL_ENABLE_MASK);

    case GRAY_MCU_INIT_SET_NORMALIZE:
        if ((DRV_GRAY_MCU_NORMALIZE_MASK != 0U) &&
            (GrayMcu_NormalizeSupported(s_gray.firmware_version) != 0U)) {
            return GrayMcu_StartWrite2(GRAY_MCU_OP_SET_NORMALIZE,
                                       DRV_GRAY_MCU_PHASE_NORMALIZE,
                                       DRV_GRAY_MCU_CMD_NORMALIZE,
                                       DRV_GRAY_MCU_NORMALIZE_MASK);
        }

        s_gray.normalize_mask = 0U;
        s_init_step = GRAY_MCU_INIT_SELECT_ANALOG;
        s_next_action_ms = now;
        return BSP_BUSY;

    case GRAY_MCU_INIT_SELECT_ANALOG:
        /* 手册方法 2：只发一次 0xB0，并产生 STOP。 */
        return GrayMcu_StartWrite1(GRAY_MCU_OP_ANALOG_SELECT,
                                   DRV_GRAY_MCU_PHASE_ANALOG_SELECT,
                                   DRV_GRAY_MCU_CMD_ANALOG_ALL);

    case GRAY_MCU_INIT_RUN:
    default:
        read_len = GrayMcu_PopCount8(s_gray.channel_enable_mask);
        if (read_len == 0U) {
            s_gray.last_status = BSP_PARAM;
            return BSP_PARAM;
        }

        /* 运行中只读，不再重复发送 0xB0。 */
        return GrayMcu_StartReadOnly(GRAY_MCU_OP_ANALOG_READ,
                                     DRV_GRAY_MCU_PHASE_ANALOG,
                                     DRV_GRAY_MCU_CMD_ANALOG_ALL,
                                     read_len);
    }
}

uint16_t Drv_GrayMcu_GetRaw(uint8_t index)
{
    if (index >= DRV_GRAY_MCU_CHANNEL_NUM) {
        return 0U;
    }
    return s_gray.raw[index];
}

uint16_t Drv_GrayMcu_GetFilt(uint8_t index)
{
    if (index >= DRV_GRAY_MCU_CHANNEL_NUM) {
        return 0U;
    }
    return s_gray.filt[index];
}

BSP_Status_t Drv_GrayMcu_GetRawArray(uint16_t *out_buf, uint8_t max_count)
{
    uint8_t i;
    uint8_t n;

    if (out_buf == 0) {
        return BSP_PARAM;
    }

    n = (max_count < DRV_GRAY_MCU_CHANNEL_NUM) ? max_count : DRV_GRAY_MCU_CHANNEL_NUM;
    for (i = 0U; i < n; i++) {
        out_buf[i] = s_gray.raw[i];
    }

    return BSP_OK;
}

BSP_Status_t Drv_GrayMcu_GetFiltArray(uint16_t *out_buf, uint8_t max_count)
{
    uint8_t i;
    uint8_t n;

    if (out_buf == 0) {
        return BSP_PARAM;
    }

    n = (max_count < DRV_GRAY_MCU_CHANNEL_NUM) ? max_count : DRV_GRAY_MCU_CHANNEL_NUM;
    for (i = 0U; i < n; i++) {
        out_buf[i] = s_gray.filt[i];
    }

    return BSP_OK;
}

BSP_Status_t Drv_GrayMcu_GetRx8(uint8_t *out_buf, uint8_t max_count)
{
    uint8_t i;
    uint8_t n;

    if (out_buf == 0) {
        return BSP_PARAM;
    }

    n = (max_count < DRV_GRAY_MCU_CHANNEL_NUM) ? max_count : DRV_GRAY_MCU_CHANNEL_NUM;
    for (i = 0U; i < n; i++) {
        out_buf[i] = s_gray.rx[i];
    }

    return BSP_OK;
}

BSP_Status_t Drv_GrayMcu_GetInfo(Drv_GrayMcu_Info_t *info)
{
    if (info == 0) {
        return BSP_PARAM;
    }

    *info = s_gray;
    return BSP_OK;
}

uint8_t Drv_GrayMcu_IsOnline(void)
{
    uint32_t now;

    if ((s_gray.online == 0U) ||
        (s_gray.valid == 0U) ||
        (s_gray.initialized == 0U)) {
        return 0U;
    }

    now = BSP_GET_TICK();
    if ((uint32_t)(now - s_gray.last_update_ms) > DRV_GRAY_MCU_STALE_TIMEOUT_MS) {
        return 0U;
    }

    return 1U;
}

uint8_t Drv_GrayMcu_IsBusy(void)
{
    return (uint8_t)(s_state != GRAY_MCU_STATE_IDLE);
}

BSP_Status_t Drv_GrayMcu_Ping(void)
{
    return GrayMcu_StartCommandRead(GRAY_MCU_OP_PING,
                                    DRV_GRAY_MCU_PHASE_PING,
                                    DRV_GRAY_MCU_CMD_PING,
                                    1U);
}

BSP_Status_t Drv_GrayMcu_ReadDigital(uint8_t *digital_data)
{
    if (digital_data == 0) {
        return BSP_PARAM;
    }

    *digital_data = s_gray.digital_data;
    return GrayMcu_StartCommandRead(GRAY_MCU_OP_DIGITAL,
                                    DRV_GRAY_MCU_PHASE_DIGITAL,
                                    DRV_GRAY_MCU_CMD_DIGITAL,
                                    1U);
}

BSP_Status_t Drv_GrayMcu_ReadErrorFlags(uint8_t *error_flags)
{
    if (error_flags == 0) {
        return BSP_PARAM;
    }

    *error_flags = s_gray.error_flags;
    return GrayMcu_StartCommandRead(GRAY_MCU_OP_ERROR,
                                    DRV_GRAY_MCU_PHASE_ERROR,
                                    DRV_GRAY_MCU_CMD_ERROR,
                                    1U);
}

BSP_Status_t Drv_GrayMcu_ReadFirmware(uint8_t *firmware_version)
{
    if (firmware_version == 0) {
        return BSP_PARAM;
    }

    *firmware_version = s_gray.firmware_version;
    return GrayMcu_StartCommandRead(GRAY_MCU_OP_FIRMWARE,
                                    DRV_GRAY_MCU_PHASE_FIRMWARE,
                                    DRV_GRAY_MCU_CMD_FIRMWARE,
                                    1U);
}

BSP_Status_t Drv_GrayMcu_SetChannelEnable(uint8_t enable_mask)
{
    if (enable_mask == 0U) {
        return BSP_PARAM;
    }

    return GrayMcu_StartWrite2(GRAY_MCU_OP_SET_CHANNEL,
                               DRV_GRAY_MCU_PHASE_CHANNEL_ENABLE,
                               DRV_GRAY_MCU_CMD_CHANNEL_ENABLE,
                               enable_mask);
}

BSP_Status_t Drv_GrayMcu_SetFlatEnable(uint8_t enable_mask)
{
    if ((enable_mask != 0U) &&
        (s_gray.firmware_version != 0U) &&
        (GrayMcu_NormalizeSupported(s_gray.firmware_version) == 0U)) {
        return BSP_PARAM;
    }

    return GrayMcu_StartWrite2(GRAY_MCU_OP_SET_NORMALIZE,
                               DRV_GRAY_MCU_PHASE_NORMALIZE,
                               DRV_GRAY_MCU_CMD_NORMALIZE,
                               enable_mask);
}

BSP_Status_t Drv_GrayMcu_Reboot(void)
{
    return GrayMcu_StartWrite1(GRAY_MCU_OP_REBOOT,
                               DRV_GRAY_MCU_PHASE_REBOOT,
                               DRV_GRAY_MCU_CMD_REBOOT);
}
