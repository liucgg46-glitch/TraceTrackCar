#include "drv_gray_yahboom_uart.h"

#include "bsp_uart.h"
#include <string.h>

#if (YAHBOOM_GRAY_DATA_MODE == YAHBOOM_GRAY_DATA_MODE_DIGITAL)
static const uint8_t s_start_command[] = "$0,0,1#";
#elif (YAHBOOM_GRAY_DATA_MODE == YAHBOOM_GRAY_DATA_MODE_ANALOG)
static const uint8_t s_start_command[] = "$0,1,0#";
#else
#error "Unsupported YAHBOOM_GRAY_DATA_MODE"
#endif

static Drv_GrayYahboom_Info_t s_gray;
static uint8_t s_frame[YAHBOOM_GRAY_FRAME_BUFFER_SIZE];
static uint16_t s_frame_length;
static uint8_t s_in_frame;
static uint16_t s_last_uart_overflow;
static uint16_t s_last_uart_error;
static uint32_t s_boot_tick;
static uint32_t s_last_command_tick;
static uint32_t s_next_update_tick;

static uint8_t GrayYahboom_TimeReached(uint32_t now, uint32_t target)
{
    return (((int32_t)(now - target) >= 0) ? 1U : 0U);
}

static uint16_t GrayYahboom_MapIndex(uint8_t index)
{
#if YAHBOOM_GRAY_REVERSE_ORDER
    return (uint16_t)(DRV_GRAY_YAHBOOM_CHANNEL_NUM - 1U - index);
#else
    return index;
#endif
}

#if (YAHBOOM_GRAY_DATA_MODE == YAHBOOM_GRAY_DATA_MODE_ANALOG)
static uint16_t GrayYahboom_Normalize(uint16_t value)
{
    uint16_t normalized;

    if (value >= DRV_GRAY_YAHBOOM_INPUT_MAX) {
        normalized = DRV_GRAY_YAHBOOM_OUTPUT_MAX;
    } else {
        normalized = value;
    }

#if YAHBOOM_GRAY_BLACK_IS_HIGH
    return (uint16_t)(DRV_GRAY_YAHBOOM_OUTPUT_MAX - normalized);
#else
    return normalized;
#endif
}

static uint16_t GrayYahboom_Filter(uint16_t old_value, uint16_t new_value)
{
#if YAHBOOM_GRAY_FILTER_SHIFT == 0U
    (void)old_value;
    return new_value;
#else
    int32_t diff = (int32_t)new_value - (int32_t)old_value;
    return (uint16_t)((int32_t)old_value +
                      (diff >> YAHBOOM_GRAY_FILTER_SHIFT));
#endif
}
#endif

static uint16_t GrayYahboom_CounterDelta(uint16_t current, uint16_t previous)
{
    if (current >= previous) {
        return (uint16_t)(current - previous);
    }

    /* BSP 统计被外部清零时，从新基准继续累计，不制造虚假回绕增量。 */
    return current;
}

BSP_Status_t Drv_GrayYahboom_ParseAnalogFrame(
    const uint8_t *frame,
    uint16_t length,
    uint16_t values[DRV_GRAY_YAHBOOM_CHANNEL_NUM])
{
    uint16_t position = 0U;
    uint8_t channel;

    if ((frame == 0) || (values == 0) || (length < 8U)) {
        return BSP_PARAM;
    }
    if ((frame[position++] != '$') ||
        (frame[position++] != 'A') ||
        (frame[position++] != ',')) {
        return BSP_ERROR;
    }

    for (channel = 0U; channel < DRV_GRAY_YAHBOOM_CHANNEL_NUM; channel++) {
        uint16_t value = 0U;
        uint8_t digit_count = 0U;

        if ((position + 3U) >= length ||
            (frame[position++] != 'x') ||
            (frame[position++] != (uint8_t)('1' + channel)) ||
            (frame[position++] != ':')) {
            return BSP_ERROR;
        }

        while ((position < length) &&
               (frame[position] >= '0') &&
               (frame[position] <= '9')) {
            value = (uint16_t)(value * 10U +
                               (uint16_t)(frame[position] - '0'));
            digit_count++;
            position++;
            if ((digit_count > 4U) ||
                (value > DRV_GRAY_YAHBOOM_INPUT_MAX)) {
                return BSP_ERROR;
            }
        }

        if (digit_count == 0U) {
            return BSP_ERROR;
        }
        values[channel] = value;

        if (channel < (DRV_GRAY_YAHBOOM_CHANNEL_NUM - 1U)) {
            if ((position >= length) || (frame[position++] != ',')) {
                return BSP_ERROR;
            }
        } else {
            if ((position >= length) || (frame[position++] != '#')) {
                return BSP_ERROR;
            }
        }
    }

    return (position == length) ? BSP_OK : BSP_ERROR;
}

#if (YAHBOOM_GRAY_DATA_MODE == YAHBOOM_GRAY_DATA_MODE_DIGITAL)
BSP_Status_t Drv_GrayYahboom_ParseDigitalFrame(
    const uint8_t *frame,
    uint16_t length,
    uint8_t values[DRV_GRAY_YAHBOOM_CHANNEL_NUM])
{
    uint16_t position = 0U;
    uint8_t channel;

    if ((frame == 0) || (values == 0) || (length < 8U)) {
        return BSP_PARAM;
    }
    if ((frame[position++] != '$') ||
        (frame[position++] != 'D') ||
        (frame[position++] != ',')) {
        return BSP_ERROR;
    }

    for (channel = 0U; channel < DRV_GRAY_YAHBOOM_CHANNEL_NUM; channel++) {
        if ((position + 3U) >= length ||
            (frame[position++] != 'x') ||
            (frame[position++] != (uint8_t)('1' + channel)) ||
            (frame[position++] != ':') ||
            (position >= length) ||
            ((frame[position] != '0') && (frame[position] != '1'))) {
            return BSP_ERROR;
        }

        values[channel] = (uint8_t)(frame[position++] - '0');
        if (channel < (DRV_GRAY_YAHBOOM_CHANNEL_NUM - 1U)) {
            if ((position >= length) || (frame[position++] != ',')) {
                return BSP_ERROR;
            }
        } else {
            if ((position >= length) || (frame[position++] != '#')) {
                return BSP_ERROR;
            }
        }
    }

    return (position == length) ? BSP_OK : BSP_ERROR;
}

static void GrayYahboom_AcceptDigitalFrame(
    const uint8_t values[DRV_GRAY_YAHBOOM_CHANNEL_NUM],
    uint32_t now)
{
    uint8_t source;
    uint16_t destination;
    uint8_t digital_value;
    uint8_t black_mask = 0U;

    for (source = 0U; source < DRV_GRAY_YAHBOOM_CHANNEL_NUM; source++) {
        destination = GrayYahboom_MapIndex(source);
        digital_value = values[source];

        /* raw 保留模块真实 0/1；filt 维持上层原有的黑低白高量程。 */
        s_gray.raw[destination] = digital_value;
        if (digital_value == YAHBOOM_GRAY_DIGITAL_BLACK_LEVEL) {
            s_gray.filt[destination] = 0U;
            black_mask |= (uint8_t)(1U << destination);
        } else {
            s_gray.filt[destination] = DRV_GRAY_YAHBOOM_OUTPUT_MAX;
        }
    }

    s_gray.digital_black_mask = black_mask;
    s_gray.valid_frame_count++;
    s_gray.last_frame_tick = now;
    s_gray.frame_age_ms = 0U;
    s_gray.valid = 1U;
}
#else
BSP_Status_t Drv_GrayYahboom_ParseDigitalFrame(
    const uint8_t *frame,
    uint16_t length,
    uint8_t values[DRV_GRAY_YAHBOOM_CHANNEL_NUM])
{
    (void)frame;
    (void)length;
    (void)values;
    return BSP_PARAM;
}

static void GrayYahboom_AcceptFrame(const uint16_t values[DRV_GRAY_YAHBOOM_CHANNEL_NUM],
                                    uint32_t now)
{
    uint8_t source;
    uint16_t destination;
    uint16_t value;
    uint8_t first_frame = (s_gray.valid_frame_count == 0U) ? 1U : 0U;

    for (source = 0U; source < DRV_GRAY_YAHBOOM_CHANNEL_NUM; source++) {
        destination = GrayYahboom_MapIndex(source);
        value = GrayYahboom_Normalize(values[source]);
        s_gray.raw[destination] = value;
        if (first_frame != 0U) {
            s_gray.filt[destination] = value;
        } else {
            s_gray.filt[destination] =
                GrayYahboom_Filter(s_gray.filt[destination], value);
        }
    }

    s_gray.valid_frame_count++;
    s_gray.last_frame_tick = now;
    s_gray.frame_age_ms = 0U;
    s_gray.valid = 1U;
}
#endif

static void GrayYahboom_RejectFrame(void)
{
    s_gray.invalid_frame_count++;
    s_gray.parse_error_count++;
}

static void GrayYahboom_ProcessByte(uint8_t data, uint32_t now)
{
#if (YAHBOOM_GRAY_DATA_MODE == YAHBOOM_GRAY_DATA_MODE_DIGITAL)
    uint8_t values[DRV_GRAY_YAHBOOM_CHANNEL_NUM];
#else
    uint16_t values[DRV_GRAY_YAHBOOM_CHANNEL_NUM];
#endif

    if (data == '$') {
        if ((s_in_frame != 0U) && (s_frame_length > 0U)) {
            GrayYahboom_RejectFrame();
        }
        s_in_frame = 1U;
        s_frame_length = 1U;
        s_frame[0] = data;
        return;
    }

    if (s_in_frame == 0U) {
        return;
    }

    if (s_frame_length >= (YAHBOOM_GRAY_FRAME_BUFFER_SIZE - 1U)) {
        GrayYahboom_RejectFrame();
        s_in_frame = 0U;
        s_frame_length = 0U;
        return;
    }

    s_frame[s_frame_length++] = data;
    if (data != '#') {
        return;
    }

    if (
#if (YAHBOOM_GRAY_DATA_MODE == YAHBOOM_GRAY_DATA_MODE_DIGITAL)
        Drv_GrayYahboom_ParseDigitalFrame(s_frame,
                                          s_frame_length,
                                          values) == BSP_OK
#else
        Drv_GrayYahboom_ParseAnalogFrame(s_frame,
                                         s_frame_length,
                                         values) == BSP_OK
#endif
    ) {
#if (YAHBOOM_GRAY_DATA_MODE == YAHBOOM_GRAY_DATA_MODE_DIGITAL)
        GrayYahboom_AcceptDigitalFrame(values, now);
#else
        GrayYahboom_AcceptFrame(values, now);
#endif
    } else {
        GrayYahboom_RejectFrame();
    }

    s_in_frame = 0U;
    s_frame_length = 0U;
}

static void GrayYahboom_UpdateUartStats(void)
{
    UART_Stats_t stats;
    uint16_t overflow_delta;
    uint16_t error_delta;

    if (BSP_UART_GetStats(UART_PORT_YAHBOOM_GRAY, &stats) != BSP_OK) {
        return;
    }

    overflow_delta = GrayYahboom_CounterDelta(stats.rx_overflow,
                                               s_last_uart_overflow);
    error_delta = GrayYahboom_CounterDelta(stats.rx_error,
                                            s_last_uart_error);
    s_last_uart_overflow = stats.rx_overflow;
    s_last_uart_error = stats.rx_error;

    if ((overflow_delta != 0U) || (error_delta != 0U)) {
        s_gray.rx_overflow_count += overflow_delta;
        s_gray.uart_error_count += error_delta;
        s_gray.valid = 0U;
        s_gray.online = 0U;
        s_in_frame = 0U;
        s_frame_length = 0U;
    }
}

static void GrayYahboom_TrySendCommand(uint32_t now)
{
    if ((s_gray.command_pending != 0U) ||
        (BSP_UART_IsTxBusy(UART_PORT_YAHBOOM_GRAY) != 0U)) {
        return;
    }

    if (BSP_UART_WriteFrame(UART_PORT_YAHBOOM_GRAY,
                            s_start_command,
                            (uint16_t)(sizeof(s_start_command) - 1U)) == BSP_OK) {
        s_gray.command_pending = 1U;
        s_gray.command_sent = 0U;
        s_gray.command_count++;
        s_last_command_tick = now;
    }
}

void Drv_GrayYahboom_Init(void)
{
    uint8_t channel;
    UART_Stats_t stats;

    memset(&s_gray, 0, sizeof(s_gray));
    memset(s_frame, 0, sizeof(s_frame));
    s_frame_length = 0U;
    s_in_frame = 0U;
    s_boot_tick = BSP_GET_TICK();
    s_last_command_tick = s_boot_tick - YAHBOOM_GRAY_COMMAND_RETRY_MS;
    s_next_update_tick = s_boot_tick;
    s_gray.data_mode = YAHBOOM_GRAY_DATA_MODE;
    s_gray.digital_black_mask = 0U;

    for (channel = 0U; channel < DRV_GRAY_YAHBOOM_CHANNEL_NUM; channel++) {
#if (YAHBOOM_GRAY_DATA_MODE == YAHBOOM_GRAY_DATA_MODE_DIGITAL)
        s_gray.raw[channel] = 1U;
#else
        s_gray.raw[channel] = DRV_GRAY_YAHBOOM_OUTPUT_MAX;
#endif
        s_gray.filt[channel] = DRV_GRAY_YAHBOOM_OUTPUT_MAX;
    }

    s_gray.initialized =
        BSP_UART_IsInitialized(UART_PORT_YAHBOOM_GRAY);
    if (BSP_UART_GetStats(UART_PORT_YAHBOOM_GRAY, &stats) == BSP_OK) {
        s_last_uart_overflow = stats.rx_overflow;
        s_last_uart_error = stats.rx_error;
    } else {
        s_last_uart_overflow = 0U;
        s_last_uart_error = 0U;
    }

    if (s_gray.initialized != 0U) {
        BSP_UART_FlushRx(UART_PORT_YAHBOOM_GRAY);
        GrayYahboom_TrySendCommand(s_boot_tick);
    }
}

BSP_Status_t Drv_GrayYahboom_Update(void)
{
    uint32_t now = BSP_GET_TICK();
    uint16_t processed = 0U;
    uint8_t data;

    s_gray.initialized =
        BSP_UART_IsInitialized(UART_PORT_YAHBOOM_GRAY);
    if (s_gray.initialized == 0U) {
        s_gray.online = 0U;
        s_gray.valid = 0U;
        return BSP_ERROR;
    }

    if (s_gray.command_pending != 0U &&
        BSP_UART_IsTxBusy(UART_PORT_YAHBOOM_GRAY) == 0U) {
        s_gray.command_pending = 0U;
        s_gray.command_sent = 1U;
    }

    if (GrayYahboom_TimeReached(now, s_next_update_tick) == 0U) {
        return BSP_BUSY;
    }
    s_next_update_tick = now + YAHBOOM_GRAY_UPDATE_PERIOD_MS;

    GrayYahboom_UpdateUartStats();

    while ((processed < YAHBOOM_GRAY_MAX_BYTES_PER_UPDATE) &&
           (BSP_UART_GetChar(UART_PORT_YAHBOOM_GRAY, &data) != 0U)) {
        GrayYahboom_ProcessByte(data, now);
        processed++;
    }

    if (s_gray.valid_frame_count != 0U) {
        s_gray.frame_age_ms = (uint32_t)(now - s_gray.last_frame_tick);
        if (s_gray.frame_age_ms > YAHBOOM_GRAY_FRAME_TIMEOUT_MS) {
            s_gray.valid = 0U;
        }
    } else {
        s_gray.frame_age_ms = 0xFFFFFFFFUL;
        s_gray.valid = 0U;
    }

    s_gray.online = 0U;
    if ((s_gray.valid != 0U) &&
        ((uint32_t)(now - s_boot_tick) >= YAHBOOM_GRAY_WARMUP_MS)) {
        s_gray.online = 1U;
    }

    if ((s_gray.valid == 0U) &&
        ((uint32_t)(now - s_last_command_tick) >=
         YAHBOOM_GRAY_COMMAND_RETRY_MS)) {
        GrayYahboom_TrySendCommand(now);
    }

    return (s_gray.online != 0U) ? BSP_OK : BSP_BUSY;
}

uint16_t Drv_GrayYahboom_GetRaw(uint8_t index)
{
    if (index >= DRV_GRAY_YAHBOOM_CHANNEL_NUM) {
        return 0U;
    }
    return s_gray.raw[index];
}

uint16_t Drv_GrayYahboom_GetFilt(uint8_t index)
{
    if (index >= DRV_GRAY_YAHBOOM_CHANNEL_NUM) {
        return 0U;
    }
    return s_gray.filt[index];
}

BSP_Status_t Drv_GrayYahboom_GetRawArray(uint16_t *out_buf,
                                         uint8_t max_count)
{
    uint8_t channel;
    uint8_t count;

    if (out_buf == 0) {
        return BSP_PARAM;
    }
    count = (max_count < DRV_GRAY_YAHBOOM_CHANNEL_NUM) ?
            max_count : DRV_GRAY_YAHBOOM_CHANNEL_NUM;
    for (channel = 0U; channel < count; channel++) {
        out_buf[channel] = s_gray.raw[channel];
    }
    return BSP_OK;
}

BSP_Status_t Drv_GrayYahboom_GetFiltArray(uint16_t *out_buf,
                                          uint8_t max_count)
{
    uint8_t channel;
    uint8_t count;

    if (out_buf == 0) {
        return BSP_PARAM;
    }
    count = (max_count < DRV_GRAY_YAHBOOM_CHANNEL_NUM) ?
            max_count : DRV_GRAY_YAHBOOM_CHANNEL_NUM;
    for (channel = 0U; channel < count; channel++) {
        out_buf[channel] = s_gray.filt[channel];
    }
    return BSP_OK;
}

BSP_Status_t Drv_GrayYahboom_GetInfo(Drv_GrayYahboom_Info_t *info)
{
    if (info == 0) {
        return BSP_PARAM;
    }
    *info = s_gray;
    if (s_gray.valid_frame_count != 0U) {
        info->frame_age_ms =
            (uint32_t)(BSP_GET_TICK() - s_gray.last_frame_tick);
    }
    return BSP_OK;
}

uint8_t Drv_GrayYahboom_IsOnline(void)
{
    return s_gray.online;
}
