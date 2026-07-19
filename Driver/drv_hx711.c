#include "drv_hx711.h"

#if (DRV_HX711_AVERAGE_SAMPLES == 0U)
#error "DRV_HX711_AVERAGE_SAMPLES must be greater than zero"
#endif

#if (DRV_HX711_AUTO_TARE_SAMPLE_COUNT == 0U) || (DRV_HX711_AUTO_TARE_SAMPLE_COUNT > 255U)
#error "DRV_HX711_AUTO_TARE_SAMPLE_COUNT must be in range 1..255"
#endif

static Drv_HX711_Info_t s_hx711;
static int32_t s_average_buffer[DRV_HX711_AVERAGE_SAMPLES];
static int64_t s_average_sum;
static uint8_t s_average_index;
static uint8_t s_average_count;
static uint8_t s_gain_change_pending;
static int64_t s_auto_tare_sum;
static int32_t s_auto_tare_min;
static int32_t s_auto_tare_max;
static uint8_t s_auto_tare_count;

/*
 * 产生约 1 us 的时钟半周期。数据手册要求 PD_SCK 高电平 0.2~50 us、
 * 低电平不小于 0.2 us；使用易移植的短忙等避免依赖额外定时器。
 */
static void HX711_ClockDelay(void)
{
    volatile uint32_t count;
    uint32_t loops = SystemCoreClock / 6000000UL;

    if (loops < 2U) {
        loops = 2U;
    }
    for (count = 0U; count < loops; count++) {
        __NOP();
    }
}

static uint8_t HX711_IsGainValid(Drv_HX711_Gain_t gain)
{
    return ((gain == DRV_HX711_CHANNEL_A_GAIN_128) ||
            (gain == DRV_HX711_CHANNEL_B_GAIN_32) ||
            (gain == DRV_HX711_CHANNEL_A_GAIN_64)) ? 1U : 0U;
}

/*
 * 完整移出 24 位数据，并用第 25~27 个脉冲选择下一次转换通道和增益。
 * 临界区只覆盖约几十微秒，防止中断把 PD_SCK 高电平拉长到 60 us 以上而掉电。
 */
static int32_t HX711_ReadFrame(void)
{
    uint32_t value = 0U;
    uint32_t primask;
    uint8_t bit_index;
    uint8_t select_pulses;

    select_pulses = (uint8_t)s_hx711.gain;
    primask = BSP_EnterCritical();

    for (bit_index = 0U; bit_index < 24U; bit_index++) {
        BSP_GPIO_Write(DRV_HX711_PD_SCK_GPIO, 1U);
        HX711_ClockDelay();
        value = (value << 1) | (uint32_t)BSP_GPIO_Read(DRV_HX711_DOUT_GPIO);
        BSP_GPIO_Write(DRV_HX711_PD_SCK_GPIO, 0U);
        HX711_ClockDelay();
    }

    for (bit_index = 0U; bit_index < select_pulses; bit_index++) {
        BSP_GPIO_Write(DRV_HX711_PD_SCK_GPIO, 1U);
        HX711_ClockDelay();
        BSP_GPIO_Write(DRV_HX711_PD_SCK_GPIO, 0U);
        HX711_ClockDelay();
    }

    BSP_ExitCritical(primask);

    /* 将 HX711 的 24 位二补码显式符号扩展到 int32_t。 */
    if ((value & 0x00800000UL) != 0U) {
        value |= 0xFF000000UL;
    }
    return (int32_t)value;
}

static void HX711_ResetAverage(int32_t first_sample)
{
    uint8_t i;

    for (i = 0U; i < DRV_HX711_AVERAGE_SAMPLES; i++) {
        s_average_buffer[i] = 0;
    }
    s_average_buffer[0] = first_sample;
    s_average_sum = first_sample;
    s_average_index = (DRV_HX711_AVERAGE_SAMPLES > 1U) ? 1U : 0U;
    s_average_count = 1U;
    s_hx711.filtered_counts = first_sample;
}

static void HX711_AddAverageSample(int32_t sample)
{
    if (s_average_count == 0U) {
        HX711_ResetAverage(sample);
        return;
    }

    if (s_average_count < DRV_HX711_AVERAGE_SAMPLES) {
        s_average_buffer[s_average_index] = sample;
        s_average_sum += sample;
        s_average_count++;
    } else {
        s_average_sum -= s_average_buffer[s_average_index];
        s_average_buffer[s_average_index] = sample;
        s_average_sum += sample;
    }

    s_average_index++;
    if (s_average_index >= DRV_HX711_AVERAGE_SAMPLES) {
        s_average_index = 0U;
    }
    s_hx711.filtered_counts = (int32_t)(s_average_sum / (int64_t)s_average_count);
}

static void HX711_UpdatePressureCache(void)
{
    if ((s_hx711.calibrated != 0U) && (s_hx711.tare_ready != 0U)) {
        s_hx711.pressure_g =
            ((float)(s_hx711.filtered_counts - s_hx711.tare_offset_counts)) /
            s_hx711.scale_counts_per_g;
    } else {
        s_hx711.pressure_g = 0.0f;
    }
}

static void HX711_ResetAutoTare(int32_t sample)
{
    s_auto_tare_sum = sample;
    s_auto_tare_min = sample;
    s_auto_tare_max = sample;
    s_auto_tare_count = 1U;
    s_hx711.tare_ready = 0U;
}

static void HX711_ProcessAutoTare(int32_t sample)
{
#if (DRV_HX711_AUTO_TARE_ENABLE != 0U)
    int32_t next_min;
    int32_t next_max;

    if (s_hx711.tare_ready != 0U) {
        return;
    }
    if (s_auto_tare_count == 0U) {
        HX711_ResetAutoTare(sample);
        return;
    }

    next_min = (sample < s_auto_tare_min) ? sample : s_auto_tare_min;
    next_max = (sample > s_auto_tare_max) ? sample : s_auto_tare_max;
    if ((next_max - next_min) > DRV_HX711_AUTO_TARE_MAX_SPAN_COUNTS) {
        /* 称重台被碰动或仍在机械回弹时重新开始稳定计数。 */
        HX711_ResetAutoTare(sample);
        return;
    }

    s_auto_tare_min = next_min;
    s_auto_tare_max = next_max;
    s_auto_tare_sum += sample;
    s_auto_tare_count++;
    if (s_auto_tare_count >= DRV_HX711_AUTO_TARE_SAMPLE_COUNT) {
        s_hx711.tare_offset_counts =
            (int32_t)(s_auto_tare_sum / (int64_t)s_auto_tare_count);
        s_hx711.tare_ready = 1U;
        HX711_UpdatePressureCache();
    }
#else
    (void)sample;
    s_hx711.tare_ready = 1U;
#endif
}

void Drv_HX711_Init(void)
{
    uint8_t i;

    s_hx711.raw_counts = 0;
    s_hx711.filtered_counts = 0;
    s_hx711.tare_offset_counts = 0;
    s_hx711.scale_counts_per_g = DRV_HX711_DEFAULT_SCALE_COUNTS_PER_G;
    s_hx711.pressure_g = 0.0f;
    s_hx711.timestamp_ms = BSP_GET_TICK();
    s_hx711.sample_count = 0U;
    s_hx711.error_count = 0U;
    s_hx711.timeout_count = 0U;
    s_hx711.initialized = 1U;
    s_hx711.online = 0U;
    s_hx711.data_valid = 0U;
    s_hx711.calibrated =
        ((DRV_HX711_DEFAULT_SCALE_COUNTS_PER_G > 0.000001f) ||
         (DRV_HX711_DEFAULT_SCALE_COUNTS_PER_G < -0.000001f)) ? 1U : 0U;
    s_hx711.tare_ready = (DRV_HX711_AUTO_TARE_ENABLE != 0U) ? 0U : 1U;
    s_hx711.powered_down = 0U;
    s_hx711.gain = DRV_HX711_CHANNEL_A_GAIN_128;

    for (i = 0U; i < DRV_HX711_AVERAGE_SAMPLES; i++) {
        s_average_buffer[i] = 0;
    }
    s_average_sum = 0;
    s_average_index = 0U;
    s_average_count = 0U;
    s_gain_change_pending = 0U;
    s_auto_tare_sum = 0;
    s_auto_tare_min = 0;
    s_auto_tare_max = 0;
    s_auto_tare_count = 0U;

    /* PD_SCK 必须保持低电平，DOUT 变低后才允许开始读取。 */
    BSP_GPIO_Write(DRV_HX711_PD_SCK_GPIO, 0U);
}

BSP_Status_t Drv_HX711_Update(void)
{
    int32_t sample;
    uint32_t now;

    if ((s_hx711.initialized == 0U) || (s_hx711.powered_down != 0U)) {
        return BSP_ERROR;
    }

    now = BSP_GET_TICK();
    if (BSP_GPIO_Read(DRV_HX711_DOUT_GPIO) != 0U) {
        if ((s_hx711.online != 0U) &&
            ((uint32_t)(now - s_hx711.timestamp_ms) >= DRV_HX711_OFFLINE_TIMEOUT_MS)) {
            s_hx711.online = 0U;
            s_hx711.timeout_count++;
        }
        return BSP_BUSY;
    }

    sample = HX711_ReadFrame();
    s_hx711.timestamp_ms = now;
    s_hx711.online = 1U;

    /* 增益切换时，本帧仍属于旧配置；脉冲结束后下一帧才是新配置。 */
    if (s_gain_change_pending != 0U) {
        s_gain_change_pending = 0U;
        s_average_count = 0U;
        s_average_sum = 0;
        s_hx711.data_valid = 0U;
        return BSP_BUSY;
    }

    /* 量程端点表示模拟输入已饱和，不让该帧污染滤波和克重缓存。 */
    if ((sample == (int32_t)0xFF800000L) || (sample == 0x007FFFFFL)) {
        s_hx711.raw_counts = sample;
        s_hx711.error_count++;
        return BSP_ERROR;
    }

    s_hx711.raw_counts = sample;
    HX711_AddAverageSample(sample);
    s_hx711.sample_count++;
    s_hx711.data_valid = 1U;
    HX711_ProcessAutoTare(sample);
    HX711_UpdatePressureCache();
    return BSP_OK;
}

BSP_Status_t Drv_HX711_GetRaw(int32_t *raw_counts)
{
    if (raw_counts == 0) {
        return BSP_PARAM;
    }
    if ((s_hx711.online == 0U) || (s_hx711.data_valid == 0U)) {
        return BSP_ERROR;
    }
    *raw_counts = s_hx711.raw_counts;
    return BSP_OK;
}

BSP_Status_t Drv_HX711_GetFiltered(int32_t *filtered_counts)
{
    if (filtered_counts == 0) {
        return BSP_PARAM;
    }
    if ((s_hx711.online == 0U) || (s_hx711.data_valid == 0U)) {
        return BSP_ERROR;
    }
    *filtered_counts = s_hx711.filtered_counts;
    return BSP_OK;
}

BSP_Status_t Drv_HX711_GetGram(float *pressure_g)
{
    if (pressure_g == 0) {
        return BSP_PARAM;
    }
    if ((s_hx711.online == 0U) ||
        (s_hx711.data_valid == 0U) ||
        (s_hx711.calibrated == 0U) ||
        (s_hx711.tare_ready == 0U)) {
        return BSP_ERROR;
    }
    *pressure_g = s_hx711.pressure_g;
    return BSP_OK;
}

BSP_Status_t Drv_HX711_GetInfo(Drv_HX711_Info_t *info)
{
    uint32_t primask;

    if (info == 0) {
        return BSP_PARAM;
    }
    primask = BSP_EnterCritical();
    *info = s_hx711;
    BSP_ExitCritical(primask);
    return BSP_OK;
}

BSP_Status_t Drv_HX711_Tare(void)
{
    if ((s_hx711.online == 0U) || (s_hx711.data_valid == 0U)) {
        return BSP_ERROR;
    }
    s_hx711.tare_offset_counts = s_hx711.filtered_counts;
    s_hx711.tare_ready = 1U;
    HX711_UpdatePressureCache();
    return BSP_OK;
}

BSP_Status_t Drv_HX711_CalibrateKnownWeight(float known_weight_g)
{
    int32_t delta;

    if (known_weight_g <= 0.0f) {
        return BSP_PARAM;
    }
    if ((s_hx711.online == 0U) || (s_hx711.data_valid == 0U)) {
        return BSP_ERROR;
    }
    if (s_hx711.tare_ready == 0U) {
        return BSP_BUSY;
    }

    delta = s_hx711.filtered_counts - s_hx711.tare_offset_counts;
    if ((delta > -DRV_HX711_MIN_CALIBRATION_DELTA_COUNTS) &&
        (delta < DRV_HX711_MIN_CALIBRATION_DELTA_COUNTS)) {
        return BSP_ERROR;
    }

    s_hx711.scale_counts_per_g = (float)delta / known_weight_g;
    s_hx711.calibrated = 1U;
    HX711_UpdatePressureCache();
    return BSP_OK;
}

BSP_Status_t Drv_HX711_SetScale(float counts_per_g)
{
    if ((counts_per_g > -0.000001f) && (counts_per_g < 0.000001f)) {
        return BSP_PARAM;
    }
    s_hx711.scale_counts_per_g = counts_per_g;
    s_hx711.calibrated = 1U;
    HX711_UpdatePressureCache();
    return BSP_OK;
}

BSP_Status_t Drv_HX711_SetGain(Drv_HX711_Gain_t gain)
{
    if (HX711_IsGainValid(gain) == 0U) {
        return BSP_PARAM;
    }
    if (s_hx711.powered_down != 0U) {
        return BSP_ERROR;
    }
    if (s_hx711.gain != gain) {
        s_hx711.gain = gain;
        s_gain_change_pending = 1U;
        s_hx711.tare_ready = (DRV_HX711_AUTO_TARE_ENABLE != 0U) ? 0U : 1U;
        s_auto_tare_sum = 0;
        s_auto_tare_min = 0;
        s_auto_tare_max = 0;
        s_auto_tare_count = 0U;
    }
    return BSP_OK;
}

void Drv_HX711_PowerDown(void)
{
    if (s_hx711.initialized == 0U) {
        return;
    }
    BSP_GPIO_Write(DRV_HX711_PD_SCK_GPIO, 0U);
    HX711_ClockDelay();
    BSP_GPIO_Write(DRV_HX711_PD_SCK_GPIO, 1U);
    s_hx711.powered_down = 1U;
    s_hx711.online = 0U;
}

void Drv_HX711_PowerUp(void)
{
    if (s_hx711.initialized == 0U) {
        return;
    }
    BSP_GPIO_Write(DRV_HX711_PD_SCK_GPIO, 0U);
    s_hx711.powered_down = 0U;
    s_hx711.online = 0U;
    s_hx711.data_valid = 0U;
    s_hx711.gain = DRV_HX711_CHANNEL_A_GAIN_128;
    s_gain_change_pending = 0U;
    s_average_count = 0U;
    s_average_sum = 0;
    s_hx711.tare_ready = (DRV_HX711_AUTO_TARE_ENABLE != 0U) ? 0U : 1U;
    s_auto_tare_sum = 0;
    s_auto_tare_min = 0;
    s_auto_tare_max = 0;
    s_auto_tare_count = 0U;
    s_hx711.timestamp_ms = BSP_GET_TICK();
}
