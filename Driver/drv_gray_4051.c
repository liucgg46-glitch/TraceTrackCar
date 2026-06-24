#include "drv_gray_4051.h"
#include "bsp_gpio.h"
#include "bsp_adc.h"
#include "bsp_common.h"

static Drv_Gray4051_Info_t s_gray;
static uint8_t s_phase;
static uint32_t s_select_tick;

static uint8_t Gray_MapIndex(uint8_t index)
{
#if DRV_GRAY_4051_INDEX_REVERSE
    return (uint8_t)(DRV_GRAY_4051_CHANNEL_NUM - 1U - index);
#else
    return index;
#endif
}

static void Gray_SelectChannel(uint8_t ch)
{
    BSP_GPIO_Write(BSP_GPIO_GRAY_S0, (uint8_t)((ch >> 0) & 0x01U));
    BSP_GPIO_Write(BSP_GPIO_GRAY_S1, (uint8_t)((ch >> 1) & 0x01U));
    BSP_GPIO_Write(BSP_GPIO_GRAY_S2, (uint8_t)((ch >> 2) & 0x01U));
}

static uint16_t Gray_Filter(uint16_t old_value, uint16_t new_value)
{
#if DRV_GRAY_4051_FILTER_SHIFT == 0
    (void)old_value;
    return new_value;
#else
    int32_t diff = (int32_t)new_value - (int32_t)old_value;
    return (uint16_t)((int32_t)old_value + (diff >> DRV_GRAY_4051_FILTER_SHIFT));
#endif
}

void Drv_Gray4051_Init(void)
{
    uint8_t i;

    for (i = 0U; i < DRV_GRAY_4051_CHANNEL_NUM; i++) {
        s_gray.raw[i] = 0U;
        s_gray.filt[i] = 0U;
    }

    s_gray.scan_index = 0U;
    s_gray.valid = 0U;
    s_phase = 0U;
    s_select_tick = BSP_GET_TICK();

    Gray_SelectChannel(0U);
}

void Drv_Gray4051_Update(void)
{
    uint8_t real_index;
    uint16_t raw;

    if (s_phase == 0U) {
        Gray_SelectChannel(s_gray.scan_index);
        s_select_tick = BSP_GET_TICK();
        s_phase = 1U;
        return;
    }

    if ((BSP_GET_TICK() - s_select_tick) < DRV_GRAY_4051_SETTLE_MS) {
        return;
    }

    real_index = Gray_MapIndex(s_gray.scan_index);
    raw = BSP_ADC_GetRaw(DRV_GRAY_4051_ADC_CH);
    s_gray.raw[real_index] = raw;

    if (s_gray.valid == 0U) {
        s_gray.filt[real_index] = raw;
    } else {
        s_gray.filt[real_index] = Gray_Filter(s_gray.filt[real_index], raw);
    }

    s_gray.scan_index++;
    if (s_gray.scan_index >= DRV_GRAY_4051_CHANNEL_NUM) {
        s_gray.scan_index = 0U;
        s_gray.valid = 1U;
    }

    s_phase = 0U;
}

uint16_t Drv_Gray4051_GetRaw(uint8_t index)
{
    if (index >= DRV_GRAY_4051_CHANNEL_NUM) return 0U;
    return s_gray.raw[index];
}

uint16_t Drv_Gray4051_GetFilt(uint8_t index)
{
    if (index >= DRV_GRAY_4051_CHANNEL_NUM) return 0U;
    return s_gray.filt[index];
}

BSP_Status_t Drv_Gray4051_GetRawArray(uint16_t *out_buf, uint8_t max_count)
{
    uint8_t i;
    uint8_t n;

    if (out_buf == 0) return BSP_PARAM;
    n = (max_count < DRV_GRAY_4051_CHANNEL_NUM) ? max_count : DRV_GRAY_4051_CHANNEL_NUM;
    for (i = 0U; i < n; i++) {
        out_buf[i] = s_gray.raw[i];
    }
    return BSP_OK;
}

BSP_Status_t Drv_Gray4051_GetFiltArray(uint16_t *out_buf, uint8_t max_count)
{
    uint8_t i;
    uint8_t n;

    if (out_buf == 0) return BSP_PARAM;
    n = (max_count < DRV_GRAY_4051_CHANNEL_NUM) ? max_count : DRV_GRAY_4051_CHANNEL_NUM;
    for (i = 0U; i < n; i++) {
        out_buf[i] = s_gray.filt[i];
    }
    return BSP_OK;
}

BSP_Status_t Drv_Gray4051_GetInfo(Drv_Gray4051_Info_t *info)
{
    if (info == 0) return BSP_PARAM;
    *info = s_gray;
    return BSP_OK;
}

uint8_t Drv_Gray4051_IsValid(void)
{
    return s_gray.valid;
}
