#include "bsp_adc.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t adc_channel;
    uint8_t sample_time;
} BSP_ADC_Cfg_t;

static const BSP_ADC_Cfg_t s_adc_cfg[BSP_ADC_CH_COUNT] = {
#if BSP_ADC_CH1_ENABLE
    [BSP_ADC_CH1] = {BSP_ADC_CH1_GPIO_PORT, BSP_ADC_CH1_GPIO_PIN, BSP_ADC_CH1_ADC_CHANNEL, BSP_ADC_CH1_SAMPLE_TIME},
#endif
#if BSP_ADC_CH2_ENABLE
    [BSP_ADC_CH2] = {BSP_ADC_CH2_GPIO_PORT, BSP_ADC_CH2_GPIO_PIN, BSP_ADC_CH2_ADC_CHANNEL, BSP_ADC_CH2_SAMPLE_TIME},
#endif
#if BSP_ADC_CH3_ENABLE
    [BSP_ADC_CH3] = {BSP_ADC_CH3_GPIO_PORT, BSP_ADC_CH3_GPIO_PIN, BSP_ADC_CH3_ADC_CHANNEL, BSP_ADC_CH3_SAMPLE_TIME},
#endif
#if BSP_ADC_CH4_ENABLE
    [BSP_ADC_CH4] = {BSP_ADC_CH4_GPIO_PORT, BSP_ADC_CH4_GPIO_PIN, BSP_ADC_CH4_ADC_CHANNEL, BSP_ADC_CH4_SAMPLE_TIME},
#endif
#if BSP_ADC_CH5_ENABLE
    [BSP_ADC_CH5] = {BSP_ADC_CH5_GPIO_PORT, BSP_ADC_CH5_GPIO_PIN, BSP_ADC_CH5_ADC_CHANNEL, BSP_ADC_CH5_SAMPLE_TIME},
#endif
#if BSP_ADC_CH6_ENABLE
    [BSP_ADC_CH6] = {BSP_ADC_CH6_GPIO_PORT, BSP_ADC_CH6_GPIO_PIN, BSP_ADC_CH6_ADC_CHANNEL, BSP_ADC_CH6_SAMPLE_TIME},
#endif
#if BSP_ADC_CH7_ENABLE
    [BSP_ADC_CH7] = {BSP_ADC_CH7_GPIO_PORT, BSP_ADC_CH7_GPIO_PIN, BSP_ADC_CH7_ADC_CHANNEL, BSP_ADC_CH7_SAMPLE_TIME},
#endif
#if BSP_ADC_CH8_ENABLE
    [BSP_ADC_CH8] = {BSP_ADC_CH8_GPIO_PORT, BSP_ADC_CH8_GPIO_PIN, BSP_ADC_CH8_ADC_CHANNEL, BSP_ADC_CH8_SAMPLE_TIME},
#endif
};

static volatile uint16_t s_adc_raw[BSP_ADC_CH_COUNT];

void BSP_ADC_Init(void)
{
    GPIO_InitTypeDef gpio;
    ADC_InitTypeDef adc;
    ADC_CommonInitTypeDef adc_common;
    DMA_InitTypeDef dma;
    uint8_t i;

    RCC_APB2PeriphClockCmd(BSP_ADC_CLOCK_MASK, ENABLE);
    RCC_AHB1PeriphClockCmd(BSP_ADC_DMA_CLOCK_MASK, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_AN;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;

    for (i = 0U; i < BSP_ADC_CH_COUNT; i++) {
        BSP_GPIO_ClockEnable(s_adc_cfg[i].port);
        gpio.GPIO_Pin = s_adc_cfg[i].pin;
        GPIO_Init(s_adc_cfg[i].port, &gpio);
    }

    DMA_Cmd(BSP_ADC_DMA_STREAM, DISABLE);
    (void)BSP_DMA_WaitDisable(BSP_ADC_DMA_STREAM, 100000UL);
    DMA_DeInit(BSP_ADC_DMA_STREAM);

    DMA_StructInit(&dma);
    dma.DMA_Channel            = BSP_ADC_DMA_CHANNEL;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&BSP_ADC_PERIPH->DR;
    dma.DMA_Memory0BaseAddr    = (uint32_t)s_adc_raw;
    dma.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize         = BSP_ADC_CH_COUNT;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode               = DMA_Mode_Circular;
    dma.DMA_Priority           = DMA_Priority_High;
    dma.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_Init(BSP_ADC_DMA_STREAM, &dma);
    DMA_Cmd(BSP_ADC_DMA_STREAM, ENABLE);

    ADC_CommonStructInit(&adc_common);
    adc_common.ADC_Mode             = ADC_Mode_Independent;
    adc_common.ADC_Prescaler        = ADC_Prescaler_Div4;
    adc_common.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
    adc_common.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&adc_common);

    ADC_DeInit();
    ADC_StructInit(&adc);
    adc.ADC_Resolution           = BSP_ADC_RESOLUTION;
    adc.ADC_ScanConvMode         = ENABLE;
    adc.ADC_ContinuousConvMode   = ENABLE;
    adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    adc.ADC_DataAlign            = ADC_DataAlign_Right;
    adc.ADC_NbrOfConversion      = BSP_ADC_CH_COUNT;
    ADC_Init(BSP_ADC_PERIPH, &adc);

    for (i = 0U; i < BSP_ADC_CH_COUNT; i++) {
        ADC_RegularChannelConfig(BSP_ADC_PERIPH,
                                 s_adc_cfg[i].adc_channel,
                                 (uint8_t)(i + 1U),
                                 s_adc_cfg[i].sample_time);
    }

    ADC_DMARequestAfterLastTransferCmd(BSP_ADC_PERIPH, ENABLE);
    ADC_DMACmd(BSP_ADC_PERIPH, ENABLE);
    ADC_Cmd(BSP_ADC_PERIPH, ENABLE);
    ADC_SoftwareStartConv(BSP_ADC_PERIPH);
}

uint16_t BSP_ADC_GetRaw(BSP_ADC_Ch_t ch)
{
    if (ch >= BSP_ADC_CH_COUNT) return 0U;
    return s_adc_raw[ch];
}

BSP_Status_t BSP_ADC_GetRawArray(uint16_t *out_buf, uint8_t max_count, uint8_t *out_count)
{
    uint8_t i;
    uint8_t n;

    if (out_buf == 0 || out_count == 0) return BSP_PARAM;

    n = (max_count < BSP_ADC_CH_COUNT) ? max_count : BSP_ADC_CH_COUNT;
    for (i = 0U; i < n; i++) {
        out_buf[i] = s_adc_raw[i];
    }
    *out_count = n;
    return BSP_OK;
}

void BSP_ADC_Start(void)
{
    ADC_Cmd(BSP_ADC_PERIPH, ENABLE);
    ADC_SoftwareStartConv(BSP_ADC_PERIPH);
}

void BSP_ADC_Stop(void)
{
    ADC_Cmd(BSP_ADC_PERIPH, DISABLE);
}
