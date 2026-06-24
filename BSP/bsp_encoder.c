#include "bsp_encoder.h"

/*
 * ============================================================================
 * 通用正交编码器 BSP 实现
 * ============================================================================
 * 本文件不写死 TIMx / GPIOx / AFx。
 * 所有硬件资源都来自 bsp_encoder.h 的配置宏。
 *
 * 关键原则：
 *   - BSP 层只输出原始 count、delta、speed_cps、total；
 *   - 不在这里做轮径换算，不在这里判断左前/右前；
 *   - 四轮映射、左右平均、单位换算放到 drv_encoder / odometer。
 */

typedef struct {
    TIM_TypeDef      *tim;
    BSP_ClockCmdFn_t  tim_clock_fn;
    uint32_t          tim_clock_mask;

    GPIO_TypeDef     *port_a;
    uint16_t          pin_a;
    uint8_t           pinsrc_a;

    GPIO_TypeDef     *port_b;
    uint16_t          pin_b;
    uint8_t           pinsrc_b;

    uint8_t           af;
    uint16_t          period;
    uint8_t           reverse;
} BSP_Encoder_Cfg_t;

typedef struct {
    uint16_t last_counter;
    int16_t  delta_count;
    int32_t  total_count;
    int32_t  speed_cps;
    uint32_t last_update_ms;
    uint32_t update_time_ms;
    uint8_t  initialized;
} BSP_Encoder_Runtime_t;

static const BSP_Encoder_Cfg_t s_enc_cfg[BSP_ENCODER_COUNT] = {
#if BSP_ENCODER_CH1_ENABLE
    [BSP_ENCODER_CH1] = {
        BSP_ENCODER_CH1_TIM,
        BSP_ENCODER_CH1_TIM_CLOCK_FN,
        BSP_ENCODER_CH1_TIM_CLOCK_MASK,
        BSP_ENCODER_CH1_GPIO_PORT_A,
        BSP_ENCODER_CH1_GPIO_PIN_A,
        BSP_ENCODER_CH1_GPIO_PINSRC_A,
        BSP_ENCODER_CH1_GPIO_PORT_B,
        BSP_ENCODER_CH1_GPIO_PIN_B,
        BSP_ENCODER_CH1_GPIO_PINSRC_B,
        BSP_ENCODER_CH1_GPIO_AF,
        BSP_ENCODER_CH1_PERIOD,
        BSP_ENCODER_CH1_REVERSE
    },
#endif
#if BSP_ENCODER_CH2_ENABLE
    [BSP_ENCODER_CH2] = {
        BSP_ENCODER_CH2_TIM,
        BSP_ENCODER_CH2_TIM_CLOCK_FN,
        BSP_ENCODER_CH2_TIM_CLOCK_MASK,
        BSP_ENCODER_CH2_GPIO_PORT_A,
        BSP_ENCODER_CH2_GPIO_PIN_A,
        BSP_ENCODER_CH2_GPIO_PINSRC_A,
        BSP_ENCODER_CH2_GPIO_PORT_B,
        BSP_ENCODER_CH2_GPIO_PIN_B,
        BSP_ENCODER_CH2_GPIO_PINSRC_B,
        BSP_ENCODER_CH2_GPIO_AF,
        BSP_ENCODER_CH2_PERIOD,
        BSP_ENCODER_CH2_REVERSE
    },
#endif
#if BSP_ENCODER_CH3_ENABLE
    [BSP_ENCODER_CH3] = {
        BSP_ENCODER_CH3_TIM,
        BSP_ENCODER_CH3_TIM_CLOCK_FN,
        BSP_ENCODER_CH3_TIM_CLOCK_MASK,
        BSP_ENCODER_CH3_GPIO_PORT_A,
        BSP_ENCODER_CH3_GPIO_PIN_A,
        BSP_ENCODER_CH3_GPIO_PINSRC_A,
        BSP_ENCODER_CH3_GPIO_PORT_B,
        BSP_ENCODER_CH3_GPIO_PIN_B,
        BSP_ENCODER_CH3_GPIO_PINSRC_B,
        BSP_ENCODER_CH3_GPIO_AF,
        BSP_ENCODER_CH3_PERIOD,
        BSP_ENCODER_CH3_REVERSE
    },
#endif
#if BSP_ENCODER_CH4_ENABLE
    [BSP_ENCODER_CH4] = {
        BSP_ENCODER_CH4_TIM,
        BSP_ENCODER_CH4_TIM_CLOCK_FN,
        BSP_ENCODER_CH4_TIM_CLOCK_MASK,
        BSP_ENCODER_CH4_GPIO_PORT_A,
        BSP_ENCODER_CH4_GPIO_PIN_A,
        BSP_ENCODER_CH4_GPIO_PINSRC_A,
        BSP_ENCODER_CH4_GPIO_PORT_B,
        BSP_ENCODER_CH4_GPIO_PIN_B,
        BSP_ENCODER_CH4_GPIO_PINSRC_B,
        BSP_ENCODER_CH4_GPIO_AF,
        BSP_ENCODER_CH4_PERIOD,
        BSP_ENCODER_CH4_REVERSE
    },
#endif
};

static volatile BSP_Encoder_Runtime_t s_enc_rt[BSP_ENCODER_COUNT];

void BSP_Encoder_Init(BSP_Encoder_Id_t id)
{
    const BSP_Encoder_Cfg_t *cfg;
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim_base;
    TIM_ICInitTypeDef tim_ic;

    if (id >= BSP_ENCODER_COUNT) {
        return;
    }

    cfg = &s_enc_cfg[id];

    BSP_GPIO_ClockEnable(cfg->port_a);
    BSP_GPIO_ClockEnable(cfg->port_b);
    cfg->tim_clock_fn(cfg->tim_clock_mask, ENABLE);

    GPIO_PinAFConfig(cfg->port_a, cfg->pinsrc_a, cfg->af);
    GPIO_PinAFConfig(cfg->port_b, cfg->pinsrc_b, cfg->af);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;

    gpio.GPIO_Pin = cfg->pin_a;
    GPIO_Init(cfg->port_a, &gpio);

    gpio.GPIO_Pin = cfg->pin_b;
    GPIO_Init(cfg->port_b, &gpio);

    TIM_DeInit(cfg->tim);

    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Prescaler     = 0U;
    tim_base.TIM_CounterMode   = TIM_CounterMode_Up;
    tim_base.TIM_Period        = cfg->period;
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(cfg->tim, &tim_base);

    /*
     * 编码器模式 TI12：CH1 和 CH2 都参与计数。
     * 极性默认 Rising。如果实际方向反了，不改极性，优先改 BSP_ENCODER_CHx_REVERSE。
     */
    TIM_EncoderInterfaceConfig(cfg->tim,
                               TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);

    /* 输入滤波：抑制编码器毛刺。若高速编码器丢脉冲，可把 Filter 改小。 */
    TIM_ICStructInit(&tim_ic);
    tim_ic.TIM_ICFilter = 6U;

    tim_ic.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(cfg->tim, &tim_ic);

    tim_ic.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(cfg->tim, &tim_ic);

    TIM_SetCounter(cfg->tim, 0U);
    TIM_ClearFlag(cfg->tim, TIM_FLAG_Update);
    TIM_Cmd(cfg->tim, ENABLE);

    s_enc_rt[id].last_counter   = 0U;
    s_enc_rt[id].delta_count    = 0;
    s_enc_rt[id].total_count    = 0;
    s_enc_rt[id].speed_cps      = 0;
    s_enc_rt[id].last_update_ms = BSP_GET_TICK();
    s_enc_rt[id].update_time_ms = s_enc_rt[id].last_update_ms;
    s_enc_rt[id].initialized    = 1U;
}

void BSP_Encoder_InitAll(void)
{
    BSP_Encoder_Id_t id;

    for (id = (BSP_Encoder_Id_t)0; id < BSP_ENCODER_COUNT; id = (BSP_Encoder_Id_t)(id + 1)) {
        BSP_Encoder_Init(id);
    }
}

void BSP_Encoder_Update(BSP_Encoder_Id_t id)
{
    const BSP_Encoder_Cfg_t *cfg;
    uint16_t now_cnt;
    int16_t delta;
    uint32_t now_ms;
    uint32_t dt_ms;

    if (id >= BSP_ENCODER_COUNT) {
        return;
    }

    if (s_enc_rt[id].initialized == 0U) {
        return;
    }

    cfg = &s_enc_cfg[id];

    now_cnt = (uint16_t)TIM_GetCounter(cfg->tim);

    /*
     * 16bit 差值自然处理溢出：
     * 例如 last=65530, now=5，now-last 转 int16_t 后仍能得到正确的小增量。
     */
    delta = (int16_t)(now_cnt - s_enc_rt[id].last_counter);
    s_enc_rt[id].last_counter = now_cnt;

    if (cfg->reverse) {
        delta = (int16_t)(-delta);
    }

    now_ms = BSP_GET_TICK();
    dt_ms = now_ms - s_enc_rt[id].last_update_ms;
    if (dt_ms == 0U) {
        dt_ms = BSP_ENCODER_UPDATE_PERIOD_MS;
    }

    s_enc_rt[id].delta_count = delta;
    s_enc_rt[id].total_count += delta;
    s_enc_rt[id].speed_cps = ((int32_t)delta * 1000L) / (int32_t)dt_ms;
    s_enc_rt[id].last_update_ms = now_ms;
    s_enc_rt[id].update_time_ms = now_ms;
}

void BSP_Encoder_UpdateAll(void)
{
    BSP_Encoder_Id_t id;

    for (id = (BSP_Encoder_Id_t)0; id < BSP_ENCODER_COUNT; id = (BSP_Encoder_Id_t)(id + 1)) {
        BSP_Encoder_Update(id);
    }
}

int16_t BSP_Encoder_GetDelta(BSP_Encoder_Id_t id)
{
    if (id >= BSP_ENCODER_COUNT) {
        return 0;
    }

    return s_enc_rt[id].delta_count;
}

int32_t BSP_Encoder_GetSpeedCps(BSP_Encoder_Id_t id)
{
    if (id >= BSP_ENCODER_COUNT) {
        return 0;
    }

    return s_enc_rt[id].speed_cps;
}

int32_t BSP_Encoder_GetTotal(BSP_Encoder_Id_t id)
{
    if (id >= BSP_ENCODER_COUNT) {
        return 0;
    }

    return s_enc_rt[id].total_count;
}

void BSP_Encoder_ClearTotal(BSP_Encoder_Id_t id)
{
    uint32_t primask;

    if (id >= BSP_ENCODER_COUNT) {
        return;
    }

    primask = BSP_EnterCritical();
    s_enc_rt[id].total_count = 0;
    s_enc_rt[id].delta_count = 0;
    s_enc_rt[id].speed_cps   = 0;
    s_enc_rt[id].last_counter = (uint16_t)TIM_GetCounter(s_enc_cfg[id].tim);
    BSP_ExitCritical(primask);
}

void BSP_Encoder_ClearAllTotal(void)
{
    BSP_Encoder_Id_t id;

    for (id = (BSP_Encoder_Id_t)0; id < BSP_ENCODER_COUNT; id = (BSP_Encoder_Id_t)(id + 1)) {
        BSP_Encoder_ClearTotal(id);
    }
}

BSP_Status_t BSP_Encoder_GetInfo(BSP_Encoder_Id_t id, BSP_Encoder_Info_t *info)
{
    if (id >= BSP_ENCODER_COUNT || info == 0) {
        return BSP_PARAM;
    }

    info->delta_count    = s_enc_rt[id].delta_count;
    info->total_count    = s_enc_rt[id].total_count;
    info->speed_cps      = s_enc_rt[id].speed_cps;
    info->update_time_ms = s_enc_rt[id].update_time_ms;

    return BSP_OK;
}
