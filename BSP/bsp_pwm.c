#include "bsp_pwm.h"

typedef struct {
    TIM_TypeDef *tim;
    BSP_ClockCmdFn_t tim_clock_fn;
    uint32_t tim_clock_mask;
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin;
    uint8_t gpio_pinsrc;
    uint8_t gpio_af;
    uint8_t channel;
    uint16_t prescaler;
    uint16_t period;
    uint16_t init_compare;
    uint8_t active_high;
} BSP_PWM_Cfg_t;

static const BSP_PWM_Cfg_t s_pwm_cfg[BSP_PWM_COUNT] = {
#if BSP_PWM_CH1_ENABLE
    [BSP_PWM_CH1] = {BSP_PWM_CH1_TIM, BSP_PWM_CH1_TIM_CLOCK_FN, BSP_PWM_CH1_TIM_CLOCK_MASK, BSP_PWM_CH1_GPIO_PORT, BSP_PWM_CH1_GPIO_PIN, BSP_PWM_CH1_GPIO_PINSRC, BSP_PWM_CH1_GPIO_AF, BSP_PWM_CH1_CHANNEL, BSP_PWM_CH1_PRESCALER, BSP_PWM_CH1_PERIOD, BSP_PWM_CH1_INIT_COMPARE, BSP_PWM_CH1_ACTIVE_HIGH},
#endif
#if BSP_PWM_CH2_ENABLE
    [BSP_PWM_CH2] = {BSP_PWM_CH2_TIM, BSP_PWM_CH2_TIM_CLOCK_FN, BSP_PWM_CH2_TIM_CLOCK_MASK, BSP_PWM_CH2_GPIO_PORT, BSP_PWM_CH2_GPIO_PIN, BSP_PWM_CH2_GPIO_PINSRC, BSP_PWM_CH2_GPIO_AF, BSP_PWM_CH2_CHANNEL, BSP_PWM_CH2_PRESCALER, BSP_PWM_CH2_PERIOD, BSP_PWM_CH2_INIT_COMPARE, BSP_PWM_CH2_ACTIVE_HIGH},
#endif
#if BSP_PWM_CH3_ENABLE
    [BSP_PWM_CH3] = {BSP_PWM_CH3_TIM, BSP_PWM_CH3_TIM_CLOCK_FN, BSP_PWM_CH3_TIM_CLOCK_MASK, BSP_PWM_CH3_GPIO_PORT, BSP_PWM_CH3_GPIO_PIN, BSP_PWM_CH3_GPIO_PINSRC, BSP_PWM_CH3_GPIO_AF, BSP_PWM_CH3_CHANNEL, BSP_PWM_CH3_PRESCALER, BSP_PWM_CH3_PERIOD, BSP_PWM_CH3_INIT_COMPARE, BSP_PWM_CH3_ACTIVE_HIGH},
#endif
#if BSP_PWM_CH4_ENABLE
    [BSP_PWM_CH4] = {BSP_PWM_CH4_TIM, BSP_PWM_CH4_TIM_CLOCK_FN, BSP_PWM_CH4_TIM_CLOCK_MASK, BSP_PWM_CH4_GPIO_PORT, BSP_PWM_CH4_GPIO_PIN, BSP_PWM_CH4_GPIO_PINSRC, BSP_PWM_CH4_GPIO_AF, BSP_PWM_CH4_CHANNEL, BSP_PWM_CH4_PRESCALER, BSP_PWM_CH4_PERIOD, BSP_PWM_CH4_INIT_COMPARE, BSP_PWM_CH4_ACTIVE_HIGH},
#endif
};

static uint8_t PWM_TimerAlreadyInited(TIM_TypeDef *tim, BSP_PWM_Id_t cur)
{
    BSP_PWM_Id_t i;
    for (i = (BSP_PWM_Id_t)0; i < cur; i = (BSP_PWM_Id_t)(i + 1)) {
        if (s_pwm_cfg[i].tim == tim) return 1U;
    }
    return 0U;
}

static void PWM_ConfigChannel(const BSP_PWM_Cfg_t *cfg)
{
    TIM_OCInitTypeDef oc;
    uint16_t polarity = cfg->active_high ? TIM_OCPolarity_High : TIM_OCPolarity_Low;

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = cfg->init_compare;
    oc.TIM_OCPolarity  = polarity;

    switch (cfg->channel) {
        case 1U: TIM_OC1Init(cfg->tim, &oc); TIM_OC1PreloadConfig(cfg->tim, TIM_OCPreload_Enable); break;
        case 2U: TIM_OC2Init(cfg->tim, &oc); TIM_OC2PreloadConfig(cfg->tim, TIM_OCPreload_Enable); break;
        case 3U: TIM_OC3Init(cfg->tim, &oc); TIM_OC3PreloadConfig(cfg->tim, TIM_OCPreload_Enable); break;
        case 4U: TIM_OC4Init(cfg->tim, &oc); TIM_OC4PreloadConfig(cfg->tim, TIM_OCPreload_Enable); break;
        default: break;
    }
}

void BSP_PWM_Init(BSP_PWM_Id_t id)
{
    const BSP_PWM_Cfg_t *cfg;
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;

    if (id >= BSP_PWM_COUNT) return;
    cfg = &s_pwm_cfg[id];

    BSP_GPIO_ClockEnable(cfg->gpio_port);
    cfg->tim_clock_fn(cfg->tim_clock_mask, ENABLE);

    GPIO_PinAFConfig(cfg->gpio_port, cfg->gpio_pinsrc, cfg->gpio_af);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin   = cfg->gpio_pin;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(cfg->gpio_port, &gpio);

    if (!PWM_TimerAlreadyInited(cfg->tim, id)) {
        TIM_TimeBaseStructInit(&tim);
        tim.TIM_Prescaler         = cfg->prescaler;
        tim.TIM_CounterMode       = TIM_CounterMode_Up;
        tim.TIM_Period            = cfg->period;
        tim.TIM_ClockDivision     = TIM_CKD_DIV1;
        tim.TIM_RepetitionCounter = 0;
        TIM_TimeBaseInit(cfg->tim, &tim);
        TIM_ARRPreloadConfig(cfg->tim, ENABLE);
    }

    PWM_ConfigChannel(cfg);
    TIM_Cmd(cfg->tim, ENABLE);

    if (cfg->tim == TIM1 || cfg->tim == TIM8) {
        TIM_CtrlPWMOutputs(cfg->tim, ENABLE);
    }
}

void BSP_PWM_InitAll(void)
{
    BSP_PWM_Id_t id;
    for (id = (BSP_PWM_Id_t)0; id < BSP_PWM_COUNT; id = (BSP_PWM_Id_t)(id + 1)) {
        BSP_PWM_Init(id);
    }
}

BSP_Status_t BSP_PWM_SetCompare(BSP_PWM_Id_t id, uint16_t compare)
{
    const BSP_PWM_Cfg_t *cfg;
    if (id >= BSP_PWM_COUNT) return BSP_PARAM;
    cfg = &s_pwm_cfg[id];

    if (compare > cfg->period) compare = cfg->period;

    switch (cfg->channel) {
        case 1U: TIM_SetCompare1(cfg->tim, compare); break;
        case 2U: TIM_SetCompare2(cfg->tim, compare); break;
        case 3U: TIM_SetCompare3(cfg->tim, compare); break;
        case 4U: TIM_SetCompare4(cfg->tim, compare); break;
        default: return BSP_PARAM;
    }

    return BSP_OK;
}

BSP_Status_t BSP_PWM_SetDutyPermille(BSP_PWM_Id_t id, uint16_t permille)
{
    uint32_t compare;
    if (id >= BSP_PWM_COUNT) return BSP_PARAM;
    if (permille > 1000U) permille = 1000U;

    compare = ((uint32_t)(s_pwm_cfg[id].period + 1U) * permille) / 1000U;
    if (compare > 0U) compare -= 1U;
    return BSP_PWM_SetCompare(id, (uint16_t)compare);
}

uint16_t BSP_PWM_GetPeriod(BSP_PWM_Id_t id)
{
    if (id >= BSP_PWM_COUNT) return 0U;
    return s_pwm_cfg[id].period;
}

void BSP_PWM_Start(BSP_PWM_Id_t id)
{
    if (id >= BSP_PWM_COUNT) return;
    TIM_Cmd(s_pwm_cfg[id].tim, ENABLE);
}

void BSP_PWM_Stop(BSP_PWM_Id_t id)
{
    if (id >= BSP_PWM_COUNT) return;
    (void)BSP_PWM_SetCompare(id, 0U);
}
