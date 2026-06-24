#include "bsp_exti.h"
#include "misc.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t port_source;
    uint8_t pin_source;
    uint32_t line;
    IRQn_Type irqn;
    EXTITrigger_TypeDef trigger;
    GPIOPuPd_TypeDef pupd;
} BSP_EXTI_Cfg_t;

typedef struct {
    BSP_EXTI_Callback_t cb;
    void *ctx;
} BSP_EXTI_Runtime_t;

#if BSP_EXTI_ANY_ENABLE
static const BSP_EXTI_Cfg_t s_exti_cfg[BSP_EXTI_COUNT] = {
#if BSP_EXTI_CH1_ENABLE
    [BSP_EXTI_CH1] = {BSP_EXTI_CH1_PORT, BSP_EXTI_CH1_PIN, BSP_EXTI_CH1_PORT_SOURCE, BSP_EXTI_CH1_PIN_SOURCE, BSP_EXTI_CH1_LINE, BSP_EXTI_CH1_IRQn, BSP_EXTI_CH1_TRIGGER, BSP_EXTI_CH1_PUPD},
#endif
#if BSP_EXTI_CH2_ENABLE
    [BSP_EXTI_CH2] = {BSP_EXTI_CH2_PORT, BSP_EXTI_CH2_PIN, BSP_EXTI_CH2_PORT_SOURCE, BSP_EXTI_CH2_PIN_SOURCE, BSP_EXTI_CH2_LINE, BSP_EXTI_CH2_IRQn, BSP_EXTI_CH2_TRIGGER, BSP_EXTI_CH2_PUPD},
#endif
#if BSP_EXTI_CH3_ENABLE
    [BSP_EXTI_CH3] = {BSP_EXTI_CH3_PORT, BSP_EXTI_CH3_PIN, BSP_EXTI_CH3_PORT_SOURCE, BSP_EXTI_CH3_PIN_SOURCE, BSP_EXTI_CH3_LINE, BSP_EXTI_CH3_IRQn, BSP_EXTI_CH3_TRIGGER, BSP_EXTI_CH3_PUPD},
#endif
};
static volatile BSP_EXTI_Runtime_t s_exti_rt[BSP_EXTI_COUNT];
#endif

void BSP_EXTI_Init(BSP_EXTI_Id_t id)
{
#if BSP_EXTI_ANY_ENABLE
    GPIO_InitTypeDef gpio;
    EXTI_InitTypeDef exti;
    NVIC_InitTypeDef nvic;
    const BSP_EXTI_Cfg_t *cfg;

    if (id >= BSP_EXTI_COUNT) return;
    cfg = &s_exti_cfg[id];

    BSP_GPIO_ClockEnable(cfg->port);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin   = cfg->pin;
    gpio.GPIO_Mode  = GPIO_Mode_IN;
    gpio.GPIO_PuPd  = cfg->pupd;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(cfg->port, &gpio);

    SYSCFG_EXTILineConfig(cfg->port_source, cfg->pin_source);

    EXTI_StructInit(&exti);
    exti.EXTI_Line    = cfg->line;
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = cfg->trigger;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    nvic.NVIC_IRQChannel = cfg->irqn;
    nvic.NVIC_IRQChannelPreemptionPriority = BSP_EXTI_IRQ_PREEMPT_PRIO;
    nvic.NVIC_IRQChannelSubPriority = BSP_EXTI_IRQ_SUB_PRIO;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
#else
    (void)id;
#endif
}

void BSP_EXTI_InitAll(void)
{
#if BSP_EXTI_ANY_ENABLE
    BSP_EXTI_Id_t id;
    for (id = (BSP_EXTI_Id_t)0; id < BSP_EXTI_COUNT; id = (BSP_EXTI_Id_t)(id + 1)) {
        BSP_EXTI_Init(id);
    }
#endif
}

BSP_Status_t BSP_EXTI_AttachCallback(BSP_EXTI_Id_t id, BSP_EXTI_Callback_t cb, void *ctx)
{
#if BSP_EXTI_ANY_ENABLE
    if (id >= BSP_EXTI_COUNT) return BSP_PARAM;
    s_exti_rt[id].cb = cb;
    s_exti_rt[id].ctx = ctx;
    return BSP_OK;
#else
    (void)id; (void)cb; (void)ctx;
    return BSP_PARAM;
#endif
}

void BSP_EXTI_DispatchIRQ(uint32_t exti_line)
{
#if BSP_EXTI_ANY_ENABLE
    BSP_EXTI_Id_t id;
    for (id = (BSP_EXTI_Id_t)0; id < BSP_EXTI_COUNT; id = (BSP_EXTI_Id_t)(id + 1)) {
        if (s_exti_cfg[id].line == exti_line) {
            if (EXTI_GetITStatus(exti_line) != RESET) {
                EXTI_ClearITPendingBit(exti_line);
                if (s_exti_rt[id].cb) {
                    s_exti_rt[id].cb(s_exti_rt[id].ctx);
                }
            }
            return;
        }
    }
#else
    (void)exti_line;
#endif
}

#if BSP_EXTI_CH1_ENABLE
void EXTI0_IRQHandler(void) { BSP_EXTI_DispatchIRQ(EXTI_Line0); }
#endif
#if BSP_EXTI_CH2_ENABLE
void EXTI1_IRQHandler(void) { BSP_EXTI_DispatchIRQ(EXTI_Line1); }
#endif
#if BSP_EXTI_CH3_ENABLE
void EXTI2_IRQHandler(void) { BSP_EXTI_DispatchIRQ(EXTI_Line2); }
#endif
