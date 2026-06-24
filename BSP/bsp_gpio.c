#include "bsp_gpio.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIOMode_TypeDef mode;
    GPIOOType_TypeDef otype;
    GPIOPuPd_TypeDef pupd;
    GPIOSpeed_TypeDef speed;
    uint8_t init_level;
} BSP_GPIO_Cfg_t;

static const BSP_GPIO_Cfg_t s_gpio_cfg[BSP_GPIO_COUNT] = {
#if BSP_GPIO_CH1_ENABLE
    [BSP_GPIO_CH1] = {BSP_GPIO_CH1_PORT, BSP_GPIO_CH1_PIN, BSP_GPIO_CH1_MODE, BSP_GPIO_CH1_OTYPE, BSP_GPIO_CH1_PUPD, BSP_GPIO_CH1_SPEED, BSP_GPIO_CH1_INIT_LEVEL},
#endif
#if BSP_GPIO_CH2_ENABLE
    [BSP_GPIO_CH2] = {BSP_GPIO_CH2_PORT, BSP_GPIO_CH2_PIN, BSP_GPIO_CH2_MODE, BSP_GPIO_CH2_OTYPE, BSP_GPIO_CH2_PUPD, BSP_GPIO_CH2_SPEED, BSP_GPIO_CH2_INIT_LEVEL},
#endif
#if BSP_GPIO_CH3_ENABLE
    [BSP_GPIO_CH3] = {BSP_GPIO_CH3_PORT, BSP_GPIO_CH3_PIN, BSP_GPIO_CH3_MODE, BSP_GPIO_CH3_OTYPE, BSP_GPIO_CH3_PUPD, BSP_GPIO_CH3_SPEED, BSP_GPIO_CH3_INIT_LEVEL},
#endif
#if BSP_GPIO_CH4_ENABLE
    [BSP_GPIO_CH4] = {BSP_GPIO_CH4_PORT, BSP_GPIO_CH4_PIN, BSP_GPIO_CH4_MODE, BSP_GPIO_CH4_OTYPE, BSP_GPIO_CH4_PUPD, BSP_GPIO_CH4_SPEED, BSP_GPIO_CH4_INIT_LEVEL},
#endif
#if BSP_GPIO_CH5_ENABLE
    [BSP_GPIO_CH5] = {BSP_GPIO_CH5_PORT, BSP_GPIO_CH5_PIN, BSP_GPIO_CH5_MODE, BSP_GPIO_CH5_OTYPE, BSP_GPIO_CH5_PUPD, BSP_GPIO_CH5_SPEED, BSP_GPIO_CH5_INIT_LEVEL},
#endif
#if BSP_GPIO_CH6_ENABLE
    [BSP_GPIO_CH6] = {BSP_GPIO_CH6_PORT, BSP_GPIO_CH6_PIN, BSP_GPIO_CH6_MODE, BSP_GPIO_CH6_OTYPE, BSP_GPIO_CH6_PUPD, BSP_GPIO_CH6_SPEED, BSP_GPIO_CH6_INIT_LEVEL},
#endif
#if BSP_GPIO_CH7_ENABLE
    [BSP_GPIO_CH7] = {BSP_GPIO_CH7_PORT, BSP_GPIO_CH7_PIN, BSP_GPIO_CH7_MODE, BSP_GPIO_CH7_OTYPE, BSP_GPIO_CH7_PUPD, BSP_GPIO_CH7_SPEED, BSP_GPIO_CH7_INIT_LEVEL},
#endif
#if BSP_GPIO_CH8_ENABLE
    [BSP_GPIO_CH8] = {BSP_GPIO_CH8_PORT, BSP_GPIO_CH8_PIN, BSP_GPIO_CH8_MODE, BSP_GPIO_CH8_OTYPE, BSP_GPIO_CH8_PUPD, BSP_GPIO_CH8_SPEED, BSP_GPIO_CH8_INIT_LEVEL},
#endif
#if BSP_GPIO_CH9_ENABLE
    [BSP_GPIO_CH9] = {BSP_GPIO_CH9_PORT, BSP_GPIO_CH9_PIN, BSP_GPIO_CH9_MODE, BSP_GPIO_CH9_OTYPE, BSP_GPIO_CH9_PUPD, BSP_GPIO_CH9_SPEED, BSP_GPIO_CH9_INIT_LEVEL},
#endif
#if BSP_GPIO_CH10_ENABLE
    [BSP_GPIO_CH10] = {BSP_GPIO_CH10_PORT, BSP_GPIO_CH10_PIN, BSP_GPIO_CH10_MODE, BSP_GPIO_CH10_OTYPE, BSP_GPIO_CH10_PUPD, BSP_GPIO_CH10_SPEED, BSP_GPIO_CH10_INIT_LEVEL},
#endif
#if BSP_GPIO_CH11_ENABLE
    [BSP_GPIO_CH11] = {BSP_GPIO_CH11_PORT, BSP_GPIO_CH11_PIN, BSP_GPIO_CH11_MODE, BSP_GPIO_CH11_OTYPE, BSP_GPIO_CH11_PUPD, BSP_GPIO_CH11_SPEED, BSP_GPIO_CH11_INIT_LEVEL},
#endif
#if BSP_GPIO_CH12_ENABLE
    [BSP_GPIO_CH12] = {BSP_GPIO_CH12_PORT, BSP_GPIO_CH12_PIN, BSP_GPIO_CH12_MODE, BSP_GPIO_CH12_OTYPE, BSP_GPIO_CH12_PUPD, BSP_GPIO_CH12_SPEED, BSP_GPIO_CH12_INIT_LEVEL},
#endif
#if BSP_GPIO_CH13_ENABLE
    [BSP_GPIO_CH13] = {BSP_GPIO_CH13_PORT, BSP_GPIO_CH13_PIN, BSP_GPIO_CH13_MODE, BSP_GPIO_CH13_OTYPE, BSP_GPIO_CH13_PUPD, BSP_GPIO_CH13_SPEED, BSP_GPIO_CH13_INIT_LEVEL},
#endif
};

void BSP_GPIO_Init(BSP_GPIO_Id_t id)
{
    GPIO_InitTypeDef gpio;
    const BSP_GPIO_Cfg_t *cfg;

    if (id >= BSP_GPIO_COUNT) return;
    cfg = &s_gpio_cfg[id];

    BSP_GPIO_ClockEnable(cfg->port);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin   = cfg->pin;
    gpio.GPIO_Mode  = cfg->mode;
    gpio.GPIO_OType = cfg->otype;
    gpio.GPIO_PuPd  = cfg->pupd;
    gpio.GPIO_Speed = cfg->speed;
    GPIO_Init(cfg->port, &gpio);

    if (cfg->mode == GPIO_Mode_OUT) {
        BSP_GPIO_Write(id, cfg->init_level);
    }
}

void BSP_GPIO_InitAll(void)
{
    BSP_GPIO_Id_t id;
    for (id = (BSP_GPIO_Id_t)0; id < BSP_GPIO_COUNT; id = (BSP_GPIO_Id_t)(id + 1)) {
        BSP_GPIO_Init(id);
    }
}

void BSP_GPIO_Write(BSP_GPIO_Id_t id, uint8_t level)
{
    const BSP_GPIO_Cfg_t *cfg;
    if (id >= BSP_GPIO_COUNT) return;
    cfg = &s_gpio_cfg[id];

    if (level) {
        GPIO_SetBits(cfg->port, cfg->pin);
    } else {
        GPIO_ResetBits(cfg->port, cfg->pin);
    }
}

void BSP_GPIO_Toggle(BSP_GPIO_Id_t id)
{
    if (BSP_GPIO_Read(id)) {
        BSP_GPIO_Write(id, 0U);
    } else {
        BSP_GPIO_Write(id, 1U);
    }
}

uint8_t BSP_GPIO_Read(BSP_GPIO_Id_t id)
{
    const BSP_GPIO_Cfg_t *cfg;
    if (id >= BSP_GPIO_COUNT) return 0U;
    cfg = &s_gpio_cfg[id];
    return (GPIO_ReadInputDataBit(cfg->port, cfg->pin) != Bit_RESET) ? 1U : 0U;
}

GPIO_TypeDef *BSP_GPIO_GetPort(BSP_GPIO_Id_t id)
{
    if (id >= BSP_GPIO_COUNT) return 0;
    return s_gpio_cfg[id].port;
}

uint16_t BSP_GPIO_GetPin(BSP_GPIO_Id_t id)
{
    if (id >= BSP_GPIO_COUNT) return 0U;
    return s_gpio_cfg[id].pin;
}
