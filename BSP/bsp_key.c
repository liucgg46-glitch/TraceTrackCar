#include "bsp_key.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIOPuPd_TypeDef pupd;
    uint8_t active_level;
} BSP_Key_Cfg_t;

typedef struct {
    uint8_t stable_pressed;
    uint8_t last_raw_pressed;
    uint8_t pressed_event;
    uint8_t released_event;
    uint32_t last_change_ms;
} BSP_Key_Runtime_t;

static const BSP_Key_Cfg_t s_key_cfg[BSP_KEY_COUNT] = {
#if BSP_KEY1_ENABLE
    [BSP_KEY1] = {BSP_KEY1_PORT, BSP_KEY1_PIN, BSP_KEY1_PUPD, BSP_KEY1_ACTIVE_LEVEL},
#endif
#if BSP_KEY2_ENABLE
    [BSP_KEY2] = {BSP_KEY2_PORT, BSP_KEY2_PIN, BSP_KEY2_PUPD, BSP_KEY2_ACTIVE_LEVEL},
#endif
#if BSP_KEY3_ENABLE
    [BSP_KEY3] = {BSP_KEY3_PORT, BSP_KEY3_PIN, BSP_KEY3_PUPD, BSP_KEY3_ACTIVE_LEVEL},
#endif
#if BSP_KEY4_ENABLE
    [BSP_KEY4] = {BSP_KEY4_PORT, BSP_KEY4_PIN, BSP_KEY4_PUPD, BSP_KEY4_ACTIVE_LEVEL},
#endif
};

static volatile BSP_Key_Runtime_t s_key_rt[BSP_KEY_COUNT];

static uint8_t Key_ReadRawPressed(BSP_Key_Id_t id)
{
    uint8_t level;
    if (id >= BSP_KEY_COUNT) return 0U;
    level = (GPIO_ReadInputDataBit(s_key_cfg[id].port, s_key_cfg[id].pin) != Bit_RESET) ? 1U : 0U;
    return (level == s_key_cfg[id].active_level) ? 1U : 0U;
}

void BSP_Key_Init(BSP_Key_Id_t id)
{
    GPIO_InitTypeDef gpio;
    uint8_t raw;

    if (id >= BSP_KEY_COUNT) return;

    BSP_GPIO_ClockEnable(s_key_cfg[id].port);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin   = s_key_cfg[id].pin;
    gpio.GPIO_Mode  = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = s_key_cfg[id].pupd;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(s_key_cfg[id].port, &gpio);

    raw = Key_ReadRawPressed(id);
    s_key_rt[id].stable_pressed = raw;
    s_key_rt[id].last_raw_pressed = raw;
    s_key_rt[id].pressed_event = 0U;
    s_key_rt[id].released_event = 0U;
    s_key_rt[id].last_change_ms = BSP_GET_TICK();
}

void BSP_Key_InitAll(void)
{
    BSP_Key_Id_t id;
    for (id = (BSP_Key_Id_t)0; id < BSP_KEY_COUNT; id = (BSP_Key_Id_t)(id + 1)) {
        BSP_Key_Init(id);
    }
}

void BSP_Key_Update(BSP_Key_Id_t id)
{
    uint8_t raw;
    uint32_t now;

    if (id >= BSP_KEY_COUNT) return;

    raw = Key_ReadRawPressed(id);
    now = BSP_GET_TICK();

    if (raw != s_key_rt[id].last_raw_pressed) {
        s_key_rt[id].last_raw_pressed = raw;
        s_key_rt[id].last_change_ms = now;
        return;
    }

    if ((uint32_t)(now - s_key_rt[id].last_change_ms) >= BSP_KEY_DEBOUNCE_MS) {
        if (raw != s_key_rt[id].stable_pressed) {
            s_key_rt[id].stable_pressed = raw;
            if (raw) {
                s_key_rt[id].pressed_event = 1U;
            } else {
                s_key_rt[id].released_event = 1U;
            }
        }
    }
}

void BSP_Key_UpdateAll(void)
{
    BSP_Key_Id_t id;
    for (id = (BSP_Key_Id_t)0; id < BSP_KEY_COUNT; id = (BSP_Key_Id_t)(id + 1)) {
        BSP_Key_Update(id);
    }
}

uint8_t BSP_Key_IsPressed(BSP_Key_Id_t id)
{
    if (id >= BSP_KEY_COUNT) return 0U;
    return s_key_rt[id].stable_pressed;
}

uint8_t BSP_Key_WasPressed(BSP_Key_Id_t id)
{
    uint8_t ret;
    uint32_t primask;

    if (id >= BSP_KEY_COUNT) return 0U;
    primask = BSP_EnterCritical();
    ret = s_key_rt[id].pressed_event;
    s_key_rt[id].pressed_event = 0U;
    BSP_ExitCritical(primask);
    return ret;
}

uint8_t BSP_Key_WasReleased(BSP_Key_Id_t id)
{
    uint8_t ret;
    uint32_t primask;

    if (id >= BSP_KEY_COUNT) return 0U;
    primask = BSP_EnterCritical();
    ret = s_key_rt[id].released_event;
    s_key_rt[id].released_event = 0U;
    BSP_ExitCritical(primask);
    return ret;
}

BSP_KeyEvent_t BSP_Key_GetEvent(BSP_Key_Id_t id)
{
    if (BSP_Key_WasPressed(id)) return BSP_KEY_EVENT_PRESSED;
    if (BSP_Key_WasReleased(id)) return BSP_KEY_EVENT_RELEASED;
    return BSP_KEY_EVENT_NONE;
}
