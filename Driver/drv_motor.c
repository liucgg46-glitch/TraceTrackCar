#include "drv_motor.h"

typedef struct {
    BSP_PWM_Id_t  pwm_id;
    BSP_GPIO_Id_t in1_gpio;
    BSP_GPIO_Id_t in2_gpio;
    uint8_t       reverse;
} Motor_Cfg_t;

static const Motor_Cfg_t s_motor_cfg[MOTOR_COUNT] = {
    [MOTOR_FL] = {MOTOR_FL_PWM_ID, MOTOR_FL_IN1_GPIO, MOTOR_FL_IN2_GPIO, MOTOR_FL_REVERSE},
    [MOTOR_FR] = {MOTOR_FR_PWM_ID, MOTOR_FR_IN1_GPIO, MOTOR_FR_IN2_GPIO, MOTOR_FR_REVERSE},
    [MOTOR_RL] = {MOTOR_RL_PWM_ID, MOTOR_RL_IN1_GPIO, MOTOR_RL_IN2_GPIO, MOTOR_RL_REVERSE},
    [MOTOR_RR] = {MOTOR_RR_PWM_ID, MOTOR_RR_IN1_GPIO, MOTOR_RR_IN2_GPIO, MOTOR_RR_REVERSE},
};

static int16_t s_motor_last[MOTOR_COUNT];

static int16_t Motor_LimitPermille(int16_t x)
{
    if (x > MOTOR_MAX_ABS_PERMILLE) return MOTOR_MAX_ABS_PERMILLE;
    if (x < -MOTOR_MAX_ABS_PERMILLE) return -MOTOR_MAX_ABS_PERMILLE;
    return x;
}

void Motor_Init(void)
{
    Motor_StopAll();
}

void Motor_SetPermille(Motor_Id_t id, int16_t pwm_permille)
{
    const Motor_Cfg_t *cfg;
    uint16_t duty;

    if (id >= MOTOR_COUNT) return;
    cfg = &s_motor_cfg[id];

    pwm_permille = Motor_LimitPermille(pwm_permille);
    if (cfg->reverse) {
        pwm_permille = (int16_t)(-pwm_permille);
    }

    s_motor_last[id] = pwm_permille;

    if (pwm_permille > 0) {
        BSP_GPIO_Write(cfg->in1_gpio, 1U);
        BSP_GPIO_Write(cfg->in2_gpio, 0U);
        duty = (uint16_t)pwm_permille;
    } else if (pwm_permille < 0) {
        BSP_GPIO_Write(cfg->in1_gpio, 0U);
        BSP_GPIO_Write(cfg->in2_gpio, 1U);
        duty = (uint16_t)(-pwm_permille);
    } else {
        /* 0 输出时采用滑行停止：IN1=0, IN2=0, PWM=0。 */
        BSP_GPIO_Write(cfg->in1_gpio, 0U);
        BSP_GPIO_Write(cfg->in2_gpio, 0U);
        duty = 0U;
    }

    (void)BSP_PWM_SetDutyPermille(cfg->pwm_id, duty);
}

void Motor_SetAllPermille(int16_t fl, int16_t fr, int16_t rl, int16_t rr)
{
    Motor_SetPermille(MOTOR_FL, fl);
    Motor_SetPermille(MOTOR_FR, fr);
    Motor_SetPermille(MOTOR_RL, rl);
    Motor_SetPermille(MOTOR_RR, rr);
}

void Motor_StopOne(Motor_Id_t id)
{
    Motor_SetPermille(id, 0);
}

void Motor_StopAll(void)
{
    Motor_SetAllPermille(0, 0, 0, 0);
}

int16_t Motor_GetLastPermille(Motor_Id_t id)
{
    if (id >= MOTOR_COUNT) return 0;
    return s_motor_last[id];
}

BSP_Status_t Motor_GetAllLastPermille(int16_t out_pwm[MOTOR_COUNT])
{
    uint8_t i;
    if (out_pwm == 0) return BSP_PARAM;
    for (i = 0U; i < MOTOR_COUNT; i++) {
        out_pwm[i] = s_motor_last[i];
    }
    return BSP_OK;
}

void Motor_SetPWM(int16_t left_pwm, int16_t right_pwm)
{
    Motor_SetAllPermille(left_pwm, right_pwm, left_pwm, right_pwm);
}

void Motor_Stop(void)
{
    Motor_StopAll();
}
