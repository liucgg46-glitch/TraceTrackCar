#include "drv_servo.h"
#include "bsp_pwm.h"

static uint16_t s_horizontal_pulse_us =
    SERVO_HORIZONTAL_CENTER_US;

static uint16_t s_pitch_pulse_us =
    SERVO_PITCH_CENTER_US;

static uint16_t s_horizontal_target_us =
    SERVO_HORIZONTAL_CENTER_US;

static uint16_t s_pitch_target_us =
    SERVO_PITCH_CENTER_US;

static uint16_t Drv_Servo_LimitU16(uint16_t value,
                                   uint16_t min_value,
                                   uint16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static uint16_t Drv_Servo_MoveOneStep(uint16_t current,
                                      uint16_t target)
{
    if (current < target) {
        if ((uint16_t)(target - current) <= SERVO_MOVE_STEP_US) {
            return target;
        }

        return (uint16_t)(current + SERVO_MOVE_STEP_US);
    }

    if (current > target) {
        if ((uint16_t)(current - target) <= SERVO_MOVE_STEP_US) {
            return target;
        }

        return (uint16_t)(current - SERVO_MOVE_STEP_US);
    }

    return current;
}

void Drv_Servo_Init(void)
{
    /*
     * PF8/TIM13和PF9/TIM14已经由BSP_InitAll()初始化。
     * 这里立即写入中位，避免舵机收到0us脉宽。
     */
    s_horizontal_pulse_us = SERVO_HORIZONTAL_CENTER_US;
    s_pitch_pulse_us = SERVO_PITCH_CENTER_US;

    s_horizontal_target_us = SERVO_HORIZONTAL_CENTER_US;
    s_pitch_target_us = SERVO_PITCH_CENTER_US;

    (void)BSP_PWM_SetCompare(
        BSP_PWM_SERVO_HORIZONTAL,
        s_horizontal_pulse_us
    );

    (void)BSP_PWM_SetCompare(
        BSP_PWM_SERVO_PITCH,
        s_pitch_pulse_us
    );
}

void Drv_Servo_Center(void)
{
    Drv_Servo_SetTarget(
        SERVO_HORIZONTAL_CENTER_US,
        SERVO_PITCH_CENTER_US
    );
}

void Drv_Servo_SetHorizontalPulse(uint16_t pulse_us)
{
    pulse_us = Drv_Servo_LimitU16(
        pulse_us,
        SERVO_HORIZONTAL_RIGHT_LIMIT_US,
        SERVO_HORIZONTAL_LEFT_LIMIT_US
    );

    s_horizontal_pulse_us = pulse_us;
    s_horizontal_target_us = pulse_us;

    (void)BSP_PWM_SetCompare(
        BSP_PWM_SERVO_HORIZONTAL,
        pulse_us
    );
}

void Drv_Servo_SetPitchPulse(uint16_t pulse_us)
{
    pulse_us = Drv_Servo_LimitU16(
        pulse_us,
        SERVO_PITCH_UP_LIMIT_US,
        SERVO_PITCH_DOWN_LIMIT_US
    );

    s_pitch_pulse_us = pulse_us;
    s_pitch_target_us = pulse_us;

    (void)BSP_PWM_SetCompare(
        BSP_PWM_SERVO_PITCH,
        pulse_us
    );
}

void Drv_Servo_SetTarget(uint16_t horizontal_target_us,
                         uint16_t pitch_target_us)
{
    s_horizontal_target_us = Drv_Servo_LimitU16(
        horizontal_target_us,
        SERVO_HORIZONTAL_RIGHT_LIMIT_US,
        SERVO_HORIZONTAL_LEFT_LIMIT_US
    );

    s_pitch_target_us = Drv_Servo_LimitU16(
        pitch_target_us,
        SERVO_PITCH_UP_LIMIT_US,
        SERVO_PITCH_DOWN_LIMIT_US
    );
}

void Drv_Servo_Update(void)
{
    uint16_t horizontal_next;
    uint16_t pitch_next;

    horizontal_next = Drv_Servo_MoveOneStep(
        s_horizontal_pulse_us,
        s_horizontal_target_us
    );

    pitch_next = Drv_Servo_MoveOneStep(
        s_pitch_pulse_us,
        s_pitch_target_us
    );

    if (horizontal_next != s_horizontal_pulse_us) {
        s_horizontal_pulse_us = horizontal_next;

        (void)BSP_PWM_SetCompare(
            BSP_PWM_SERVO_HORIZONTAL,
            s_horizontal_pulse_us
        );
    }

    if (pitch_next != s_pitch_pulse_us) {
        s_pitch_pulse_us = pitch_next;

        (void)BSP_PWM_SetCompare(
            BSP_PWM_SERVO_PITCH,
            s_pitch_pulse_us
        );
    }
}

uint16_t Drv_Servo_GetHorizontalPulse(void)
{
    return s_horizontal_pulse_us;
}

uint16_t Drv_Servo_GetPitchPulse(void)
{
    return s_pitch_pulse_us;
}

uint16_t Drv_Servo_GetHorizontalTarget(void)
{
    return s_horizontal_target_us;
}

uint16_t Drv_Servo_GetPitchTarget(void)
{
    return s_pitch_target_us;
}

uint8_t Drv_Servo_IsAtTarget(void)
{
    return (uint8_t)(
        (s_horizontal_pulse_us == s_horizontal_target_us) &&
        (s_pitch_pulse_us == s_pitch_target_us)
    );
}