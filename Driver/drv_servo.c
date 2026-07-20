#include "drv_servo.h"
#include "bsp_pwm.h"
#include "bsp_systick.h"

/*
 * 当前实测标定值。只有 Driver 层可以接触 PWM 脉宽，APP 使用归一化位置。
 * 实机机械限位尚未最终验收前，不应扩大以下范围。
 */
#define SERVO_HORIZONTAL_RIGHT_LIMIT_US    1000U
#define SERVO_HORIZONTAL_CENTER_US         1248U
#define SERVO_HORIZONTAL_LEFT_LIMIT_US     1500U

#define SERVO_PITCH_UP_LIMIT_US            1200U
#define SERVO_PITCH_CENTER_US              1450U
#define SERVO_PITCH_DOWN_LIMIT_US          1830U

/* 每 20 ms 最多改变 1 us，保持原实现的缓动速度。 */
#define SERVO_TASK_PERIOD_MS               20U
#define SERVO_MOVE_STEP_US                 1U

static uint16_t s_horizontal_pulse_us = SERVO_HORIZONTAL_CENTER_US;
static uint16_t s_pitch_pulse_us = SERVO_PITCH_CENTER_US;
static uint16_t s_horizontal_target_us = SERVO_HORIZONTAL_CENTER_US;
static uint16_t s_pitch_target_us = SERVO_PITCH_CENTER_US;
static Drv_Servo_Position_t s_current_position = {0, 0};
static Drv_Servo_Position_t s_target_position = {0, 0};
static uint32_t s_last_task_ms;

static int16_t Drv_Servo_LimitPosition(int16_t value)
{
    if (value < DRV_SERVO_POSITION_MIN_PERMILLE) {
        return DRV_SERVO_POSITION_MIN_PERMILLE;
    }
    if (value > DRV_SERVO_POSITION_MAX_PERMILLE) {
        return DRV_SERVO_POSITION_MAX_PERMILLE;
    }
    return value;
}

static uint16_t Drv_Servo_PositionToPulse(int16_t position,
                                          uint16_t negative_limit_us,
                                          uint16_t center_us,
                                          uint16_t positive_limit_us)
{
    uint32_t offset_us;

    position = Drv_Servo_LimitPosition(position);
    if (position < 0) {
        offset_us = ((uint32_t)(-position) *
                     (uint32_t)(center_us - negative_limit_us)) / 1000U;
        return (uint16_t)(center_us - offset_us);
    }

    offset_us = ((uint32_t)position *
                 (uint32_t)(positive_limit_us - center_us)) / 1000U;
    return (uint16_t)(center_us + offset_us);
}

static int16_t Drv_Servo_PulseToPosition(uint16_t pulse_us,
                                         uint16_t negative_limit_us,
                                         uint16_t center_us,
                                         uint16_t positive_limit_us)
{
    uint32_t position;

    if (pulse_us < center_us) {
        position = ((uint32_t)(center_us - pulse_us) * 1000U) /
                   (uint32_t)(center_us - negative_limit_us);
        return (int16_t)(-(int32_t)position);
    }

    position = ((uint32_t)(pulse_us - center_us) * 1000U) /
               (uint32_t)(positive_limit_us - center_us);
    return (int16_t)position;
}

static uint16_t Drv_Servo_MoveOneStep(uint16_t current, uint16_t target)
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

static void Drv_Servo_RefreshCurrentPosition(void)
{
    s_current_position.horizontal_permille = Drv_Servo_PulseToPosition(
        s_horizontal_pulse_us,
        SERVO_HORIZONTAL_RIGHT_LIMIT_US,
        SERVO_HORIZONTAL_CENTER_US,
        SERVO_HORIZONTAL_LEFT_LIMIT_US
    );
    s_current_position.pitch_permille = Drv_Servo_PulseToPosition(
        s_pitch_pulse_us,
        SERVO_PITCH_UP_LIMIT_US,
        SERVO_PITCH_CENTER_US,
        SERVO_PITCH_DOWN_LIMIT_US
    );
}

static BSP_Status_t Drv_Servo_WriteBoth(uint16_t horizontal_pulse_us,
                                        uint16_t pitch_pulse_us)
{
    BSP_Status_t status;

    status = BSP_PWM_SetCompare(
        BSP_PWM_SERVO_HORIZONTAL,
        horizontal_pulse_us
    );
    if (status != BSP_OK) {
        return status;
    }

    return BSP_PWM_SetCompare(BSP_PWM_SERVO_PITCH, pitch_pulse_us);
}

void Drv_Servo_Init(void)
{
    s_horizontal_pulse_us = SERVO_HORIZONTAL_CENTER_US;
    s_pitch_pulse_us = SERVO_PITCH_CENTER_US;
    s_horizontal_target_us = SERVO_HORIZONTAL_CENTER_US;
    s_pitch_target_us = SERVO_PITCH_CENTER_US;
    s_current_position.horizontal_permille = 0;
    s_current_position.pitch_permille = 0;
    s_target_position = s_current_position;
    s_last_task_ms = BSP_GetTickMs();

    (void)Drv_Servo_WriteBoth(
        s_horizontal_pulse_us,
        s_pitch_pulse_us
    );
}

void Drv_Servo_Task(void)
{
    uint16_t horizontal_next;
    uint16_t pitch_next;

    if (BSP_TimeElapsed(&s_last_task_ms, SERVO_TASK_PERIOD_MS) == 0U) {
        return;
    }

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

    if (Drv_Servo_IsCommandReached() != 0U) {
        s_current_position = s_target_position;
    } else {
        Drv_Servo_RefreshCurrentPosition();
    }
}

BSP_Status_t Drv_Servo_SetTargetPosition(
    const Drv_Servo_Position_t *position
)
{
    if (position == 0) {
        return BSP_PARAM;
    }

    s_target_position.horizontal_permille = Drv_Servo_LimitPosition(
        position->horizontal_permille
    );
    s_target_position.pitch_permille = Drv_Servo_LimitPosition(
        position->pitch_permille
    );

    s_horizontal_target_us = Drv_Servo_PositionToPulse(
        s_target_position.horizontal_permille,
        SERVO_HORIZONTAL_RIGHT_LIMIT_US,
        SERVO_HORIZONTAL_CENTER_US,
        SERVO_HORIZONTAL_LEFT_LIMIT_US
    );
    s_pitch_target_us = Drv_Servo_PositionToPulse(
        s_target_position.pitch_permille,
        SERVO_PITCH_UP_LIMIT_US,
        SERVO_PITCH_CENTER_US,
        SERVO_PITCH_DOWN_LIMIT_US
    );
    return BSP_OK;
}

BSP_Status_t Drv_Servo_SetImmediatePosition(
    const Drv_Servo_Position_t *position
)
{
    Drv_Servo_Position_t limited;
    uint16_t horizontal_pulse_us;
    uint16_t pitch_pulse_us;
    BSP_Status_t status;

    if (position == 0) {
        return BSP_PARAM;
    }

    limited.horizontal_permille = Drv_Servo_LimitPosition(
        position->horizontal_permille
    );
    limited.pitch_permille = Drv_Servo_LimitPosition(
        position->pitch_permille
    );
    horizontal_pulse_us = Drv_Servo_PositionToPulse(
        limited.horizontal_permille,
        SERVO_HORIZONTAL_RIGHT_LIMIT_US,
        SERVO_HORIZONTAL_CENTER_US,
        SERVO_HORIZONTAL_LEFT_LIMIT_US
    );
    pitch_pulse_us = Drv_Servo_PositionToPulse(
        limited.pitch_permille,
        SERVO_PITCH_UP_LIMIT_US,
        SERVO_PITCH_CENTER_US,
        SERVO_PITCH_DOWN_LIMIT_US
    );

    status = Drv_Servo_WriteBoth(horizontal_pulse_us, pitch_pulse_us);
    if (status != BSP_OK) {
        return status;
    }

    s_horizontal_pulse_us = horizontal_pulse_us;
    s_pitch_pulse_us = pitch_pulse_us;
    s_horizontal_target_us = horizontal_pulse_us;
    s_pitch_target_us = pitch_pulse_us;
    s_current_position = limited;
    s_target_position = limited;
    return BSP_OK;
}

void Drv_Servo_Center(void)
{
    const Drv_Servo_Position_t center = {0, 0};
    (void)Drv_Servo_SetTargetPosition(&center);
}

BSP_Status_t Drv_Servo_GetInfo(Drv_Servo_Info_t *info)
{
    if (info == 0) {
        return BSP_PARAM;
    }

    info->current = s_current_position;
    info->target = s_target_position;
    info->command_reached = Drv_Servo_IsCommandReached();
    return BSP_OK;
}

uint8_t Drv_Servo_IsCommandReached(void)
{
    return (uint8_t)(
        (s_horizontal_pulse_us == s_horizontal_target_us) &&
        (s_pitch_pulse_us == s_pitch_target_us)
    );
}
