#include "drv_servo.h"
#include "bsp_pwm.h"
#include "bsp_systick.h"

/*
 * S20F 180°舵机水平通道的标称全行程范围。
 * Driver层统一保存PWM脉宽、角度和归一化位置之间的映射。
 */
#define SERVO_HORIZONTAL_RIGHT_LIMIT_US    500U
#define SERVO_HORIZONTAL_CENTER_US         1500U
#define SERVO_HORIZONTAL_LEFT_LIMIT_US     2500U
#define SERVO_HORIZONTAL_MIN_ANGLE_DEG     0U
#define SERVO_HORIZONTAL_MAX_ANGLE_DEG     180U
#define SERVO_HORIZONTAL_MIN_ANGLE_X10     0U
#define SERVO_HORIZONTAL_MAX_ANGLE_X10     1800U

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

static uint16_t Drv_Servo_LimitHorizontalPulse(uint16_t pulse_us)
{
    if (pulse_us < SERVO_HORIZONTAL_RIGHT_LIMIT_US) {
        return SERVO_HORIZONTAL_RIGHT_LIMIT_US;
    }
    if (pulse_us > SERVO_HORIZONTAL_LEFT_LIMIT_US) {
        return SERVO_HORIZONTAL_LEFT_LIMIT_US;
    }
    return pulse_us;
}

static uint16_t Drv_Servo_LimitHorizontalAngle(uint16_t angle_deg)
{
    if (angle_deg > SERVO_HORIZONTAL_MAX_ANGLE_DEG) {
        return SERVO_HORIZONTAL_MAX_ANGLE_DEG;
    }
    return angle_deg;
}

static uint16_t Drv_Servo_LimitHorizontalAngleX10(uint16_t angle_x10)
{
    if (angle_x10 > SERVO_HORIZONTAL_MAX_ANGLE_X10) {
        return SERVO_HORIZONTAL_MAX_ANGLE_X10;
    }
    return angle_x10;
}

static uint16_t Drv_Servo_HorizontalAngleX10ToPulse(uint16_t angle_x10)
{
    uint32_t span_us;
    uint32_t pulse_us;

    angle_x10 = Drv_Servo_LimitHorizontalAngleX10(angle_x10);
    span_us = (uint32_t)(SERVO_HORIZONTAL_LEFT_LIMIT_US -
                         SERVO_HORIZONTAL_RIGHT_LIMIT_US);
    pulse_us = (uint32_t)SERVO_HORIZONTAL_RIGHT_LIMIT_US +
               (((uint32_t)angle_x10 * span_us) +
                (SERVO_HORIZONTAL_MAX_ANGLE_X10 / 2U)) /
                   SERVO_HORIZONTAL_MAX_ANGLE_X10;
    return (uint16_t)pulse_us;
}

static uint16_t Drv_Servo_HorizontalPulseToAngleX10(uint16_t pulse_us)
{
    uint32_t span_us;
    uint32_t angle_x10;

    pulse_us = Drv_Servo_LimitHorizontalPulse(pulse_us);
    span_us = (uint32_t)(SERVO_HORIZONTAL_LEFT_LIMIT_US -
                         SERVO_HORIZONTAL_RIGHT_LIMIT_US);
    angle_x10 = (((uint32_t)(pulse_us - SERVO_HORIZONTAL_RIGHT_LIMIT_US) *
                  SERVO_HORIZONTAL_MAX_ANGLE_X10) +
                 (span_us / 2U)) /
                span_us;
    return (uint16_t)angle_x10;
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

BSP_Status_t Drv_Servo_SetHorizontalPulseUs(uint16_t pulse_us)
{
    int16_t horizontal_position;
    BSP_Status_t status;

    pulse_us = Drv_Servo_LimitHorizontalPulse(pulse_us);
    status = BSP_PWM_SetCompare(BSP_PWM_SERVO_HORIZONTAL, pulse_us);
    if (status != BSP_OK) {
        return status;
    }

    horizontal_position = Drv_Servo_PulseToPosition(
        pulse_us,
        SERVO_HORIZONTAL_RIGHT_LIMIT_US,
        SERVO_HORIZONTAL_CENTER_US,
        SERVO_HORIZONTAL_LEFT_LIMIT_US
    );

    /*
     * 微秒标定命令是立即输出；当前值和缓动目标必须同步，
     * 防止Drv_Servo_Task()在下一周期把PF8拉回旧目标。
     */
    s_horizontal_pulse_us = pulse_us;
    s_horizontal_target_us = pulse_us;
    s_current_position.horizontal_permille = horizontal_position;
    s_target_position.horizontal_permille = horizontal_position;
    return BSP_OK;
}

uint16_t Drv_Servo_GetHorizontalPulseUs(void)
{
    return s_horizontal_pulse_us;
}

BSP_Status_t Drv_Servo_SetHorizontalAngleDeg(uint16_t angle_deg)
{
    angle_deg = Drv_Servo_LimitHorizontalAngle(angle_deg);
    return Drv_Servo_SetHorizontalAngleX10((uint16_t)(angle_deg * 10U));
}

uint16_t Drv_Servo_GetHorizontalAngleDeg(void)
{
    return (uint16_t)((Drv_Servo_GetHorizontalAngleX10() + 5U) / 10U);
}

BSP_Status_t Drv_Servo_SetHorizontalAngleX10(uint16_t angle_x10)
{
    return Drv_Servo_SetHorizontalPulseUs(
        Drv_Servo_HorizontalAngleX10ToPulse(angle_x10)
    );
}

uint16_t Drv_Servo_GetHorizontalAngleX10(void)
{
    return Drv_Servo_HorizontalPulseToAngleX10(s_horizontal_pulse_us);
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
    info->horizontal_pulse_us = s_horizontal_pulse_us;
    info->pitch_pulse_us = s_pitch_pulse_us;
    info->horizontal_angle_x10 = Drv_Servo_HorizontalPulseToAngleX10(
        s_horizontal_pulse_us
    );
    info->horizontal_target_angle_x10 = Drv_Servo_HorizontalPulseToAngleX10(
        s_horizontal_target_us
    );
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
