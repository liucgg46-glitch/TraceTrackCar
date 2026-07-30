#include "ball_balance_app.h"

#include "bsp_systick.h"
#include "drv_servo.h"
#include "project_critical.h"

#define BALL_BALANCE_APP_POSITION_ABS_MAX_MM_X10     1200
#define BALL_BALANCE_HOLD_MAX_MS                     150U

static uint8_t s_initialized;
static uint8_t s_enabled;
static uint8_t s_pending_sample;
static uint8_t s_has_sequence;
static uint8_t s_last_sequence;
static uint8_t s_last_sample_state;
static uint8_t s_last_sample_valid;
static uint8_t s_low_confidence;
static uint8_t s_position_out_of_range;
static uint8_t s_duplicate_sequence;
static uint8_t s_hold_active;
static uint8_t s_hold_expired;
static uint8_t s_has_valid_measurement;
static uint8_t s_servo_fault;
static uint32_t s_last_valid_sample_ms;
static uint32_t s_last_hold_sample_ms;
static uint32_t s_pushed_sample_count;
static uint32_t s_consumed_sample_count;
static uint32_t s_rejected_sample_count;
static uint32_t s_valid_sample_count;
static uint32_t s_hold_sample_count;
static uint32_t s_lost_sample_count;
static BallBalance_VisionSample_t s_pending_vision_sample;
static BallBalance_VisionSample_t s_last_vision_sample;
static BSP_Status_t s_last_servo_status = BSP_OK;

static uint32_t BallBalance_App_IncrementU32(uint32_t value)
{
    return (value < 0xFFFFFFFFUL) ? (value + 1UL) : value;
}

static int16_t BallBalance_App_AbsI16(int16_t value)
{
    if (value >= 0) {
        return value;
    }
    if (value == (int16_t)-32768) {
        return 32767;
    }
    return (int16_t)(-value);
}

static uint8_t BallBalance_App_IsPositionInRange(int16_t position_mm_x10)
{
    return (BallBalance_App_AbsI16(position_mm_x10) <=
            BALL_BALANCE_APP_POSITION_ABS_MAX_MM_X10) ? 1U : 0U;
}

static BSP_Status_t BallBalance_App_OutputNeutral(void)
{
    return Drv_Servo_SetHorizontalAngleX10(
        BALL_BALANCE_NEUTRAL_ANGLE_X10
    );
}

static void BallBalance_App_ClearVisionState(void)
{
    uint32_t primask;

    primask = Project_EnterCritical();
    s_pending_sample = 0U;
    s_has_sequence = 0U;
    s_last_sequence = 0U;
    s_last_sample_state = BALL_BALANCE_VISION_LOST;
    s_last_sample_valid = 0U;
    s_low_confidence = 0U;
    s_position_out_of_range = 0U;
    s_duplicate_sequence = 0U;
    s_hold_active = 0U;
    s_hold_expired = 0U;
    s_has_valid_measurement = 0U;
    s_last_valid_sample_ms = 0U;
    s_last_hold_sample_ms = 0U;
    s_pending_vision_sample.position_mm_x10 = 0;
    s_pending_vision_sample.state = BALL_BALANCE_VISION_LOST;
    s_pending_vision_sample.valid = 0U;
    s_pending_vision_sample.confidence = 0U;
    s_pending_vision_sample.sequence = 0U;
    s_pending_vision_sample.timestamp_ms = 0U;
    s_last_vision_sample = s_pending_vision_sample;
    Project_ExitCritical(primask);
}

static uint8_t BallBalance_App_NormalizeSample(
    const BallBalance_VisionSample_t *sample,
    BallBalance_VisionSample_t *checked_sample,
    uint8_t *low_confidence,
    uint8_t *position_out_of_range,
    uint8_t *hold_expired
)
{
    uint8_t sample_valid;
    uint32_t hold_age_ms;

    *checked_sample = *sample;
    *low_confidence = 0U;
    *position_out_of_range = 0U;
    *hold_expired = 0U;
    sample_valid = 0U;

    if (checked_sample->timestamp_ms == 0U) {
        checked_sample->timestamp_ms = BSP_GetTickMs();
    }

    switch (sample->state) {
        case BALL_BALANCE_VISION_VALID:
            checked_sample->valid = 1U;
            if (sample->confidence < BALL_BALANCE_MIN_CONFIDENCE) {
                checked_sample->state = BALL_BALANCE_VISION_LOST;
                checked_sample->valid = 0U;
                *low_confidence = 1U;
                break;
            }
            if (BallBalance_App_IsPositionInRange(
                    sample->position_mm_x10
                ) == 0U) {
                checked_sample->state = BALL_BALANCE_VISION_LOST;
                checked_sample->valid = 0U;
                *position_out_of_range = 1U;
                break;
            }
            sample_valid = 1U;
            break;

        case BALL_BALANCE_VISION_HOLD:
            checked_sample->valid = 1U;
            if (s_has_valid_measurement == 0U) {
                checked_sample->state = BALL_BALANCE_VISION_LOST;
                checked_sample->valid = 0U;
                break;
            }
            if (BallBalance_App_IsPositionInRange(
                    sample->position_mm_x10
                ) == 0U) {
                checked_sample->state = BALL_BALANCE_VISION_LOST;
                checked_sample->valid = 0U;
                *position_out_of_range = 1U;
                break;
            }

            hold_age_ms =
                checked_sample->timestamp_ms - s_last_valid_sample_ms;
            if (hold_age_ms > BALL_BALANCE_HOLD_MAX_MS) {
                checked_sample->state = BALL_BALANCE_VISION_LOST;
                checked_sample->valid = 0U;
                *hold_expired = 1U;
                break;
            }

            sample_valid = 1U;
            break;

        case BALL_BALANCE_VISION_LOST:
        default:
            checked_sample->state = BALL_BALANCE_VISION_LOST;
            checked_sample->valid = 0U;
            break;
    }

    return sample_valid;
}

void BallBalance_App_Init(void)
{
    BallBalance_Control_Init();
    BallBalance_App_ClearVisionState();
    s_enabled = 0U;
    s_servo_fault = 0U;
    s_pushed_sample_count = 0U;
    s_consumed_sample_count = 0U;
    s_rejected_sample_count = 0U;
    s_valid_sample_count = 0U;
    s_hold_sample_count = 0U;
    s_lost_sample_count = 0U;
    s_last_servo_status = BallBalance_App_OutputNeutral();
    s_initialized = 1U;
}

void BallBalance_App_Update(void)
{
    BallBalance_VisionSample_t sample;
    BallBalance_ControlInfo_t control_info;
    uint8_t has_sample;
    uint32_t primask;
    uint32_t now_ms;
    uint16_t command_angle_x10;

    if (s_initialized == 0U) {
        return;
    }

    now_ms = BSP_GetTickMs();
    has_sample = 0U;
    primask = Project_EnterCritical();
    if (s_pending_sample != 0U) {
        sample = s_pending_vision_sample;
        s_pending_sample = 0U;
        has_sample = 1U;
    }
    Project_ExitCritical(primask);

    if (has_sample != 0U) {
        s_consumed_sample_count = BallBalance_App_IncrementU32(
            s_consumed_sample_count
        );
        if (((sample.state == BALL_BALANCE_VISION_VALID) ||
             (sample.state == BALL_BALANCE_VISION_HOLD)) &&
            (sample.valid != 0U)) {
            BallBalance_Control_PushMeasurement(
                sample.position_mm_x10,
                sample.sequence,
                sample.timestamp_ms
            );
        } else {
            BallBalance_Control_InvalidateMeasurement(sample.timestamp_ms);
        }
    }

    BallBalance_Control_Update(now_ms);
    if (BallBalance_Control_GetInfo(&control_info) != PROJECT_OK) {
        return;
    }

    if ((s_enabled == 0U) ||
        (control_info.enabled == 0U) ||
        (control_info.measurement_valid == 0U) ||
        (control_info.data_timeout != 0U)) {
        command_angle_x10 = BALL_BALANCE_NEUTRAL_ANGLE_X10;
    } else {
        command_angle_x10 = control_info.command_angle_x10;
    }

    s_last_servo_status = Drv_Servo_SetHorizontalAngleX10(command_angle_x10);
    if (s_last_servo_status != BSP_OK) {
        s_servo_fault = 1U;
        s_enabled = 0U;
        BallBalance_Control_SetEnabled(0U);
        (void)BallBalance_App_OutputNeutral();
    }
}

void BallBalance_App_Enable(void)
{
    if (s_initialized == 0U) {
        return;
    }

    s_enabled = 1U;
    s_servo_fault = 0U;
    BallBalance_Control_SetEnabled(1U);
}

void BallBalance_App_Disable(void)
{
    if (s_initialized == 0U) {
        return;
    }

    s_enabled = 0U;
    BallBalance_Control_SetEnabled(0U);
    s_last_servo_status = BallBalance_App_OutputNeutral();
}

uint8_t BallBalance_App_IsEnabled(void)
{
    return s_enabled;
}

void BallBalance_App_SetTargetMmX10(int16_t target_mm_x10)
{
    BallBalance_Control_SetTargetMmX10(target_mm_x10);
}

void BallBalance_App_SetGains(float kp, float ki, float kd)
{
    BallBalance_Control_SetGains(kp, ki, kd);
}

void BallBalance_App_PushVisionSample(
    const BallBalance_VisionSample_t *sample
)
{
    BallBalance_VisionSample_t checked_sample;
    uint8_t sample_valid;
    uint8_t low_confidence;
    uint8_t position_out_of_range;
    uint8_t hold_expired;
    uint32_t primask;

    if (sample == 0) {
        return;
    }

    sample_valid = BallBalance_App_NormalizeSample(
        sample,
        &checked_sample,
        &low_confidence,
        &position_out_of_range,
        &hold_expired
    );

    primask = Project_EnterCritical();
    if ((s_has_sequence != 0U) &&
        (checked_sample.sequence == s_last_sequence)) {
        s_duplicate_sequence = 1U;
        Project_ExitCritical(primask);
        return;
    }

    s_pending_vision_sample = checked_sample;
    s_last_vision_sample = checked_sample;
    s_pending_sample = 1U;
    s_last_sequence = checked_sample.sequence;
    s_has_sequence = 1U;
    s_last_sample_state = checked_sample.state;
    s_last_sample_valid = sample_valid;
    s_low_confidence = low_confidence;
    s_position_out_of_range = position_out_of_range;
    s_duplicate_sequence = 0U;
    s_hold_expired = hold_expired;
    s_pushed_sample_count = BallBalance_App_IncrementU32(
        s_pushed_sample_count
    );

    if ((checked_sample.state == BALL_BALANCE_VISION_VALID) &&
        (sample_valid != 0U)) {
        s_has_valid_measurement = 1U;
        s_hold_active = 0U;
        s_last_valid_sample_ms = checked_sample.timestamp_ms;
        s_valid_sample_count = BallBalance_App_IncrementU32(
            s_valid_sample_count
        );
    } else if ((checked_sample.state == BALL_BALANCE_VISION_HOLD) &&
               (sample_valid != 0U)) {
        s_hold_active = 1U;
        s_last_hold_sample_ms = checked_sample.timestamp_ms;
        s_hold_sample_count = BallBalance_App_IncrementU32(
            s_hold_sample_count
        );
    } else {
        s_hold_active = 0U;
        s_lost_sample_count = BallBalance_App_IncrementU32(
            s_lost_sample_count
        );
    }

    if (sample_valid == 0U) {
        s_rejected_sample_count = BallBalance_App_IncrementU32(
            s_rejected_sample_count
        );
    }
    Project_ExitCritical(primask);
}

BSP_Status_t BallBalance_App_GetInfo(BallBalance_AppInfo_t *info)
{
    uint32_t primask;

    if (info == 0) {
        return BSP_PARAM;
    }

    info->initialized = s_initialized;
    info->enabled = s_enabled;
    info->servo_fault = s_servo_fault;
    info->last_servo_status = s_last_servo_status;

    primask = Project_EnterCritical();
    info->pending_sample = s_pending_sample;
    info->last_sample_state = s_last_sample_state;
    info->last_sample_valid = s_last_sample_valid;
    info->low_confidence = s_low_confidence;
    info->position_out_of_range = s_position_out_of_range;
    info->duplicate_sequence = s_duplicate_sequence;
    info->hold_active = s_hold_active;
    info->hold_expired = s_hold_expired;
    info->last_valid_sample_ms = s_last_valid_sample_ms;
    info->last_hold_sample_ms = s_last_hold_sample_ms;
    info->pushed_sample_count = s_pushed_sample_count;
    info->consumed_sample_count = s_consumed_sample_count;
    info->rejected_sample_count = s_rejected_sample_count;
    info->valid_sample_count = s_valid_sample_count;
    info->hold_sample_count = s_hold_sample_count;
    info->lost_sample_count = s_lost_sample_count;
    info->last_sample = s_last_vision_sample;
    Project_ExitCritical(primask);

    if (BallBalance_Control_GetInfo(&info->control) != PROJECT_OK) {
        return BSP_ERROR;
    }
    return BSP_OK;
}
