#include "ball_balance_app.h"

#include "ball_balance_config.h"
#include "ball_equilibrium_map.h"
#include "bsp_systick.h"
#include "drv_servo.h"
#include "project_critical.h"

static uint8_t s_initialized;
static uint8_t s_enabled;
static BallBalance_AppState_t s_state;
static uint8_t s_pending_sample;
static uint8_t s_has_sequence;
static uint8_t s_last_sequence;
static uint8_t s_last_sample_state;
static uint8_t s_last_sample_valid;
static uint8_t s_low_confidence;
static uint8_t s_position_out_of_range;
static uint8_t s_duplicate_sequence;
static uint8_t s_tracking_ready;
static uint8_t s_valid_streak;
static uint8_t s_estimator_reject_streak;
static uint8_t s_ever_active;
static uint8_t s_data_timeout;
static uint8_t s_settled;
static uint8_t s_servo_fault;
static uint8_t s_vehicle_feedforward_enabled;
static uint8_t s_vehicle_disturbance_valid;
static int16_t s_target_mm_x10;
static float s_vehicle_disturbance_mm_s2;
static float s_equilibrium_angle_deg;
static float s_last_applied_dynamic_angle_deg;
static uint32_t s_vehicle_disturbance_timestamp_ms;
static uint32_t s_last_valid_sample_ms;
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

static float BallBalance_App_AbsF(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static int16_t BallBalance_App_LimitTargetX10(int16_t target_mm_x10)
{
    if (target_mm_x10 > BALL_BALANCE_POSITION_ABS_MAX_MM_X10) {
        return BALL_BALANCE_POSITION_ABS_MAX_MM_X10;
    }
    if (target_mm_x10 < -BALL_BALANCE_POSITION_ABS_MAX_MM_X10) {
        return -BALL_BALANCE_POSITION_ABS_MAX_MM_X10;
    }
    return target_mm_x10;
}

static uint8_t BallBalance_App_IsPositionInRange(
    int16_t position_mm_x10
)
{
    int32_t position = position_mm_x10;

    if (position < 0) {
        position = -position;
    }
    return (position <= BALL_BALANCE_POSITION_ABS_MAX_MM_X10) ?
           1U : 0U;
}

static void BallBalance_App_ClearSamples(void)
{
    uint32_t primask = Project_EnterCritical();

    s_pending_sample = 0U;
    s_has_sequence = 0U;
    s_last_sequence = 0U;
    s_last_sample_state = BALL_BALANCE_VISION_LOST;
    s_last_sample_valid = 0U;
    s_low_confidence = 0U;
    s_position_out_of_range = 0U;
    s_duplicate_sequence = 0U;
    s_pending_vision_sample.position_mm_x10 = 0;
    s_pending_vision_sample.state = BALL_BALANCE_VISION_LOST;
    s_pending_vision_sample.valid = 0U;
    s_pending_vision_sample.confidence = 0U;
    s_pending_vision_sample.sequence = 0U;
    s_pending_vision_sample.timestamp_ms = 0U;
    s_last_vision_sample = s_pending_vision_sample;
    Project_ExitCritical(primask);
}

static uint8_t BallBalance_App_TakeSample(
    BallBalance_VisionSample_t *sample
)
{
    uint8_t available = 0U;
    uint32_t primask = Project_EnterCritical();

    if (s_pending_sample != 0U) {
        *sample = s_pending_vision_sample;
        s_pending_sample = 0U;
        available = 1U;
    }
    Project_ExitCritical(primask);
    return available;
}

static void BallBalance_App_AcquireTracking(float position_mm)
{
    BallStateEstimator_Reset(position_mm);
    BallReference_Init(position_mm);
    BallReference_SetTargetMm((float)s_target_mm_x10 * 0.1f);
    BallReference_Pause();

    /*
     * 每次重新获得视觉跟踪都清除闭环内部记忆，避免超时前的目标速度、
     * 速度滤波或积分角在恢复后重新生效。
     */
    BallBalance_Control_Reset();
    s_last_applied_dynamic_angle_deg = 0.0f;

    s_tracking_ready = 1U;
    s_data_timeout = 0U;
    s_ever_active = 1U;
    s_estimator_reject_streak = 0U;
}

static void BallBalance_App_ResetEstimatorAtMeasurement(
    float position_mm
)
{
    BallStateEstimator_Reset(position_mm);
    /*
     * 这里只修正估计器，不重置参考轨迹。
     * 边缘救球时如果把参考也重置到边缘，反馈会短暂变小，舵机会显得慢半拍。
     */
    s_last_applied_dynamic_angle_deg = 0.0f;
    s_estimator_reject_streak = 0U;
}

static uint8_t BallBalance_App_GetVehicleDisturbance(
    uint32_t now_ms,
    float *disturbance_mm_s2
)
{
    if ((s_vehicle_feedforward_enabled == 0U) ||
        (s_vehicle_disturbance_valid == 0U) ||
        ((uint32_t)(now_ms - s_vehicle_disturbance_timestamp_ms) >
         BALL_VEHICLE_IMU_TIMEOUT_MS)) {
        *disturbance_mm_s2 = 0.0f;
        return 0U;
    }

    *disturbance_mm_s2 = s_vehicle_disturbance_mm_s2;
    return 1U;
}

void BallBalance_App_Init(void)
{
    BallBalance_App_ClearSamples();
    s_initialized = 0U;
    s_enabled = 0U;
    s_state = BALL_BALANCE_APP_DISABLED;
    s_tracking_ready = 0U;
    s_valid_streak = 0U;
    s_estimator_reject_streak = 0U;
    s_ever_active = 0U;
    s_data_timeout = 0U;
    s_settled = 0U;
    s_servo_fault = 0U;
    s_vehicle_feedforward_enabled = 0U;
    s_vehicle_disturbance_valid = 0U;
    s_target_mm_x10 = 0;
    s_vehicle_disturbance_mm_s2 = 0.0f;
    s_equilibrium_angle_deg = BALL_BALANCE_LEVEL_ANGLE_DEG;
    s_last_applied_dynamic_angle_deg = 0.0f;
    s_vehicle_disturbance_timestamp_ms = 0U;
    s_last_valid_sample_ms = 0U;
    s_pushed_sample_count = 0U;
    s_consumed_sample_count = 0U;
    s_rejected_sample_count = 0U;
    s_valid_sample_count = 0U;
    s_hold_sample_count = 0U;
    s_lost_sample_count = 0U;
    s_last_servo_status =
        Drv_Servo_SetHorizontalAngleX10(
            BALL_BALANCE_LEVEL_ANGLE_X10
        );
    if (s_last_servo_status != BSP_OK) {
        s_servo_fault = 1U;
        s_state = BALL_BALANCE_APP_FAULT;
    }
    s_initialized = 1U;
}

void BallBalance_App_Enable(void)
{
    if ((s_initialized == 0U) || (s_servo_fault != 0U)) {
        return;
    }
    s_enabled = 1U;
    s_settled = 0U;
}

void BallBalance_App_Disable(void)
{
    if (s_initialized == 0U) {
        return;
    }
    s_enabled = 0U;
    s_settled = 0U;
    BallReference_Pause();
}

uint8_t BallBalance_App_IsEnabled(void)
{
    return s_enabled;
}

void BallBalance_App_SetTargetMmX10(int16_t target_mm_x10)
{
    s_target_mm_x10 =
        BallBalance_App_LimitTargetX10(target_mm_x10);
    BallReference_SetTargetMm((float)s_target_mm_x10 * 0.1f);
    s_settled = 0U;
}

BSP_Status_t BallBalance_App_SetControlProfile(
    const BallBalance_ControlProfile_t *profile
)
{
    return (BallBalance_Control_SetProfile(profile) == PROJECT_OK) ?
           BSP_OK : BSP_PARAM;
}

void BallBalance_App_ResetControlProfile(void)
{
    BallBalance_Control_ResetProfile();
}

void BallBalance_App_PushVisionSample(
    const BallBalance_VisionSample_t *sample
)
{
    BallBalance_VisionSample_t checked;
    uint8_t sample_valid;
    uint32_t primask;

    if (sample == 0) {
        return;
    }

    checked = *sample;
    if (checked.timestamp_ms == 0U) {
        checked.timestamp_ms = BSP_GetTickMs();
    }
    if (checked.state > BALL_BALANCE_VISION_VALID) {
        checked.state = BALL_BALANCE_VISION_LOST;
    }

    sample_valid =
        ((checked.state == BALL_BALANCE_VISION_VALID) &&
         (checked.confidence >= BALL_BALANCE_MIN_CONFIDENCE) &&
         (BallBalance_App_IsPositionInRange(
              checked.position_mm_x10) != 0U)) ? 1U : 0U;
    checked.valid = sample_valid;

    primask = Project_EnterCritical();
    if ((s_has_sequence != 0U) &&
        (checked.sequence == s_last_sequence)) {
        s_duplicate_sequence = 1U;
        Project_ExitCritical(primask);
        return;
    }

    s_pending_vision_sample = checked;
    s_last_vision_sample = checked;
    s_pending_sample = 1U;
    s_has_sequence = 1U;
    s_last_sequence = checked.sequence;
    s_last_sample_state = checked.state;
    s_last_sample_valid = sample_valid;
    s_low_confidence =
        (checked.confidence < BALL_BALANCE_MIN_CONFIDENCE) ? 1U : 0U;
    s_position_out_of_range =
        (BallBalance_App_IsPositionInRange(
             checked.position_mm_x10) == 0U) ? 1U : 0U;
    s_duplicate_sequence = 0U;
    s_pushed_sample_count =
        BallBalance_App_IncrementU32(s_pushed_sample_count);

    if (checked.state == BALL_BALANCE_VISION_VALID) {
        s_valid_sample_count =
            BallBalance_App_IncrementU32(s_valid_sample_count);
    } else if (checked.state == BALL_BALANCE_VISION_HOLD) {
        s_hold_sample_count =
            BallBalance_App_IncrementU32(s_hold_sample_count);
    } else {
        s_lost_sample_count =
            BallBalance_App_IncrementU32(s_lost_sample_count);
    }
    if (sample_valid == 0U) {
        s_rejected_sample_count =
            BallBalance_App_IncrementU32(s_rejected_sample_count);
    }
    Project_ExitCritical(primask);
}

void BallBalance_App_SetVehicleDisturbanceMmS2(
    float disturbance_mm_s2,
    uint8_t valid,
    uint32_t timestamp_ms
)
{
    s_vehicle_disturbance_mm_s2 = disturbance_mm_s2;
    s_vehicle_disturbance_valid = (valid != 0U) ? 1U : 0U;
    s_vehicle_disturbance_timestamp_ms = timestamp_ms;
}

void BallBalance_App_SetVehicleFeedforwardEnabled(uint8_t enabled)
{
    s_vehicle_feedforward_enabled = (enabled != 0U) ? 1U : 0U;
}

uint8_t BallBalance_App_IsSettled(void)
{
    return s_settled;
}

void BallBalance_App_Update(void)
{
    BallBalance_VisionSample_t sample;
    BallStateEstimator_Info_t estimator;
    BallBalance_ControlInput_t control_input;
    BallBalance_ControlOutput_t control_output;
    float vehicle_disturbance;
    uint8_t has_sample;
    float measured_position_mm;
    uint32_t now_ms;

    if (s_initialized == 0U) {
        return;
    }

    now_ms = BSP_GetTickMs();
    has_sample = BallBalance_App_TakeSample(&sample);
    measured_position_mm = 0.0f;

    if (has_sample != 0U) {
        s_consumed_sample_count =
            BallBalance_App_IncrementU32(s_consumed_sample_count);
        if (sample.valid != 0U) {
            measured_position_mm =
                (float)sample.position_mm_x10 * 0.1f;
            s_last_valid_sample_ms = sample.timestamp_ms;
            if (s_valid_streak < 0xFFU) {
                s_valid_streak++;
            }

            if (s_tracking_ready == 0U) {
                if (s_valid_streak == 1U) {
                    BallStateEstimator_Reset(measured_position_mm);
                }
                if (s_valid_streak >=
                    BALL_BALANCE_REACQUIRE_VALID_COUNT) {
                    BallBalance_App_AcquireTracking(
                        measured_position_mm
                    );
                }
            } else {
                if (BallStateEstimator_UpdatePosition(
                        measured_position_mm
                    ) == PROJECT_OK) {
                    s_estimator_reject_streak = 0U;
                } else {
                    if (s_estimator_reject_streak < 0xFFU) {
                        s_estimator_reject_streak++;
                    }
                    if ((BallBalance_App_AbsF(measured_position_mm) >=
                         BALL_ESTIMATOR_EDGE_RESET_POSITION_MM) ||
                        (s_estimator_reject_streak >=
                         BALL_ESTIMATOR_REJECT_RESET_COUNT)) {
                        BallBalance_App_ResetEstimatorAtMeasurement(
                            measured_position_mm
                        );
                    }
                }
            }
        } else {
            s_valid_streak = 0U;
            s_estimator_reject_streak = 0U;
        }
    }

    if ((s_tracking_ready != 0U) &&
        ((uint32_t)(now_ms - s_last_valid_sample_ms) >
         BALL_BALANCE_VALID_TIMEOUT_MS)) {
        s_tracking_ready = 0U;
        s_valid_streak = 0U;
        s_estimator_reject_streak = 0U;
        s_data_timeout = 1U;
        BallReference_Pause();
    }

    (void)BallBalance_App_GetVehicleDisturbance(
        now_ms,
        &vehicle_disturbance
    );

    BallStateEstimator_Predict(
        s_last_applied_dynamic_angle_deg,
        vehicle_disturbance,
        BALL_BALANCE_CONTROL_PERIOD_S
    );
    if (BallStateEstimator_GetInfo(&estimator) != PROJECT_OK) {
        return;
    }

    if ((s_enabled != 0U) && (s_tracking_ready != 0U)) {
        s_state = BALL_BALANCE_APP_ACTIVE;
        s_data_timeout = 0U;
    } else {
        BallReference_Pause();
        if (s_servo_fault != 0U) {
            s_state = BALL_BALANCE_APP_FAULT;
        } else if (s_enabled == 0U) {
            s_state = BALL_BALANCE_APP_DISABLED;
        } else if (s_ever_active != 0U) {
            s_state = BALL_BALANCE_APP_DEGRADED;
        } else {
            s_state = BALL_BALANCE_APP_WAIT_VALID;
        }
    }

    s_equilibrium_angle_deg =
        BallEquilibriumMap_GetAngleDeg(estimator.position_mm);

    control_input.control_enabled =
        (s_state == BALL_BALANCE_APP_ACTIVE) ? 1U : 0U;
    control_input.data_valid =
        ((s_tracking_ready != 0U) &&
         (s_last_sample_valid != 0U)) ? 1U : 0U;
    control_input.now_ms = now_ms;
    control_input.dt_s = BALL_BALANCE_CONTROL_PERIOD_S;
    control_input.target_position_mm =
        (float)s_target_mm_x10 * 0.1f;
    control_input.estimated_position_mm = estimator.position_mm;
    control_input.estimated_velocity_mm_s = estimator.velocity_mm_s;
    control_input.equilibrium_angle_deg = s_equilibrium_angle_deg;

    if (BallBalance_Control_Update(
            &control_input,
            &control_output
        ) != PROJECT_OK) {
        return;
    }
    s_last_applied_dynamic_angle_deg =
        control_output.applied_dynamic_angle_deg;
    s_settled = control_output.target_locked;

    s_last_servo_status =
        Drv_Servo_SetHorizontalAngleX10(
            control_output.command_angle_x10
        );
    if (s_last_servo_status != BSP_OK) {
        s_servo_fault = 1U;
        s_enabled = 0U;
        s_state = BALL_BALANCE_APP_FAULT;
        (void)Drv_Servo_SetHorizontalAngleX10(
            BALL_BALANCE_LEVEL_ANGLE_X10
        );
    }
}

BSP_Status_t BallBalance_App_GetInfo(BallBalance_AppInfo_t *info)
{
    uint32_t primask;

    if (info == 0) {
        return BSP_PARAM;
    }

    info->initialized = s_initialized;
    info->enabled = s_enabled;
    info->state = s_state;
    info->tracking_ready = s_tracking_ready;
    info->valid_streak = s_valid_streak;
    info->data_timeout = s_data_timeout;
    info->settled = s_settled;
    info->servo_fault = s_servo_fault;
    info->last_servo_status = s_last_servo_status;
    info->vehicle_feedforward_enabled =
        s_vehicle_feedforward_enabled;
    info->vehicle_disturbance_valid =
        s_vehicle_disturbance_valid;
    info->vehicle_disturbance_mm_s2 =
        s_vehicle_disturbance_mm_s2;
    info->vehicle_disturbance_timestamp_ms =
        s_vehicle_disturbance_timestamp_ms;
    info->last_valid_sample_ms = s_last_valid_sample_ms;
    info->target_mm_x10 = s_target_mm_x10;
    info->equilibrium_angle_deg = s_equilibrium_angle_deg;

    primask = Project_EnterCritical();
    info->pending_sample = s_pending_sample;
    info->last_sample_state = s_last_sample_state;
    info->last_sample_valid = s_last_sample_valid;
    info->low_confidence = s_low_confidence;
    info->position_out_of_range = s_position_out_of_range;
    info->duplicate_sequence = s_duplicate_sequence;
    info->pushed_sample_count = s_pushed_sample_count;
    info->consumed_sample_count = s_consumed_sample_count;
    info->rejected_sample_count = s_rejected_sample_count;
    info->valid_sample_count = s_valid_sample_count;
    info->hold_sample_count = s_hold_sample_count;
    info->lost_sample_count = s_lost_sample_count;
    info->last_sample = s_last_vision_sample;
    Project_ExitCritical(primask);

    if ((BallStateEstimator_GetInfo(&info->estimator) != PROJECT_OK) ||
        (BallReference_GetInfo(&info->reference) != PROJECT_OK) ||
        (BallBalance_Control_GetInfo(&info->control) != PROJECT_OK)) {
        return BSP_ERROR;
    }
    return BSP_OK;
}
