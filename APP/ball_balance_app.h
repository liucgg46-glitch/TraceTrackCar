#ifndef __BALL_BALANCE_APP_H
#define __BALL_BALANCE_APP_H

#include "ball_balance_control.h"
#include "ball_reference_generator.h"
#include "ball_state_estimator.h"
#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BALL_BALANCE_VISION_LOST = 0U,
    BALL_BALANCE_VISION_HOLD = 1U,
    BALL_BALANCE_VISION_VALID = 2U
} BallBalance_VisionState_t;

typedef enum {
    BALL_BALANCE_APP_DISABLED = 0U,
    BALL_BALANCE_APP_WAIT_VALID,
    BALL_BALANCE_APP_ACTIVE,
    BALL_BALANCE_APP_DEGRADED,
    BALL_BALANCE_APP_FAULT
} BallBalance_AppState_t;

typedef struct {
    int16_t position_mm_x10;
    uint8_t state;
    uint8_t valid;
    uint8_t confidence;
    uint8_t sequence;
    uint32_t timestamp_ms;
} BallBalance_VisionSample_t;

typedef struct {
    uint8_t initialized;
    uint8_t enabled;
    BallBalance_AppState_t state;
    uint8_t tracking_ready;
    uint8_t valid_streak;
    uint8_t data_timeout;
    uint8_t settled;
    uint8_t servo_fault;
    BSP_Status_t last_servo_status;
    uint8_t pending_sample;
    uint8_t last_sample_state;
    uint8_t last_sample_valid;
    uint8_t low_confidence;
    uint8_t position_out_of_range;
    uint8_t duplicate_sequence;
    uint8_t vehicle_feedforward_enabled;
    uint8_t vehicle_disturbance_valid;
    float vehicle_disturbance_mm_s2;
    uint32_t vehicle_disturbance_timestamp_ms;
    uint32_t last_valid_sample_ms;
    uint32_t pushed_sample_count;
    uint32_t consumed_sample_count;
    uint32_t rejected_sample_count;
    uint32_t valid_sample_count;
    uint32_t hold_sample_count;
    uint32_t lost_sample_count;
    int16_t target_mm_x10;
    float equilibrium_angle_deg;
    BallBalance_VisionSample_t last_sample;
    BallStateEstimator_Info_t estimator;
    BallReference_Info_t reference;
    BallBalance_ControlInfo_t control;
} BallBalance_AppInfo_t;

void BallBalance_App_Init(void);
void BallBalance_App_Enable(void);
void BallBalance_App_Disable(void);
uint8_t BallBalance_App_IsEnabled(void);
void BallBalance_App_SetTargetMmX10(int16_t target_mm_x10);
void BallBalance_App_PushVisionSample(
    const BallBalance_VisionSample_t *sample
);
void BallBalance_App_SetVehicleDisturbanceMmS2(
    float disturbance_mm_s2,
    uint8_t valid,
    uint32_t timestamp_ms
);
void BallBalance_App_SetVehicleFeedforwardEnabled(uint8_t enabled);
uint8_t BallBalance_App_IsSettled(void);
void BallBalance_App_Update(void);
BSP_Status_t BallBalance_App_GetInfo(BallBalance_AppInfo_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __BALL_BALANCE_APP_H */
