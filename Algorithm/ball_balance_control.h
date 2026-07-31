#ifndef __BALL_BALANCE_CONTROL_H
#define __BALL_BALANCE_CONTROL_H

#include "ball_balance_config.h"
#include "project_status.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BREAKAWAY_IDLE = 0U,
    BREAKAWAY_RAMP,
    BREAKAWAY_DECAY,
    BREAKAWAY_COOLDOWN
} BallBalance_BreakawayState_t;

typedef struct {
    uint8_t control_enabled;
    uint8_t data_valid;
    uint8_t allow_breakaway_growth;
    uint8_t position_measurement_valid;
    uint32_t now_ms;
    float dt_s;
    float target_position_mm;
    float measured_position_mm;
    float reference_position_mm;
    float reference_velocity_mm_s;
    float reference_acceleration_mm_s2;
    float estimated_position_mm;
    float estimated_velocity_mm_s;
    float estimated_disturbance_mm_s2;
    float vehicle_disturbance_mm_s2;
    float equilibrium_angle_deg;
} BallBalance_ControlInput_t;

typedef struct {
    float position_error_mm;
    float velocity_error_mm_s;
    float filtered_velocity_mm_s;
    float filtered_disturbance_mm_s2;
    float reference_accel_feedforward_mm_s2;
    float desired_acceleration_mm_s2;
    float required_control_acceleration_mm_s2;
    float requested_dynamic_angle_deg;
    float limited_dynamic_angle_deg;
    float breakaway_angle_deg;
    float breakaway_start_angle_deg;
    float breakaway_progress_mm;
    float hold_servo_angle_deg;
    float requested_servo_angle_deg;
    float servo_angle_deg;
    float servo_speed_deg_s;
    float applied_dynamic_angle_deg;
    uint16_t command_angle_x10;
    uint8_t dynamic_limited;
    uint8_t absolute_limited;
    uint8_t motion_limited;
    uint8_t breakaway_active;
    uint8_t hold_active;
    uint8_t target_locked;
    BallBalance_BreakawayState_t breakaway_state;
} BallBalance_ControlOutput_t;

typedef struct {
    uint8_t initialized;
    BallBalance_ControlInput_t input;
    BallBalance_ControlOutput_t output;
    uint32_t update_count;
    uint32_t output_limit_count;
    uint32_t breakaway_elapsed_ms;
    uint32_t breakaway_update_count;
} BallBalance_ControlInfo_t;

void BallBalance_Control_Init(void);
void BallBalance_Control_Reset(void);
Project_Status_t BallBalance_Control_Update(
    const BallBalance_ControlInput_t *input,
    BallBalance_ControlOutput_t *output
);
Project_Status_t BallBalance_Control_GetInfo(
    BallBalance_ControlInfo_t *info
);

#ifdef __cplusplus
}
#endif

#endif /* __BALL_BALANCE_CONTROL_H */
