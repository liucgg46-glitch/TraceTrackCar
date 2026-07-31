#ifndef __BALL_BALANCE_CONTROL_H
#define __BALL_BALANCE_CONTROL_H

#include "ball_balance_config.h"
#include "project_status.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t control_enabled;
    uint8_t data_valid;
    uint32_t now_ms;
    float dt_s;
    float target_position_mm;
    float estimated_position_mm;
    float estimated_velocity_mm_s;
    float equilibrium_angle_deg;
} BallBalance_ControlInput_t;

typedef struct {
    float target_position_mm;
    float estimated_position_mm;
    float position_error_mm;
    float target_velocity_mm_s;
    float filtered_velocity_mm_s;
    float velocity_error_mm_s;
    float velocity_integral_angle_deg;
    float proportional_angle_deg;
    float dynamic_angle_deg;
    float equilibrium_angle_deg;
    float requested_servo_angle_deg;
    float servo_angle_deg;
    float servo_speed_deg_s;
    float hold_servo_angle_deg;
    float applied_dynamic_angle_deg;
    uint16_t command_angle_x10;
    uint8_t absolute_limited;
    uint8_t motion_limited;
    uint8_t hold_active;
    uint8_t target_locked;
    uint8_t integral_blocked;
} BallBalance_ControlOutput_t;

typedef struct {
    uint8_t initialized;
    BallBalance_ControlInput_t input;
    BallBalance_ControlOutput_t output;
    uint32_t update_count;
    uint32_t output_limit_count;
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
