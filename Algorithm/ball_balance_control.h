#ifndef __BALL_BALANCE_CONTROL_H
#define __BALL_BALANCE_CONTROL_H

#include "project_status.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 车载平衡滚球控制器。
 * 本模块只处理位置、速度和PID/PD计算，不读取K210、不访问舵机、不输出日志。
 */
#define BALL_BALANCE_KP                         0.06f
#define BALL_BALANCE_KI                         0.00f
#define BALL_BALANCE_KD                         0.015f

#define BALL_BALANCE_NEUTRAL_ANGLE_X10          900U
#define BALL_BALANCE_OUTPUT_MIN_X10             850U
#define BALL_BALANCE_OUTPUT_MAX_X10             950U
#define BALL_BALANCE_ABS_SAFE_MIN_X10           0U
#define BALL_BALANCE_ABS_SAFE_MAX_X10           1800U
#define BALL_BALANCE_UPDATE_PERIOD_MS           10U
/* K210实际帧周期受YOLO推理速度影响，后续应按实测周期调整该超时值。 */
#define BALL_BALANCE_DATA_TIMEOUT_MS            150U
#define BALL_BALANCE_MIN_CONFIDENCE             60U
#define BALL_BALANCE_POSITION_DEADBAND_MM       1.0f
#define BALL_BALANCE_INTEGRAL_ACTIVE_MM         30.0f
#define BALL_BALANCE_SLEW_X10_PER_UPDATE        5U

typedef struct {
    uint8_t enabled;
    uint8_t measurement_valid;
    uint8_t data_timeout;
    int16_t target_mm_x10;
    int16_t raw_position_mm_x10;
    int16_t filtered_position_mm_x10;
    int16_t error_mm_x10;
    float velocity_mm_s;
    float integral;
    float kp;
    float ki;
    float kd;
    float p_term;
    float i_term;
    float d_term;
    float pid_output_deg;
    uint16_t command_angle_x10;
    uint8_t last_sequence;
    uint32_t last_sample_ms;
    uint32_t update_count;
    uint32_t valid_sample_count;
    uint32_t invalid_sample_count;
    uint32_t timeout_count;
    uint32_t output_limit_count;
} BallBalance_ControlInfo_t;

void BallBalance_Control_Init(void);
void BallBalance_Control_Reset(void);

void BallBalance_Control_SetEnabled(uint8_t enabled);
uint8_t BallBalance_Control_IsEnabled(void);

void BallBalance_Control_SetTargetMmX10(int16_t target_mm_x10);
int16_t BallBalance_Control_GetTargetMmX10(void);

void BallBalance_Control_SetGains(float kp, float ki, float kd);

void BallBalance_Control_PushMeasurement(int16_t position_mm_x10,
                                         uint8_t sequence,
                                         uint32_t timestamp_ms);
void BallBalance_Control_InvalidateMeasurement(uint32_t timestamp_ms);

void BallBalance_Control_Update(uint32_t now_ms);

Project_Status_t BallBalance_Control_GetInfo(
    BallBalance_ControlInfo_t *info
);

#ifdef __cplusplus
}
#endif

#endif /* __BALL_BALANCE_CONTROL_H */
