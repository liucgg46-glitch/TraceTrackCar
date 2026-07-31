#ifndef __BALL_STATE_ESTIMATOR_H
#define __BALL_STATE_ESTIMATOR_H

#include "project_status.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t initialized;
    uint8_t measurement_valid;
    float position_mm;
    float velocity_mm_s;
    float disturbance_mm_s2;
    float innovation_mm;
    uint8_t innovation_rejected;
    uint32_t prediction_count;
    uint32_t measurement_count;
    uint32_t reject_count;
} BallStateEstimator_Info_t;

void BallStateEstimator_Init(void);
void BallStateEstimator_Reset(float position_mm);
void BallStateEstimator_Predict(float dynamic_angle_deg,
                                float vehicle_disturbance_mm_s2,
                                float dt_s);
Project_Status_t BallStateEstimator_UpdatePosition(float position_mm);
Project_Status_t BallStateEstimator_GetInfo(BallStateEstimator_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __BALL_STATE_ESTIMATOR_H */
