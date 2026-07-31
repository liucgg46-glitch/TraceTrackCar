#ifndef __BALL_REFERENCE_GENERATOR_H
#define __BALL_REFERENCE_GENERATOR_H

#include "project_status.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t initialized;
    uint8_t paused;
    float target_position_mm;
    float reference_position_mm;
    float reference_velocity_mm_s;
    float reference_acceleration_mm_s2;
} BallReference_Info_t;

void BallReference_Init(float initial_position_mm);
void BallReference_SetTargetMm(float target_mm);
void BallReference_Pause(void);
void BallReference_Resume(void);
void BallReference_Update(float dt_s);
Project_Status_t BallReference_GetInfo(BallReference_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __BALL_REFERENCE_GENERATOR_H */
