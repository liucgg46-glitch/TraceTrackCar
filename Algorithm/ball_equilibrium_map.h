#ifndef __BALL_EQUILIBRIUM_MAP_H
#define __BALL_EQUILIBRIUM_MAP_H

#include "ball_balance_config.h"
#include "project_status.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t point_count;
    float position_mm[BALL_EQUILIBRIUM_MAP_POINT_COUNT];
    float angle_deg[BALL_EQUILIBRIUM_MAP_POINT_COUNT];
} BallEquilibriumMap_Info_t;

void BallEquilibriumMap_Init(void);
float BallEquilibriumMap_GetAngleDeg(float position_mm);
Project_Status_t BallEquilibriumMap_SetPoint(uint8_t index,
                                             float position_mm,
                                             float angle_deg);
Project_Status_t BallEquilibriumMap_GetInfo(BallEquilibriumMap_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __BALL_EQUILIBRIUM_MAP_H */
