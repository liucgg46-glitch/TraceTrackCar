#ifndef __BALL_BALANCE_VEHICLE_IMU_ADAPTER_H
#define __BALL_BALANCE_VEHICLE_IMU_ADAPTER_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t initialized;
    uint8_t source_available;
    uint8_t valid;
    float disturbance_mm_s2;
    uint32_t timestamp_ms;
    uint32_t invalid_count;
} BallBalance_VehicleImuAdapterInfo_t;

void BallBalance_VehicleImuAdapter_Init(void);
void BallBalance_VehicleImuAdapter_Update(void);
BSP_Status_t BallBalance_VehicleImuAdapter_GetInfo(
    BallBalance_VehicleImuAdapterInfo_t *info
);

#ifdef __cplusplus
}
#endif

#endif /* __BALL_BALANCE_VEHICLE_IMU_ADAPTER_H */
