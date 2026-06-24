#ifndef __HEADING_ESTIMATOR_H
#define __HEADING_ESTIMATOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== Heading source select ==================== */

#define HEADING_SOURCE_ENCODER      0U
#define HEADING_SOURCE_IMU          1U

/* Use encoder differential yaw first. Switch this to IMU after drv_imu is ready. */
#define HEADING_ESTIMATOR_SOURCE    HEADING_SOURCE_ENCODER


/* ==================== Encoder heading parameters ==================== */

/* Wheel base in mm.
 * Turn angle too small: increase this.
 * Turn angle too large: decrease this.
 */
#define HEADING_ENCODER_WHEEL_BASE_MM     165.0f

/* Set to 1 if encoder-estimated yaw sign is reversed. */
#define HEADING_ENCODER_YAW_REVERSE       0U


/* ==================== IMU heading parameters ==================== */

/* Set to 1 if IMU yaw sign is reversed. */
#define HEADING_IMU_YAW_REVERSE           0U


void Heading_Init(void);
void Heading_Reset(void);
void Heading_Update(void);

float Heading_GetYawDeg(void);
float Heading_GetErrorDeg(float target_deg);

#ifdef __cplusplus
}
#endif

#endif /* __HEADING_ESTIMATOR_H */
