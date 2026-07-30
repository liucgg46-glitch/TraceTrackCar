#ifndef __BALL_BALANCE_K210_ADAPTER_H
#define __BALL_BALANCE_K210_ADAPTER_H

#include "bsp_common.h"
#include "ball_balance_app.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t initialized;
    uint8_t sequence;
    uint8_t last_k210_state;
    uint8_t last_app_state;
    int16_t last_position_mm_x10;
    uint8_t last_confidence;
    uint32_t last_sample_ms;
    uint32_t pushed_count;
    uint32_t busy_count;
    uint32_t error_count;
} BallBalance_K210AdapterInfo_t;

void BallBalance_K210Adapter_Init(void);
void BallBalance_K210Adapter_Update(void);
BSP_Status_t BallBalance_K210Adapter_GetInfo(
    BallBalance_K210AdapterInfo_t *info
);

#ifdef __cplusplus
}
#endif

#endif /* __BALL_BALANCE_K210_ADAPTER_H */
