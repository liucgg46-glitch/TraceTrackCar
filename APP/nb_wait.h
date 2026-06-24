#ifndef __NB_WAIT_H
#define __NB_WAIT_H

#include <stdint.h>
#include "bsp_systick.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 非阻塞等待工具
 * ============================================================================
 * 用于替代 delay_ms/HAL_Delay。它本身不会卡住 CPU，只记录开始时间，
 * 由 Update/IsDone 根据 BSP_GetTickMs() 判断是否到时。
 */

typedef enum {
    NB_WAIT_IDLE = 0,
    NB_WAIT_RUNNING,
    NB_WAIT_DONE
} NB_WaitState_t;

typedef struct {
    NB_WaitState_t state;
    uint32_t start_time_ms;
    uint32_t duration_ms;
} NB_Wait_t;

void     NB_Wait_Init(NB_Wait_t *wait);
void     NB_Wait_Start(NB_Wait_t *wait, uint32_t duration_ms);
void     NB_Wait_Update(NB_Wait_t *wait);
void     NB_Wait_Stop(NB_Wait_t *wait);
uint8_t  NB_Wait_IsDone(NB_Wait_t *wait);
uint8_t  NB_Wait_IsRunning(NB_Wait_t *wait);

#ifdef __cplusplus
}
#endif

#endif /* __NB_WAIT_H */
