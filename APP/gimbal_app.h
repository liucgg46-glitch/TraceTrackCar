#ifndef __GIMBAL_APP_H
#define __GIMBAL_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GIMBAL_APP_IDLE = 0,
    GIMBAL_APP_SQUARE_TEST,
    GIMBAL_APP_TRACK,
    GIMBAL_APP_STOP
} GimbalApp_State_t;

void GimbalApp_Init(void);
void GimbalApp_StartSquareTest(void);
void GimbalApp_StartTrack(void);
void GimbalApp_Stop(void);
void GimbalApp_Update(void);

GimbalApp_State_t GimbalApp_GetState(void);
uint8_t GimbalApp_GetSquareSegment(void);

#ifdef __cplusplus
}
#endif

#endif /* __GIMBAL_APP_H */