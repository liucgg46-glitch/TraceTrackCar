#ifndef __LINE_FOLLOW_APP_H
#define __LINE_FOLLOW_APP_H

#include "bsp_common.h"
#include "line_track.h"
#include <stdint.h>

typedef enum {
    LINE_FOLLOW_STOP = 0,
    LINE_FOLLOW_RUN
} LineFollow_State_t;

BSP_Status_t LineFollow_Start(void);
void LineFollow_StopPreserveRoute(void);
BSP_Status_t LineFollow_SetSpeedProfile(int16_t base_speed_cps,
                                        int16_t cross_speed_cps,
                                        int16_t min_track_speed_cps);
BSP_Status_t LineFollow_SetControlProfile(
    const LineTrack_ControlProfile_t *profile);
void LineFollow_ResetControlProfile(void);
LineFollow_State_t LineFollow_GetState(void);

#endif /* __LINE_FOLLOW_APP_H */
