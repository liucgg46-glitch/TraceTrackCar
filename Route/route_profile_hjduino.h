#ifndef __ROUTE_PROFILE_HJDUINO_H
#define __ROUTE_PROFILE_HJDUINO_H

#include "route_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HJD_ROUTE_NORMAL_BEFORE_LOOP = 0,
    HJD_ROUTE_ENTER_LOOP_RIGHT,
    HJD_ROUTE_IN_LOOP,
    HJD_ROUTE_ERROR
} HJduinoRoute_State_t;

typedef struct {
    HJduinoRoute_State_t state;
    uint16_t entry_confirm_samples;
    uint32_t running_ms;
    uint32_t transition_count;
} HJduinoRoute_Info_t;

void HJduinoRoute_Init(void);
void HJduinoRoute_Reset(void);
Route_ControlMode_t HJduinoRoute_Update(const LineDetect_Result_t *line,
                                        LineTrack_Output_t *out);
BSP_Status_t HJduinoRoute_GetInfo(HJduinoRoute_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __ROUTE_PROFILE_HJDUINO_H */
