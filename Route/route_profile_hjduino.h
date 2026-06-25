#ifndef __ROUTE_PROFILE_HJDUINO_H
#define __ROUTE_PROFILE_HJDUINO_H

#include "route_common.h"

#ifdef __cplusplus
extern "C" {
#endif

void HJduinoRoute_Init(void);
void HJduinoRoute_Reset(void);
void HJduinoRoute_Update(const LineDetect_Result_t *line, LineTrack_Output_t *out);

#ifdef __cplusplus
}
#endif

#endif