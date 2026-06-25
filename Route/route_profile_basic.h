#ifndef __ROUTE_PROFILE_BASIC_H
#define __ROUTE_PROFILE_BASIC_H

#include "route_common.h"

#ifdef __cplusplus
extern "C" {
#endif

void BasicRoute_Init(void);
void BasicRoute_Reset(void);
void BasicRoute_Update(const LineDetect_Result_t *line, LineTrack_Output_t *out);

#ifdef __cplusplus
}
#endif

#endif