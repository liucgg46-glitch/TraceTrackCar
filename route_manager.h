#ifndef __ROUTE_MANAGER_H
#define __ROUTE_MANAGER_H

#include "route_common.h"

#ifdef __cplusplus
extern "C" {
#endif

void RouteManager_Init(void);
void RouteManager_Reset(void);
void RouteManager_Update(const LineDetect_Result_t *line, LineTrack_Output_t *out);

#ifdef __cplusplus
}
#endif

#endif