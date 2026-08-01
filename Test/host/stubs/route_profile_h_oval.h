#ifndef __ROUTE_PROFILE_H_OVAL_H
#define __ROUTE_PROFILE_H_OVAL_H

#include "project_status.h"
#include <stdint.h>

typedef struct {
    uint8_t finish_armed;
    int32_t relative_distance_mm;
    int16_t turn_output;
} HOvalRoute_Info_t;

Project_Status_t HRoute_GetH2Info(HOvalRoute_Info_t *info);

#endif /* __ROUTE_PROFILE_H_OVAL_H */
