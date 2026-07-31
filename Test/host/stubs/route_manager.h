#ifndef __ROUTE_MANAGER_H
#define __ROUTE_MANAGER_H

#include "project_status.h"
#include <stdint.h>

typedef enum {
    ROUTE_EVENT_NONE         = 0x00U,
    ROUTE_EVENT_LEFT_A       = 0x01U,
    ROUTE_EVENT_PASSED_B     = 0x02U,
    ROUTE_EVENT_LAP_COMPLETE = 0x04U,
    ROUTE_EVENT_LINE_LOST    = 0x08U,
    ROUTE_EVENT_ACTION_ERROR = 0x10U,
    ROUTE_EVENT_FAILED       = 0x20U
} Route_Event_t;

uint32_t RouteManager_GetEvents(void);
void RouteManager_ClearEvents(uint32_t events);

#endif /* __ROUTE_MANAGER_H */
