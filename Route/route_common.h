#ifndef __ROUTE_COMMON_H
#define __ROUTE_COMMON_H

#include "bsp_common.h"
#include "line_detect.h"
#include "line_track.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exactly one module owns the chassis command in each scheduler cycle. */
typedef enum {
    ROUTE_CONTROL_STOP = 0,
    ROUTE_CONTROL_LINE_TRACK,
    ROUTE_CONTROL_MOTION,
    ROUTE_CONTROL_ERROR
} Route_ControlMode_t;

#ifdef __cplusplus
}
#endif

#endif /* __ROUTE_COMMON_H */
