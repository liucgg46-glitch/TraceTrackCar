#ifndef __ROUTE_COMMON_H
#define __ROUTE_COMMON_H

#include "bsp_common.h"
#include "line_detect.h"
#include "line_track.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t Route_CountBlackBits(uint8_t mask);
uint8_t Route_IsCrossLike(const LineDetect_Result_t *line);
uint8_t Route_IsStableSingleLine(const LineDetect_Result_t *line);
uint8_t Route_IsLeftEdge(const LineDetect_Result_t *line);
uint8_t Route_IsRightEdge(const LineDetect_Result_t *line);
uint8_t Route_IsMiddleOnLine(const LineDetect_Result_t *line);

#ifdef __cplusplus
}
#endif

#endif