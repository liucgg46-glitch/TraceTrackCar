#ifndef __BSP_COMMON_H
#define __BSP_COMMON_H

#include "project_status.h"
#include <stdint.h>

typedef Project_Status_t BSP_Status_t;
#define BSP_OK       PROJECT_OK
#define BSP_ERROR    PROJECT_ERROR
#define BSP_BUSY     PROJECT_BUSY
#define BSP_TIMEOUT  PROJECT_TIMEOUT
#define BSP_PARAM    PROJECT_PARAM

#ifndef BSP_WEAK
#define BSP_WEAK
#endif

uint32_t GetTick(void);
#define BSP_GET_TICK() GetTick()

#endif /* __BSP_COMMON_H */
