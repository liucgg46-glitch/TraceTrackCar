#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "bsp_common.h"

typedef enum {
    CHASSIS_FAULT_NONE = 0,
    CHASSIS_FAULT_COMMAND_TIMEOUT
} Chassis_Fault_t;

void Chassis_EmergencyStop(void);
Chassis_Fault_t Chassis_GetFault(void);

#endif /* __CHASSIS_H */
