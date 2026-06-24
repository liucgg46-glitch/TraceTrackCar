#include "app_all.h"
#include "chassis.h"
#include "odometer.h"
#include "angle_control.h"
#include "motion_action.h"

void App_Init(void)
{
    Chassis_Init();
    Odometer_Init();
    AngleControl_Init();
    Motion_Init();
}
