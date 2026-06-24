#include "app_all.h"
#include "chassis.h"
#include "odometer.h"
#include "heading_estimator.h"
#include "motion_action.h"
#include "sensor_manager.h"
#include "line_follow_app.h"
#include "lcd_ui.h"

void App_Init(void)
{
    Chassis_Init();
    Odometer_Init();
    Heading_Init();
    Motion_Init();
    SensorManager_Init();
    LineFollow_Init();
    LcdUi_ShowBoot();
}
