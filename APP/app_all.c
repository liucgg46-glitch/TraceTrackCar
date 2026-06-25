#include "app_all.h"
#include "chassis.h"
#include "odometer.h"
#include "heading_estimator.h"
#include "motion_action.h"
#include "sensor_manager.h"
#include "line_follow_app.h"

void App_Init(void)
{
    Chassis_Init();
    Odometer_Init();
    Heading_Init();
    Motion_Init();
    SensorManager_Init();
    LineFollow_Init();
	
    /* 上电后自动进入循迹，不再需要串口发送 '1' */
    LineFollow_Start();
}
