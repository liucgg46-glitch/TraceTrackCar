
#include "app_all.h"
#include "chassis.h"
#include "odometer.h"
#include "heading_estimator.h"
#include "motion_action.h"
#include "sensor_manager.h"
#include "line_follow_app.h"
#include "lcd_ui.h"
#include "oled_ui.h"

void App_Init(void)
{
    Chassis_Init();
    Odometer_Init();
    Heading_Init();
    Motion_Init();
    SensorManager_Init();
    LineFollow_Init();

     /* 这里只请求显示启动页，不重新初始化 LCD/OLED 驱动 */
    LcdUi_ShowBoot();
    OledUi_ShowBoot();

	LineFollow_Start();
}

