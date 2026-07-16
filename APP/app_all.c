
#include "app_all.h"
#include "chassis.h"
#include "odometer.h"
#include "attitude_estimator.h"
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
    Attitude_Init();
    Heading_Init();
    Motion_Init();
    SensorManager_Init();
    LineFollow_Init();

     /* 这里只请求显示启动页，不重新初始化 LCD/OLED 驱动 */
    //LcdUi_ShowBoot();
    //OledUi_ShowBoot();
	 /*
    /*
     * 初始化K210通信协议层。
     *
     * USART2底层已经由BSP_InitAll()初始化，
     * 这里只初始化协议状态机。
     */
    K210_Comm_Init();
	//LineFollow_Start();
}

