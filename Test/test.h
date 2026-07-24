#ifndef __TEST_H
#define __TEST_H

#include "test_config.h"

#if (PROJECT_TEST_TASKS_ENABLE != 0U)

/* 板级资源与基础外设测试。 */
void Test_GPIO_Toggle(void);
void Test_StatusLight_Update(void);
void Test_Buzzer_Update(void);
void Test_PWM_Ramp(void);
void Test_Encoder_Log(void);
void Test_Gray4051_Update(void);
void Test_Gray4051_Log(void);
void Test_Key_Update(void);
void Test_EXTI_Init(void);
void Test_EXTI_Log(void);
void Test_UART_Echo(void);
void Test_UART_Stats(void);
void Test_I2C_Scan(void);

/* Driver与器件测试。 */
void Test_E220_Link_Update(void);
void Test_DrvEncoder_Log(void);
void Test_CountPerRev_Update(void);
void Test_VL53L1X_Update(void);
void Test_HX711_Update(void);
void Test_ICM20948_Update(void);
void Test_ICM20948_Mag_Update(void);
void Test_LCD_Ascii_Update(void);
void Test_OLED_Ascii_Update(void);

/* APP、Algorithm与Route联调入口。 */
void Test_MotorCmd_Update(void);
void Test_MotorCmd_Log(void);
void Test_ChassisCmd_Update(void);
void Test_ChassisWatchdog_Update(void);
void Test_ChassisCmd_Log(void);
void Test_MotionCmd_Update(void);
void Test_MotionCmd_Log(void);
void Test_LineCmd_Update(void);
void Test_LineCmd_Log(void);
void Test_RouteCmd_Update(void);
void Test_RouteLog(void);
void Test_TaskFSM_Log(void);
void Test_Attitude_Update(void);
void Test_K210_CommUpdate(void);
void Test_DriveProfile_Update(void);

#endif /* PROJECT_TEST_TASKS_ENABLE */

#endif
