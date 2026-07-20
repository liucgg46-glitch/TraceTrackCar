#ifndef __TEST_H
#define __TEST_H

void Test_GPIO_Toggle(void); //{ Test_GPIO_Toggle,       10U,   0U },  /* LED闪烁任务，判断程序是否正常运行 */
void Test_PWM_Ramp(void);    //{ Test_PWM_Ramp, 10U, 0U },
void Test_Encoder_Log(void); //{ Test_Encoder_Log,   200U, 0U },

void Test_Gray4051_Update(void);//{ Test_Gray4051_Update,  1U,   0U },
void Test_Gray4051_Log(void);//{ Test_Gray4051_Log,     200U, 0U },

void Test_Key_Update(void);//五按键事件测试：先注册 { Key_Update, 10U, 0U }，再注册本函数

void Test_EXTI_Init(void); //在主函数BSP_InitAll（）后，Scheduler_Init();前初始化才行
void Test_EXTI_Log(void);  //{ Test_EXTI_Log,         200U, 0U },

void Test_UART_Echo(void);//{ Test_UART_Echo,          5U,   0U },
void Test_UART_Stats(void);//{ Test_UART_Stats,        200U,   0U },
void Test_E220_Link_Update(void);//E220 双车通信测试；每秒发送递增数字并在 LCD/OLED 显示接收结果

void Test_I2C_Scan(void);//扫描i2c设备，主函数中调用一次即可

void Test_SPI2_LCD(void);//主函数调用一次即可

void Test_MotorCmd_Update(void);// 测试电机方向（开环测试）   { Test_MotorCmd_Update,  10U,  0U },
void Test_MotorCmd_Log(void);//    { Test_MotorCmd_Log,     200U, 0U },

void Test_ChassisCmd_Update(void);//五按键底盘速度控制测试；必须先注册Key_Update和Encoder_Update
void Test_ChassisWatchdog_Update(void);//底盘命令租约专项测试；与Test_ChassisCmd_Update二选一
void Test_ChassisCmd_Log(void);//打印日志   { Test_ChassisCmd_Log,   200U, 0U },

void Test_DrvEncoder_Log(void);//测试编码器方向    { Test_DrvEncoder_Log,   200U, 0U },

static void Test_CountPerRev_Print(void);//测试脉冲
void Test_CountPerRev_Update(void);// 测试一圈的脉冲数{ Test_CountPerRev_Update,10U,  0U },

void Test_MotionCmd_Update(void);//五按键动作库测试；必须先注册 { Key_Update,10U,0U }
void Test_MotionCmd_Log(void);//{ Test_MotionCmd_Log,     200U, 0U },

void Test_LineCmd_Update(void);// { Test_LineCmd_Update,   10U,   0U },
void Test_RouteCmd_Update(void);// KEY1 start, KEY4 stop; requires Key_Update before this task
void Test_RouteLog(void);//激活路线LCD/OLED调试页；同时注册对应显示刷新任务
void Test_LineCmd_Log(void);//测试巡线函数日志（包含灰度数据和校准后的灰度数据，巡线状态等等）{ Test_LineCmd_Log,     200U,   0U }, 

void Test_VL53L1X_Update(void);//VL53L1X日志测试 { Test_VL53L1X_Update, 200U, 0U },
void Test_HX711_Update(void);//HX711克重输出 { Test_HX711_Update, 20U, 0U }，内部每200ms打印

void Test_ICM20948_Update(void);//易读IMU日志 { Test_ICM20948_Update, 500U, 0U },
void Test_ICM20948_Mag_Update(void);//AK09916专项测试 { Test_ICM20948_Mag_Update, 500U, 0U },
void Test_Attitude_Update(void);//独立姿态融合日志与标定 { Test_Attitude_Update, 10U, 0U }，内部每500ms打印

void Test_LCD_Ascii_Update(void);//测试lcd{ Test_LCD_Ascii_Update, 50U, 0U },
void Test_OLED_Ascii_Update(void);//测试OLED{ Test_OLED_Ascii_Update, 20U, 0U },

void Test_AsyncDisplay_Update(void);//
void Test_K210_CommUpdate(void);//K210测试

void Test_DriveProfile_Update(void);

#endif
