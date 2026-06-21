#include "bsp_timer.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "bsp_led.h"
#include "bsp_uart.h"
#include "bsp_i2c.h"
#include "bsp_pwm.h"
#include "k210_protocol.h"
#include "task_scheduler.h"
#include "app_mpu_task.h"
#include "app_bmp280_task.h"
#include "app_vl53l1x_task.h"
#include "app_pmw3901_task.h"
#include "app_attitude_task.h"
#include "app_flight_data_task.h"
#include "app_altitude_estimator_task.h"
#include "app_rc_task.h"
#include "bsp_uart3.h"
#include "app_altitude_pid_task.h"
#include "app_alt_hold_task.h"
#include "app_attitude_pid_task.h"
#include "app_motor_mixer_task.h"
#include "app_motor_output_task.h"
#include "app_rc_command_task.h"
#include "app_takeoff_tune.h"

#define APP_MPU_CALIB_SAMPLES      500U   // MPU 静止校准采样数，越大越稳但启动越�?

#define APP_ENABLE_I2C_BOOT_SCAN   1U     // 上电扫描 I2C 设备，调试用
#define APP_ENABLE_RC_INPUT        1U     // 启用遥控�?CH1~CH5 输入读取
#define APP_ENABLE_RC_MOTOR_TEST   0U     // 旧的遥控器直控电机测试，使用新输出链路时必须�?
#define APP_ENABLE_UART3_ECHO      1U     // UART3 回显测试开�?

#define APP_DEBUG_MPU_RAW          1U     // 打印 MPU 数据，调试时可开，正式控制建议关
#define APP_DEBUG_ATTITUDE_RAW     0U     // 打印姿态角 ATT 原始输出
#define APP_DEBUG_BMP280_RAW       0U     // 打印 BMP280 气压计数�?
#define APP_DEBUG_VL53L1_RAW       0U     // 打印 VL53L1X 测距数据
#define APP_DEBUG_PMW3901_RAW      0U     // 打印 PMW3901 光流数据
#define APP_DEBUG_FLIGHT_DATA      0U     // 打印飞行数据汇�?FD
#define APP_DEBUG_ALTITUDE_EST     0U     // 打印高度估计 ALT
#define APP_DEBUG_ATTITUDE_PID     0U     // 打印姿�?PID 输出 ATT_PID
#define APP_DEBUG_MOTOR_MIXER      0U     // 打印电机混控 MIX
#define APP_DEBUG_K210_RAW         0U     // 打印 K210 巡线/二维码数�?
#define APP_DEBUG_RC               0U     // 打印遥控器原始通道
#define APP_DEBUG_MOTOR_OUTPUT     1U     // 打印电机输出和安全锁 MOUT
#define APP_DEBUG_RC_COMMAND       1U     // 打印遥控器转换后的目标量 RC_CMD

#define APP_ENABLE_MOTOR_OUTPUT    1U     // 启用新的电机输出�?
#define APP_MOTOR_OUTPUT_ARM       1U     // 软件总解锁开关，0 会强制不输出
#define APP_MOTOR_OUTPUT_WRITE_PWM 1U     // 是否真正�?PWM�? 只打印不转电�?

#define APP_ENABLE_ESC_CALIB        0U      // 1=进入电调校准模式�?=正常飞控模式

#define APP_ESC_CALIB_HIGH_US       2000U   // 电调校准最高油�?
#define APP_ESC_CALIB_LOW_US        1000U   // 电调校准最低油�?

extern void task_led_blink(void);

static void App_ESC_CalibrationMode(void);
static void App_BlockDelayMs(uint32_t ms);

static void Task_UART1Echo(void);
static void Task_K210Parse(void);
static void Task_K210DebugPrint(void);

static void App_BoardInit(void);
static void App_RegisterCoreTasks(void);
static void App_StartSensors(void);
static void App_StartFlightData(void);
static void App_StartAltitudeEstimator(void);
static void App_StartAltHold(void);
static void App_StartAttitudeEstimator(void);
static void App_StartMotorMixer(void);
static void App_StartMotorOutput(void);
static void App_StartRCCommand(void);
static void App_StartOptionalRc(void);
static void App_ServiceFastLoop(void);

//=========================输出四个电机参数===========================================
static void Task_MotorPwmDebugPrint(void);
static task_t s_task_motor_pwm_debug = {
    .func = Task_MotorPwmDebugPrint,
    .interval_ms = 200,
    .last_run = 0
};
static void Task_MotorPwmDebugPrint(void)
{
    char buf[128];

    snprintf(buf, sizeof(buf),
             "PWM CCR M1:%lu M2:%lu M3:%lu M4:%lu\r\n",
             (unsigned long)TIM1->CCR1,
             (unsigned long)TIM1->CCR2,
             (unsigned long)TIM1->CCR3,
             (unsigned long)TIM1->CCR4);

    UART1_SendData_NonBlocking((uint8_t *)buf, (uint16_t)strlen(buf));
}
//=========================================================================

static task_t s_task_uart1_echo = {
    .func = Task_UART1Echo,
    .interval_ms = 5,
    .last_run = 0
};

static task_t s_task_k210_parse = {
    .func = Task_K210Parse,
    .interval_ms = 5,
    .last_run = 0
};

static task_t s_task_led = {
    .func = task_led_blink,
    .interval_ms = 500,
    .last_run = 0
};

static task_t s_task_k210_debug = {
    .func = Task_K210DebugPrint,
    .interval_ms = 200,
    .last_run = 0
};

static void Task_UART1Echo(void)
{
    uint8_t ch;

    while (UART1_GetChar(&ch)) {
        UART1_SendData_NonBlocking(&ch, 1);
    }
}

static void Task_K210Parse(void)
{
    K210_ParseTask();
}

static void Task_K210DebugPrint(void)
{
    const k210_data_t *data;
    char buf[64];

    data = K210_GetData();
    if (data->line_valid) {
        snprintf(buf, sizeof(buf), "LINE: offset=%d\r\n", (int)data->line_offset);
        UART1_SendData_NonBlocking((uint8_t *)buf, (uint16_t)strlen(buf));
    }

    if (data->qr_valid) {
        snprintf(buf, sizeof(buf), "QR: x=%d y=%d\r\n", (int)data->qr_x, (int)data->qr_y);
        UART1_SendData_NonBlocking((uint8_t *)buf, (uint16_t)strlen(buf));
    }
}

static void App_BoardInit(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    led_init();
    UART1_Init();
    UART2_Init();
    K210_Init();
    I2C1_Init();
    UART3_Init();

    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1) {
        }
    }

    __enable_irq();

#if APP_ENABLE_I2C_BOOT_SCAN
    UART1_SendData_NonBlocking((uint8_t *)"I2C boot scan start\r\n", 21);
    I2C1_ScanDevices();
#endif
}

static void App_RegisterCoreTasks(void)
{
    scheduler_register(&s_task_led);
    scheduler_register(&s_task_uart1_echo);
    scheduler_register(&s_task_k210_parse);

#if APP_DEBUG_K210_RAW
    scheduler_register(&s_task_k210_debug);
#endif

//=============================================
	scheduler_register(&s_task_motor_pwm_debug);
//================================================
}

static void App_StartSensors(void)
{
    uint8_t ret;
    char buf[64];

    ret = App_MPU9250_InitAndCalibrate(GYRO_FS_500,
                                       ACCEL_FS_4,
                                       APP_MPU_CALIB_SAMPLES,
                                       MPU9250_CALIB_MODE_GYRO_ONLY);
    if (ret == 0U) {
        App_MPU9250_SetDebugPrint(APP_DEBUG_MPU_RAW);
        App_MPU9250_RegisterTasks();
    } else {
        snprintf(buf, sizeof(buf), "MPU9250 init fail: ret=%u\r\n", ret);
        UART1_SendData_NonBlocking((uint8_t *)buf, (uint16_t)strlen(buf));
    }

    ret = App_BMP280_Init();
    if (ret == 0U) {
        App_BMP280_SetDebugPrint(APP_DEBUG_BMP280_RAW);
        App_BMP280_RegisterTasks();
    }

    if (App_VL53L1_Init() == APP_VL53L1_OK) {
        App_VL53L1_SetDebugPrint(APP_DEBUG_VL53L1_RAW);
        App_VL53L1_RegisterTasks();
    }

    ret = App_PMW3901_Init();
    if (ret == 0U) {
        App_PMW3901_SetDebugPrint(APP_DEBUG_PMW3901_RAW);
        App_PMW3901_RegisterTasks();
    }
}

static void App_StartFlightData(void)
{
    App_FlightData_Init();
    App_FlightData_SetDebugPrint(APP_DEBUG_FLIGHT_DATA);
    App_FlightData_RegisterTasks();
}

static void App_StartAltitudeEstimator(void)
{
    App_AltitudeEstimator_Init();
    App_AltitudeEstimator_SetDebugPrint(APP_DEBUG_ALTITUDE_EST);
    App_AltitudeEstimator_RegisterTasks();
	
//========================高度测试==========

	
	App_AltitudePID_Init();
	App_AltitudePID_SetTargetHeight(TAKEOFF_ALT_TARGET_M);      // 目标高度 20cm
	App_AltitudePID_SetThrottleLimit(TAKEOFF_ALT_THROTTLE_MIN_US, TAKEOFF_ALT_THROTTLE_MAX_US);
	App_AltitudePID_SetBaseThrottleUs(TAKEOFF_ALT_BASE_THROTTLE_US);     // 只是建议油门，不会输出给电机
	App_AltitudePID_SetGains(TAKEOFF_ALT_KP, TAKEOFF_ALT_KI, TAKEOFF_ALT_KD);
	App_AltitudePID_SetOutputLimit(TAKEOFF_ALT_OUTPUT_MIN_US, TAKEOFF_ALT_OUTPUT_MAX_US);
	App_AltitudePID_SetDebugPrint(APP_DEBUG_ALTITUDE_EST);
	App_AltitudePID_SetEnable(1);
	App_AltitudePID_RegisterTasks();
//=====================================================
}

static void App_StartAltHold(void)
{
    App_AltHold_Init();
    App_AltHold_SetParam(TAKEOFF_ALT_HOLD_CENTER_US,
                         TAKEOFF_ALT_HOLD_DEADBAND_US,
                         TAKEOFF_ALT_HOLD_CLIMB_RATE_MPS,
                         TAKEOFF_ALT_HOLD_DESCEND_RATE_MPS,
                         TAKEOFF_ALT_HOLD_MIN_M,
                         TAKEOFF_ALT_HOLD_MAX_M,
                         TAKEOFF_ALT_HOLD_INIT_M);
    App_AltHold_SetEnable(TAKEOFF_ALT_HOLD_ENABLE);
    App_AltHold_SetDebugPrint(TAKEOFF_ALT_HOLD_DEBUG);
    App_AltHold_RegisterTasks();
}
static void App_StartAttitudeEstimator(void)
{
	App_Attitude_Init();
	App_Attitude_SetDebugPrint(APP_DEBUG_ATTITUDE_RAW);
	App_Attitude_RegisterTasks();
	App_AttitudePID_Init();

	App_AttitudePID_SetTargets(TAKEOFF_ATT_INIT_ROLL_DEG, TAKEOFF_ATT_INIT_PITCH_DEG, TAKEOFF_ATT_INIT_YAW_RATE_DPS);   // RCCommand will own runtime attitude targets.
	App_AttitudePID_SetAngleKp(TAKEOFF_ATT_ROLL_ANGLE_KP, TAKEOFF_ATT_PITCH_ANGLE_KP);
	App_AttitudePID_SetMaxAngleRate(TAKEOFF_ATT_MAX_ANGLE_RATE_DPS);

	App_AttitudePID_SetRateGains(TAKEOFF_RATE_ROLL_KP, TAKEOFF_RATE_ROLL_KI, TAKEOFF_RATE_ROLL_KD,
								 TAKEOFF_RATE_PITCH_KP, TAKEOFF_RATE_PITCH_KI, TAKEOFF_RATE_PITCH_KD,
								 TAKEOFF_RATE_YAW_KP, TAKEOFF_RATE_YAW_KI, TAKEOFF_RATE_YAW_KD);

	App_AttitudePID_SetOutputLimit(TAKEOFF_ATT_OUTPUT_MIN, TAKEOFF_ATT_OUTPUT_MAX);
	App_AttitudePID_SetIntegratorLimit(TAKEOFF_ATT_INTEGRATOR_MIN, TAKEOFF_ATT_INTEGRATOR_MAX);

	App_AttitudePID_SetDebugPrint(APP_DEBUG_ATTITUDE_PID);
	App_AttitudePID_SetEnable(1);
	App_AttitudePID_RegisterTasks();
}

static void App_StartMotorMixer(void)
{
    App_MotorMixer_Init();

    /*
     * 1 = 使用高度 PID �?throttle_cmd_us�?
     * 0 = 使用手动油门，方便只测试姿态混控�?
     */
    //App_MotorMixer_SetUseAltitudeThrottle(1);
#if TAKEOFF_ALT_HOLD_ENABLE
    App_MotorMixer_SetThrottleSource(APP_MOTOR_MIXER_THROTTLE_ALT);
#else
    App_MotorMixer_SetThrottleSource(APP_MOTOR_MIXER_THROTTLE_RC);
#endif

    /*
     * 如果只想测试姿态混控方向，不想依赖高度 PID�?
     * 可以把上面改�?0，然后这里用 1100us�?
     */
    App_MotorMixer_SetManualThrottleUs(TAKEOFF_MIX_MANUAL_THROTTLE_US);

    /* 当前只打印，仍然保守限制�?1000~1300us�?*/
    App_MotorMixer_SetMotorLimit(TAKEOFF_MIX_MOTOR_MIN_US, TAKEOFF_MIX_MOTOR_MAX_US);

    /*
     * 默认不反向�?
     * 后面如果发现某个轴修正方向反了，再把对应参数改成 1�?
     */
    App_MotorMixer_SetAxisReverse(1, 0,0);
    App_MotorMixer_SetAxisScale(1.0f, 1.0f, 1.0f);

    App_MotorMixer_SetDebugPrint(APP_DEBUG_MOTOR_MIXER);
    App_MotorMixer_SetEnable(1);
    App_MotorMixer_RegisterTasks();
}

static void App_StartMotorOutput(void)
{
    App_MotorOutput_Init();

    App_MotorOutput_SetEnable(APP_ENABLE_MOTOR_OUTPUT);
    App_MotorOutput_SetArm(APP_MOTOR_OUTPUT_ARM);
    App_MotorOutput_SetWritePwm(APP_MOTOR_OUTPUT_WRITE_PWM);

    App_MotorOutput_SetRcSafetyEnable(1);
    App_MotorOutput_SetRcThresholds(TAKEOFF_RC_SAFETY_THROTTLE_LOW_US, TAKEOFF_RC_SAFETY_ARM_ON_US, TAKEOFF_RC_SAFETY_ARM_OFF_US);

    App_MotorOutput_SetOutputLimit(TAKEOFF_MOTOR_OUTPUT_MIN_US, TAKEOFF_MOTOR_OUTPUT_MAX_US);
    App_MotorOutput_SetTiltSafety(TAKEOFF_TILT_CUTOFF_ENABLE, TAKEOFF_TILT_CUTOFF_DEG);
    App_MotorOutput_SetMotorScale(TAKEOFF_MOTOR_SCALE_M1,
                                  TAKEOFF_MOTOR_SCALE_M2,
                                  TAKEOFF_MOTOR_SCALE_M3,
                                  TAKEOFF_MOTOR_SCALE_M4);
    App_MotorOutput_SetMixerTimeout(TAKEOFF_MOTOR_MIXER_TIMEOUT_MS);

    App_MotorOutput_SetDebugPrint(APP_DEBUG_MOTOR_OUTPUT);
    App_MotorOutput_RegisterTasks();
}

static void App_StartRCCommand(void)
{
    App_RCCommand_Init();

    App_RCCommand_SetEnable(1);

    /* 第一次测试角度不要太�?*/
    App_RCCommand_SetAngleLimit(TAKEOFF_RC_MAX_ROLL_DEG, TAKEOFF_RC_MAX_PITCH_DEG);
    App_RCCommand_SetYawRateLimit(TAKEOFF_RC_MAX_YAW_RATE_DPS);

    /* CH3 映射到手动油门，第一次先保守 */
    App_RCCommand_SetThrottleRange(TAKEOFF_RC_THROTTLE_MIN_US, TAKEOFF_RC_THROTTLE_IDLE_US, TAKEOFF_RC_THROTTLE_MAX_US);

    /* 中位、死区、油门最低阈�?*/
    App_RCCommand_SetInputConfig(TAKEOFF_RC_CENTER_US, TAKEOFF_RC_DEADBAND_US, TAKEOFF_RC_THROTTLE_LOW_US);

    /* 如果 CH1/CH2/CH4 方向反了，后面改这里 */
    App_RCCommand_SetAxisReverse(1, 0, 1);
    App_RCCommand_SetLevelOffset(TAKEOFF_LEVEL_ROLL_OFFSET_DEG, TAKEOFF_LEVEL_PITCH_OFFSET_DEG);

    App_RCCommand_SetDebugPrint(APP_DEBUG_RC_COMMAND);
    App_RCCommand_RegisterTasks();
}

static void App_StartOptionalRc(void)
{
#if APP_ENABLE_RC_INPUT
    App_RC_Init();
    App_RC_SetDebugPrint(APP_DEBUG_RC);
    App_RC_RegisterTasks();
#endif

#if APP_ENABLE_RC_MOTOR_TEST
    App_RC_MotorTest_Init();
    App_RC_MotorTest_Enable(1);
    UART1_SendData_NonBlocking((uint8_t *)"RC motor test start\r\n", 21);
#endif
}

static void App_ServiceFastLoop(void)
{
#if APP_ENABLE_UART3_ECHO
    uint8_t ch;

    while (UART3_GetChar(&ch)) {
        UART3_SendByte_NonBlocking(ch);
    }
#endif
}

static void App_BlockDelayMs(uint32_t ms)
{
    uint32_t start = GetTick();

    while ((GetTick() - start) < ms) {
        App_ServiceFastLoop();
    }
}

static void App_ESC_CalibrationMode(void)
{
    uint8_t ch;

    UART1_SendData_NonBlocking((uint8_t *)"\r\n==== ESC CALIB MODE ====\r\n", 28);
    UART1_SendData_NonBlocking((uint8_t *)"REMOVE PROPELLERS FIRST!\r\n", 26);
    UART1_SendData_NonBlocking((uint8_t *)"Commands: h=HIGH 2000us, l=LOW 1000us, s=STOP 1000us\r\n", 64);
    UART1_SendData_NonBlocking((uint8_t *)"Power STM32 first. Keep ESC battery disconnected.\r\n", 50);

    PWM_Init();

    /* 上电默认先给低油门，避免程序一启动电机直接高速转 */
    PWM_SetAllMotorUs(APP_ESC_CALIB_LOW_US,
                      APP_ESC_CALIB_LOW_US,
                      APP_ESC_CALIB_LOW_US,
                      APP_ESC_CALIB_LOW_US);

    while (1) {
        if (UART1_GetChar(&ch)) {
            if (ch == 'h' || ch == 'H') {
                PWM_SetAllMotorUs(APP_ESC_CALIB_HIGH_US,
                                  APP_ESC_CALIB_HIGH_US,
                                  APP_ESC_CALIB_HIGH_US,
                                  APP_ESC_CALIB_HIGH_US);

                UART1_SendData_NonBlocking((uint8_t *)
                    "HIGH 2000us output. Now connect ESC battery and wait for high-throttle beeps.\r\n",
                    80);
            } else if (ch == 'l' || ch == 'L') {
                PWM_SetAllMotorUs(APP_ESC_CALIB_LOW_US,
                                  APP_ESC_CALIB_LOW_US,
                                  APP_ESC_CALIB_LOW_US,
                                  APP_ESC_CALIB_LOW_US);

                UART1_SendData_NonBlocking((uint8_t *)
                    "LOW 1000us output. Wait for confirmation beeps.\r\n",
                    52);
            } else if (ch == 's' || ch == 'S') {
                PWM_SetAllMotorUs(1000, 1000, 1000, 1000);

                UART1_SendData_NonBlocking((uint8_t *)
                    "STOP/HOLD 1000us.\r\n",
                    20);
            }
        }

        App_ServiceFastLoop();
    }
}

int main(void)
{
    App_BoardInit();
	
#if APP_ENABLE_ESC_CALIB
    App_ESC_CalibrationMode();
#endif
	
    App_RegisterCoreTasks();
	
    App_StartSensors();
	
    App_StartOptionalRc();
	App_StartRCCommand();       // 再把遥控器通道转成目标�?
	
    App_StartAltitudeEstimator();
	App_StartAltHold();
	App_StartAttitudeEstimator();
	App_StartMotorMixer();
	App_StartMotorOutput();
	
	App_StartFlightData();


    while (1) {
        scheduler_run();
        App_ServiceFastLoop();
    }
}
