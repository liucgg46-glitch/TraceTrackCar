#include "test.h"
#include "bsp_gpio.h"
#include "bsp_pwm.h"
#include "bsp_encoder.h"
#include "bsp_adc.h"
#include "bsp_key.h"
#include "bsp_exti.h"
#include "bsp_uart.h"
#include "bsp_i2c.h"
#include "bsp_spi.h"

#include "drv_motor.h"
#include "drv_encoder.h"
#include "chassis.h"
#include "motion_action.h"
#include "odometer.h"
#include "angle_control.h"

#include <stdio.h>
#include <stdint.h>

#include "bsp_systick.h"

//测试函数，OLED闪烁
void Test_GPIO_Toggle(void)
{
    static uint32_t last = 0;

    if (BSP_TimeElapsed(&last, 500U)) {
        BSP_GPIO_Toggle(BSP_GPIO_CH1);
    }
}

//测试代码，电机转速逐渐变快在变慢
void Test_PWM_Ramp(void)
{
    static uint32_t last = 0;
    static uint16_t duty = 0;
    static int8_t dir = 1;

    if (!BSP_TimeElapsed(&last, 20U)) {
        return;
    }

    BSP_PWM_SetDutyPermille(BSP_PWM_CH1, duty);

    if (dir > 0) {
        duty += 10;
        if (duty >= 1000U) {
            duty = 1000U;
            dir = -1;
        }
    } else {
        if (duty >= 10U) duty -= 10U;
        else {
            duty = 0U;
            dir = 1;
        }
    }
}

//测试编码器
void Test_Encoder_Log(void)
{
    char buf[96];
    int n;

    n = sprintf(buf,
                "ENC L: d=%d cps=%d total=%d | R: d=%d cps=%d total=%d\r\n",
                BSP_Encoder_GetDelta(BSP_ENCODER_CH1),
                BSP_Encoder_GetSpeedCps(BSP_ENCODER_CH1),
                BSP_Encoder_GetTotal(BSP_ENCODER_CH1),
                BSP_Encoder_GetDelta(BSP_ENCODER_CH2),
                BSP_Encoder_GetSpeedCps(BSP_ENCODER_CH2),
                BSP_Encoder_GetTotal(BSP_ENCODER_CH2));

    BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
}

/*
 * 74HC4051 多路复用灰度模块最小测试代码
 *
 * 硬件连接：
 *   灰度 OUT/SIG/AO -> PC0 / ADC1_IN10 / BSP_ADC_CH1
 *   灰度 S0        -> PD10 / BSP_GPIO_GRAY_S0
 *   灰度 S1        -> PD11 / BSP_GPIO_GRAY_S1
 *   灰度 S2        -> PD12 / BSP_GPIO_GRAY_S2
 *
 * 任务表建议：
 *   { AppTask_BSP_Background, 1U,   0U },
 *   { Test_Gray4051_Update,  1U,   0U },
 *   { Test_Gray4051_Log,     200U, 0U },
 */

#define GRAY_ADC_READ_RAW()   BSP_ADC_GetRaw(BSP_ADC_CH1)

static uint16_t s_gray_raw[8];
static uint8_t  s_gray_channel = 0U;
static uint8_t  s_gray_phase = 0U;

static void Gray4051_Select(uint8_t ch)
{
    BSP_GPIO_Write(BSP_GPIO_GRAY_S0, (ch & 0x01U) ? 1U : 0U);
    BSP_GPIO_Write(BSP_GPIO_GRAY_S1, (ch & 0x02U) ? 1U : 0U);
    BSP_GPIO_Write(BSP_GPIO_GRAY_S2, (ch & 0x04U) ? 1U : 0U);
}

void Test_Gray4051_Update(void)
{
    if (s_gray_phase == 0U) {
        Gray4051_Select(s_gray_channel);
        s_gray_phase = 1U;
    } else {
        s_gray_raw[s_gray_channel] = GRAY_ADC_READ_RAW();

        s_gray_channel++;
        if (s_gray_channel >= 8U) {
            s_gray_channel = 0U;
        }

        s_gray_phase = 0U;
    }
}

void Test_Gray4051_Log(void)
{
    char buf[160];
    int n;

    n = sprintf(buf,
                "GRAY: %4u %4u %4u %4u %4u %4u %4u %4u\r\n",
                s_gray_raw[0], s_gray_raw[1], s_gray_raw[2], s_gray_raw[3],
                s_gray_raw[4], s_gray_raw[5], s_gray_raw[6], s_gray_raw[7]);

    if (n > 0 && n < (int)sizeof(buf)) {
        (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
    }
}

void Test_Key_LED(void)
{
    if (BSP_Key_WasPressed(BSP_KEY1)) {
        BSP_GPIO_Toggle(BSP_GPIO_CH1);
    }
}

/*
*中断测试
*/
static volatile uint32_t g_exti_count = 0;

static void Test_EXTI_Callback(void *ctx)
{
    (void)ctx;
    g_exti_count++;   /* 中断里只做计数，不 printf */
}

void Test_EXTI_Init(void)
{
    BSP_EXTI_AttachCallback(BSP_EXTI_CH1, Test_EXTI_Callback, 0);
}

void Test_EXTI_Log(void)
{
    char buf[64];
    int n = sprintf(buf, "EXTI count=%lu\r\n", (unsigned long)g_exti_count);
    BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
}

//串口测试
void Test_UART_Echo(void)
{
    uint8_t ch;

    while (BSP_UART_GetChar(UART_PORT1, &ch)) {
        BSP_UART_WriteFrame(UART_PORT1, &ch, 1U);
    }
}

void Test_UART_Stats(void)
{
    UART_Stats_t st;
    char buf[96];
    int n;

    if (BSP_UART_GetStats(UART_PORT1, &st) != BSP_OK) {
        return;
    }

    n = sprintf(buf, "UART rx=%u tx=%u ov=%u drop=%u\r\n",
                st.rx_count, st.tx_count, st.rx_overflow, st.tx_drop);
    BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
}

//测试i2c
void Test_I2C_Scan(void)
{
    uint8_t addr[16];
    uint8_t found = 0;
    char buf[128];
    int n;

    if (BSP_I2C_ScanBus(I2C_BUS1, addr, 16, &found) != BSP_OK) {
        BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)"I2C scan error\r\n", 16);
        return;
    }

    n = sprintf(buf, "I2C found %u:", found);
    for (uint8_t i = 0; i < found; i++) {
        n += sprintf(&buf[n], " 0x%02X", addr[i]);
    }
    n += sprintf(&buf[n], "\r\n");

    BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
}

//spi测试
/* 按你的实际 GPIO 通道改 */
#define LCD_CS_LOW()     BSP_GPIO_Write(BSP_GPIO_CH2, 0)
#define LCD_CS_HIGH()    BSP_GPIO_Write(BSP_GPIO_CH2, 1)

#define LCD_DC_CMD()     BSP_GPIO_Write(BSP_GPIO_CH3, 0)
#define LCD_DC_DATA()    BSP_GPIO_Write(BSP_GPIO_CH3, 1)

#define LCD_BL_ON()      BSP_GPIO_Write(BSP_GPIO_CH5, 1)

/* 如果你这个是 1.8寸常见彩屏，通常是 128x160 */
#define LCD_W            240
#define LCD_H            280

static void lcd_delay(volatile uint32_t t)
{
    while (t--) {
        __NOP();
    }
}

static void LCD_WriteCmd(uint8_t cmd)
{
    LCD_CS_LOW();
    LCD_DC_CMD();
    BSP_SPI_ReadWriteByte(SPI_BUS2, cmd);
    LCD_CS_HIGH();
}

static void LCD_WriteData(uint8_t data)
{
    LCD_CS_LOW();
    LCD_DC_DATA();
    BSP_SPI_ReadWriteByte(SPI_BUS2, data);
    LCD_CS_HIGH();
}

static void LCD_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    LCD_WriteCmd(0x2A);
    LCD_WriteData(x0 >> 8);
    LCD_WriteData(x0);
    LCD_WriteData(x1 >> 8);
    LCD_WriteData(x1);

    LCD_WriteCmd(0x2B);
    LCD_WriteData(y0 >> 8);
    LCD_WriteData(y0);
    LCD_WriteData(y1 >> 8);
    LCD_WriteData(y1);

    LCD_WriteCmd(0x2C);
}

void Test_SPI2_LCD(void)
{
    uint32_t i;

    BSP_SPI_Init(SPI_BUS2);

    LCD_CS_HIGH();
    LCD_DC_DATA();
    LCD_BL_ON();

    /* 没有RST脚，上电后先等一会儿 */
    lcd_delay(3000000);

    LCD_WriteCmd(0x11);      /* Sleep Out */
    lcd_delay(3000000);

    LCD_WriteCmd(0x3A);      /* 颜色格式 */
    LCD_WriteData(0x55);     /* RGB565 */

    LCD_WriteCmd(0x29);      /* Display On */
    lcd_delay(1000000);

    LCD_SetWindow(0, 0, LCD_W - 1, LCD_H - 1);

    LCD_CS_LOW();
    LCD_DC_DATA();

    for (i = 0; i < (uint32_t)LCD_W * LCD_H; i++) {
        BSP_SPI_ReadWriteByte(SPI_BUS2, 0xF8);   /* 红色高字节 */
        BSP_SPI_ReadWriteByte(SPI_BUS2, 0x00);   /* 红色低字节 */
    }

    LCD_CS_HIGH();
}

/*
 * 电机开环命令测试。
 * 串口发送：
 *   w：四轮前进 300‰
 *   s：四轮后退 300‰
 *   a：原地左转
 *   d：原地右转
 *   0：停止
 *
 * 注意：第一次测试必须架空小车，确认方向后再落地。
 */
void Test_MotorCmd_Update(void)
{
    uint8_t ch;

    while (BSP_UART_GetChar(UART_PORT1, &ch)) {
        if (ch == 'w') {
            Motor_SetAllPermille(300, 300, 300, 300);
        } else if (ch == 's') {
            Motor_SetAllPermille(-300, -300, -300, -300);
        } else if (ch == 'a') {
            Motor_SetAllPermille(-300, 300, -300, 300);
        } else if (ch == 'd') {
            Motor_SetAllPermille(300, -300, 300, -300);
        } else if (ch == '0') {
            Motor_StopAll();
        }
    }
}

void Test_MotorCmd_Log(void)
{
    char buf[96];
    int16_t pwm[MOTOR_COUNT];
    int n;

    if (Motor_GetAllLastPermille(pwm) != BSP_OK) return;

    n = sprintf(buf,
                "MOTOR pwm: FL=%d FR=%d RL=%d RR=%d\r\n",
                pwm[MOTOR_FL], pwm[MOTOR_FR], pwm[MOTOR_RL], pwm[MOTOR_RR]);

    if (n > 0 && n < (int)sizeof(buf)) {
        (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
    }
}

void Test_DrvEncoder_Log(void)
{
    char buf[192];
    int n;

    n = sprintf(buf,
                "ENC FL=%d FR=%d RL=%d RR=%d | L=%d R=%d\r\n",
                Drv_Encoder_GetWheelSpeedCps(WHEEL_FL),
                Drv_Encoder_GetWheelSpeedCps(WHEEL_FR),
                Drv_Encoder_GetWheelSpeedCps(WHEEL_RL),
                Drv_Encoder_GetWheelSpeedCps(WHEEL_RR),
                Drv_Encoder_GetLeftSpeedCps(),
                Drv_Encoder_GetRightSpeedCps());

    if (n > 0 && n < (int)sizeof(buf)) {
        BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
    }
}

/*
 * 底盘速度闭环命令测试。
 * 串口发送：
 *   g：目标 linear=600 cps, turn=0，直行闭环
 *   b：目标 linear=-600 cps, turn=0，后退闭环
 *   l：原地左转 turn=-400 cps
 *   r：原地右转 turn=400 cps
 *   x 或 0：停止
 *
 * 测试前提：
 *   1. 四路编码器方向已确认：前进时 cps 均为正；
 *   2. 四个电机方向已确认：正 PWM 时小车前进；
 *   3. 第一次闭环测试必须架空小车。
 */
void Test_ChassisCmd_Update(void)
{
    uint8_t ch;

    while (BSP_UART_GetChar(UART_PORT1, &ch)) {
        if (ch == 'g') {
            Chassis_SetSpeed(600, 0);
        } else if (ch == 'b') {
            Chassis_SetSpeed(-600, 0);
        } else if (ch == 'l') {
            Chassis_SetSpeed(0, 400);
        } else if (ch == 'r') {
            Chassis_SetSpeed(0, -400);
        } else if (ch == 'x' || ch == '0') {
            Chassis_Stop();
        }
    }
}

void Test_ChassisCmd_Log(void)
{
    Chassis_Info_t info;
    char buf[192];
    int n;

    if (Chassis_GetInfo(&info) != BSP_OK) return;

    n = sprintf(buf,
                "CHS mode=%d tgt L=%d R=%d | fb FL=%d FR=%d RL=%d RR=%d | out %d %d %d %d\r\n",
                (int)info.mode,
                info.left_target_cps,
                info.right_target_cps,
                info.fl_feedback_cps,
                info.fr_feedback_cps,
                info.rl_feedback_cps,
                info.rr_feedback_cps,
                info.fl_output,
                info.fr_output,
                info.rl_output,
                info.rr_output);

    if (n > 0 && n < (int)sizeof(buf)) {
        (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
    }
}

//测试编码器脉冲
static void Test_CountPerRev_Print(void)
{
    char buf[192];
    int n;

    n = sprintf(buf,
                "TOTAL FL=%ld FR=%ld RL=%ld RR=%ld\r\n",
                (long)Drv_Encoder_GetWheelTotalCount(WHEEL_FL),
                (long)Drv_Encoder_GetWheelTotalCount(WHEEL_FR),
                (long)Drv_Encoder_GetWheelTotalCount(WHEEL_RL),
                (long)Drv_Encoder_GetWheelTotalCount(WHEEL_RR));

    if (n > 0 && n < (int)sizeof(buf)) {
        BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
    }
}

void Test_CountPerRev_Update(void)
{
    uint8_t ch;

    while (BSP_UART_GetChar(UART_PORT1, &ch)) {
        if (ch == 'c' || ch == 'C') {
            Drv_Encoder_ClearAllTotal();
            BSP_UART_WriteFrame(UART_PORT1,
                                (const uint8_t *)"encoder total cleared\r\n",
                                23);
        } else if (ch == 'p' || ch == 'P') {
            Test_CountPerRev_Print();
        }
    }
}

/*
 * Part3 非阻塞动作库测试。
 * 串口发送：
 *   f：前进 500mm
 *   v：后退 500mm
 *   L：左转 90°
 *   R：右转 90°
 *   x / 0：停止动作并停车
 *
 * 测试任务表必须包含：Encoder_Update、Motion_Update、Chassis_Update。
 */
void Test_MotionCmd_Update(void)
{
    uint8_t ch;

    while (BSP_UART_GetChar(UART_PORT1, &ch)) {
        if (ch == 'f') {
            (void)Motion_GoDistance(500, 800);
        } else if (ch == 'v') {
            (void)Motion_GoDistance(-500, 800);
        } else if (ch == 'L') {
            (void)Motion_TurnAngle(90, 600);
        } else if (ch == 'R') {
            (void)Motion_TurnAngle(-90, 600);
        } else if (ch == 'x' || ch == '0') {
            Motion_Stop();
        }
    }
}

void Test_MotionCmd_Log(void)
{
    Motion_Info_t motion;
    Chassis_Info_t chassis;
    char buf[256];
    int n;

    if (Motion_GetInfo(&motion) != BSP_OK) return;
    if (Chassis_GetInfo(&chassis) != BSP_OK) return;

    n = sprintf(buf,
                "MOT st=%d act=%d dist=%ld/%ld yaw=%d tgtL=%d tgtR=%d out=%d %d %d %d\r\n",
                (int)motion.state,
                (int)motion.action,
                (long)motion.current_distance_mm,
                (long)motion.target_distance_mm,
                (int)motion.current_yaw_deg,
                chassis.left_target_cps,
                chassis.right_target_cps,
                chassis.fl_output,
                chassis.fr_output,
                chassis.rl_output,
                chassis.rr_output);

    if (n > 0 && n < (int)sizeof(buf)) {
        (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
    }
}

