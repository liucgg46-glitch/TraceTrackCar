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
#include "heading_estimator.h"
#include "line_follow_app.h"
#include "line_detect.h"
#include "line_track.h"
#include "drv_gray_sensor.h"
#include "drv_gray_mcu_i2c.h"
#include "drv_lcd_tft.h"
#include "drv_oled_i2c.h"

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

//测试代码，电机转速逐渐变快在变�?
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

//测试编码�?
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
 * 74HC4051 多路复用灰度模块最小测试代�?
 *
 * 硬件连接�?
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
    g_exti_count++;   /* 中断里只做计数，�?printf */
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
/* 按你的实�?GPIO 通道�?*/
void Test_SPI2_LCD(void)
{
    Drv_LcdTft_Init();
}

/*
 * 电机开环命令测试�?
 * 串口发送：
 *   w：四轮前�?300�?
 *   s：四轮后退 300�?
 *   a：原地左�?
 *   d：原地右�?
 *   0：停�?
 *
 * 注意：第一次测试必须架空小车，确认方向后再落地�?
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
 * 底盘速度闭环命令测试�?
 * 串口发送：
 *   g：目�?linear=600 cps, turn=0，直行闭�?
 *   b：目�?linear=-600 cps, turn=0，后退闭环
 *   l：原地左�?turn=-400 cps
 *   r：原地右�?turn=400 cps
 *   x �?0：停�?
 *
 * 测试前提�?
 *   1. 四路编码器方向已确认：前进时 cps 均为正；
 *   2. 四个电机方向已确认：�?PWM 时小车前进；
 *   3. 第一次闭环测试必须架空小车�?
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

//测试编码器脉�?
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
 * Part3 非阻塞动作库测试�?
 * 串口发送：
 *   f：前�?500mm
 *   v：后退 500mm
 *   L：左�?90°
 *   R：右�?90°
 *   x / 0：停止动作并停车
 *
 * 测试任务表必须包含：Encoder_Update、Motion_Update、Chassis_Update�?
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

//灰度传感器巡线测�?
/*
 * Part4 灰度循迹测试命令�?
 *   1：启动循�?
 *   0 / x：停止循迹并停车
 *   w：把当前 8 路灰度采样记录为白底
 *   b：把当前 8 路灰度采样记录为黑线
 *   t：根据白�?黑线记录生成阈�?
 *   d：恢复默认统一阈�?LINE_DETECT_DEFAULT_THRESHOLD
 *   p：立即打印一�?raw/threshold/mask/error/type/output
 *
 * 推荐标定流程�?
 *   1. �?8 路传感器都对着白底，发�?w�?
 *   2. �?8 路传感器都压在黑线上，发�?b�?
 *   3. 发�?t 生成阈值；
 *   4. 发�?p �?mask 是否合理�?
 *   5. 发�?1 开始循迹�?
 */

static const char *LineTypeName(LineType_t type)
{
    switch (type) {
        case LINE_TYPE_LOST:         return "LOST";
        case LINE_TYPE_SINGLE:       return "SINGLE";
        case LINE_TYPE_LEFT_BRANCH:  return "LEFT";
        case LINE_TYPE_RIGHT_BRANCH: return "RIGHT";
        case LINE_TYPE_CROSS:        return "CROSS";
        case LINE_TYPE_FULL_BLACK:   return "FULL";
        default:                     return "?";
    }
}

static void Test_Line_Print(void)
{
    LineFollow_Info_t info;
    Drv_GraySensor_Info_t sensor;
    Drv_GrayMcu_Info_t mcu;
    BSP_I2C_Debug_t i2c;
    uint16_t th[LINE_DETECT_SENSOR_NUM];
    char buf[192];
    int n;

    if (LineFollow_GetInfo(&info) != BSP_OK) return;
    if (Drv_GraySensor_GetInfo(&sensor) != BSP_OK) return;
    (void)Drv_GrayMcu_GetInfo(&mcu);
    (void)BSP_I2C_GetDebug(I2C_BUS1, &i2c);
    (void)LineDetect_GetThresholdArray(th, LINE_DETECT_SENSOR_NUM);

    n = sprintf(buf,
                "GRAY src=%u on=%u valid=%u busy=%u st=%d i2c=%d op=%u upd=%lu fw=0x%02X gaddr=0x%02X ph=%u lph=%u dph=%u scan=%u/0x%02X err=%lu\r\n",
                (unsigned int)sensor.source,
                (unsigned int)sensor.online,
                (unsigned int)mcu.valid,
                (unsigned int)Drv_GrayMcu_IsBusy(),
                (int)mcu.last_status,
                mcu.last_i2c_result,
                (unsigned int)mcu.last_op,
                (unsigned long)mcu.update_count,
                (unsigned int)mcu.firmware_version,
                (unsigned int)mcu.active_addr,
                (unsigned int)mcu.current_phase,
                (unsigned int)mcu.last_phase,
                (unsigned int)mcu.done_phase,
                (unsigned int)mcu.scan_count,
                (unsigned int)mcu.scan_mask,
                (unsigned long)mcu.error_count);
    if (n > 0 && n < (int)sizeof(buf)) {
        (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
    }

    n = sprintf(buf,
                "I2C s=%u a=0x%02X tx=%u/%u rx=%u rd=%u sr1=0x%04X sr2=0x%04X\r\n",
                (unsigned int)i2c.state,
                (unsigned int)i2c.dev_addr,
                (unsigned int)i2c.tx_pos,
                (unsigned int)i2c.tx_len,
                (unsigned int)i2c.rx_len,
                (unsigned int)i2c.need_read,
                (unsigned int)i2c.sr1,
                (unsigned int)i2c.sr2);
    if (n > 0 && n < (int)sizeof(buf)) {
        (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
    }

    n = sprintf(buf,
                "OLED r=%u b=%u st=%d e=%u | LINE st=%d type=%s mask=0x%02X cnt=%d err=%d out(v=%d,t=%d)\r\n",
                (unsigned int)Drv_OledI2c_IsReady(),
                (unsigned int)Drv_OledI2c_IsBusy(),
                (int)Drv_OledI2c_GetLastStatus(),
                (unsigned int)Drv_OledI2c_GetErrorCount(),
                (int)info.state,
                LineTypeName(info.detect.type),
                (unsigned int)info.detect.black_mask,
                (int)info.detect.black_count,
                (int)info.detect.error_x1000,
                (int)info.output.linear_cps,
                (int)info.output.turn_cps);
    if (n > 0 && n < (int)sizeof(buf)) {
        (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
    }

    n = sprintf(buf,
                "RAW %u %u %u %u %u %u %u %u\r\n",
                (unsigned int)sensor.raw[0], (unsigned int)sensor.raw[1],
                (unsigned int)sensor.raw[2], (unsigned int)sensor.raw[3],
                (unsigned int)sensor.raw[4], (unsigned int)sensor.raw[5],
                (unsigned int)sensor.raw[6], (unsigned int)sensor.raw[7]);
    if (n > 0 && n < (int)sizeof(buf)) {
        (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
    }

    n = sprintf(buf,
                "FLT %u %u %u %u %u %u %u %u\r\n",
                (unsigned int)sensor.filt[0], (unsigned int)sensor.filt[1],
                (unsigned int)sensor.filt[2], (unsigned int)sensor.filt[3],
                (unsigned int)sensor.filt[4], (unsigned int)sensor.filt[5],
                (unsigned int)sensor.filt[6], (unsigned int)sensor.filt[7]);
    if (n > 0 && n < (int)sizeof(buf)) {
        (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
    }

    n = sprintf(buf,
                "TH  %u %u %u %u %u %u %u %u\r\n",
                (unsigned int)th[0], (unsigned int)th[1],
                (unsigned int)th[2], (unsigned int)th[3],
                (unsigned int)th[4], (unsigned int)th[5],
                (unsigned int)th[6], (unsigned int)th[7]);
    if (n > 0 && n < (int)sizeof(buf)) {
        (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
    }
}
void Test_LineCmd_Update(void)
{
    uint8_t ch;
    uint16_t raw[LINE_DETECT_SENSOR_NUM];

    while (BSP_UART_GetChar(UART_PORT1, &ch)) {
        if (ch == '1') {
            LineFollow_Start();
            (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)"line follow start\r\n", (uint16_t)(sizeof("line follow start\r\n") - 1U));
        } else if (ch == '0' || ch == 'x') {
            LineFollow_Stop();
            (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)"line follow stop\r\n", (uint16_t)(sizeof("line follow stop\r\n") - 1U));
        } else if (ch == 'w') {
            (void)Drv_GraySensor_GetFiltArray(raw, LINE_DETECT_SENSOR_NUM);
            LineDetect_CaptureWhite(raw);
            (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)"capture white ok\r\n", (uint16_t)(sizeof("capture white ok\r\n") - 1U));
        } else if (ch == 'b') {
            (void)Drv_GraySensor_GetFiltArray(raw, LINE_DETECT_SENSOR_NUM);
            LineDetect_CaptureBlack(raw);
            (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)"capture black ok\r\n", (uint16_t)(sizeof("capture black ok\r\n") - 1U));
        } else if (ch == 't') {
            LineDetect_MakeThresholdFromWhiteBlack();
            (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)"make threshold ok\r\n", (uint16_t)(sizeof("make threshold ok\r\n") - 1U));
        } else if (ch == 'd') {
            LineDetect_SetAllThreshold(LINE_DETECT_DEFAULT_THRESHOLD);
            (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)"default threshold\r\n", (uint16_t)(sizeof("default threshold\r\n") - 1U));
        } else if (ch == 'p') {
            Test_Line_Print();
        }
    }
}

void Test_LineCmd_Log(void)
{
    Test_Line_Print();
}

void Test_LCD_Ascii_Update(void)
{
    Test_AsyncDisplay_Update();
}

void Test_AsyncDisplay_Update(void)
{
    static uint32_t last_oled_ms = 0U;
    static uint32_t last_lcd_ms = 0U;
    static uint32_t oled_cnt = 0U;
    static uint32_t lcd_cnt = 0U;
    static uint8_t lcd_base_done = 0U;
    static uint8_t lcd_base_step = 0U;
    static uint8_t lcd_refresh_pending = 0U;
    static uint8_t lcd_step = 0U;
    uint16_t bar_w;
    char buf[32];
    BSP_Status_t ret = BSP_BUSY;

    if ((Drv_OledI2c_IsReady() != 0U) &&
        (Drv_OledI2c_IsBusy() == 0U) &&
        (BSP_TimeElapsed(&last_oled_ms, 200U) != 0U)) {
        oled_cnt++;
        Drv_OledI2c_Clear();
        Drv_OledI2c_DrawRect(0U, 0U, 128U, 64U, DRV_OLED_COLOR_ON);
        Drv_OledI2c_DrawString5x7(6U, 6U,  "OLED TEST", DRV_OLED_COLOR_ON);
        Drv_OledI2c_DrawString5x7(6U, 18U, "I2C DMA OK", DRV_OLED_COLOR_ON);
        (void)sprintf(buf, "CNT:%lu", (unsigned long)oled_cnt);
        Drv_OledI2c_DrawString5x7(6U, 32U, buf, DRV_OLED_COLOR_ON);
        bar_w = (uint16_t)(8U + ((oled_cnt * 5U) % 104U));
        Drv_OledI2c_DrawRect(6U, 48U, 116U, 10U, DRV_OLED_COLOR_ON);
        Drv_OledI2c_FillRect(8U, 50U, (uint8_t)bar_w, 6U, DRV_OLED_COLOR_ON);
        Drv_OledI2c_Flush();
    }

    if ((Drv_LcdTft_IsReady() == 0U) || (Drv_LcdTft_IsBusy() != 0U)) {
        return;
    }

    if (lcd_base_done == 0U) {
        switch (lcd_base_step) {
            case 0U:
                ret = Drv_LcdTft_TryClear(DRV_LCD_COLOR_BLACK);
                break;

            case 1U:
                ret = Drv_LcdTft_TryDrawRect(4U, 4U, 232U, 232U, DRV_LCD_COLOR_BLUE);
                break;

            case 2U:
                ret = Drv_LcdTft_TryDrawString5x7(16U, 18U, "LCD DISPLAY TEST",
                                                  DRV_LCD_COLOR_WHITE,
                                                  DRV_LCD_COLOR_BLACK);
                break;

            case 3U:
                ret = Drv_LcdTft_TryDrawString5x7(16U, 38U, "SPI2 DMA ASYNC",
                                                  DRV_LCD_COLOR_CYAN,
                                                  DRV_LCD_COLOR_BLACK);
                break;

            case 4U:
                ret = Drv_LcdTft_TryDrawString5x7(16U, 58U, "REALTIME REFRESH",
                                                  DRV_LCD_COLOR_YELLOW,
                                                  DRV_LCD_COLOR_BLACK);
                break;

            default:
                lcd_base_done = 1U;
                lcd_step = 0U;
                return;
        }

        if (ret == BSP_OK) {
            lcd_base_step++;
        }
        return;
    }

    if ((lcd_refresh_pending == 0U) &&
        (BSP_TimeElapsed(&last_lcd_ms, 200U) != 0U)) {
        lcd_cnt++;
        lcd_refresh_pending = 1U;
        lcd_step = 0U;
    }

    if (lcd_refresh_pending == 0U) {
        return;
    }

    switch (lcd_step) {
        case 0U:
            (void)sprintf(buf, "CNT:%lu        ", (unsigned long)lcd_cnt);
            ret = Drv_LcdTft_TryDrawString5x7(16U, 88U, buf,
                                              DRV_LCD_COLOR_WHITE,
                                              DRV_LCD_COLOR_BLACK);
            break;

        case 1U:
            ret = Drv_LcdTft_TryFillRect(16U, 114U, 208U, 14U, DRV_LCD_COLOR_BLACK);
            break;

        case 2U:
            bar_w = (uint16_t)(8U + ((lcd_cnt * 9U) % 200U));
            ret = Drv_LcdTft_TryFillRect(16U, 114U, bar_w, 14U, DRV_LCD_COLOR_GREEN);
            break;

        case 3U:
            (void)sprintf(buf, "ERR:%u ST:%u     ",
                          (unsigned int)Drv_LcdTft_GetErrorCount(),
                          (unsigned int)Drv_LcdTft_GetAsyncStage());
            ret = Drv_LcdTft_TryDrawString5x7(16U, 146U, buf,
                                              DRV_LCD_COLOR_CYAN,
                                              DRV_LCD_COLOR_BLACK);
            break;

        default:
            lcd_refresh_pending = 0U;
            return;
    }

    if (ret == BSP_OK) {
        lcd_step++;
    }
}