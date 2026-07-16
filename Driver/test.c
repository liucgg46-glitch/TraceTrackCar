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
#include "sensor_manager.h"
#include "odometer.h"
#include "attitude_estimator.h"
#include "heading_estimator.h"
#include "line_follow_app.h"
#include "line_detect.h"
#include "line_track.h"
#include "drv_gray_sensor.h"
#include "drv_gray_mcu_i2c.h"
#include "drv_lcd_tft.h"
#include "drv_oled_i2c.h"
#include "drv_vl53l1x.h"
#include "drv_icm20948.h"

#include <stdio.h>
#include <stdint.h>

#include "bsp_systick.h"
#include "test.h"

#include "k210_comm.h"
#include "bsp_uart.h"
#include "bsp_systick.h"

#include <stdio.h>

void Test_K210_CommUpdate(void)
{
    static uint32_t last_status_ms = 0U;

    uint8_t digit;
    uint8_t valid;
    uint8_t confidence;

    K210_Comm_Info_t info;

    char buf[160];
    int n;

    /*
     * 读取新的数字结果。
     */
    if (K210_Comm_GetNewDigit(
            &digit,
            &valid,
            &confidence) == BSP_OK) {
        n = snprintf(
            buf,
            sizeof(buf),
            "NEW DIGIT=%u valid=%u confidence=%u\r\n",
            (unsigned int)digit,
            (unsigned int)valid,
            (unsigned int)confidence
        );

        if ((n > 0) &&
            (n < (int)sizeof(buf))) {
            (void)BSP_UART_WriteFrame(
                UART_PORT1,
                (const uint8_t *)buf,
                (uint16_t)n
            );
        }
    }

    /*
     * 每500ms打印一次通信状态。
     */
    if ((uint32_t)(
            BSP_GetTickMs() -
            last_status_ms
        ) < 500U) {
        return;
    }

    last_status_ms = BSP_GetTickMs();

    if (K210_Comm_GetInfo(&info) != BSP_OK) {
        return;
    }

    n = snprintf(
        buf,
        sizeof(buf),
        "K210 online=%u frames=%lu checksum_err=%lu format_err=%lu last_rx=%lu\r\n",
        (unsigned int)info.online,
        (unsigned long)info.valid_frame_count,
        (unsigned long)info.checksum_error_count,
        (unsigned long)info.format_error_count,
        (unsigned long)info.last_rx_ms
    );

    if ((n > 0) &&
        (n < (int)sizeof(buf))) {
        (void)BSP_UART_WriteFrame(
            UART_PORT1,
            (const uint8_t *)buf,
            (uint16_t)n
        );
    }
}





//娴嬭瘯鍑芥暟锛孫LED闂儊
void Test_GPIO_Toggle(void)
{
    static uint32_t last = 0;

    if (BSP_TimeElapsed(&last, 500U)) {
        BSP_GPIO_Toggle(BSP_GPIO_CH1);
    }
}

//娴嬭瘯浠ｇ爜锛岀數鏈鸿浆閫熼€愭笎鍙樺揩鍦ㄥ彉鎱?
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

//娴嬭瘯缂栫爜鍣?
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
 * 74HC4051 澶氳矾澶嶇敤鐏板害妯″潡鏈€灏忔祴璇曚唬鐮?
 *
 * 纭欢杩炴帴锛?
 *   鐏板害 OUT/SIG/AO -> PC0 / ADC1_IN10 / BSP_ADC_CH1
 *   鐏板害 S0        -> PD10 / BSP_GPIO_GRAY_S0
 *   鐏板害 S1        -> PD11 / BSP_GPIO_GRAY_S1
 *   鐏板害 S2        -> PD12 / BSP_GPIO_GRAY_S2
 *
 * 浠诲姟琛ㄥ缓璁細
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
*涓柇娴嬭瘯
*/
static volatile uint32_t g_exti_count = 0;

static void Test_EXTI_Callback(void *ctx)
{
    (void)ctx;
    g_exti_count++;   /* 涓柇閲屽彧鍋氳鏁帮紝涓?printf */
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

//涓插彛娴嬭瘯
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

//娴嬭瘯i2c
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

//spi娴嬭瘯
/* 鎸変綘鐨勫疄闄?GPIO 閫氶亾鏀?*/
void Test_SPI2_LCD(void)
{
    Drv_LcdTft_Init();
}

void Test_DriveProfile_Update(void)
{
    static uint8_t printed = 0U;
    char line[180];
    int length;

    if (printed != 0U) return;
    printed = 1U;

    length = snprintf(line,
                      sizeof(line),
                      "DRIVE PROFILE=%s motor FL/FR/RL/RR=%u/%u/%u/%u encoder=%u/%u/%u/%u SPI1_pins_free=%u\r\n",
                      VEHICLE_DRIVE_MODE_NAME,
                      (unsigned int)Motor_IsEnabled(MOTOR_FL),
                      (unsigned int)Motor_IsEnabled(MOTOR_FR),
                      (unsigned int)Motor_IsEnabled(MOTOR_RL),
                      (unsigned int)Motor_IsEnabled(MOTOR_RR),
                      (unsigned int)Drv_Encoder_IsWheelEnabled(WHEEL_FL),
                      (unsigned int)Drv_Encoder_IsWheelEnabled(WHEEL_FR),
                      (unsigned int)Drv_Encoder_IsWheelEnabled(WHEEL_RL),
                      (unsigned int)Drv_Encoder_IsWheelEnabled(WHEEL_RR),
                      (unsigned int)VEHICLE_SPI1_PINS_AVAILABLE);
    if ((length > 0) && (length < (int)sizeof(line))) {
        (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)line, (uint16_t)length);
    }
}

/*
 * 鐢垫満寮€鐜懡浠ゆ祴璇曘€?
 * 涓插彛鍙戦€侊細
 *   w锛氬洓杞墠杩?300鈥?
 *   s锛氬洓杞悗閫€ 300鈥?
 *   a锛氬師鍦板乏杞?
 *   d锛氬師鍦板彸杞?
 *   0锛氬仠姝?
 *
 * 娉ㄦ剰锛氱涓€娆℃祴璇曞繀椤绘灦绌哄皬杞︼紝纭鏂瑰悜鍚庡啀钀藉湴銆?
 */
void Test_MotorCmd_Update(void)
{
    uint8_t ch;

    while (BSP_UART_GetChar(UART_PORT1, &ch)) {
        if (ch == 'w') {
            Motor_SetPWM(300, 300);
        } else if (ch == 's') {
            Motor_SetPWM(-300, -300);
        } else if (ch == 'a') {
            Motor_SetPWM(-300, 300);
        } else if (ch == 'd') {
            Motor_SetPWM(300, -300);
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
 * 搴曠洏閫熷害闂幆鍛戒护娴嬭瘯銆?
 * 涓插彛鍙戦€侊細
 *   g锛氱洰鏍?linear=600 cps, turn=0锛岀洿琛岄棴鐜?
 *   b锛氱洰鏍?linear=-600 cps, turn=0锛屽悗閫€闂幆
 *   l锛氬師鍦板乏杞?turn=-400 cps
 *   r锛氬師鍦板彸杞?turn=400 cps
 *   x 鎴?0锛氬仠姝?
 *
 * 娴嬭瘯鍓嶆彁锛?
 *   1. 鍥涜矾缂栫爜鍣ㄦ柟鍚戝凡纭锛氬墠杩涙椂 cps 鍧囦负姝ｏ紱
 *   2. 鍥涗釜鐢垫満鏂瑰悜宸茬‘璁わ細姝?PWM 鏃跺皬杞﹀墠杩涳紱
 *   3. 绗竴娆￠棴鐜祴璇曞繀椤绘灦绌哄皬杞︺€?
 */
void Test_ChassisCmd_Update(void)
{
    uint8_t ch;

    while (BSP_UART_GetChar(UART_PORT1, &ch)) {
        if (ch == 'g') {
            Chassis_SetSpeed(2000, 0);
        } else if (ch == 'b') {
            Chassis_SetSpeed(-2000, 0);
        } else if (ch == 'l') {
            Chassis_SetSpeed(0, 1200);
        } else if (ch == 'r') {
            Chassis_SetSpeed(0, -1200);
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

//娴嬭瘯缂栫爜鍣ㄨ剦鍐?
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
 * Part3 闈為樆濉炲姩浣滃簱娴嬭瘯銆?
 * 涓插彛鍙戦€侊細
 *   f锛氬墠杩?500mm
 *   v锛氬悗閫€ 500mm
 *   L锛氬乏杞?90掳
 *   R锛氬彸杞?90掳
 *   x / 0锛氬仠姝㈠姩浣滃苟鍋滆溅
 *
 * 娴嬭瘯浠诲姟琛ㄥ繀椤诲寘鍚細Encoder_Update銆丮otion_Update銆丆hassis_Update銆?
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

//鐏板害浼犳劅鍣ㄥ贰绾挎祴璇?
/*
 * Part4 鐏板害寰抗娴嬭瘯鍛戒护锛?
 *   1锛氬惎鍔ㄥ惊杩?
 *   0 / x锛氬仠姝㈠惊杩瑰苟鍋滆溅
 *   w锛氭妸褰撳墠 8 璺伆搴﹂噰鏍疯褰曚负鐧藉簳
 *   b锛氭妸褰撳墠 8 璺伆搴﹂噰鏍疯褰曚负榛戠嚎
 *   t锛氭牴鎹櫧搴?榛戠嚎璁板綍鐢熸垚闃堝€?
 *   d锛氭仮澶嶉粯璁ょ粺涓€闃堝€?LINE_DETECT_DEFAULT_THRESHOLD
 *   p锛氱珛鍗虫墦鍗颁竴娆?raw/threshold/mask/error/type/output
 *
 * 鎺ㄨ崘鏍囧畾娴佺▼锛?
 *   1. 璁?8 璺紶鎰熷櫒閮藉鐫€鐧藉簳锛屽彂閫?w锛?
 *   2. 璁?8 璺紶鎰熷櫒閮藉帇鍦ㄩ粦绾夸笂锛屽彂閫?b锛?
 *   3. 鍙戦€?t 鐢熸垚闃堝€硷紱
 *   4. 鍙戦€?p 鐪?mask 鏄惁鍚堢悊锛?
 *   5. 鍙戦€?1 寮€濮嬪惊杩广€?
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

static void Test_Line_PrintThreshold(void)
{
    uint16_t threshold[LINE_DETECT_SENSOR_NUM];
    char buf[128];
    int n;

    /* Read the 8 thresholds currently used by line_detect. */
    if (LineDetect_GetThresholdArray(threshold, LINE_DETECT_SENSOR_NUM) != BSP_OK) {
        return;
    }

    n = sprintf(buf,
                "TH   %4u %4u %4u %4u %4u %4u %4u %4u\r\n",
                (unsigned int)threshold[0],
                (unsigned int)threshold[1],
                (unsigned int)threshold[2],
                (unsigned int)threshold[3],
                (unsigned int)threshold[4],
                (unsigned int)threshold[5],
                (unsigned int)threshold[6],
                (unsigned int)threshold[7]);

    if ((n > 0) && (n < (int)sizeof(buf))) {
        (void)BSP_UART_WriteFrame(UART_PORT1,
                                  (const uint8_t *)buf,
                                  (uint16_t)n);
    }
}

static void Test_Line_Print(void)
{
    LineFollow_Info_t info;
    Drv_GraySensor_Info_t sensor;
    Drv_GrayMcu_Info_t mcu;
    BSP_I2C_Debug_t i2c;
    char buf[192];
    int n;

    if (LineFollow_GetInfo(&info) != BSP_OK) return;
    if (Drv_GraySensor_GetInfo(&sensor) != BSP_OK) return;

    (void)Drv_GrayMcu_GetInfo(&mcu);
    (void)BSP_I2C_GetDebug(I2C_BUS1, &i2c);

    n = sprintf(buf,
                "GRAY on=%u valid=%u busy=%u st=%d i2c=%d ph=%u op=%u reg=0x%02X len=%u ping=0x%02X upd=%lu err=%lu pok=%lu perr=%lu addr=0x%02X\r\n",
                (unsigned int)sensor.online,
                (unsigned int)mcu.valid,
                (unsigned int)Drv_GrayMcu_IsBusy(),
                (int)mcu.last_status,
                (int)mcu.last_i2c_result,
                (unsigned int)mcu.last_phase,
                (unsigned int)mcu.last_op,
                (unsigned int)mcu.last_reg,
                (unsigned int)mcu.last_rx_len,
                (unsigned int)mcu.ping_value,
                (unsigned long)mcu.update_count,
                (unsigned long)mcu.error_count,
                (unsigned long)mcu.ping_ok_count,
                (unsigned long)mcu.ping_error_count,
                (unsigned int)mcu.active_addr);

    if (n > 0 && n < (int)sizeof(buf)) {
        (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
    }

    n = sprintf(buf,
                "I2C s=%u a=0x%02X tx=%u/%u rx=%u rd=%u sr1=0x%04X sr2=0x%04X | "
                "ERR src=%u s=%u a=0x%02X tx=%u/%u rx=%u sr1=0x%04X sr2=0x%04X n=%lu\r\n",
                (unsigned int)i2c.state,
                (unsigned int)i2c.dev_addr,
                (unsigned int)i2c.tx_pos,
                (unsigned int)i2c.tx_len,
                (unsigned int)i2c.rx_len,
                (unsigned int)i2c.need_read,
                (unsigned int)i2c.sr1,
                (unsigned int)i2c.sr2,
                (unsigned int)i2c.error_source,
                (unsigned int)i2c.error_state,
                (unsigned int)i2c.error_dev_addr,
                (unsigned int)i2c.error_tx_pos,
                (unsigned int)i2c.error_tx_len,
                (unsigned int)i2c.error_rx_len,
                (unsigned int)i2c.error_sr1,
                (unsigned int)i2c.error_sr2,
                (unsigned long)i2c.error_count);

    if (n > 0 && n < (int)sizeof(buf)) {
        (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
    }

    n = sprintf(buf,
                "RAW  %4u %4u %4u %4u %4u %4u %4u %4u | FILT %4u %4u %4u %4u %4u %4u %4u %4u\r\n",
                (unsigned int)sensor.raw[0], (unsigned int)sensor.raw[1],
                (unsigned int)sensor.raw[2], (unsigned int)sensor.raw[3],
                (unsigned int)sensor.raw[4], (unsigned int)sensor.raw[5],
                (unsigned int)sensor.raw[6], (unsigned int)sensor.raw[7],
                (unsigned int)sensor.filt[0], (unsigned int)sensor.filt[1],
                (unsigned int)sensor.filt[2], (unsigned int)sensor.filt[3],
                (unsigned int)sensor.filt[4], (unsigned int)sensor.filt[5],
                (unsigned int)sensor.filt[6], (unsigned int)sensor.filt[7]);

    if (n > 0 && n < (int)sizeof(buf)) {
        (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
    }

    Test_Line_PrintThreshold();

    n = sprintf(buf,
                "LINE st=%d type=%s mask=0x%02X cnt=%d err=%d out(v=%d,t=%d)\r\n",
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
}

void Test_I2C1_Scan_Print(void)
{
    uint8_t addr[16];
    uint8_t found = 0U;
    char buf[160];
    int n;
    uint8_t i;

    (void)BSP_I2C_ScanBus(I2C_BUS1, addr, (uint8_t)sizeof(addr), &found);

    n = sprintf(buf, "I2C_SCAN found=%u:", (unsigned int)found);
    if (n > 0 && n < (int)sizeof(buf)) {
        (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
    }

    for (i = 0U; i < found; i++) {
        n = sprintf(buf, " 0x%02X", (unsigned int)addr[i]);
        if (n > 0 && n < (int)sizeof(buf)) {
            (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
        }
    }

    (void)BSP_UART_WriteFrame(UART_PORT1,
                              (const uint8_t *)"\r\n",
                              2U);
}

void Test_LineCmd_Update(void)
{
    uint8_t ch;
    uint16_t raw[LINE_DETECT_SENSOR_NUM];

    while (BSP_UART_GetChar(UART_PORT1, &ch)) {
        if ((ch == '1') || (ch == 'l') || (ch == 'L')) {
            LineFollow_Start();
            if (LineFollow_GetState() == LINE_FOLLOW_RUN) {
                (void)BSP_UART_WriteFrame(UART_PORT1,
                                          (const uint8_t *)"line follow RUN\r\n",
                                          (uint16_t)(sizeof("line follow RUN\r\n") - 1U));
            } else {
                (void)BSP_UART_WriteFrame(UART_PORT1,
                                          (const uint8_t *)"line follow rejected: wait for IMU calibration\r\n",
                                          (uint16_t)(sizeof("line follow rejected: wait for IMU calibration\r\n") - 1U));
            }
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
            Test_Line_PrintThreshold();
        } else if (ch == 'd') {
            LineDetect_SetAllThreshold(LINE_DETECT_DEFAULT_THRESHOLD);
            (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)"default threshold\r\n", (uint16_t)(sizeof("default threshold\r\n") - 1U));
            Test_Line_PrintThreshold();
        } else if (ch == 'p') {
            Test_Line_Print();
        } else if (ch == 'k') {
            Test_I2C1_Scan_Print();
        }
    }
}

void Test_LineCmd_Log(void)
{
    Test_Line_Print();
}


/*
 * VL53L1X independent log test.
 * The ranging state machine is advanced by Sensor_Update(); this function
 * only reads the current driver snapshot and prints it through USART1.
 * Suggested scheduler entry:
 *   { Test_VL53L1X_Update, 200U, 0U },
 */
static const char *Test_VL53L1X_StateName(Drv_VL53L1X_State_t state)
{
    switch (state) {
        case DRV_VL53L1X_STATE_DISABLED:               return "DISABLED";
        case DRV_VL53L1X_STATE_XSHUT_LOW:              return "XSHUT_LOW";
        case DRV_VL53L1X_STATE_POWER_ON_WAIT:          return "POWER_WAIT";
        case DRV_VL53L1X_STATE_BOOT_CHECK:             return "BOOT";
        case DRV_VL53L1X_STATE_SENSOR_INIT:            return "INIT";
        case DRV_VL53L1X_STATE_SET_DISTANCE_MODE:      return "SET_MODE";
        case DRV_VL53L1X_STATE_SET_TIMING_BUDGET:      return "SET_TB";
        case DRV_VL53L1X_STATE_SET_INTER_MEASUREMENT:  return "SET_IM";
        case DRV_VL53L1X_STATE_SET_INTERRUPT_POLARITY: return "SET_INT";
        case DRV_VL53L1X_STATE_SET_OFFSET:             return "SET_OFFSET";
        case DRV_VL53L1X_STATE_SET_XTALK:              return "SET_XTALK";
        case DRV_VL53L1X_STATE_START_RANGING:          return "START";
        case DRV_VL53L1X_STATE_CHECK_DATA_READY:       return "RUN";
        case DRV_VL53L1X_STATE_READ_RESULT:            return "READ";
        case DRV_VL53L1X_STATE_CLEAR_INTERRUPT:        return "CLEAR";
        case DRV_VL53L1X_STATE_ERROR_WAIT:             return "ERROR_WAIT";
        case DRV_VL53L1X_STATE_STOPPED:                return "STOPPED";
        default:                                        return "?";
    }
}

void Test_VL53L1X_Update(void)
{
    Drv_VL53L1X_Info_t info;
    char buf[256];
    int n;

    if (Drv_VL53L1X_GetInfo(&info) != BSP_OK) {
        return;
    }

    n = snprintf(buf,
                 sizeof(buf),
                 "TOF en=%u on=%u init=%u run=%u valid=%u new=%u ready=%u "
                 "st=%s api=%u id=0x%04X rs=%u raw=%umm filt=%umm "
                 "amb=%u sig=%u spad=%u cnt=%lu ok=%lu err=%lu "
                 "busy=%lu reinit=%lu\r\n",
                 (unsigned int)info.enabled,
                 (unsigned int)info.online,
                 (unsigned int)info.initialized,
                 (unsigned int)info.ranging,
                 (unsigned int)info.data_valid,
                 (unsigned int)info.new_data,
                 (unsigned int)info.data_ready,
                 Test_VL53L1X_StateName(info.state),
                 (unsigned int)info.last_api_status,
                 (unsigned int)info.sensor_id,
                 (unsigned int)info.range_status,
                 (unsigned int)info.raw_distance_mm,
                 (unsigned int)info.distance_mm,
                 (unsigned int)info.ambient_kcps,
                 (unsigned int)info.signal_per_spad_kcps,
                 (unsigned int)info.spad_count,
                 (unsigned long)info.measurement_count,
                 (unsigned long)info.valid_count,
                 (unsigned long)info.error_count,
                 (unsigned long)info.busy_skip_count,
                 (unsigned long)info.reinit_count);

    if ((n > 0) && (n < (int)sizeof(buf))) {
        (void)BSP_UART_WriteFrame(UART_PORT1,
                                  (const uint8_t *)buf,
                                  (uint16_t)n);
    }
}


/*
 * 将放大后的有符号整数格式化成带小数点的 ASCII 文本。
 *
 * 示例：
 *   scaled = -8339, divisor = 1000, digits = 3
 *   输出 "-8.339"
 *
 * 不使用 printf 浮点功能，兼容 Keil ARMCC 默认配置。
 */
static void Test_FormatFixed(char *out,
                             uint16_t out_size,
                             long scaled,
                             unsigned long divisor,
                             uint8_t digits)
{
    unsigned long absolute_value;
    unsigned long integer_part;
    unsigned long fraction_part;
    char sign;

    if ((out == 0) || (out_size == 0U) || (divisor == 0UL)) {
        return;
    }

    sign = (scaled < 0L) ? '-' : '+';
    absolute_value = (scaled < 0L) ?
                     (unsigned long)(-(scaled + 1L)) + 1UL :
                     (unsigned long)scaled;
    integer_part = absolute_value / divisor;
    fraction_part = absolute_value % divisor;

    if (digits == 3U) {
        (void)snprintf(out,
                       out_size,
                       "%c%lu.%03lu",
                       sign,
                       integer_part,
                       fraction_part);
    } else if (digits == 2U) {
        (void)snprintf(out,
                       out_size,
                       "%c%lu.%02lu",
                       sign,
                       integer_part,
                       fraction_part);
    } else if (digits == 1U) {
        (void)snprintf(out,
                       out_size,
                       "%c%lu.%01lu",
                       sign,
                       integer_part,
                       fraction_part);
    } else {
        (void)snprintf(out, out_size, "%c%lu", sign, integer_part);
    }
}

static void Test_ICM20948_Print(const char *text)
{
    uint16_t length = 0U;

    if (text == 0) {
        return;
    }

    while ((text[length] != '\0') && (length < 500U)) {
        length++;
    }

    if (length != 0U) {
        (void)BSP_UART_WriteFrame(UART_PORT1,
                                  (const uint8_t *)text,
                                  length);
    }
}

/*
 * ICM-20948 数据独立日志测试。
 *
 * 本函数只读取驱动缓存，不推进传感器状态机。任务表必须同时保留：
 *   { Sensor_Update,          1U,   0U },
 *
 * 建议注册：
 *   { Test_ICM20948_Update, 500U,   0U },
 *
 * 输出单位：
 *   加速度      g
 *   角速度      deg/s（度/秒，不是姿态角）
 *   磁场强度    uT
 *   温度        degC
 */
void Test_ICM20948_Update(void)
{
    Drv_ICM20948_Info_t info;
    Drv_ICM20948_Data_t data;
    BSP_Status_t data_status;
    char line[256];
    char ax[20], ay[20], az[20];
    char gx[20], gy[20], gz[20];
    char mx[20], my[20], mz[20];
    char temp[20];
    char bx[20], by[20], bz[20];

    if (Drv_ICM20948_GetInfo(&info) != BSP_OK) {
        return;
    }

    data_status = Drv_ICM20948_GetData(&data);

    Test_ICM20948_Print("\r\n========== ICM-20948 SENSOR ==========\r\n");

    (void)snprintf(line,
                   sizeof(line),
                   "Main chip       : %s (WHO_AM_I=0x%02X, expected 0xEA)\r\n"
                   "Sampling        : %s | Valid data: %s | Gyro calibration: %s\r\n",
                   (info.online != 0U) ? "ONLINE" : "OFFLINE",
                   (unsigned int)info.who_am_i,
                   (info.running != 0U) ? "RUNNING" : "STOPPED",
                   (info.data_valid != 0U) ? "YES" : "NO",
                   (info.calibrating != 0U) ? "IN PROGRESS" :
                   ((info.gyro_cal_samples != 0U) ? "DONE" : "NOT STARTED"));
    Test_ICM20948_Print(line);

    if (data_status == BSP_OK) {
        Test_FormatFixed(ax, sizeof(ax),
                         (long)(data.accel_filtered_g.x * 1000.0f),
                         1000UL, 3U);
        Test_FormatFixed(ay, sizeof(ay),
                         (long)(data.accel_filtered_g.y * 1000.0f),
                         1000UL, 3U);
        Test_FormatFixed(az, sizeof(az),
                         (long)(data.accel_filtered_g.z * 1000.0f),
                         1000UL, 3U);

        Test_FormatFixed(gx, sizeof(gx),
                         (long)(data.gyro_filtered_dps.x * 1000.0f),
                         1000UL, 3U);
        Test_FormatFixed(gy, sizeof(gy),
                         (long)(data.gyro_filtered_dps.y * 1000.0f),
                         1000UL, 3U);
        Test_FormatFixed(gz, sizeof(gz),
                         (long)(data.gyro_filtered_dps.z * 1000.0f),
                         1000UL, 3U);

        Test_FormatFixed(temp, sizeof(temp),
                         (long)(data.temperature_filtered_c * 100.0f),
                         100UL, 2U);

        (void)snprintf(line,
                       sizeof(line),
                       "Accelerometer   : X=%s g, Y=%s g, Z=%s g\r\n"
                       "Gyroscope rate  : X=%s deg/s, Y=%s deg/s, Z=%s deg/s\r\n"
                       "Chip temperature: %s degC\r\n",
                       ax, ay, az,
                       gx, gy, gz,
                       temp);
        Test_ICM20948_Print(line);

        if ((info.mag_valid != 0U) && (data.mag_valid != 0U)) {
            Test_FormatFixed(mx, sizeof(mx),
                             (long)(data.mag_filtered_uT.x * 100.0f),
                             100UL, 2U);
            Test_FormatFixed(my, sizeof(my),
                             (long)(data.mag_filtered_uT.y * 100.0f),
                             100UL, 2U);
            Test_FormatFixed(mz, sizeof(mz),
                             (long)(data.mag_filtered_uT.z * 100.0f),
                             100UL, 2U);

            (void)snprintf(line,
                           sizeof(line),
                           "Magnetometer    : X=%s uT, Y=%s uT, Z=%s uT"
                           " (WIA1=0x%02X, WIA2=0x%02X)\r\n",
                           mx, my, mz,
                           (unsigned int)info.mag_wia1,
                           (unsigned int)info.mag_wia2);
        } else {
            (void)snprintf(line,
                           sizeof(line),
                           "Magnetometer    : INVALID / NOT DETECTED"
                           " (WIA1=0x%02X, WIA2=0x%02X, ST1=0x%02X, ST2=0x%02X)\r\n",
                           (unsigned int)info.mag_wia1,
                           (unsigned int)info.mag_wia2,
                           (unsigned int)info.mag_st1,
                           (unsigned int)info.mag_st2);
        }
        Test_ICM20948_Print(line);
    } else {
        Test_ICM20948_Print("Sensor values   : unavailable until valid sampling starts\r\n");
    }

    Test_FormatFixed(bx, sizeof(bx),
                     (long)(info.gyro_bias_dps.x * 1000.0f),
                     1000UL, 3U);
    Test_FormatFixed(by, sizeof(by),
                     (long)(info.gyro_bias_dps.y * 1000.0f),
                     1000UL, 3U);
    Test_FormatFixed(bz, sizeof(bz),
                     (long)(info.gyro_bias_dps.z * 1000.0f),
                     1000UL, 3U);

    (void)snprintf(line,
                   sizeof(line),
                   "Gyro zero bias  : X=%s deg/s, Y=%s deg/s, Z=%s deg/s"
                   " (%u samples)\r\n"
                   "Counters        : samples=%lu, valid=%lu, mag_valid=%lu,"
                   " errors=%lu, reinit=%lu\r\n",
                   bx, by, bz,
                   (unsigned int)info.gyro_cal_samples,
                   (unsigned long)info.sample_count,
                   (unsigned long)info.valid_count,
                   (unsigned long)info.mag_valid_count,
                   (unsigned long)info.error_count,
                   (unsigned long)info.reinit_count);
    Test_ICM20948_Print(line);
}

/*
 * AK09916 磁力计专项诊断。
 *
 * 只用于定位内部辅助 I2C 通信，不会修改驱动状态。测试时可暂时只注册本函数，
 * 避免与完整 IMU 日志混在一起：
 *   { Test_ICM20948_Mag_Update, 500U, 0U },
 */
void Test_ICM20948_Mag_Update(void)
{
    Drv_ICM20948_Info_t info;
    Drv_ICM20948_Data_t data;
    BSP_Status_t data_status;
    char line[500];
    char mx[20], my[20], mz[20];
    const char *method;
    const char *diagnosis;
    uint8_t identity_valid;
    int length;

    if (Drv_ICM20948_GetInfo(&info) != BSP_OK) {
        return;
    }

    data_status = Drv_ICM20948_GetData(&data);
    method = (info.mag_init_method == 2U) ? "SLV0+SLV1" :
             ((info.mag_init_method == 1U) ? "SLV4" : "unknown");
    identity_valid = ((info.mag_wia1 == DRV_ICM20948_MAG_WIA1_EXPECTED) ||
                      (info.mag_wia2 == DRV_ICM20948_MAG_WIA2_EXPECTED)) ? 1U : 0U;

    if ((info.last_i2c_mst_status & 0x01U) != 0U) {
        diagnosis = "SLV0 NACK at 0x0C";
    } else if ((info.last_i2c_mst_status & 0x02U) != 0U) {
        diagnosis = "SLV1 NACK at 0x0C";
    } else if (identity_valid == 0U) {
        diagnosis = "neither identity byte matches";
    } else if ((info.mag_st2 & 0x08U) != 0U) {
        diagnosis = "magnetic overflow";
    } else if ((info.mag_st1 & 0x01U) == 0U) {
        diagnosis = "identity OK, waiting for DRDY";
    } else {
        diagnosis = "identity and DRDY OK";
    }

    length = snprintf(line,
                      sizeof(line),
                      "\r\n========== AK09916 MAG TEST ==========\r\n"
                      "init=%s method=%s identity=%s\r\n"
                      "WIA1=0x%02X WIA2=0x%02X ST1=0x%02X ST2=0x%02X MST=0x%02X retry=%u\r\n"
                      "USER_CTRL=0x%02X LP_CONFIG=0x%02X I2C_MST_CTRL=0x%02X\r\n"
                      "SLV0 addr/ctrl=0x%02X/0x%02X SLV1 addr/ctrl=0x%02X/0x%02X\r\n"
                      "count valid=%lu not_ready=%lu overflow=%lu nack=%lu\r\n"
                      "diagnosis=%s\r\n",
                      (info.mag_valid != 0U) ? "SUCCESS" : "WAITING/FAILED",
                      method,
                      (identity_valid != 0U) ? "PASS" : "FAIL",
                      (unsigned int)info.mag_wia1,
                      (unsigned int)info.mag_wia2,
                      (unsigned int)info.mag_st1,
                      (unsigned int)info.mag_st2,
                      (unsigned int)info.last_i2c_mst_status,
                      (unsigned int)info.mag_retry_count,
                      (unsigned int)info.user_ctrl_readback,
                      (unsigned int)info.lp_config_readback,
                      (unsigned int)info.i2c_mst_ctrl_readback,
                      (unsigned int)info.slv0_addr_readback,
                      (unsigned int)info.slv0_ctrl_readback,
                      (unsigned int)info.slv1_addr_readback,
                      (unsigned int)info.slv1_ctrl_readback,
                      (unsigned long)info.mag_valid_count,
                      (unsigned long)info.mag_not_ready_count,
                      (unsigned long)info.mag_overflow_count,
                      (unsigned long)info.mag_nack_count,
                      diagnosis);
    if (length < 0) {
        return;
    }
    if (length >= (int)sizeof(line)) {
        length = (int)sizeof(line) - 1;
    }

    if ((data_status == BSP_OK) &&
        (info.mag_valid != 0U) &&
        (data.mag_valid != 0U)) {
        Test_FormatFixed(mx, sizeof(mx),
                         (long)(data.mag_filtered_uT.x * 100.0f),
                         100UL, 2U);
        Test_FormatFixed(my, sizeof(my),
                         (long)(data.mag_filtered_uT.y * 100.0f),
                         100UL, 2U);
        Test_FormatFixed(mz, sizeof(mz),
                         (long)(data.mag_filtered_uT.z * 100.0f),
                         100UL, 2U);

        (void)snprintf(&line[length],
                       sizeof(line) - (uint16_t)length,
                       "mag X=%s uT Y=%s uT Z=%s uT\r\n",
                       mx, my, mz);
    } else {
        (void)snprintf(&line[length],
                       sizeof(line) - (uint16_t)length,
                       "mag unavailable\r\n");
    }

    /* 单帧写入，避免 512 字节 UART 环形缓冲区被多次日志调用挤满后丢行。 */
    Test_ICM20948_Print(line);
}

/*
 * 独立的姿态融合诊断与标定测试，不修改其他测试函数：
 *   M：开始磁力计 min/max 标定；N：结束并计算参数；Y：当前融合 Yaw 置零。
 */
void Test_Attitude_Update(void)
{
    static uint32_t last_log_ms = 0U;
    Attitude_Info_t info;
    Sensor_Attitude_t sensor_attitude;
    Drv_ICM20948_Info_t imu_info;
    Attitude_MagCalibration_t calibration;
    BSP_Status_t calibration_status;
    uint8_t ch;
    char line[500];
    char response[220];
    char roll[20], pitch[20], yaw[20];
    char startup_bx[20], startup_by[20], startup_bz[20];
    char online_bx[20], online_by[20], online_bz[20];
    char cal_gyro_max[20], cal_accel_norm[20];
    char mag_norm[20];
    int length;

    /*
     * 该测试函数独占本轮姿态测试所需的串口命令，不修改或依赖其他测试函数：
     *   M：开始磁力计标定；N：完成标定；Y：把当前融合航向设为相对零点。
     */
    while (BSP_UART_GetChar(UART_PORT1, &ch)) {
        if (ch == 'G') {
            Chassis_Stop();
            Drv_ICM20948_StartGyroCalibration();
            Attitude_Reset();
            Heading_Reset();
            (void)BSP_UART_WriteFrame(
                UART_PORT1,
                (const uint8_t *)"gyro calibration RESTARTED: keep the vehicle completely still\r\n",
                (uint16_t)(sizeof("gyro calibration RESTARTED: keep the vehicle completely still\r\n") - 1U));
        } else if (ch == 'M') {
            Attitude_MagCalibrationStart();
            (void)BSP_UART_WriteFrame(
                UART_PORT1,
                (const uint8_t *)"mag calibration START: rotate slowly through all axes, then send N\r\n",
                (uint16_t)(sizeof("mag calibration START: rotate slowly through all axes, then send N\r\n") - 1U));
        } else if (ch == 'N') {
            calibration_status = Attitude_MagCalibrationFinish(&calibration);
            if (calibration_status == BSP_OK) {
                length = snprintf(response,
                                  sizeof(response),
                                  "mag calibration OK: offset x100=%ld,%ld,%ld scale x1000=%ld,%ld,%ld\r\n",
                                  (long)(calibration.offset_uT[0] * 100.0f),
                                  (long)(calibration.offset_uT[1] * 100.0f),
                                  (long)(calibration.offset_uT[2] * 100.0f),
                                  (long)(calibration.scale[0] * 1000.0f),
                                  (long)(calibration.scale[1] * 1000.0f),
                                  (long)(calibration.scale[2] * 1000.0f));
                if ((length > 0) && (length < (int)sizeof(response))) {
                    (void)BSP_UART_WriteFrame(UART_PORT1,
                                              (const uint8_t *)response,
                                              (uint16_t)length);
                }
            } else {
                (void)BSP_UART_WriteFrame(
                    UART_PORT1,
                    (const uint8_t *)"mag calibration FAILED: need >=300 samples and full XYZ rotation; send M to retry\r\n",
                    (uint16_t)(sizeof("mag calibration FAILED: need >=300 samples and full XYZ rotation; send M to retry\r\n") - 1U));
            }
        } else if (ch == 'Y') {
            Attitude_ZeroYaw();
            Heading_Reset();
            (void)BSP_UART_WriteFrame(
                UART_PORT1,
                (const uint8_t *)"gyro yaw zeroed\r\n",
                (uint16_t)(sizeof("gyro yaw zeroed\r\n") - 1U));
        }
    }

    /* 10 ms 运行一次以免漏串口命令，但姿态日志只每 500 ms 输出一次。 */
    if (!BSP_TimeElapsed(&last_log_ms, 500U)) {
        return;
    }

    /* 角度只通过 SensorManager 公共接口读取；Info 仅补充测试诊断计数。 */
    if (Drv_ICM20948_GetInfo(&imu_info) != BSP_OK) {
        Test_ICM20948_Print("\r\nIMU INFO unavailable\r\n");
        return;
    }
    Test_FormatFixed(cal_gyro_max, sizeof(cal_gyro_max),
                     (long)(imu_info.gyro_cal_last_max_abs_dps * 100.0f), 100UL, 2U);
    Test_FormatFixed(cal_accel_norm, sizeof(cal_accel_norm),
                     (long)(imu_info.gyro_cal_last_accel_norm_g * 100.0f), 100UL, 2U);

    /* Before attitude becomes valid, show why motion is still interlocked. */
    if ((Sensor_GetAttitude(&sensor_attitude) != BSP_OK) ||
        (Attitude_GetInfo(&info) != BSP_OK)) {
        length = snprintf(line,
                          sizeof(line),
                          "\r\n========== GYRO ATTITUDE TEST ==========\r\n"
                          "WAITING: keep vehicle completely still; motors are disabled\r\n"
                          "imu state=%u online/init/run/cal/data=%u/%u/%u/%u/%u\r\n"
                          "gyro calibration samples=%u/%u (must reach the full count)\r\n"
                          "last max gyro=%s dps accel norm=%s g reject=%lu\r\n"
                          "command G=restart gyro calibration\r\n",
                          (unsigned int)imu_info.state,
                          (unsigned int)imu_info.online,
                          (unsigned int)imu_info.initialized,
                          (unsigned int)imu_info.running,
                          (unsigned int)imu_info.calibrating,
                          (unsigned int)imu_info.data_valid,
                          (unsigned int)imu_info.gyro_cal_samples,
                          (unsigned int)DRV_ICM20948_GYRO_CAL_SAMPLE_COUNT,
                          cal_gyro_max,
                          cal_accel_norm,
                          (unsigned long)imu_info.gyro_cal_reject_count);
        if ((length > 0) && (length < (int)sizeof(line))) {
            Test_ICM20948_Print(line);
        }
        return;
    }

    Test_FormatFixed(roll, sizeof(roll),
                     (long)(sensor_attitude.roll_deg * 100.0f), 100UL, 2U);
    Test_FormatFixed(pitch, sizeof(pitch),
                     (long)(sensor_attitude.pitch_deg * 100.0f), 100UL, 2U);
    Test_FormatFixed(yaw, sizeof(yaw),
                     (long)(sensor_attitude.yaw_deg * 100.0f), 100UL, 2U);
    Test_FormatFixed(startup_bx, sizeof(startup_bx),
                     (long)(imu_info.gyro_bias_dps.x * 1000.0f), 1000UL, 3U);
    Test_FormatFixed(startup_by, sizeof(startup_by),
                     (long)(imu_info.gyro_bias_dps.y * 1000.0f), 1000UL, 3U);
    Test_FormatFixed(startup_bz, sizeof(startup_bz),
                     (long)(imu_info.gyro_bias_dps.z * 1000.0f), 1000UL, 3U);
    Test_FormatFixed(online_bx, sizeof(online_bx),
                     (long)(info.online_gyro_bias_dps[0] * 1000.0f), 1000UL, 3U);
    Test_FormatFixed(online_by, sizeof(online_by),
                     (long)(info.online_gyro_bias_dps[1] * 1000.0f), 1000UL, 3U);
    Test_FormatFixed(online_bz, sizeof(online_bz),
                     (long)(info.online_gyro_bias_dps[2] * 1000.0f), 1000UL, 3U);
    Test_FormatFixed(mag_norm, sizeof(mag_norm),
                     (long)(info.mag_norm_uT * 100.0f), 100UL, 2U);

    length = snprintf(line,
                      sizeof(line),
                      "\r\n========== GYRO ATTITUDE TEST ==========\r\n"
                      "angle roll=%s pitch=%s yaw=%s deg\r\n"
                      "mode yaw=GYRO+MAG encoder=0 mag_slow_kp_x100=%ld\r\n"
                      "imu ready=%u init/run/cal/data=%u/%u/%u/%u samples=%u/%u\r\n"
                      "bias startup=%s,%s,%s online=%s,%s,%s dps\r\n"
                      "mag available/calibrated/healthy/used=%u/%u/%u/%u norm=%s uT\r\n"
                      "state valid=%u stationary=%u update=%lu\r\n"
                      "command G=restart gyro cal, Y=zero yaw\r\n",
                      roll, pitch, yaw,
                      (long)(ATTITUDE_MAHONY_MAG_KP * 100.0f),
                      (unsigned int)Sensor_IsImuReadyForMotion(),
                      (unsigned int)imu_info.initialized,
                      (unsigned int)imu_info.running,
                      (unsigned int)imu_info.calibrating,
                      (unsigned int)imu_info.data_valid,
                      (unsigned int)imu_info.gyro_cal_samples,
                      (unsigned int)DRV_ICM20948_GYRO_CAL_SAMPLE_COUNT,
                      startup_bx, startup_by, startup_bz,
                      online_bx, online_by, online_bz,
                      (unsigned int)info.mag_available,
                      (unsigned int)info.mag_calibrated,
                      (unsigned int)info.mag_healthy,
                      (unsigned int)info.mag_used,
                      mag_norm,
                      (unsigned int)info.valid,
                      (unsigned int)info.stationary,
                      (unsigned long)info.update_count);

    if ((length > 0) && (length < (int)sizeof(line))) {
        Test_ICM20948_Print(line);
    }
}

void Test_LCD_Ascii_Update(void)
{
    Test_AsyncDisplay_Update();
}

void Test_OLED_Ascii_Update(void)
{
    static uint32_t last_oled_ms = 0U;
    static uint32_t oled_cnt = 0U;
    uint16_t bar_w;
    char buf[24];

    if ((Drv_OledI2c_IsReady() == 0U) ||
        (Drv_OledI2c_IsBusy() != 0U) ||
        (BSP_TimeElapsed(&last_oled_ms, 200U) == 0U)) {
        return;
    }

    oled_cnt++;
    Drv_OledI2c_Clear();
    Drv_OledI2c_DrawRect(0U, 0U, 128U, 64U, DRV_OLED_COLOR_ON);
    Drv_OledI2c_DrawString5x7(6U, 6U, "OLED TEST", DRV_OLED_COLOR_ON);
    Drv_OledI2c_DrawString5x7(6U, 18U, "I2C DMA OK", DRV_OLED_COLOR_ON);
    (void)snprintf(buf, sizeof(buf), "CNT:%lu", (unsigned long)oled_cnt);
    Drv_OledI2c_DrawString5x7(6U, 32U, buf, DRV_OLED_COLOR_ON);
    bar_w = (uint16_t)(8U + ((oled_cnt * 5U) % 104U));
    Drv_OledI2c_DrawRect(6U, 48U, 116U, 10U, DRV_OLED_COLOR_ON);
    Drv_OledI2c_FillRect(8U, 50U, (uint8_t)bar_w, 6U, DRV_OLED_COLOR_ON);
    Drv_OledI2c_Flush();
}

void Test_AsyncDisplay_Update(void)
{
    static uint32_t last_lcd_ms = 0U;
    static uint32_t lcd_cnt = 0U;
    static uint8_t lcd_base_done = 0U;
    static uint8_t lcd_base_step = 0U;
    static uint8_t lcd_refresh_pending = 0U;
    static uint8_t lcd_step = 0U;
    uint16_t bar_w;
    char buf[32];
    BSP_Status_t ret = BSP_BUSY;

    Test_OLED_Ascii_Update();

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
                ret = Drv_LcdTft_TryDrawString5x7(16U, 38U, "SPI1 DMA ASYNC",
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
