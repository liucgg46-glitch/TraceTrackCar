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
#include "lcd_ui.h"
#include "oled_ui.h"
#include "motion_action.h"
#include "sensor_manager.h"
#include "odometer.h"
#include "attitude_estimator.h"
#include "heading_estimator.h"
#include "line_follow_app.h"
#include "line_detect.h"
#include "line_track.h"
#include "route_manager.h"
#include "task_profile_config.h"
#include "task_profile_h2_round_stop.h"
#include "route_profile_h_oval.h"
#include "drv_gray_sensor.h"
#include "drv_gray_mcu_i2c.h"
#include "drv_gray_yahboom_uart.h"
#include "drv_lcd_tft.h"
#include "drv_oled_i2c.h"
#include "drv_vl53l1x.h"
#include "drv_icm20948.h"
#include "drv_status_light.h"
#include "drv_buzzer.h"
#include "task_fsm.h"
#include "k210_comm.h"
#include "drv_servo.h"
#include "ball_balance_app.h"
#include "ball_balance_k210_adapter.h"
#include "bsp_systick.h"

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

#define TEST_SERVO_CAL_STEP_ANGLE_DEG             10U

/* LCD公开测试入口使用的内部异步显示状态机，不对任务表暴露。 */
void Test_AsyncDisplay_Update(void);

/*
 * K210与STM32通信测试任务。
 *
 * 功能：
 *   1. 读取K210发送的完整多数字快照；
 *   2. 通过USART1输出数字、置信度和横坐标；
 *   3. 每500ms输出一次通信状态和错误统计。
 *
 * 建议任务周期：
 *   { Test_K210_CommUpdate, 10U, 0U },
 */
void Test_K210_CommUpdate(void)
{
    static uint32_t last_status_ms = 0U;

    K210_DigitSnapshot_t snapshot;
    K210_Comm_Info_t info;

    char buf[240];
    int n;
    int used;
    uint8_t i;
    /*
     * 读取一个已经完整接收的数字快照。
     *
     * 快照中的数字已经按照画面横坐标
     * 从左到右排列。
     */
    if (K210_Comm_GetNewSnapshot(&snapshot) == BSP_OK) {
        used = snprintf(
            buf,
            sizeof(buf),
            "K210 SNAP seq=%u status=%u count=%u",
            (unsigned int)snapshot.sequence,
            (unsigned int)snapshot.status,
            (unsigned int)snapshot.count
        );

        /*
         * 将本次快照中的全部数字追加到日志。
         *
         * 输出格式：
         *   [序号]=数字/置信度/x横坐标
         */
        for (i = 0U;
             (i < snapshot.count) &&
             (used > 0) &&
             (used < (int)sizeof(buf));
             i++) {
            n = snprintf(
                &buf[used],
                sizeof(buf) - (size_t)used,
                " [%u]=%u/%u/x%u",
                (unsigned int)i,
                (unsigned int)snapshot.items[i].digit,
                (unsigned int)snapshot.items[i].confidence,
                (unsigned int)snapshot.items[i].center_x
            );

            /*
             * snprintf返回值大于等于剩余空间时，
             * 表示日志内容被截断。
             */
            if ((n <= 0) ||
                (n >= ((int)sizeof(buf) - used))) {
                used = (int)sizeof(buf);
                break;
            }

            used += n;
        }

        /*
         * 缓冲区仍有空间时添加回车换行，
         * 然后通过USART1发送到串口助手。
         */
        if ((used > 0) &&
            (used <= ((int)sizeof(buf) - 3))) {
            buf[used++] = '\r';
            buf[used++] = '\n';
            buf[used] = '\0';

            (void)BSP_UART_WriteFrame(
                UART_PORT1,
                (const uint8_t *)buf,
                (uint16_t)used
            );
        }
    }

    /* 通信状态每500ms输出一次 */
    if ((uint32_t)(
            BSP_GetTickMs() -
            last_status_ms
        ) < 500U) {
        return;
    }

    last_status_ms = BSP_GetTickMs();

    /* 获取K210通信状态和错误计数 */
    if (K210_Comm_GetInfo(&info) != BSP_OK) {
        return;
    }

    n = snprintf(
        buf,
        sizeof(buf),
        "K210 online=%u frames=%lu "
        "snapshots=%lu check_err=%lu "
        "format_err=%lu snap_err=%lu "
        "overwrite=%lu last_rx=%lu\r\n",
        (unsigned int)info.online,
        (unsigned long)info.valid_frame_count,
        (unsigned long)info.snapshot_count,
        (unsigned long)info.checksum_error_count,
        (unsigned long)info.format_error_count,
        (unsigned long)info.snapshot_error_count,
        (unsigned long)info.snapshot_overwrite_count,
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

/*
 * ============================================================================
 * K210 LAB调参UART命令测试
 * ============================================================================
 *
 * 测试目的：
 *   只验证 STM32 -> K210 的 0x41 命令。
 *
 * 测试行为：
 *   STM32启动3秒后，只发送一次：
 *
 *       K210_CMD_LAB_TUNE_MODE
 *       DATA1 = K210_LAB_TUNE_RED
 *
 * 预期：
 *   K210从正常道路画面切换到RED LAB二值化调参页面。
 *
 * 建议任务周期：
 *   { Test_K210_LabTuneUpdate, 20U, 0U },
 *
 * 请使用UTF-8编码保存。
 * ============================================================================
 */
/*
 * ============================================================================
 * K210 LAB调参UART完整链路测试
 * ============================================================================
 *
 * 测试顺序：
 *
 *   3秒：
 *       0x41 -> 进入 RED LAB TUNE
 *
 *   5秒：
 *       0x42 -> 选择 L_MAX
 *
 *   7秒：
 *       0x43 -> L_MAX +10
 *
 *   9秒：
 *       0x43 -> L_MAX +10
 *
 *   11秒：
 *       0x43 -> L_MAX -10
 *
 * 预期：
 *   K210屏幕上的 > 参数选择改变；
 *   L_MAX数值改变；
 *   黑白二值化区域同步发生变化。
 *
 * 建议任务周期：
 *   { Test_K210_LabTuneUpdate, 20U, 0U },
 *
 * 请使用UTF-8编码保存。
 * ============================================================================
 */
/*
 * ============================================================================
 * K210 LAB现场按键调参测试
 * ============================================================================
 *
 * KEY1：
 *   进入/退出LAB调参模式
 *
 * KEY2：
 *   选择下一个LAB参数
 *
 *   L_MIN -> L_MAX -> A_MIN -> A_MAX
 *         -> B_MIN -> B_MAX -> L_MIN
 *
 * KEY3：
 *   当前参数 +1
 *
 * KEY4：
 *   当前参数 -1
 *
 * KEY5：
 *   RED / BLACK 切换
 *
 * K210 LCD：
 *   调参模式下显示实时黑白二值化结果。
 *
 * 建议任务周期：
 *
 *   { Key_Update,               10U, 0U },
 *   { K210_Comm_Update,          5U, 0U },
 *   { Test_K210_LabTuneUpdate,  20U, 0U },
 *
 * 文件请使用UTF-8编码保存。
 * ============================================================================
 */
/*
 * ============================================================================
 * K210 LAB四按键现场调参测试
 * ============================================================================
 *
 * KEY1：
 *   OFF -> RED -> BLACK -> OFF
 *
 * KEY2：
 *   选择下一个LAB参数
 *
 * KEY3：
 *   当前参数 +1
 *
 * KEY4：
 *   当前参数 -1
 *
 * 文件请使用 UTF-8 编码保存。
 * ============================================================================
 */
void Test_K210_LabTuneUpdate(void)
{
    static uint8_t initialized = 0U;

    static uint8_t tune_mode =
        K210_LAB_TUNE_OFF;

    static uint8_t selected_param =
        K210_LAB_PARAM_L_MIN;

    BSP_Status_t status;


    /* ========================================================
     * 启动提示
     * ======================================================== */

    if (initialized == 0U) {

        static const char message[] =
            "\r\n"
            "===== K210 LAB KEY TEST =====\r\n"
            "KEY1 : OFF -> RED -> BLACK -> OFF\r\n"
            "KEY2 : NEXT PARAM\r\n"
            "KEY3 : +1\r\n"
            "KEY4 : -1\r\n"
            "=============================\r\n";

        (void)BSP_UART_WriteFrame(
            UART_PORT1,
            (const uint8_t *)message,
            (uint16_t)(sizeof(message) - 1U)
        );

        initialized = 1U;
    }


    /* ========================================================
     * KEY1
     *
     * OFF -> RED -> BLACK -> OFF
     * ======================================================== */

#if BSP_KEY1_ENABLE

    if (BSP_Key_WasPressed(BSP_KEY1) != 0U) {

        if (tune_mode == K210_LAB_TUNE_OFF) {

            tune_mode =
                K210_LAB_TUNE_RED;
        }
        else if (tune_mode == K210_LAB_TUNE_RED) {

            tune_mode =
                K210_LAB_TUNE_BLACK;
        }
        else {

            tune_mode =
                K210_LAB_TUNE_OFF;
        }


        /*
         * 每次切换模式，
         * 参数重新从L_MIN开始。
         */
        selected_param =
            K210_LAB_PARAM_L_MIN;


        status =
            K210_Comm_SetLabTuneMode(
                tune_mode
            );


        if (status == BSP_OK) {

            /*
             * RED / BLACK模式下同步参数选择。
             */
            if (tune_mode != K210_LAB_TUNE_OFF) {

                (void)K210_Comm_SelectLabParam(
                    selected_param
                );
            }


            if (tune_mode == K210_LAB_TUNE_RED) {

                static const char message[] =
                    "LAB MODE -> RED\r\n";

                (void)BSP_UART_WriteFrame(
                    UART_PORT1,
                    (const uint8_t *)message,
                    (uint16_t)(sizeof(message) - 1U)
                );
            }

            else if (tune_mode == K210_LAB_TUNE_BLACK) {

                static const char message[] =
                    "LAB MODE -> BLACK\r\n";

                (void)BSP_UART_WriteFrame(
                    UART_PORT1,
                    (const uint8_t *)message,
                    (uint16_t)(sizeof(message) - 1U)
                );
            }

            else {

                static const char message[] =
                    "LAB MODE -> OFF\r\n";

                (void)BSP_UART_WriteFrame(
                    UART_PORT1,
                    (const uint8_t *)message,
                    (uint16_t)(sizeof(message) - 1U)
                );
            }
        }

        return;
    }

#endif


    /* ========================================================
     * 正常模式下忽略KEY2~KEY4
     * ======================================================== */

    if (tune_mode == K210_LAB_TUNE_OFF) {
        return;
    }


    /* ========================================================
     * KEY2
     *
     * 下一个LAB参数
     * ======================================================== */

#if BSP_KEY2_ENABLE

    if (BSP_Key_WasPressed(BSP_KEY2) != 0U) {

        selected_param++;

        if (
            selected_param >
            K210_LAB_PARAM_B_MAX
        ) {
            selected_param =
                K210_LAB_PARAM_L_MIN;
        }


        status =
            K210_Comm_SelectLabParam(
                selected_param
            );


        if (status == BSP_OK) {

            char message[48];
            int length;

            length = snprintf(
                message,
                sizeof(message),
                "LAB SELECT=%u\r\n",
                (unsigned int)selected_param
            );

            if (
                length > 0 &&
                length < (int)sizeof(message)
            ) {
                (void)BSP_UART_WriteFrame(
                    UART_PORT1,
                    (const uint8_t *)message,
                    (uint16_t)length
                );
            }
        }

        return;
    }

#endif

/* ========================================================
 * KEY3
 *
 * 当前参数 +5
 * ======================================================== */

#if BSP_KEY3_ENABLE

    if (BSP_Key_WasPressed(BSP_KEY3) != 0U) {

        status =
            K210_Comm_AdjustLabParam(
                selected_param,
                +5
            );


        if (status == BSP_OK) {

            char message[48];
            int length;

            length = snprintf(
                message,
                sizeof(message),
                "LAB PARAM=%u +5\r\n",
                (unsigned int)selected_param
            );

            if (
                length > 0 &&
                length < (int)sizeof(message)
            ) {
                (void)BSP_UART_WriteFrame(
                    UART_PORT1,
                    (const uint8_t *)message,
                    (uint16_t)length
                );
            }
        }

        return;
    }

#endif

   /* ========================================================
 * KEY4
 *
 * 当前参数 -5
 * ======================================================== */

#if BSP_KEY4_ENABLE

    if (BSP_Key_WasPressed(BSP_KEY4) != 0U) {

        status =
            K210_Comm_AdjustLabParam(
                selected_param,
                -5
            );


        if (status == BSP_OK) {

            char message[48];
            int length;

            length = snprintf(
                message,
                sizeof(message),
                "LAB PARAM=%u -5\r\n",
                (unsigned int)selected_param
            );

            if (
                length > 0 &&
                length < (int)sizeof(message)
            ) {
                (void)BSP_UART_WriteFrame(
                    UART_PORT1,
                    (const uint8_t *)message,
                    (uint16_t)length
                );
            }
        }

        return;
    }

#endif
}
//娴嬭瘯鍑芥暟锛孫LED闂儊
void Test_GPIO_Toggle(void)
{
    static uint32_t last = 0;

    if (BSP_TimeElapsed(&last, 500U)) {
        BSP_GPIO_Toggle(BSP_GPIO_CH1);
    }
}

/*
 * 红绿状态灯非阻塞测试：红灯、绿灯、全部熄灭各保持 1 秒并循环。
 * 建议以 10 ms 周期注册：{ Test_StatusLight_Update, 10U, 0U }。
 */
void Test_StatusLight_Update(void)
{
    static uint32_t last_switch_ms = 0U;
    static uint8_t phase = 0U;

    if (BSP_TimeElapsed(&last_switch_ms, 1000U) == 0U) {
        return;
    }

    phase++;
    if (phase == 1U) {
        Drv_StatusLight_SetRed();
    } else if (phase == 2U) {
        Drv_StatusLight_SetGreen();
    } else {
        Drv_StatusLight_Off();
        phase = 0U;
    }
}

//娴嬭瘯浠ｇ爜锛岀數鏈鸿浆閫熼€愭笎鍙樺揩鍦ㄥ彉鎱?
/*
 * 蜂鸣器按键测试（PG7，低电平有效）：
 * KEY1：按下一次后持续鸣响；即使按下边沿被上电保护清除，稳定按住也能进入长响。
 * KEY2：进入间歇鸣响模式，每 500 ms 翻转一次。
 * KEY3：立即停止，且停止优先级最高。
 * 必须先以 10 ms 周期注册 Key_Update，再注册本任务。
 */
void Test_Buzzer_Update(void)
{
    enum {
        TEST_BUZZER_MODE_OFF = 0U,
        TEST_BUZZER_MODE_CONTINUOUS,
        TEST_BUZZER_MODE_INTERVAL
    };

    static uint8_t mode = TEST_BUZZER_MODE_OFF;
    static uint8_t armed = 0U;
    static uint8_t released_samples = 0U;
    static uint32_t last_toggle_ms = 0U;
    uint8_t key1_down = 0U;
    uint8_t key1_event = 0U;
    uint8_t key2_event = 0U;
    uint8_t key3_event = 0U;
    uint8_t all_released = 1U;

#if BSP_KEY1_ENABLE
    key1_down = BSP_Key_IsPressed(BSP_KEY1);
    if (key1_down != 0U) {
        all_released = 0U;
    }
#endif
#if BSP_KEY2_ENABLE
    if (BSP_Key_IsPressed(BSP_KEY2) != 0U) {
        all_released = 0U;
    }
#endif
#if BSP_KEY3_ENABLE
    if (BSP_Key_IsPressed(BSP_KEY3) != 0U) {
        all_released = 0U;
    }
#endif

    /* 上电后先确认三个按键均稳定松开约 100 ms，避免上电假触发。 */
    if (armed == 0U) {
        Drv_Buzzer_Off();
        mode = TEST_BUZZER_MODE_OFF;

#if BSP_KEY1_ENABLE
        (void)BSP_Key_WasPressed(BSP_KEY1);
        (void)BSP_Key_WasReleased(BSP_KEY1);
#endif
#if BSP_KEY2_ENABLE
        (void)BSP_Key_WasPressed(BSP_KEY2);
        (void)BSP_Key_WasReleased(BSP_KEY2);
#endif
#if BSP_KEY3_ENABLE
        (void)BSP_Key_WasPressed(BSP_KEY3);
        (void)BSP_Key_WasReleased(BSP_KEY3);
#endif

        if (all_released != 0U) {
            if (released_samples < 10U) {
                released_samples++;
            }
            if (released_samples >= 10U) {
                armed = 1U;
                released_samples = 0U;
                last_toggle_ms = BSP_GET_TICK();
            }
        } else {
            released_samples = 0U;
        }
        return;
    }

    /* 每个边沿只读取一次，避免事件被重复读取或提前清除。 */
#if BSP_KEY1_ENABLE
    key1_event = BSP_Key_WasPressed(BSP_KEY1);
#endif
#if BSP_KEY2_ENABLE
    key2_event = BSP_Key_WasPressed(BSP_KEY2);
#endif
#if BSP_KEY3_ENABLE
    key3_event = BSP_Key_WasPressed(BSP_KEY3);
#endif

    /* KEY3停止优先；KEY1同时使用边沿和稳定电平，保证长响命令不会丢失。 */
    if (key3_event != 0U) {
        mode = TEST_BUZZER_MODE_OFF;
        Drv_Buzzer_Off();
    } else if ((key1_event != 0U) || (key1_down != 0U)) {
        mode = TEST_BUZZER_MODE_CONTINUOUS;
        Drv_Buzzer_On();
    } else if (key2_event != 0U) {
        mode = TEST_BUZZER_MODE_INTERVAL;
        last_toggle_ms = BSP_GET_TICK();
        Drv_Buzzer_On();
    }

    /* 每个调度周期刷新物理输出，防止其他初始化或瞬态写操作覆盖蜂鸣器状态。 */
    if (mode == TEST_BUZZER_MODE_OFF) {
        Drv_Buzzer_Off();
    } else if (mode == TEST_BUZZER_MODE_CONTINUOUS) {
        Drv_Buzzer_On();
    } else if (BSP_TimeElapsed(&last_toggle_ms, 500U) != 0U) {
        Drv_Buzzer_Toggle();
    }
}

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
                "ENC L: d=%ld cps=%ld total=%ld | R: d=%ld cps=%ld total=%ld\r\n",
                (long)BSP_Encoder_GetDelta(BSP_ENCODER_CH1),
                (long)BSP_Encoder_GetSpeedCps(BSP_ENCODER_CH1),
                (long)BSP_Encoder_GetTotal(BSP_ENCODER_CH1),
                (long)BSP_Encoder_GetDelta(BSP_ENCODER_CH2),
                (long)BSP_Encoder_GetSpeedCps(BSP_ENCODER_CH2),
                (long)BSP_Encoder_GetTotal(BSP_ENCODER_CH2));

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

static void Test_Key_Send(const char *message, uint16_t length)
{
    (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)message, length);
}

/*
 * 五按键最小测试任务：
 *   KEY1=PE4、KEY2=PE3、KEY3=PE2、KEY4=PE1、KEY5=PA15。
 * Key_Update() 统一完成按键扫描和消抖，本函数只消费按下/松开事件。
 */
void Test_Key_Update(void)
{
    static uint8_t banner_sent = 0U;

    if (banner_sent == 0U) {
        static const char banner[] =
            "KEY TEST READY: KEY1=PE4 KEY2=PE3 KEY3=PE2 KEY4=PE1 KEY5=PA15\r\n";
        Test_Key_Send(banner, (uint16_t)(sizeof(banner) - 1U));
        banner_sent = 1U;
    }

#if BSP_KEY1_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY1)) {
        static const char message[] = "KEY1 PRESSED (PE4)\r\n";
        Test_Key_Send(message, (uint16_t)(sizeof(message) - 1U));
    }
    if (BSP_Key_WasReleased(BSP_KEY1)) {
        static const char message[] = "KEY1 RELEASED (PE4)\r\n";
        Test_Key_Send(message, (uint16_t)(sizeof(message) - 1U));
    }
#endif
#if BSP_KEY2_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY2)) {
        static const char message[] = "KEY2 PRESSED (PE3)\r\n";
        Test_Key_Send(message, (uint16_t)(sizeof(message) - 1U));
    }
    if (BSP_Key_WasReleased(BSP_KEY2)) {
        static const char message[] = "KEY2 RELEASED (PE3)\r\n";
        Test_Key_Send(message, (uint16_t)(sizeof(message) - 1U));
    }
#endif
#if BSP_KEY3_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY3)) {
        static const char message[] = "KEY3 PRESSED (PE2)\r\n";
        Test_Key_Send(message, (uint16_t)(sizeof(message) - 1U));
    }
    if (BSP_Key_WasReleased(BSP_KEY3)) {
        static const char message[] = "KEY3 RELEASED (PE2)\r\n";
        Test_Key_Send(message, (uint16_t)(sizeof(message) - 1U));
    }
#endif
#if BSP_KEY4_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY4)) {
        static const char message[] = "KEY4 PRESSED (PE1)\r\n";
        Test_Key_Send(message, (uint16_t)(sizeof(message) - 1U));
    }
    if (BSP_Key_WasReleased(BSP_KEY4)) {
        static const char message[] = "KEY4 RELEASED (PE1)\r\n";
        Test_Key_Send(message, (uint16_t)(sizeof(message) - 1U));
    }
#endif
#if BSP_KEY5_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY5)) {
        static const char message[] = "KEY5 PRESSED (PA15)\r\n";
        Test_Key_Send(message, (uint16_t)(sizeof(message) - 1U));
    }
    if (BSP_Key_WasReleased(BSP_KEY5)) {
        static const char message[] = "KEY5 RELEASED (PA15)\r\n";
        Test_Key_Send(message, (uint16_t)(sizeof(message) - 1U));
    }
#endif
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

#if VEHICLE_UART1_E220_ENABLE

#define TEST_E220_SEND_PERIOD_MS          1000U
#define TEST_E220_RX_TIMEOUT_MS           2500U
#define TEST_E220_START_PHASE_MIN_MS      100U
#define TEST_E220_START_PHASE_SPAN_MS     800U
#define TEST_E220_RX_FRAME_MAX_LEN        32U
#define TEST_E220_UID_BASE_ADDRESS        0x1FFF7A10UL

typedef struct {
    uint32_t tx_value;
    uint32_t rx_value;
    uint32_t rx_count;
    uint32_t last_rx_ms;
    uint32_t revision;
    uint16_t self_id;
    uint16_t peer_id;
    uint8_t rx_valid;
    uint8_t online;
} Test_E220_LinkState_t;

static Test_E220_LinkState_t s_test_e220_state;

static uint32_t Test_E220_GetUidHash(void)
{
    volatile const uint32_t *uid = (volatile const uint32_t *)TEST_E220_UID_BASE_ADDRESS;

    return uid[0] ^ uid[1] ^ uid[2] ^ (uid[0] >> 16U) ^ (uid[2] << 7U);
}

static uint8_t Test_E220_HexToValue(uint8_t ch, uint8_t *value)
{
    if ((ch >= (uint8_t)'0') && (ch <= (uint8_t)'9')) {
        *value = (uint8_t)(ch - (uint8_t)'0');
        return 1U;
    }
    if ((ch >= (uint8_t)'A') && (ch <= (uint8_t)'F')) {
        *value = (uint8_t)(ch - (uint8_t)'A' + 10U);
        return 1U;
    }
    if ((ch >= (uint8_t)'a') && (ch <= (uint8_t)'f')) {
        *value = (uint8_t)(ch - (uint8_t)'a' + 10U);
        return 1U;
    }
    return 0U;
}

static uint8_t Test_E220_ParseFrame(const uint8_t *frame,
                                    uint8_t length,
                                    uint16_t *peer_id,
                                    uint32_t *value)
{
    static const uint8_t prefix[] = "E220,";
    uint32_t parsed_value = 0U;
    uint16_t parsed_id = 0U;
    uint8_t digit;
    uint8_t i;

    if ((frame == 0) || (peer_id == 0) || (value == 0) || (length < 11U)) {
        return 0U;
    }

    for (i = 0U; i < (uint8_t)(sizeof(prefix) - 1U); i++) {
        if (frame[i] != prefix[i]) {
            return 0U;
        }
    }

    for (i = 5U; i < 9U; i++) {
        if (Test_E220_HexToValue(frame[i], &digit) == 0U) {
            return 0U;
        }
        parsed_id = (uint16_t)((parsed_id << 4U) | digit);
    }

    if (frame[9] != (uint8_t)',') {
        return 0U;
    }

    for (i = 10U; i < length; i++) {
        if ((frame[i] < (uint8_t)'0') || (frame[i] > (uint8_t)'9')) {
            return 0U;
        }
        digit = (uint8_t)(frame[i] - (uint8_t)'0');
        if (parsed_value > ((0xFFFFFFFFUL - digit) / 10U)) {
            return 0U;
        }
        parsed_value = (parsed_value * 10U) + digit;
    }

    *peer_id = parsed_id;
    *value = parsed_value;
    return 1U;
}

static void Test_E220_OledUpdate(void)
{
    static uint32_t displayed_revision = 0U;
    char text[24];

    if ((displayed_revision == s_test_e220_state.revision) ||
        (Drv_OledI2c_IsReady() == 0U) ||
        (Drv_OledI2c_IsBusy() != 0U)) {
        return;
    }

    Drv_OledI2c_Clear();
    Drv_OledI2c_DrawString5x7(2U, 0U, "E220 CAR LINK", DRV_OLED_COLOR_ON);
    (void)snprintf(text, sizeof(text), "SELF:%04X",
                   (unsigned int)s_test_e220_state.self_id);
    Drv_OledI2c_DrawString5x7(2U, 10U, text, DRV_OLED_COLOR_ON);
    (void)snprintf(text, sizeof(text), "TX:%lu",
                   (unsigned long)s_test_e220_state.tx_value);
    Drv_OledI2c_DrawString5x7(2U, 20U, text, DRV_OLED_COLOR_ON);

    if (s_test_e220_state.rx_valid != 0U) {
        (void)snprintf(text, sizeof(text), "PEER:%04X",
                       (unsigned int)s_test_e220_state.peer_id);
    } else {
        (void)snprintf(text, sizeof(text), "PEER:----");
    }
    Drv_OledI2c_DrawString5x7(2U, 30U, text, DRV_OLED_COLOR_ON);

    if (s_test_e220_state.rx_valid != 0U) {
        (void)snprintf(text, sizeof(text), "RX:%lu",
                       (unsigned long)s_test_e220_state.rx_value);
    } else {
        (void)snprintf(text, sizeof(text), "RX:----");
    }
    Drv_OledI2c_DrawString5x7(2U, 40U, text, DRV_OLED_COLOR_ON);

    (void)snprintf(text, sizeof(text), "N:%lu %s",
                   (unsigned long)s_test_e220_state.rx_count,
                   (s_test_e220_state.online != 0U) ? "OK" : "WAIT");
    Drv_OledI2c_DrawString5x7(2U, 52U, text, DRV_OLED_COLOR_ON);
    Drv_OledI2c_Flush();
    displayed_revision = s_test_e220_state.revision;
}

static void Test_E220_LcdUpdate(void)
{
    static Test_E220_LinkState_t snapshot;
    static uint32_t displayed_revision = 0U;
    static uint32_t active_revision = 0U;
    static uint8_t refresh_pending = 0U;
    static uint8_t base_ready = 0U;
    static uint8_t step = 0U;
    BSP_Status_t status = BSP_BUSY;
    char text[24];

    if (Drv_LcdTft_IsReady() == 0U) {
        return;
    }

    if (refresh_pending == 0U) {
        if (displayed_revision == s_test_e220_state.revision) {
            return;
        }
        snapshot = s_test_e220_state;
        active_revision = s_test_e220_state.revision;
        step = (base_ready != 0U) ? 3U : 0U;
        refresh_pending = 1U;
    }

    if (Drv_LcdTft_IsBusy() != 0U) {
        return;
    }

    switch (step) {
        case 0U:
            status = Drv_LcdTft_TryClear(DRV_LCD_COLOR_BLACK);
            break;

        case 1U:
            status = Drv_LcdTft_TryDrawRect(4U, 4U, 232U, 232U, DRV_LCD_COLOR_BLUE);
            break;

        case 2U:
            status = Drv_LcdTft_TryDrawString5x7(16U, 18U, "E220 CAR LINK TEST",
                                                 DRV_LCD_COLOR_CYAN,
                                                 DRV_LCD_COLOR_BLACK);
            break;

        case 3U:
            (void)snprintf(text, sizeof(text), "SELF ID:%04X",
                           (unsigned int)snapshot.self_id);
            status = Drv_LcdTft_TryDrawString5x7(16U, 48U, text,
                                                 DRV_LCD_COLOR_WHITE,
                                                 DRV_LCD_COLOR_BLACK);
            break;

        case 4U:
            (void)snprintf(text, sizeof(text), "TX:%10lu",
                           (unsigned long)snapshot.tx_value);
            status = Drv_LcdTft_TryDrawString5x7(16U, 72U, text,
                                                 DRV_LCD_COLOR_WHITE,
                                                 DRV_LCD_COLOR_BLACK);
            break;

        case 5U:
            if (snapshot.rx_valid != 0U) {
                (void)snprintf(text, sizeof(text), "PEER ID:%04X",
                               (unsigned int)snapshot.peer_id);
            } else {
                (void)snprintf(text, sizeof(text), "PEER ID:----");
            }
            status = Drv_LcdTft_TryDrawString5x7(16U, 96U, text,
                                                 DRV_LCD_COLOR_YELLOW,
                                                 DRV_LCD_COLOR_BLACK);
            break;

        case 6U:
            if (snapshot.rx_valid != 0U) {
                (void)snprintf(text, sizeof(text), "RX:%10lu",
                               (unsigned long)snapshot.rx_value);
            } else {
                (void)snprintf(text, sizeof(text), "RX:----------");
            }
            status = Drv_LcdTft_TryDrawString5x7(16U, 120U, text,
                                                 DRV_LCD_COLOR_YELLOW,
                                                 DRV_LCD_COLOR_BLACK);
            break;

        case 7U:
            (void)snprintf(text, sizeof(text), "RX COUNT:%8lu",
                           (unsigned long)snapshot.rx_count);
            status = Drv_LcdTft_TryDrawString5x7(16U, 144U, text,
                                                 DRV_LCD_COLOR_WHITE,
                                                 DRV_LCD_COLOR_BLACK);
            break;

        case 8U:
            status = Drv_LcdTft_TryDrawString5x7(
                16U, 168U,
                (snapshot.online != 0U) ? "LINK:OK            " : "LINK:WAIT          ",
                (snapshot.online != 0U) ? DRV_LCD_COLOR_GREEN : DRV_LCD_COLOR_RED,
                DRV_LCD_COLOR_BLACK);
            break;

        default:
            base_ready = 1U;
            displayed_revision = active_revision;
            refresh_pending = 0U;
            return;
    }

    if (status == BSP_OK) {
        step++;
    }
}

#endif

void Test_E220_Link_Update(void)
{
#if VEHICLE_UART1_E220_ENABLE
    static uint8_t initialized = 0U;
    static uint8_t rx_frame[TEST_E220_RX_FRAME_MAX_LEN];
    static uint8_t rx_length = 0U;
    static uint8_t discard_until_newline = 0U;
    static uint32_t next_send_ms = 0U;
    static uint32_t next_value = 1U;
    uint32_t uid_hash;
    uint32_t received_value;
    uint32_t now_ms = BSP_GET_TICK();
    uint16_t received_id;
    uint8_t ch;
    char tx_frame[32];
    int length;

    if (initialized == 0U) {
        uid_hash = Test_E220_GetUidHash();
        s_test_e220_state.self_id = (uint16_t)(uid_hash ^ (uid_hash >> 16U));
        next_send_ms = now_ms + TEST_E220_START_PHASE_MIN_MS +
                       (uid_hash % TEST_E220_START_PHASE_SPAN_MS);
        s_test_e220_state.revision++;
        initialized = 1U;
    }

    while (BSP_UART_GetChar(UART_PORT1, &ch) != 0U) {
        if (ch == (uint8_t)'\n') {
            if ((discard_until_newline == 0U) &&
                (Test_E220_ParseFrame(rx_frame, rx_length,
                                      &received_id, &received_value) != 0U) &&
                (received_id != s_test_e220_state.self_id)) {
                s_test_e220_state.peer_id = received_id;
                s_test_e220_state.rx_value = received_value;
                s_test_e220_state.rx_count++;
                s_test_e220_state.last_rx_ms = now_ms;
                s_test_e220_state.rx_valid = 1U;
                s_test_e220_state.online = 1U;
                s_test_e220_state.revision++;
            }
            rx_length = 0U;
            discard_until_newline = 0U;
        } else if (ch != (uint8_t)'\r') {
            if ((discard_until_newline == 0U) &&
                (rx_length < (TEST_E220_RX_FRAME_MAX_LEN - 1U))) {
                rx_frame[rx_length++] = ch;
            } else {
                /* 超长或破损帧一直丢弃到换行，避免把帧尾误当成新帧。 */
                discard_until_newline = 1U;
            }
        }
    }

    if ((s_test_e220_state.online != 0U) &&
        ((uint32_t)(now_ms - s_test_e220_state.last_rx_ms) >= TEST_E220_RX_TIMEOUT_MS)) {
        s_test_e220_state.online = 0U;
        s_test_e220_state.revision++;
    }

    if ((int32_t)(now_ms - next_send_ms) >= 0) {
        length = snprintf(tx_frame, sizeof(tx_frame), "E220,%04X,%lu\r\n",
                          (unsigned int)s_test_e220_state.self_id,
                          (unsigned long)next_value);
        if ((length > 0) && (length < (int)sizeof(tx_frame)) &&
            (BSP_UART_WriteFrame(UART_PORT1,
                                 (const uint8_t *)tx_frame,
                                 (uint16_t)length) == BSP_OK)) {
            s_test_e220_state.tx_value = next_value;
            next_value++;
            if (next_value == 0U) {
                next_value = 1U;
            }
            next_send_ms = now_ms + TEST_E220_SEND_PERIOD_MS;
            s_test_e220_state.revision++;
        }
    }

    Test_E220_OledUpdate();
    Test_E220_LcdUpdate();
#endif
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
                "ENC FL=%ld FR=%ld RL=%ld RR=%ld | L=%ld R=%ld\r\n",
                (long)Drv_Encoder_GetWheelSpeedCps(WHEEL_FL),
                (long)Drv_Encoder_GetWheelSpeedCps(WHEEL_FR),
                (long)Drv_Encoder_GetWheelSpeedCps(WHEEL_RL),
                (long)Drv_Encoder_GetWheelSpeedCps(WHEEL_RR),
                (long)Drv_Encoder_GetLeftSpeedCps(),
                (long)Drv_Encoder_GetRightSpeedCps());

    if (n > 0 && n < (int)sizeof(buf)) {
        BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)buf, (uint16_t)n);
    }
}

#define TEST_CHASSIS_INITIAL_LINEAR_CPS  1600
#define TEST_CHASSIS_LINEAR_STEP_CPS      200
#define TEST_CHASSIS_LINEAR_MAX_CPS      4800
#define TEST_CHASSIS_TURN_CPS            1600

static BSP_Status_t Test_ChassisSetCommand(int16_t linear_cps,
                                           int16_t turn_cps)
{
    BSP_Status_t status;

    status = Chassis_AcquireControl(CHASSIS_OWNER_TEST);
    if (status != BSP_OK) {
        return status;
    }
    return Chassis_SetSpeed(CHASSIS_OWNER_TEST, linear_cps, turn_cps);
}

/*
 * 五按键底盘速度控制测试，必须先运行Key_Update()和Encoder_Update()。
 * KEY1 以 1600 cps 前进并重置加速档位；KEY2 每次增加 200 cps，
 * 最大 4800 cps；KEY3/KEY4 原地转向；KEY5 立即停止并尝试清除故障。
 */
void Test_ChassisCmd_Update(void)
{
    static int16_t linear_speed_cps = TEST_CHASSIS_INITIAL_LINEAR_CPS;
    static int16_t active_linear_cps = 0;
    static int16_t active_turn_cps = 0;
    static uint8_t command_active = 0U;
    static uint8_t forward_started = 0U;
    static uint8_t banner_sent = 0U;
    char message[96];
    int length;
    BSP_Status_t status;

    if (banner_sent == 0U) {
        static const char banner[] =
            "CHASSIS KEY TEST: K1=FWD1600 K2=+200(MAX4800) K3=LEFT K4=RIGHT K5=STOP/CLEAR\r\n";
        banner_sent = 1U;
        LcdUi_ChassisTestBegin();
        Test_Key_Send(banner, (uint16_t)(sizeof(banner) - 1U));
    }

    /* 正式测试入口每10 ms刷新一次命令租约，按键只改变当前命令。 */
    if (command_active != 0U) {
        status = Chassis_SetSpeed(CHASSIS_OWNER_TEST,
                                  active_linear_cps,
                                  active_turn_cps);
        if (status != BSP_OK) {
            command_active = 0U;
            forward_started = 0U;
        }
    }

#if BSP_KEY1_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY1)) {
        linear_speed_cps = TEST_CHASSIS_INITIAL_LINEAR_CPS;
        status = Test_ChassisSetCommand(linear_speed_cps, 0);
        if (status != BSP_OK) {
            static const char busy[] =
                "CHASSIS KEY1 REJECTED: CONTROL BUSY OR FAULT\r\n";
            Test_Key_Send(busy, (uint16_t)(sizeof(busy) - 1U));
            forward_started = 0U;
            command_active = 0U;
            return;
        }
        forward_started = 1U;
        active_linear_cps = linear_speed_cps;
        active_turn_cps = 0;
        command_active = 1U;
        length = snprintf(message, sizeof(message),
                          "CHASSIS KEY1 FORWARD: linear=%d cps\r\n",
                          (int)linear_speed_cps);
        if ((length > 0) && (length < (int)sizeof(message))) {
            Test_Key_Send(message, (uint16_t)length);
        }
    }
#endif

#if BSP_KEY2_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY2)) {
        if (forward_started == 0U) {
            static const char response[] =
                "CHASSIS KEY2 IGNORED: PRESS KEY1 FIRST\r\n";
            Test_Key_Send(response, (uint16_t)(sizeof(response) - 1U));
        } else {
            if (linear_speed_cps < TEST_CHASSIS_LINEAR_MAX_CPS) {
                linear_speed_cps = (int16_t)(linear_speed_cps + TEST_CHASSIS_LINEAR_STEP_CPS);
                if (linear_speed_cps > TEST_CHASSIS_LINEAR_MAX_CPS) {
                    linear_speed_cps = TEST_CHASSIS_LINEAR_MAX_CPS;
                }
            }
            status = Test_ChassisSetCommand(linear_speed_cps, 0);
            if (status != BSP_OK) {
                static const char busy[] =
                    "CHASSIS KEY2 REJECTED: CONTROL BUSY OR FAULT\r\n";
                Test_Key_Send(busy, (uint16_t)(sizeof(busy) - 1U));
                command_active = 0U;
                return;
            }
            active_linear_cps = linear_speed_cps;
            active_turn_cps = 0;
            command_active = 1U;
            length = snprintf(message, sizeof(message),
                              "CHASSIS KEY2 ACCEL: linear=%d cps%s\r\n",
                              (int)linear_speed_cps,
                              (linear_speed_cps >= TEST_CHASSIS_LINEAR_MAX_CPS) ? " (MAX)" : "");
            if ((length > 0) && (length < (int)sizeof(message))) {
                Test_Key_Send(message, (uint16_t)length);
            }
        }
    }
#endif

#if BSP_KEY3_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY3)) {
        static const char response[] = "CHASSIS KEY3: TURN LEFT 1600 cps\r\n";
        status = Test_ChassisSetCommand(0, TEST_CHASSIS_TURN_CPS);
        if (status != BSP_OK) {
            static const char busy[] =
                "CHASSIS KEY3 REJECTED: CONTROL BUSY OR FAULT\r\n";
            Test_Key_Send(busy, (uint16_t)(sizeof(busy) - 1U));
            command_active = 0U;
            return;
        }
        active_linear_cps = 0;
        active_turn_cps = TEST_CHASSIS_TURN_CPS;
        command_active = 1U;
        Test_Key_Send(response, (uint16_t)(sizeof(response) - 1U));
    }
#endif

#if BSP_KEY4_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY4)) {
        static const char response[] = "CHASSIS KEY4: TURN RIGHT 1600 cps\r\n";
        status = Test_ChassisSetCommand(0, -TEST_CHASSIS_TURN_CPS);
        if (status != BSP_OK) {
            static const char busy[] =
                "CHASSIS KEY4 REJECTED: CONTROL BUSY OR FAULT\r\n";
            Test_Key_Send(busy, (uint16_t)(sizeof(busy) - 1U));
            command_active = 0U;
            return;
        }
        active_linear_cps = 0;
        active_turn_cps = (int16_t)(-TEST_CHASSIS_TURN_CPS);
        command_active = 1U;
        Test_Key_Send(response, (uint16_t)(sizeof(response) - 1U));
    }
#endif

#if BSP_KEY5_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY5)) {
        static const char cleared[] =
            "CHASSIS KEY5: STOPPED / FAULT CLEARED\r\n";
        static const char wait_stop[] =
            "CHASSIS KEY5: STOPPED / WAIT WHEELS THEN CLEAR AGAIN\r\n";
        forward_started = 0U;
        command_active = 0U;
        active_linear_cps = 0;
        active_turn_cps = 0;
        Chassis_EmergencyStop();
        status = Chassis_ClearFault();
        if (status == BSP_OK) {
            Test_Key_Send(cleared, (uint16_t)(sizeof(cleared) - 1U));
        } else {
            Test_Key_Send(wait_stop, (uint16_t)(sizeof(wait_stop) - 1U));
        }
    }
#endif
}

/*
 * 命令租约看门狗专项测试：KEY1只发送一次速度命令，故意不再刷新；
 * KEY5急停并尝试清除故障。必须与普通底盘按键测试二选一注册。
 */
void Test_ChassisWatchdog_Update(void)
{
    static uint8_t banner_sent = 0U;
    BSP_Status_t status;

    if (banner_sent == 0U) {
        static const char banner[] =
            "CHASSIS WATCHDOG TEST: K1=ONE SHOT K5=STOP/CLEAR\r\n";
        banner_sent = 1U;
        LcdUi_ChassisTestBegin();
        Test_Key_Send(banner, (uint16_t)(sizeof(banner) - 1U));
    }

#if BSP_KEY1_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY1)) {
        static const char started[] =
            "CHASSIS WATCHDOG: ONE SHOT 1600 cps\r\n";
        static const char rejected[] =
            "CHASSIS WATCHDOG: START REJECTED\r\n";

        status = Test_ChassisSetCommand(TEST_CHASSIS_INITIAL_LINEAR_CPS, 0);
        if (status == BSP_OK) {
            Test_Key_Send(started, (uint16_t)(sizeof(started) - 1U));
        } else {
            Test_Key_Send(rejected, (uint16_t)(sizeof(rejected) - 1U));
        }
    }
#endif

#if BSP_KEY5_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY5)) {
        static const char cleared[] =
            "CHASSIS WATCHDOG: STOPPED / FAULT CLEARED\r\n";
        static const char wait_stop[] =
            "CHASSIS WATCHDOG: WAIT WHEELS THEN CLEAR AGAIN\r\n";

        Chassis_EmergencyStop();
        status = Chassis_ClearFault();
        if (status == BSP_OK) {
            Test_Key_Send(cleared, (uint16_t)(sizeof(cleared) - 1U));
        } else {
            Test_Key_Send(wait_stop, (uint16_t)(sizeof(wait_stop) - 1U));
        }
    }
#endif
}

void Test_ChassisCmd_Log(void)
{
    Chassis_Info_t info;
    char buf[400];
    int n;

    if (Chassis_GetInfo(&info) != BSP_OK) return;

    n = snprintf(buf, sizeof(buf),
                "CHS loop=%u fault=%d mask=%02X age=%lu owner=%d mode=%d "
                "cmd=%d/%d tgt=%d/%d "
                "raw=%ld/%ld/%ld/%ld fb=%ld/%ld/%ld/%ld "
                "ff=%d/%d/%d/%d "
                "pi=%d/%d/%d/%d out=%d/%d/%d/%d\r\n",
                (unsigned int)info.speed_loop_enabled,
                (int)info.fault,
                (unsigned int)info.fault_wheel_mask,
                (unsigned long)info.command_age_ms,
                (int)info.owner,
                (int)info.mode,
                info.left_target_cps,
                info.right_target_cps,
                info.left_applied_target_cps,
                info.right_applied_target_cps,
                (long)Drv_Encoder_GetWheelRawSpeedCps(WHEEL_FL),
                (long)Drv_Encoder_GetWheelRawSpeedCps(WHEEL_FR),
                (long)Drv_Encoder_GetWheelRawSpeedCps(WHEEL_RL),
                (long)Drv_Encoder_GetWheelRawSpeedCps(WHEEL_RR),
                (long)info.fl_feedback_cps,
                (long)info.fr_feedback_cps,
                (long)info.rl_feedback_cps,
                (long)info.rr_feedback_cps,
                info.fl_feedforward,
                info.fr_feedforward,
                info.rl_feedforward,
                info.rr_feedforward,
                info.fl_pid_correction,
                info.fr_pid_correction,
                info.rl_pid_correction,
                info.rr_pid_correction,
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
 * 五按键非阻塞动作库测试，必须先运行 Key_Update()。
 * KEY1/KEY2 分别左转/右转 90 度；KEY3/KEY4 分别前进/后退
 * 500 mm；KEY5 取消当前动作并停车。
 */
void Test_MotionCmd_Update(void)
{
    static uint8_t banner_sent = 0U;
    BSP_Status_t status;

    if (banner_sent == 0U) {
        static const char banner[] =
            "MOTION KEY TEST: K1=LEFT90 K2=RIGHT90 K3=FWD500 K4=BACK500 K5=STOP\r\n";
        banner_sent = 1U;
        Test_Key_Send(banner, (uint16_t)(sizeof(banner) - 1U));
    }

#if BSP_KEY1_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY1)) {
        static const char started[] = "MOTION KEY1: LEFT 90 STARTED\r\n";
        static const char not_ready[] =
            "MOTION KEY1 LEFT90 REJECTED: IMU NOT READY\r\n";
        static const char busy[] = "MOTION KEY1 LEFT90 REJECTED: BUSY\r\n";

        status = Motion_TurnAngle(90);
        if (status == BSP_OK) {
            Test_Key_Send(started, (uint16_t)(sizeof(started) - 1U));
        } else if (status == BSP_ERROR) {
            Test_Key_Send(not_ready, (uint16_t)(sizeof(not_ready) - 1U));
        } else {
            Test_Key_Send(busy, (uint16_t)(sizeof(busy) - 1U));
        }
    }
#endif

#if BSP_KEY2_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY2)) {
        static const char started[] = "MOTION KEY2: RIGHT 90 STARTED\r\n";
        static const char not_ready[] =
            "MOTION KEY2 RIGHT90 REJECTED: IMU NOT READY\r\n";
        static const char busy[] = "MOTION KEY2 RIGHT90 REJECTED: BUSY\r\n";

        status = Motion_TurnAngle(-90);
        if (status == BSP_OK) {
            Test_Key_Send(started, (uint16_t)(sizeof(started) - 1U));
        } else if (status == BSP_ERROR) {
            Test_Key_Send(not_ready, (uint16_t)(sizeof(not_ready) - 1U));
        } else {
            Test_Key_Send(busy, (uint16_t)(sizeof(busy) - 1U));
        }
    }
#endif

#if BSP_KEY3_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY3)) {
        static const char started[] = "MOTION KEY3: FORWARD 500MM STARTED\r\n";
        static const char busy[] = "MOTION KEY3 FORWARD REJECTED: BUSY\r\n";
        status = Motion_GoDistance(500, 800);
        Test_Key_Send((status == BSP_OK) ? started : busy,
                      (status == BSP_OK) ?
                          (uint16_t)(sizeof(started) - 1U) :
                          (uint16_t)(sizeof(busy) - 1U));
    }
#endif

#if BSP_KEY4_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY4)) {
        static const char started[] = "MOTION KEY4: BACKWARD 500MM STARTED\r\n";
        static const char busy[] = "MOTION KEY4 BACKWARD REJECTED: BUSY\r\n";
        status = Motion_GoDistance(-500, 800);
        Test_Key_Send((status == BSP_OK) ? started : busy,
                      (status == BSP_OK) ?
                          (uint16_t)(sizeof(started) - 1U) :
                          (uint16_t)(sizeof(busy) - 1U));
    }
#endif

#if BSP_KEY5_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY5)) {
        static const char response[] = "MOTION KEY5: STOP\r\n";
        Motion_Stop();
        Test_Key_Send(response, (uint16_t)(sizeof(response) - 1U));
    }
#endif
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

#define TEST_LINE_LOG_BUF_SIZE 896U

static uint8_t Test_Line_Append(char *buf,
                                uint16_t size,
                                uint16_t *used,
                                const char *fmt,
                                ...)
{
    va_list args;
    int n;
    uint16_t remain;

    if ((buf == 0) || (used == 0) || (fmt == 0) || (*used >= size)) {
        return 0U;
    }

    remain = (uint16_t)(size - *used);
    va_start(args, fmt);
    n = vsnprintf(&buf[*used], remain, fmt, args);
    va_end(args);

    if ((n <= 0) || (n >= (int)remain)) {
        return 0U;
    }

    *used = (uint16_t)(*used + (uint16_t)n);
    return 1U;
}

static uint8_t Test_Line_AppendThreshold(char *buf,
                                         uint16_t size,
                                         uint16_t *used)
{
    uint16_t threshold[LINE_DETECT_SENSOR_NUM];

    if (LineDetect_GetThresholdArray(threshold, LINE_DETECT_SENSOR_NUM) != BSP_OK) {
        return 0U;
    }

    return Test_Line_Append(buf,
                            size,
                            used,
                            "TH   %4u %4u %4u %4u %4u %4u %4u %4u\r\n",
                            (unsigned int)threshold[0],
                            (unsigned int)threshold[1],
                            (unsigned int)threshold[2],
                            (unsigned int)threshold[3],
                            (unsigned int)threshold[4],
                            (unsigned int)threshold[5],
                            (unsigned int)threshold[6],
                            (unsigned int)threshold[7]);
}

static void Test_Line_SendBuffer(const char *buf, uint16_t used)
{
    if ((buf != 0) && (used != 0U)) {
        (void)BSP_UART_WriteFrame(UART_PORT1,
                                  (const uint8_t *)buf,
                                  used);
    }
}

static void Test_Line_SendMessageAndThreshold(const char *message)
{
    char buf[192];
    uint16_t used = 0U;

    if (message != 0) {
        (void)Test_Line_Append(buf, sizeof(buf), &used, "%s", message);
    }
    (void)Test_Line_AppendThreshold(buf, sizeof(buf), &used);
    Test_Line_SendBuffer(buf, used);
}

static void Test_Line_Print(void)
{
    LineFollow_Info_t info;
    Drv_GraySensor_Info_t sensor;
#if (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_MCU_I2C)
    Drv_GrayMcu_Info_t mcu;
    BSP_I2C_Debug_t i2c;
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_YAHBOOM_UART)
    Drv_GrayYahboom_Info_t yahboom;
#endif
    static char buf[TEST_LINE_LOG_BUF_SIZE];
    uint16_t used = 0U;

    if (LineFollow_GetInfo(&info) != BSP_OK) return;
    if (Drv_GraySensor_GetInfo(&sensor) != BSP_OK) return;
    buf[0] = '\0';

#if (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_MCU_I2C)
    (void)Drv_GrayMcu_GetInfo(&mcu);
    (void)BSP_I2C_GetDebug(I2C_BUS1, &i2c);

    (void)Test_Line_Append(
        buf,
        sizeof(buf),
        &used,
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
#elif (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_YAHBOOM_UART)
    (void)Drv_GrayYahboom_GetInfo(&yahboom);
    (void)Test_Line_Append(
        buf,
        sizeof(buf),
        &used,
        "GRAY src=YB mode=%u mask=0x%02X on=%u valid=%u init=%u cmd=%u/%u "
        "age=%lu vf=%lu bad=%lu parse=%lu ovf=%lu uart=%lu\r\n",
        (unsigned int)yahboom.data_mode,
        (unsigned int)yahboom.digital_black_mask,
        (unsigned int)yahboom.online,
        (unsigned int)yahboom.valid,
        (unsigned int)yahboom.initialized,
        (unsigned int)yahboom.command_sent,
        (unsigned int)yahboom.command_count,
        (unsigned long)yahboom.frame_age_ms,
        (unsigned long)yahboom.valid_frame_count,
        (unsigned long)yahboom.invalid_frame_count,
        (unsigned long)yahboom.parse_error_count,
        (unsigned long)yahboom.rx_overflow_count,
        (unsigned long)yahboom.uart_error_count);
#else
    (void)Test_Line_Append(buf,
                           sizeof(buf),
                           &used,
                           "GRAY src=4051 on=%u valid=%u\r\n",
                           (unsigned int)sensor.online,
                           (unsigned int)sensor.valid);
#endif

#if (GRAY_SENSOR_SOURCE == GRAY_SENSOR_SOURCE_MCU_I2C)
    (void)Test_Line_Append(
        buf,
        sizeof(buf),
        &used,
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
#endif

    (void)Test_Line_Append(
        buf,
        sizeof(buf),
        &used,
        "RAW  %4u %4u %4u %4u %4u %4u %4u %4u | FILT %4u %4u %4u %4u %4u %4u %4u %4u\r\n",
        (unsigned int)sensor.raw[0], (unsigned int)sensor.raw[1],
        (unsigned int)sensor.raw[2], (unsigned int)sensor.raw[3],
        (unsigned int)sensor.raw[4], (unsigned int)sensor.raw[5],
        (unsigned int)sensor.raw[6], (unsigned int)sensor.raw[7],
        (unsigned int)sensor.filt[0], (unsigned int)sensor.filt[1],
        (unsigned int)sensor.filt[2], (unsigned int)sensor.filt[3],
        (unsigned int)sensor.filt[4], (unsigned int)sensor.filt[5],
        (unsigned int)sensor.filt[6], (unsigned int)sensor.filt[7]);

    (void)Test_Line_AppendThreshold(buf, sizeof(buf), &used);

    (void)Test_Line_Append(
        buf,
        sizeof(buf),
        &used,
        "LINE st=%d type=%s mask=0x%02X cnt=%d err=%d out(v=%d,t=%d)\r\n",
        (int)info.state,
        LineTypeName(info.detect.type),
        (unsigned int)info.detect.black_mask,
        (int)info.detect.black_count,
        (int)info.detect.error_x1000,
        (int)info.output.linear_cps,
        (int)info.output.turn_cps);

    Test_Line_SendBuffer(buf, used);
}

void Test_LineCmd_Update(void)
{
    uint8_t ch;
    uint16_t raw[LINE_DETECT_SENSOR_NUM];
    BSP_Status_t status;

    while (BSP_UART_GetChar(UART_PORT1, &ch)) {
        if ((ch == '1') || (ch == 'l') || (ch == 'L')) {
            status = LineFollow_Start();
            if (status == BSP_OK) {
                (void)BSP_UART_WriteFrame(UART_PORT1,
                                          (const uint8_t *)"line follow RUN\r\n",
                                          (uint16_t)(sizeof("line follow RUN\r\n") - 1U));
            } else if (status == BSP_ERROR) {
                (void)BSP_UART_WriteFrame(UART_PORT1,
                                          (const uint8_t *)"line follow rejected: gray sensor offline\r\n",
                                          (uint16_t)(sizeof("line follow rejected: gray sensor offline\r\n") - 1U));
            } else {
                (void)BSP_UART_WriteFrame(UART_PORT1,
                                          (const uint8_t *)"line follow rejected: control busy\r\n",
                                          (uint16_t)(sizeof("line follow rejected: control busy\r\n") - 1U));
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
            Test_Line_SendMessageAndThreshold("make threshold ok\r\n");
        } else if (ch == 'd') {
            LineDetect_SetAllThreshold(LINE_DETECT_DEFAULT_THRESHOLD);
            Test_Line_SendMessageAndThreshold("default threshold\r\n");
        } else if (ch == 'p') {
            Test_Line_Print();
        } else if (ch == 'k') {
            Test_I2C_Scan();
        }
    }
}

void Test_LineCmd_Log(void)
{
    Test_Line_Print();
}

void Test_RouteLog(void)
{
#if (TASK_PROFILE_SELECT == TASK_PROFILE_H2_ROUND_STOP)
    H2Task_Info_t task;
    HOvalRoute_Info_t route;
    char buf[384];
    int length;

    if ((H2Task_GetInfo(&task) != BSP_OK) ||
        (HRoute_GetH2Info(&route) != PROJECT_OK)) {
        return;
    }

    length = snprintf(
        buf,
        sizeof(buf),
        "H2 task=%u route=%u elapsed=%lu mask=0x%02X "
        "black=%u span=%u err=%d turn=%d curve_ms=%lu "
        "armed=%u cand=%u speed=%ld/%ld enc_mm=%ld "
        "stop=immediate stop_offset=%ld enc_ok=%u fault=%u/%u\r\n",
        (unsigned int)task.state,
        (unsigned int)route.state,
        (unsigned long)task.elapsed_ms,
        (unsigned int)route.gray_mask,
        (unsigned int)route.black_count,
        (unsigned int)route.black_span,
        (int)route.line_error,
        (int)route.turn_output,
        (unsigned long)route.curve_confirm_ms,
        (unsigned int)route.finish_armed,
        (unsigned int)route.finish_candidate,
        (long)task.left_speed_cps,
        (long)task.right_speed_cps,
        (long)task.encoder_distance_mm,
        (long)task.stop_offset_mm,
        (unsigned int)task.encoder_reliable,
        (unsigned int)task.fault,
        (unsigned int)route.fault);
    if ((length > 0) && (length < (int)sizeof(buf))) {
        Test_Key_Send(buf, (uint16_t)length);
    }
#else
    LcdUi_RouteTestBegin();
    OledUi_RouteTestBegin();
#endif
}

void Test_RouteCmd_Update(void)
{
    uint8_t ch;
    BSP_Status_t status;

    /* Key_Update() 生成消抖事件，本任务只处理 KEY1 和 KEY4。 */

#if BSP_KEY1_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY1)) {
        status = LineFollow_Start();
        if (status == BSP_OK) {
            static const char message[] = "ROUTE START: KEY1\r\n";
            Test_Key_Send(message, (uint16_t)(sizeof(message) - 1U));
        } else if (status == BSP_ERROR) {
            static const char message[] = "ROUTE START REJECTED: GRAY OFFLINE\r\n";
            Test_Key_Send(message, (uint16_t)(sizeof(message) - 1U));
        } else {
            static const char message[] = "ROUTE START REJECTED: CONTROL BUSY\r\n";
            Test_Key_Send(message, (uint16_t)(sizeof(message) - 1U));
        }
    }
#endif

#if BSP_KEY4_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY4)) {
        static const char message[] = "ROUTE STOP: KEY4\r\n";
        LineFollow_Stop();
        Test_Key_Send(message, (uint16_t)(sizeof(message) - 1U));
    }
#endif

    /* USART1 只保留路线复位命令，路线状态改由 LCD 显示。 */
    while (BSP_UART_GetChar(UART_PORT1, &ch)) {
        if ((ch == 'r') || (ch == 'R')) {
            RouteManager_Reset(BSP_GetTickMs());
        }
    }
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

static void Test_HX711_Print(void)
{
    float pressure_g;
    long pressure_tenth_g;
    char pressure_text[24];
    const char *display_text;
    char line[48];
    int length;

    /* 自动去皮或传感器未就绪期间保持静默，只输出有效克重。 */
    if (Sensor_GetPressureGram(&pressure_g) != BSP_OK) {
        return;
    }

    pressure_tenth_g = (long)((pressure_g * 10.0f) +
                              ((pressure_g >= 0.0f) ? 0.5f : -0.5f));
    Test_FormatFixed(pressure_text,
                     sizeof(pressure_text),
                     pressure_tenth_g,
                     10UL,
                     1U);
    display_text = (pressure_text[0] == '+') ? &pressure_text[1] : pressure_text;
    length = snprintf(line, sizeof(line), "WEIGHT=%s g\r\n", display_text);
    if ((length > 0) && (length < (int)sizeof(line))) {
        (void)BSP_UART_WriteFrame(UART_PORT1, (const uint8_t *)line, (uint16_t)length);
    }
}

/*
 * HX711 克重输出任务。任务表必须同时保留 Sensor_Update 以推进采样。
 * 本函数不读取串口命令；上电空载自动去皮完成后，每 200 ms 只输出克重。
 */
void Test_HX711_Update(void)
{
    static uint32_t last_print_ms = 0U;

    if ((uint32_t)(BSP_GET_TICK() - last_print_ms) >= 200U) {
        last_print_ms = BSP_GET_TICK();
        Test_HX711_Print();
    }
}

/* 正式总任务状态机联调日志，状态数字与task_fsm.h中的枚举保持一致。 */
void Test_TaskFSM_Log(void)
{
    Mission_Info_t info;
    char line[360];
    int length;

    if (TaskFSM_GetInfo(&info) != BSP_OK) {
        return;
    }

    length = snprintf(
        line,
        sizeof(line),
        "MISSION mode=%u state=%u sub=%u result=%u ready=%u/%u/%u "
        "run=%u fin=%u tgt=%d custom=%d pos=%d err=%d max=%d "
        "time=%lu score=%lu safe=%lu events=%02lX fault=%u ff=%u\r\n",
        (unsigned int)info.mode,
        (unsigned int)info.state,
        (unsigned int)info.substate,
        (unsigned int)info.result,
        (unsigned int)info.armed_ready,
        (unsigned int)info.route_ready,
        (unsigned int)info.ball_ready,
        (unsigned int)info.running,
        (unsigned int)info.finished,
        (int)info.active_ball_target_mm_x10,
        (int)info.custom_ball_target_mm_x10,
        (int)info.current_ball_position_mm_x10,
        (int)info.current_ball_error_mm_x10,
        (int)info.max_ball_error_mm_x10,
        (unsigned long)info.elapsed_ms,
        (unsigned long)info.score_limit_ms,
        (unsigned long)info.safety_timeout_ms,
        (unsigned long)info.route_events,
        (unsigned int)info.fault_code,
        (unsigned int)info.vehicle_feedforward_enabled
    );

    if ((length > 0) && (length < (int)sizeof(line))) {
        (void)BSP_UART_WriteFrame(
            UART_PORT1,
            (const uint8_t *)line,
            (uint16_t)length
        );
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
            Chassis_EmergencyStop();
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

/*
 * K210钢球位置通信测试。
 * USART2接收0x32钢球帧，USART1输出到USB-TTL串口助手。
 */
void Test_K210_BallCommUpdate(void)
{
    static uint32_t last_status_ms = 0U;

    K210_Comm_Info_t info;
    int16_t position_tenth_mm;
    uint8_t state;
    uint8_t confidence;
    uint32_t now_ms;
    char buf[192];
    int length;
    int32_t position_value;
    uint32_t position_abs;
    const char *state_text;

    if (K210_Comm_GetNewBallPosition(
            &position_tenth_mm,
            &state,
            &confidence
        ) == BSP_OK) {
        if (state == K210_BALL_STATE_VALID) {
            state_text = "VALID";
        } else if (state == K210_BALL_STATE_HOLD) {
            state_text = "HOLD";
        } else {
            state_text = "LOST";
        }

        position_value = position_tenth_mm;
        if (position_value < 0) {
            position_abs = (uint32_t)(-position_value);
        } else {
            position_abs = (uint32_t)position_value;
        }

        length = snprintf(
            buf,
            sizeof(buf),
            "BALL position=%c%lu.%02lucm "
            "state=%s confidence=%u\r\n",
            (position_value < 0) ? '-' : '+',
            (unsigned long)(position_abs / 100U),
            (unsigned long)(position_abs % 100U),
            state_text,
            (unsigned int)confidence
        );

        if ((length > 0) &&
            (length < (int)sizeof(buf))) {
            (void)BSP_UART_WriteFrame(
                UART_PORT_DEBUG,
                (const uint8_t *)buf,
                (uint16_t)length
            );
        }
    }

    now_ms = BSP_GetTickMs();
    if ((uint32_t)(now_ms - last_status_ms) < 500U) {
        return;
    }
    last_status_ms = now_ms;

    if (K210_Comm_GetInfo(&info) != BSP_OK) {
        return;
    }

    length = snprintf(
        buf,
        sizeof(buf),
        "K210 STATUS online=%u frames=%lu "
        "check_err=%lu format_err=%lu "
        "timeout=%lu last_rx=%lu\r\n",
        (unsigned int)info.online,
        (unsigned long)info.valid_frame_count,
        (unsigned long)info.checksum_error_count,
        (unsigned long)info.format_error_count,
        (unsigned long)info.timeout_count,
        (unsigned long)info.last_rx_ms
    );

    if ((length > 0) &&
        (length < (int)sizeof(buf))) {
        (void)BSP_UART_WriteFrame(
            UART_PORT_DEBUG,
            (const uint8_t *)buf,
            (uint16_t)length
        );
    }
}

static void Test_ServoCal_SendText(const char *text)
{
    uint16_t length;

    if (text == 0) {
        return;
    }
    length = 0U;
    while ((text[length] != '\0') &&
           (length < 255U)) {
        length++;
    }
    if (length > 0U) {
        (void)BSP_UART_WriteFrame(
            DEBUG_UART_PORT,
            (const uint8_t *)text,
            length
        );
    }
}

static void Test_ServoCal_Apply(uint16_t angle_deg,
                                const char *command)
{
    BSP_Status_t status;
    char line[128];
    int length;

    status = Drv_Servo_SetHorizontalAngleDeg(angle_deg);
    length = snprintf(
        line,
        sizeof(line),
        "[ServoCal] angle=%udeg pulse=%uus command=%s status=%s\r\n",
        (unsigned int)angle_deg,
        (unsigned int)Drv_Servo_GetHorizontalPulseUs(),
        command,
        (status == BSP_OK) ? "OK" : "ERROR"
    );
    if ((length > 0) &&
        (length < (int)sizeof(line))) {
        Test_ServoCal_SendText(line);
    }
}

void Test_ServoBeamCalibration_Update(void)
{
    static uint8_t initialized = 0U;
    static uint16_t angle_deg = 0U;

    if (initialized == 0U) {
        Chassis_EmergencyStop();
        Test_ServoCal_SendText(
            "[ServoCal] range=0..180deg pulse=500..2500us step=10deg\r\n"
            "[ServoCal] KEY1=0deg KEY2=180deg KEY3=+10deg KEY4=-10deg\r\n"
            "[ServoCal] KEY5=stop/0deg\r\n"
        );
        Test_ServoCal_Apply(angle_deg, "INIT_0DEG");
        initialized = 1U;
    }

#if BSP_KEY1_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY1) != 0U) {
        angle_deg = 0U;
        Test_ServoCal_Apply(angle_deg, "KEY1_0DEG");
        return;
    }
#endif
#if BSP_KEY2_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY2) != 0U) {
        angle_deg = 180U;
        Test_ServoCal_Apply(angle_deg, "KEY2_180DEG");
        return;
    }
#endif
#if BSP_KEY3_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY3) != 0U) {
        if (angle_deg <=
            (180U - TEST_SERVO_CAL_STEP_ANGLE_DEG)) {
            angle_deg =
                (uint16_t)(
                    angle_deg +
                    TEST_SERVO_CAL_STEP_ANGLE_DEG
                );
        } else {
            angle_deg = 180U;
        }
        Test_ServoCal_Apply(angle_deg, "KEY3_PLUS");
        return;
    }
#endif
#if BSP_KEY4_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY4) != 0U) {
        if (angle_deg >= TEST_SERVO_CAL_STEP_ANGLE_DEG) {
            angle_deg =
                (uint16_t)(
                    angle_deg -
                    TEST_SERVO_CAL_STEP_ANGLE_DEG
                );
        } else {
            angle_deg = 0U;
        }
        Test_ServoCal_Apply(angle_deg, "KEY4_MINUS");
        return;
    }
#endif
#if BSP_KEY5_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY5) != 0U) {
        Chassis_EmergencyStop();
        angle_deg = 0U;
        Test_ServoCal_Apply(angle_deg, "KEY5_STOP");
    }
#endif
}
static const char *Test_BallState_StateText(uint8_t state)
{
    switch (state) {
        case BALL_BALANCE_VISION_VALID:
            return "VALID";

        case BALL_BALANCE_VISION_HOLD:
            return "HOLD";

        default:
            return "LOST";
    }
}

static int32_t Test_BallState_FloatToX10(float value)
{
    float scaled;

    scaled = value * 10.0f;

    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }

    return (int32_t)scaled;
}

static int32_t Test_BallState_FloatToX100(float value)
{
    float scaled;

    scaled = value * 100.0f;

    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }

    return (int32_t)scaled;
}

static float Test_BallState_AbsF(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static void Test_BallState_SendLine(const char *line)
{
    uint16_t length;

    if (line == 0) {
        return;
    }

    length = 0U;

    while ((line[length] != '\0') &&
           (length < 500U)) {
        length++;
    }

    if (length > 0U) {
        (void)BSP_UART_WriteFrame(
            UART_PORT_DEBUG,
            (const uint8_t *)line,
            length
        );
    }
}

static void Test_BallState_PrintStatus(const char *reason)
{
    K210_Comm_Info_t k210_info;
    BallBalance_AppInfo_t app_info;
    BallBalance_K210AdapterInfo_t adapter_info;
    Drv_Servo_Info_t servo_info;
    uint8_t servo_info_ok;
    char line[256];
    int n;

    if (K210_Comm_GetInfo(&k210_info) != BSP_OK) {
        return;
    }

    if (BallBalance_App_GetInfo(&app_info) != BSP_OK) {
        return;
    }

    if (BallBalance_K210Adapter_GetInfo(&adapter_info) != BSP_OK) {
        return;
    }

    servo_info_ok =
        (Drv_Servo_GetInfo(&servo_info) == BSP_OK) ?
        1U :
        0U;

    n = snprintf(
        line,
        sizeof(line),
        "[BallState] %s k210_online=%u frames=%lu "
        "checksum_err=%lu format_err=%lu\r\n",
        (reason == 0) ? "STATUS" : reason,
        (unsigned int)k210_info.online,
        (unsigned long)k210_info.valid_frame_count,
        (unsigned long)k210_info.checksum_error_count,
        (unsigned long)k210_info.format_error_count
    );

    if ((n > 0) && (n < (int)sizeof(line))) {
        Test_BallState_SendLine(line);
    }

    n = snprintf(
        line,
        sizeof(line),
        "[BallState] app=%u vision=%s conf=%u "
        "seq=%u streak=%u timeout=%u pushed=%lu\r\n",
        (unsigned int)app_info.state,
        Test_BallState_StateText(
            app_info.last_sample_state
        ),
        (unsigned int)app_info.last_sample.confidence,
        (unsigned int)adapter_info.sequence,
        (unsigned int)app_info.valid_streak,
        (unsigned int)app_info.data_timeout,
        (unsigned long)adapter_info.pushed_count
    );

    if ((n > 0) && (n < (int)sizeof(line))) {
        Test_BallState_SendLine(line);
    }

    n = snprintf(
        line,
        sizeof(line),
        "[BallState] target=%d.%dmm raw=%d.%dmm "
        "est=%ld.%ldmm vel=%ld.%ldmm/s "
        "dist=%ld.%ldmm/s2\r\n",
        (int)(app_info.target_mm_x10 / 10),
        (int)(
            (
                app_info.target_mm_x10 < 0 ?
                -app_info.target_mm_x10 :
                app_info.target_mm_x10
            ) % 10
        ),
        (int)(app_info.last_sample.position_mm_x10 / 10),
        (int)(
            (
                app_info.last_sample.position_mm_x10 < 0 ?
                -app_info.last_sample.position_mm_x10 :
                app_info.last_sample.position_mm_x10
            ) % 10
        ),
        (long)(
            Test_BallState_FloatToX10(
                app_info.estimator.position_mm
            ) / 10
        ),
        (long)(
            Test_BallState_FloatToX10(
                Test_BallState_AbsF(
                    app_info.estimator.position_mm
                )
            ) % 10
        ),
        (long)(
            Test_BallState_FloatToX10(
                app_info.estimator.velocity_mm_s
            ) / 10
        ),
        (long)(
            Test_BallState_FloatToX10(
                Test_BallState_AbsF(
                    app_info.estimator.velocity_mm_s
                )
            ) % 10
        ),
        (long)(
            Test_BallState_FloatToX10(
                app_info.estimator.disturbance_mm_s2
            ) / 10
        ),
        (long)(
            Test_BallState_FloatToX10(
                Test_BallState_AbsF(
                    app_info.estimator.disturbance_mm_s2
                )
            ) % 10
        )
    );

    if ((n > 0) && (n < (int)sizeof(line))) {
        Test_BallState_SendLine(line);
    }

    n = snprintf(
        line,
        sizeof(line),
        "[BallState] ref=%ld.%ldmm "
        "dyn=%ld.%02lddeg "
        "breakaway=%ld.%02lddeg "
        "servo=%u.%udeg speed=%ld.%lddeg/s "
        "now=%u.%udeg "
        "locked=%u reject=%u fault=%u\r\n",
        (long)(
            Test_BallState_FloatToX10(
                app_info.reference.reference_position_mm
            ) / 10
        ),
        (long)(
            Test_BallState_FloatToX10(
                Test_BallState_AbsF(
                    app_info.reference.reference_position_mm
                )
            ) % 10
        ),
        (long)(
            Test_BallState_FloatToX100(
                app_info.control.output
                    .limited_dynamic_angle_deg
            ) / 100
        ),
        (long)(
            Test_BallState_FloatToX100(
                Test_BallState_AbsF(
                    app_info.control.output
                        .limited_dynamic_angle_deg
                )
            ) % 100
        ),
        (long)(
            Test_BallState_FloatToX100(
                app_info.control.output
                    .breakaway_angle_deg
            ) / 100
        ),
        (long)(
            Test_BallState_FloatToX100(
                Test_BallState_AbsF(
                    app_info.control.output
                        .breakaway_angle_deg
                )
            ) % 100
        ),
        (unsigned int)(
            app_info.control.output.command_angle_x10 /
            10U
        ),
        (unsigned int)(
            app_info.control.output.command_angle_x10 %
            10U
        ),
        (long)(
            Test_BallState_FloatToX10(
                app_info.control.output.servo_speed_deg_s
            ) / 10
        ),
        (long)(
            Test_BallState_FloatToX10(
                Test_BallState_AbsF(
                    app_info.control.output.servo_speed_deg_s
                )
            ) % 10
        ),
        (servo_info_ok != 0U) ?
            (unsigned int)(
                servo_info.horizontal_angle_x10 / 10U
            ) :
            0U,
        (servo_info_ok != 0U) ?
            (unsigned int)(
                servo_info.horizontal_angle_x10 % 10U
            ) :
            0U,
        (unsigned int)app_info.settled,
        (unsigned int)
            app_info.estimator.innovation_rejected,
        (unsigned int)app_info.servo_fault
    );

    if ((n > 0) && (n < (int)sizeof(line))) {
        Test_BallState_SendLine(line);
    }
}

static void Test_BallState_PrintCsv(
    uint32_t now_ms,
    const BallBalance_AppInfo_t *app
)
{
    char line[320];
    int n;

    if (app == 0) {
        return;
    }

    n = snprintf(
        line,
        sizeof(line),
        "BB,%lu,%u,%u,%u,%d,"
        "%ld,%ld,%ld,%ld,%ld,"
        "%ld,%ld,%ld,%ld,"
        "%ld,%ld,%ld,%ld,%ld,%ld,"
        "%u,%u,%u\r\n",
        (unsigned long)now_ms,
        (unsigned int)app->state,
        (unsigned int)app->last_sample_state,
        (unsigned int)app->last_sample.confidence,
        (int)app->last_sample.position_mm_x10,
        (long)Test_BallState_FloatToX10(
            app->estimator.position_mm
        ),
        (long)Test_BallState_FloatToX10(
            app->estimator.velocity_mm_s
        ),
        (long)Test_BallState_FloatToX10(
            app->control.output.filtered_velocity_mm_s
        ),
        (long)Test_BallState_FloatToX10(
            app->estimator.disturbance_mm_s2
        ),
        (long)Test_BallState_FloatToX10(
            app->control.output.filtered_disturbance_mm_s2
        ),
        (long)Test_BallState_FloatToX10(
            app->reference.reference_position_mm
        ),
        (long)Test_BallState_FloatToX10(
            app->reference.reference_velocity_mm_s
        ),
        (long)Test_BallState_FloatToX10(
            app->reference.reference_acceleration_mm_s2
        ),
        (long)Test_BallState_FloatToX10(
            app->control.output
                .reference_accel_feedforward_mm_s2
        ),
        (long)Test_BallState_FloatToX100(
            app->equilibrium_angle_deg
        ),
        (long)Test_BallState_FloatToX100(
            app->control.output.limited_dynamic_angle_deg
        ),
        (long)Test_BallState_FloatToX100(
            app->control.output.breakaway_angle_deg
        ),
        (long)Test_BallState_FloatToX10(
            app->vehicle_disturbance_mm_s2
        ),
        (long)Test_BallState_FloatToX100(
            app->control.output.servo_angle_deg
        ),
        (long)Test_BallState_FloatToX10(
            app->control.output.servo_speed_deg_s
        ),
        (unsigned int)app->data_timeout,
        (unsigned int)
            app->estimator.innovation_rejected,
        (unsigned int)app->settled
    );

    if ((n > 0) && (n < (int)sizeof(line))) {
        Test_BallState_SendLine(line);
    }
}

void Test_BallModelIdentify_Update(void)
{
    static uint8_t initialized = 0U;
    static uint16_t servo_angle_x10 =
        BALL_BALANCE_LEVEL_ANGLE_X10;
    static uint32_t last_print_ms = 0U;

    BallBalance_K210AdapterInfo_t adapter;
    char line[192];
    int position_abs;
    int n;
    uint32_t now_ms;

    if (initialized == 0U) {
        BallBalance_App_Disable();
        BallBalance_App_SetVehicleFeedforwardEnabled(0U);

        (void)Drv_Servo_SetHorizontalAngleX10(
            servo_angle_x10
        );

        Test_BallState_SendLine(
            "[BallModel] READY "
            "KEY1=LEVEL KEY2=+0.5 KEY3=-0.5 "
            "KEY4=+1.0 KEY5=-1.0\r\n"
        );

        initialized = 1U;
    }

#if BSP_KEY1_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY1) != 0U) {
        servo_angle_x10 =
            BALL_BALANCE_LEVEL_ANGLE_X10;
    }
#endif

#if BSP_KEY2_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY2) != 0U) {
        servo_angle_x10 =
            (uint16_t)(
                BALL_BALANCE_LEVEL_ANGLE_X10 + 5U
            );
    }
#endif

#if BSP_KEY3_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY3) != 0U) {
        servo_angle_x10 =
            (uint16_t)(
                BALL_BALANCE_LEVEL_ANGLE_X10 - 5U
            );
    }
#endif

#if BSP_KEY4_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY4) != 0U) {
        servo_angle_x10 =
            (uint16_t)(
                BALL_BALANCE_LEVEL_ANGLE_X10 + 10U
            );
    }
#endif

#if BSP_KEY5_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY5) != 0U) {
        servo_angle_x10 =
            (uint16_t)(
                BALL_BALANCE_LEVEL_ANGLE_X10 - 10U
            );
    }
#endif

    (void)Drv_Servo_SetHorizontalAngleX10(
        servo_angle_x10
    );

    now_ms = BSP_GetTickMs();

    if ((uint32_t)(
            now_ms - last_print_ms
        ) < 50U) {
        return;
    }

    last_print_ms = now_ms;

    if (BallBalance_K210Adapter_GetInfo(
            &adapter
        ) != BSP_OK) {
        return;
    }

    position_abs = adapter.last_position_mm_x10;

    if (position_abs < 0) {
        position_abs = -position_abs;
    }

    n = snprintf(
        line,
        sizeof(line),
        "BMODEL,%lu,%s,%u,%c%d.%d,%u.%u\r\n",
        (unsigned long)now_ms,
        Test_BallState_StateText(
            adapter.last_app_state
        ),
        (unsigned int)adapter.last_confidence,
        (adapter.last_position_mm_x10 < 0) ?
            '-' :
            '+',
        position_abs / 10,
        position_abs % 10,
        (unsigned int)(servo_angle_x10 / 10U),
        (unsigned int)(servo_angle_x10 % 10U)
    );

    if ((n > 0) && (n < (int)sizeof(line))) {
        Test_BallState_SendLine(line);
    }
}

void Test_BallBalanceControl_Update(void)
{
    static uint8_t banner_sent = 0U;
    static uint8_t last_servo_fault = 0U;
    static uint8_t last_timeout = 0U;
    static uint32_t last_debug_ms = 0U;
    static const char *pending_status_reason = 0;

    BallBalance_AppInfo_t app_info;
    char banner_line[160];
    int banner_len;

    if (banner_sent == 0U) {
        banner_len = snprintf(
            banner_line,
            sizeof(banner_line),
            "[BallState] READY level=%u.%udeg "
            "KEY1=O KEY2=+50.0mm KEY3=-50.0mm "
            "KEY4=STOP KEY5=STATUS\r\n",
            (unsigned int)(
                BALL_BALANCE_LEVEL_ANGLE_X10 / 10U
            ),
            (unsigned int)(
                BALL_BALANCE_LEVEL_ANGLE_X10 % 10U
            )
        );

        if ((banner_len > 0) &&
            (banner_len < (int)sizeof(banner_line))) {
            Test_BallState_SendLine(banner_line);
        }

        Test_BallState_SendLine(
            "[BallState] CSV BB,time_ms,app,vision,conf,raw_x10,"
            "est_x10,vel_x10,fvel_x10,dist_x10,fdist_x10,"
            "ref_x10,refv_x10,refa_x10,refaff_x10,"
            "eq_x100,dyn_x100,breakaway_x100,vehicle_x10,"
            "servo_x100,servov_x10,timeout,reject,locked\r\n"
        );
        Test_BallState_PrintStatus("INIT");
        banner_sent = 1U;
    }

    /*
     * 本测试任务运行在BallBalance_App_Update之前。
     * 按键修改目标后延后一拍打印，使日志反映APP
     * 已经完成一次计算后的状态。
     */
    if (pending_status_reason != 0) {
        Test_BallState_PrintStatus(
            pending_status_reason
        );

        pending_status_reason = 0;
        return;
    }

#if BSP_KEY1_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY1) != 0U) {
        BallBalance_App_SetTargetMmX10(0);
        BallBalance_App_Enable();

        pending_status_reason = "KEY1_CENTER";
        return;
    }
#endif

#if BSP_KEY2_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY2) != 0U) {
        BallBalance_App_SetTargetMmX10(500);
        BallBalance_App_Enable();

        pending_status_reason = "KEY2_PLUS_50";
        return;
    }
#endif

#if BSP_KEY3_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY3) != 0U) {
        BallBalance_App_SetTargetMmX10(-500);
        BallBalance_App_Enable();

        pending_status_reason = "KEY3_MINUS_50";
        return;
    }
#endif

#if BSP_KEY4_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY4) != 0U) {
        BallBalance_App_Disable();

        Test_BallState_PrintStatus(
            "KEY4_STOP"
        );
        return;
    }
#endif

#if BSP_KEY5_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY5) != 0U) {
        Test_BallState_PrintStatus(
            "KEY5_STATUS"
        );
        return;
    }
#endif

    if (BallBalance_App_GetInfo(
            &app_info
        ) != BSP_OK) {
        return;
    }

    if (app_info.servo_fault != last_servo_fault) {
        last_servo_fault = app_info.servo_fault;

        Test_BallState_PrintStatus(
            "SERVO_FAULT_CHANGE"
        );
    } else if (app_info.data_timeout != last_timeout) {
        last_timeout = app_info.data_timeout;

        Test_BallState_PrintStatus(
            "TIMEOUT_CHANGE"
        );
    }

#if (BALL_BALANCE_DEBUG_STREAM_ENABLE != 0U)
    if ((uint32_t)(
            BSP_GetTickMs() - last_debug_ms
        ) >= BALL_BALANCE_DEBUG_STREAM_PERIOD_MS) {
        last_debug_ms = BSP_GetTickMs();

        Test_BallState_PrintCsv(
            last_debug_ms,
            &app_info
        );
    }
#endif
}

static void Test_K210_GrayTuneSendText(
    const char *text
)
{
    uint16_t length;

    if (text == 0) {
        return;
    }

    length = 0U;

    while ((text[length] != '\0') &&
           (length < 255U)) {
        length++;
    }

    if (length > 0U) {
        (void)BSP_UART_WriteFrame(
            UART_PORT_DEBUG,
            (const uint8_t *)text,
            length
        );
    }
}

/*
 * K210灰度钢球四按键现场调参：
 *
 * KEY6：正常画面/二值调参画面切换；
 * KEY7：循环选择BALL_MIN、BALL_MAX、
 *       PIPE_MIN、PIPE_MAX；
 * KEY8：当前参数增加5；
 * KEY9：当前参数减少5。
 */
void Test_K210_GrayTuneUpdate(void)
{
    static uint8_t initialized = 0U;
    static uint8_t tune_enabled = 0U;
    static uint8_t selected_param =
        K210_GRAY_PARAM_BALL_MIN;

    static const char *const param_name[
        K210_GRAY_PARAM_COUNT
    ] = {
        "BALL_MIN",
        "BALL_MAX",
        "PIPE_MIN",
        "PIPE_MAX"
    };

    BSP_Status_t status;
    char message[64];
    int length;

    if (initialized == 0U) {
        Test_K210_GrayTuneSendText(
            "\r\n"
            "===== K210 GRAY KEY TUNE =====\r\n"
            "KEY6 : NORMAL / BINARY\r\n"
            "KEY7 : NEXT PARAM\r\n"
            "KEY8 : +5\r\n"
            "KEY9 : -5\r\n"
            "================================\r\n"
        );

        initialized = 1U;
    }

#if BSP_KEY6_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY6) != 0U) {
        tune_enabled =
            (tune_enabled == 0U) ?
            1U :
            0U;

        status = K210_Comm_SetLabTuneMode(
            (tune_enabled != 0U) ?
                K210_GRAY_TUNE_ON :
                K210_GRAY_TUNE_OFF
        );

        if ((status == BSP_OK) &&
            (tune_enabled != 0U)) {
            (void)K210_Comm_SelectLabParam(
                selected_param
            );
        }

        Test_K210_GrayTuneSendText(
            (tune_enabled != 0U) ?
                "GRAY TUNE -> BINARY\r\n" :
                "GRAY TUNE -> NORMAL\r\n"
        );

        return;
    }
#endif

    if (tune_enabled == 0U) {
        return;
    }

#if BSP_KEY7_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY7) != 0U) {
        selected_param++;

        if (selected_param >=
            K210_GRAY_PARAM_COUNT) {
            selected_param =
                K210_GRAY_PARAM_BALL_MIN;
        }

        status = K210_Comm_SelectLabParam(
            selected_param
        );

        if (status == BSP_OK) {
            length = snprintf(
                message,
                sizeof(message),
                "GRAY SELECT -> %s\r\n",
                param_name[selected_param]
            );

            if ((length > 0) &&
                (length < (int)sizeof(message))) {
                Test_K210_GrayTuneSendText(
                    message
                );
            }
        }

        return;
    }
#endif

#if BSP_KEY8_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY8) != 0U) {
        status = K210_Comm_AdjustLabParam(
            selected_param,
            +5
        );

        if (status == BSP_OK) {
            length = snprintf(
                message,
                sizeof(message),
                "GRAY %s +5\r\n",
                param_name[selected_param]
            );

            if ((length > 0) &&
                (length < (int)sizeof(message))) {
                Test_K210_GrayTuneSendText(
                    message
                );
            }
        }

        return;
    }
#endif

#if BSP_KEY9_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY9) != 0U) {
        status = K210_Comm_AdjustLabParam(
            selected_param,
            -5
        );

        if (status == BSP_OK) {
            length = snprintf(
                message,
                sizeof(message),
                "GRAY %s -5\r\n",
                param_name[selected_param]
            );

            if ((length > 0) &&
                (length < (int)sizeof(message))) {
                Test_K210_GrayTuneSendText(
                    message
                );
            }
        }

        return;
    }
#endif
}
