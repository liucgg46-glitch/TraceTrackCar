#include "test_k210_comm.h"

#include "k210_comm.h"
#include "bsp_uart.h"
#include "bsp_systick.h"
#include "bsp_key.h"
#include <stdio.h>

/*
 * ============================================================================
 * K210统一通信测试
 * ============================================================================
 *
 * 文件编码：UTF-8
 *
 * 端口由BSP语义宏统一选择：
 *   UART_PORT_K210连接K210；
 *   DEBUG_UART_PORT输出调试日志。
 *
 * 本文件统一包含单数字、多数字、道路、视觉和配置档位通信测试。
 * 不依赖printf串口重定向，可在STM32F407和MSPM0G3519工程间直接复用。
 * ============================================================================
 */

#define K210_DIGIT_LOG_INTERVAL_MS   500U
#define K210_ROAD_LOG_INTERVAL_MS    200U
#define K210_TEST_TEXT_MAX_LENGTH    500U

volatile K210_Comm_Info_t g_k210_digit_debug_info;
volatile K210_Comm_Info_t g_k210_road_debug_info;
volatile uint8_t g_k210_profile_test_selected;
volatile uint8_t g_k210_profile_test_remaining;
volatile uint32_t g_k210_profile_test_tx_count;
volatile uint32_t g_k210_profile_test_busy_count;

static void Test_K210_SendText(const char *text)
{
    uint16_t length;

    if (text == 0) {
        return;
    }

    length = 0U;

    while ((text[length] != '\0') &&
           (length < K210_TEST_TEXT_MAX_LENGTH)) {
        length++;
    }

    if (length == 0U) {
        return;
    }

    (void)BSP_UART_WriteFrame(
        DEBUG_UART_PORT,
        (const uint8_t *)text,
        length
    );
}

static const char *Test_K210_LineSourceText(uint8_t source)
{
    switch (source) {
        case K210_LINE_SOURCE_NEAR:
            return "NEAR";

        case K210_LINE_SOURCE_MID:
            return "MID";

        default:
            return "NONE";
    }
}

static const char *Test_K210_RoadEventText(uint8_t event)
{
    switch (event) {
        case K210_ROAD_EVENT_CROSS_ENTER:
            return "CROSS_ENTER";

        case K210_ROAD_EVENT_CROSS_LEAVE:
            return "CROSS_LEAVE";

        case K210_ROAD_EVENT_STOP_ENTER:
            return "STOP_ENTER";

        case K210_ROAD_EVENT_STOP_LEAVE:
            return "STOP_LEAVE";

        default:
            return "NONE";
    }
}

void Test_K210_DigitCommUpdate(void)
{
    static uint32_t last_status_ms = 0U;

    K210_DigitSnapshot_t snapshot;
    K210_Comm_Info_t info;

    char buf[256];
    int used;
    int n;
    uint8_t i;

    if (K210_Comm_GetNewSnapshot(&snapshot) == BSP_OK) {
        used = snprintf(
            buf,
            sizeof(buf),
            "K210 SNAP seq=%u status=%u count=%u",
            (unsigned int)snapshot.sequence,
            (unsigned int)snapshot.status,
            (unsigned int)snapshot.count
        );

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

            if ((n <= 0) ||
                (n >= ((int)sizeof(buf) - used))) {
                used = (int)sizeof(buf);
                break;
            }

            used += n;
        }

        if ((used > 0) &&
            (used <= ((int)sizeof(buf) - 3))) {
            buf[used++] = '\r';
            buf[used++] = '\n';
            buf[used] = '\0';

            Test_K210_SendText(buf);
        }
    }

    if ((uint32_t)(
            BSP_GetTickMs() -
            last_status_ms
        ) < K210_DIGIT_LOG_INTERVAL_MS) {
        return;
    }

    last_status_ms = BSP_GetTickMs();

    if (K210_Comm_GetInfo(&info) != BSP_OK) {
        Test_K210_SendText(
            "K210 DIGIT GET INFO ERROR\r\n"
        );
        return;
    }

    g_k210_digit_debug_info = info;

    used = snprintf(
        buf,
        sizeof(buf),
        "K210 DIGIT online=%u frames=%lu snapshots=%lu "
        "check_err=%lu format_err=%lu "
        "snap_err=%lu overwrite=%lu last_rx=%lu\r\n",
        (unsigned int)info.online,
        (unsigned long)info.valid_frame_count,
        (unsigned long)info.snapshot_count,
        (unsigned long)info.checksum_error_count,
        (unsigned long)info.format_error_count,
        (unsigned long)info.snapshot_error_count,
        (unsigned long)info.snapshot_overwrite_count,
        (unsigned long)info.last_rx_ms
    );

    if ((used > 0) &&
        (used < (int)sizeof(buf))) {
        Test_K210_SendText(buf);
    }
}

void Test_K210_RoadCommUpdate(void)
{
    static uint32_t last_log_ms = 0U;

    K210_Comm_Info_t info;
    K210_RoadState_t road;

    uint8_t cross_count;
    uint8_t stop_count;
    uint8_t road_event;

    char buf[256];
    int length;

    if (K210_Comm_GetNewRoadEvent(
            &cross_count,
            &stop_count,
            &road_event
        ) == BSP_OK) {

        length = snprintf(
            buf,
            sizeof(buf),
            "K210 EVENT=%s cross_total=%u stop_total=%u\r\n",
            Test_K210_RoadEventText(road_event),
            (unsigned int)cross_count,
            (unsigned int)stop_count
        );

        if ((length > 0) &&
            (length < (int)sizeof(buf))) {
            Test_K210_SendText(buf);
        }
    }

    if ((uint32_t)(
            BSP_GetTickMs() -
            last_log_ms
        ) < K210_ROAD_LOG_INTERVAL_MS) {
        return;
    }

    last_log_ms = BSP_GetTickMs();

    if (K210_Comm_GetInfo(&info) != BSP_OK) {
        Test_K210_SendText(
            "K210 ROAD GET INFO ERROR\r\n"
        );
        return;
    }

    g_k210_road_debug_info = info;

    if (K210_Comm_ReadRoadState(&road) == BSP_OK) {
        length = snprintf(
            buf,
            sizeof(buf),
            "K210 ROAD online=%u frames=%lu "
            "check_err=%lu format_err=%lu timeout=%lu "
            "valid=%u source=%s x=%u error=%d "
            "cross=%u stop=%u "
            "cross_total=%u stop_total=%u\r\n",
            (unsigned int)info.online,
            (unsigned long)info.valid_frame_count,
            (unsigned long)info.checksum_error_count,
            (unsigned long)info.format_error_count,
            (unsigned long)info.timeout_count,
            (unsigned int)road.line_valid,
            Test_K210_LineSourceText(road.line_source),
            (unsigned int)road.line_x,
            (int)road.line_error,
            (unsigned int)road.cross_valid,
            (unsigned int)road.stop_valid,
            (unsigned int)road.cross_count,
            (unsigned int)road.stop_count
        );
    } else {
        length = snprintf(
            buf,
            sizeof(buf),
            "K210 ROAD online=%u frames=%lu "
            "check_err=%lu format_err=%lu timeout=%lu "
            "road=no_data last_rx=%lu\r\n",
            (unsigned int)info.online,
            (unsigned long)info.valid_frame_count,
            (unsigned long)info.checksum_error_count,
            (unsigned long)info.format_error_count,
            (unsigned long)info.timeout_count,
            (unsigned long)info.last_rx_ms
        );
    }

    if ((length > 0) &&
        (length < (int)sizeof(buf))) {
        Test_K210_SendText(buf);
    }
}

void Test_K210_VisionCommUpdate(void)
{
    /*
     * 后续激光、目标中心或通用视觉结果测试入口。
     * 当前暂不执行操作。
     */
}
void Test_K210_RoadProfileUpdate(void)
{
    static uint32_t last_log_ms = 0U;

    K210_Comm_Info_t info;
    char buf[192];
    int length;

    /*
     * KEY1：CURRENT，P=0
     * KEY2：OLD，P=1
     *
     * KEY3、KEY4目前预留，暂不发送P=2/P=3，
     * 因为K210端还没有配置BRIGHT和DARK参数。
     */

    if (BSP_Key_WasPressed(BSP_KEY1) != 0U) {
        (void)K210_Comm_SelectRoadProfile(
            K210_ROAD_PROFILE_CURRENT
        );

        Test_K210_SendText(
            "KEY1 -> ROAD PROFILE CURRENT\r\n"
        );
    }

    if (BSP_Key_WasPressed(BSP_KEY2) != 0U) {
        (void)K210_Comm_SelectRoadProfile(
            K210_ROAD_PROFILE_OLD
        );

        Test_K210_SendText(
            "KEY2 -> ROAD PROFILE OLD\r\n"
        );
    }

    if (BSP_Key_WasPressed(BSP_KEY3) != 0U) {
        Test_K210_SendText(
            "KEY3 -> BRIGHT NOT ENABLED\r\n"
        );
    }

    if (BSP_Key_WasPressed(BSP_KEY4) != 0U) {
        Test_K210_SendText(
            "KEY4 -> DARK NOT ENABLED\r\n"
        );
    }

    if ((uint32_t)(
            BSP_GetTickMs() -
            last_log_ms
        ) < 200U) {
        return;
    }

    last_log_ms = BSP_GetTickMs();

    if (K210_Comm_GetInfo(&info) != BSP_OK) {
        return;
    }

    g_k210_profile_test_selected =
        info.selected_road_profile;

    g_k210_profile_test_remaining =
        info.road_profile_tx_remaining;

    g_k210_profile_test_tx_count =
        info.road_profile_tx_count;

    g_k210_profile_test_busy_count =
        info.road_profile_tx_busy_count;

    length = snprintf(
        buf,
        sizeof(buf),
        "K210 PROFILE selected=%u remaining=%u "
        "tx=%lu busy=%lu last_ok=%u\r\n",
        (unsigned int)info.selected_road_profile,
        (unsigned int)info.road_profile_tx_remaining,
        (unsigned long)info.road_profile_tx_count,
        (unsigned long)info.road_profile_tx_busy_count,
        (unsigned int)info.road_profile_last_tx_ok
    );

    if ((length > 0) &&
        (length < (int)sizeof(buf))) {
        Test_K210_SendText(buf);
    }
}



void Test_K210_SingleDigitCommUpdate(void)
{
    static uint32_t last_status_ms = 0U;

    K210_Comm_Info_t info;

    uint8_t digit;
    uint8_t valid;
    uint8_t confidence;

    char buf[192];
    int length;


    /*
     * ========================================================
     * 读取K210刚刚发送的新数字帧
     *
     * 协议：
     * AA 55 01 DIGIT VALID CONF CHECK
     * ========================================================
     */

    if (K210_Comm_GetNewDigit(
            &digit,
            &valid,
            &confidence
        ) == BSP_OK) {

        length = snprintf(
            buf,
            sizeof(buf),
            "K210 DIGIT RX digit=%u valid=%u confidence=%u\r\n",
            (unsigned int)digit,
            (unsigned int)valid,
            (unsigned int)confidence
        );

        if ((length > 0) &&
            (length < (int)sizeof(buf))) {

            Test_K210_SendText(
                buf
            );
        }
    }


    /*
     * ========================================================
     * 每500ms打印一次通信总状态
     * ========================================================
     */

    if ((uint32_t)(
            BSP_GetTickMs() -
            last_status_ms
        ) < K210_DIGIT_LOG_INTERVAL_MS) {

        return;
    }


    last_status_ms =
        BSP_GetTickMs();


    if (K210_Comm_GetInfo(
            &info
        ) != BSP_OK) {

        Test_K210_SendText(
            "K210 GET INFO ERROR\r\n"
        );

        return;
    }


    /*
     * 保存到全局变量。
     *
     * Debug模式下可以直接在Watch窗口观察：
     *
     * g_k210_digit_debug_info.digit
     * g_k210_digit_debug_info.digit_valid
     * g_k210_digit_debug_info.digit_confidence
     * g_k210_digit_debug_info.valid_frame_count
     * g_k210_digit_debug_info.checksum_error_count
     * g_k210_digit_debug_info.format_error_count
     */

    g_k210_digit_debug_info =
        info;


    length = snprintf(
        buf,
        sizeof(buf),

        "K210 STATUS "
        "online=%u "
        "frames=%lu "
        "digit=%u "
        "valid=%u "
        "conf=%u "
        "check_err=%lu "
        "format_err=%lu "
        "timeout=%lu "
        "last_rx=%lu\r\n",

        (unsigned int)info.online,

        (unsigned long)
            info.valid_frame_count,

        (unsigned int)
            info.digit,

        (unsigned int)
            info.digit_valid,

        (unsigned int)
            info.digit_confidence,

        (unsigned long)
            info.checksum_error_count,

        (unsigned long)
            info.format_error_count,

        (unsigned long)
            info.timeout_count,

        (unsigned long)
            info.last_rx_ms
    );


    if ((length > 0) &&
        (length < (int)sizeof(buf))) {

        Test_K210_SendText(
            buf
        );
    }
}