#include "test_k210_road_comm.h"

#include "k210_comm.h"
#include "bsp_uart.h"
#include "bsp_systick.h"

#include <stdio.h>

/*
 * ============================================================================
 * K210道路视觉通信调试
 * ============================================================================
 *
 * 文件编码：
 *   UTF-8
 *
 * K210通信：
 *   K210 IO6 TX -> STM32 PA3 USART2_RX
 *
 * TTL调试输出：
 *   STM32 PA9 USART1_TX -> USB-TTL RX
 *
 * 本文件不依赖printf串口重定向。
 * 所有日志都直接通过BSP_UART_WriteFrame(UART_PORT1, ...)输出。
 * ============================================================================
 */

#define K210_ROAD_LOG_INTERVAL_MS    200U

volatile K210_Comm_Info_t g_k210_road_debug_info;


/*
 * 将循迹来源转换成文本。
 */
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


/*
 * 将道路事件转换成文本。
 */
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


/*
 * 直接通过USART1发送调试字符串。
 */
static void Test_K210_SendText(const char *text)
{
    uint16_t length;

    if (text == 0) {
        return;
    }

    length = 0U;

    while (
        (text[length] != '\0') &&
        (length < 500U)
    ) {
        length++;
    }

    if (length == 0U) {
        return;
    }

    (void)BSP_UART_WriteFrame(
        UART_PORT1,
        (const uint8_t *)text,
        length
    );
}


/*
 * K210道路通信测试任务。
 *
 * 建议任务周期：
 *
 *   { Test_K210_RoadCommUpdate, 10U, 0U },
 *
 * 函数内部每200ms打印一次状态。
 * 道路事件收到后立即打印。
 */
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

    /*
     * --------------------------------------------------------
     * 读取并打印新的道路事件
     * --------------------------------------------------------
     */

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

        if (
            (length > 0) &&
            (length < (int)sizeof(buf))
        ) {
            Test_K210_SendText(buf);
        }
    }

    /*
     * --------------------------------------------------------
     * 每200ms输出一次当前通信状态
     * --------------------------------------------------------
     */

    if ((uint32_t)(
            BSP_GetTickMs() -
            last_log_ms
        ) < K210_ROAD_LOG_INTERVAL_MS) {
        return;
    }

    last_log_ms = BSP_GetTickMs();

    if (K210_Comm_GetInfo(&info) != BSP_OK) {
        Test_K210_SendText(
            "K210 GET INFO ERROR\r\n"
        );
        return;
    }

    /*
     * 复制到公开变量，方便Keil Watch窗口观察。
     */
    g_k210_road_debug_info = info;

    /*
     * 在线并且已经收到道路状态。
     */
    if (K210_Comm_ReadRoadState(&road) == BSP_OK) {

        length = snprintf(
            buf,
            sizeof(buf),
            "K210 online=%u frames=%lu "
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
            Test_K210_LineSourceText(
                road.line_source
            ),
            (unsigned int)road.line_x,
            (int)road.line_error,

            (unsigned int)road.cross_valid,
            (unsigned int)road.stop_valid,

            (unsigned int)road.cross_count,
            (unsigned int)road.stop_count
        );

    } else {

        /*
         * 尚未收到有效道路状态。
         */
        length = snprintf(
            buf,
            sizeof(buf),
            "K210 online=%u frames=%lu "
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

    if (
        (length > 0) &&
        (length < (int)sizeof(buf))
    ) {
        Test_K210_SendText(buf);
    }
}