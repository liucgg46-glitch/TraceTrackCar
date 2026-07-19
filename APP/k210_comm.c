#include "k210_comm.h"

#include "bsp_uart.h"
#include "bsp_systick.h"

#include <string.h>

/*
 * K210固定使用USART2。
 *
 * USART1留给USB转TTL调试：
 *   PA9  -> USB转TTL RX
 *   PA10 <- USB转TTL TX
 *
 * USART2连接K210：
 *   PA2  -> K210 RX
 *   PA3  <- K210 TX
 */
#define K210_UART_PORT             UART_PORT2

/*
 * 超过该时间没有收到有效数据帧，
 * 则认为K210已经离线。
 */
#define K210_OFFLINE_TIMEOUT_MS    1000U

/*
 * 接收状态机状态。
 */
typedef enum {
    K210_RX_WAIT_HEAD1 = 0,
    K210_RX_WAIT_HEAD2,
    K210_RX_RECEIVING
} K210_RxState_t;

/*
 * K210通信状态缓存。
 */
static K210_Comm_Info_t s_k210_info;

/*
 * 接收状态机变量。
 */
static K210_RxState_t s_rx_state;
static uint8_t s_rx_frame[K210_FRAME_SIZE];
static uint8_t s_rx_index;

/*
 * 计算帧校验和。
 *
 * CHECKSUM等于指定数据的累加和低8位。
 */
static uint8_t K210_Comm_CalcChecksum(const uint8_t *data,
                                      uint8_t length)
{
    uint16_t sum;
    uint8_t i;

    if (data == 0) {
        return 0U;
    }

    sum = 0U;

    for (i = 0U; i < length; i++) {
        sum += data[i];
    }

    return (uint8_t)(sum & 0xFFU);
}

/*
 * 将两个8位数据组合为一个16位无符号整数。
 *
 * 协议采用高字节在前、低字节在后。
 *
 * 示例：
 *   high = 0x01
 *   low  = 0x40
 *
 * 结果：
 *   0x0140 = 320
 */
static uint16_t K210_Comm_MakeU16(uint8_t high,
                                  uint8_t low)
{
    return (uint16_t)(
        ((uint16_t)high << 8U) |
        (uint16_t)low
    );
}

/*
 * 解析一帧已经通过校验的数据。
 */
static void K210_Comm_ParseFrame(const uint8_t *frame)
{
    uint8_t command;
    uint8_t data1;
    uint8_t data2;
    uint8_t data3;

    if (frame == 0) {
        return;
    }

    command = frame[2];
    data1 = frame[3];
    data2 = frame[4];
    data3 = frame[5];

    /*
     * 只要收到一帧校验正确的数据，
     * 就认为K210当前在线。
     */
    s_k210_info.online = 1U;
    s_k210_info.last_rx_ms = BSP_GetTickMs();
    s_k210_info.valid_frame_count++;

    switch (command) {
        /*
         * 数字识别结果帧：
         *
         * AA 55 01 DIGIT VALID CONFIDENCE CHECKSUM
         *
         * DIGIT：
         *   0~9
         *
         * VALID：
         *   0：当前识别结果无效；
         *   1：当前识别结果有效。
         *
         * CONFIDENCE：
         *   0~100。
         */
        case K210_CMD_DIGIT_RESULT:
            if ((data1 <= 9U) &&
                (data2 <= 1U) &&
                (data3 <= 100U)) {
                s_k210_info.digit = data1;
                s_k210_info.digit_valid = data2;
                s_k210_info.digit_confidence = data3;
                s_k210_info.new_digit = 1U;
            } else {
                s_k210_info.format_error_count++;
            }
            break;

        /*
         * 目标中心坐标帧：
         *
         * AA 55 02 X_H X_L Y CHECKSUM
         *
         * X：
         *   16位坐标，高字节在前。
         *
         * Y：
         *   8位坐标。
         *
         * Y=0xFF：
         *   表示当前没有检测到目标。
         */
        case K210_CMD_TARGET_POINT:
            s_k210_info.target_x =
                K210_Comm_MakeU16(data1, data2);

            if (data3 == 0xFFU) {
                s_k210_info.target_y = 0U;
                s_k210_info.target_valid = 0U;
            } else {
                s_k210_info.target_y = data3;
                s_k210_info.target_valid = 1U;
            }

            s_k210_info.new_target = 1U;
            break;

        /*
         * 激光点坐标帧：
         *
         * AA 55 03 X_H X_L Y CHECKSUM
         *
         * Y=0xFF：
         *   表示当前没有检测到激光点。
         */
        case K210_CMD_LASER_POINT:
            s_k210_info.laser_x =
                K210_Comm_MakeU16(data1, data2);

            if (data3 == 0xFFU) {
                s_k210_info.laser_y = 0U;
                s_k210_info.laser_valid = 0U;
            } else {
                s_k210_info.laser_y = data3;
                s_k210_info.laser_valid = 1U;
            }

            s_k210_info.new_laser = 1U;
            break;

        /*
         * 目标状态帧：
         *
         * AA 55 04 STATE 00 00 CHECKSUM
         *
         * STATE：
         *   0：目标丢失；
         *   1：目标有效。
         */
        case K210_CMD_TARGET_STATE:
            if (data1 <= 1U) {
                s_k210_info.target_valid = data1;
            } else {
                s_k210_info.format_error_count++;
            }
            break;

        /*
         * 心跳帧。
         *
         * 不携带业务数据，
         * 只用于刷新K210在线状态。
         */
        case K210_CMD_HEARTBEAT:
            break;

        /*
         * 未定义的命令。
         */
        default:
            s_k210_info.format_error_count++;
            break;
    }
}

/*
 * 将一个接收到的字节送入协议状态机。
 *
 * 状态机支持：
 *   1. 半包；
 *   2. 连续多帧；
 *   3. 帧头错位；
 *   4. 校验错误后重新同步；
 *   5. 连续收到AA AA时重新寻找帧头。
 */
static void K210_Comm_InputByte(uint8_t byte)
{
    uint8_t checksum;

    switch (s_rx_state) {
        /*
         * 等待第一个帧头0xAA。
         */
        case K210_RX_WAIT_HEAD1:
            if (byte == K210_FRAME_HEAD1) {
                s_rx_frame[0] = byte;
                s_rx_state = K210_RX_WAIT_HEAD2;
            }
            break;

        /*
         * 已收到0xAA，等待第二个帧头0x55。
         */
        case K210_RX_WAIT_HEAD2:
            if (byte == K210_FRAME_HEAD2) {
                s_rx_frame[1] = byte;
                s_rx_index = 2U;
                s_rx_state = K210_RX_RECEIVING;
            } else if (byte == K210_FRAME_HEAD1) {
                /*
                 * 收到AA AA时，
                 * 第二个AA仍可能是新帧的第一个字节。
                 */
                s_rx_frame[0] = byte;
            } else {
                s_rx_index = 0U;
                s_rx_state = K210_RX_WAIT_HEAD1;
            }
            break;

        /*
         * 接收帧中剩余的5个字节。
         */
        case K210_RX_RECEIVING:
            /*
             * 正常情况下s_rx_index范围为2~6。
             */
            if (s_rx_index < K210_FRAME_SIZE) {
                s_rx_frame[s_rx_index] = byte;
                s_rx_index++;
            } else {
                /*
                 * 防止异常状态造成数组越界。
                 */
                s_rx_index = 0U;
                s_rx_state = K210_RX_WAIT_HEAD1;
                break;
            }

            /*
             * 收满7个字节后进行校验。
             */
            if (s_rx_index >= K210_FRAME_SIZE) {
                checksum =
                    K210_Comm_CalcChecksum(
                        s_rx_frame,
                        K210_FRAME_SIZE - 1U
                    );

                if (checksum == s_rx_frame[6]) {
                    K210_Comm_ParseFrame(s_rx_frame);
                } else {
                    s_k210_info.checksum_error_count++;
                }

                /*
                 * 无论校验成功还是失败，
                 * 都重新等待下一帧。
                 */
                s_rx_index = 0U;
                s_rx_state = K210_RX_WAIT_HEAD1;
            }
            break;

        /*
         * 异常状态恢复。
         */
        default:
            s_rx_index = 0U;
            s_rx_state = K210_RX_WAIT_HEAD1;
            break;
    }
}

void K210_Comm_Init(void)
{
    /*
     * 清空通信状态和接收帧缓存。
     */
    memset(&s_k210_info, 0, sizeof(s_k210_info));
    memset(s_rx_frame, 0, sizeof(s_rx_frame));

    /*
     * 初始化协议接收状态机。
     */
    s_rx_state = K210_RX_WAIT_HEAD1;
    s_rx_index = 0U;

    /*
     * 清空USART2底层接收缓存。
     *
     * USART2硬件已经由BSP_InitAll()完成初始化。
     */
    BSP_UART_FlushRx(K210_UART_PORT);
}

void K210_Comm_Update(void)
{
    uint8_t byte;
    uint32_t now_ms;

    /*
     * 推进USART2底层接收状态。
     *
     * 如果USART2使用DMA接收，
     * 该函数会将DMA数据搬运到软件缓冲区。
     */
    BSP_UART_Task(K210_UART_PORT);

    /*
     * 将USART2软件缓冲区中的所有字节
     * 逐个送入协议状态机。
     */
    while (BSP_UART_GetChar(K210_UART_PORT, &byte) != 0U) {
        K210_Comm_InputByte(byte);
    }

    /*
     * 检查K210通信是否超时。
     */
    now_ms = BSP_GetTickMs();

    if (s_k210_info.online != 0U) {
        if ((uint32_t)(
                now_ms -
                s_k210_info.last_rx_ms
            ) > K210_OFFLINE_TIMEOUT_MS) {
            /*
             * 超过1秒没有收到校验正确的数据帧，
             * 判定K210离线。
             */
            s_k210_info.online = 0U;

            /*
             * 离线后，当前识别结果不再视为有效。
             */
            s_k210_info.digit_valid = 0U;
            s_k210_info.target_valid = 0U;
            s_k210_info.laser_valid = 0U;
        }
    }
}

BSP_Status_t K210_Comm_GetInfo(K210_Comm_Info_t *info)
{
    if (info == 0) {
        return BSP_PARAM;
    }

    /*
     * 返回完整通信状态快照。
     */
    *info = s_k210_info;

    return BSP_OK;
}

BSP_Status_t K210_Comm_GetNewDigit(uint8_t *digit,
                                   uint8_t *valid,
                                   uint8_t *confidence)
{
    if ((digit == 0) ||
        (valid == 0) ||
        (confidence == 0)) {
        return BSP_PARAM;
    }

    /*
     * 当前没有新的数字识别结果。
     */
    if (s_k210_info.new_digit == 0U) {
        return BSP_BUSY;
    }

    /*
     * 复制最新数字结果。
     */
    *digit = s_k210_info.digit;
    *valid = s_k210_info.digit_valid;
    *confidence = s_k210_info.digit_confidence;

    /*
     * 本次读取完成后清除新数据标志。
     */
    s_k210_info.new_digit = 0U;

    /*
     * 即使valid=0，也说明成功读取到一条新结果，
     * 因此仍然返回BSP_OK。
     */
    return BSP_OK;
}

BSP_Status_t K210_Comm_GetNewTarget(uint16_t *x,
                                    uint16_t *y,
                                    uint8_t *valid)
{
    if ((x == 0) ||
        (y == 0) ||
        (valid == 0)) {
        return BSP_PARAM;
    }

    if (s_k210_info.new_target == 0U) {
        return BSP_BUSY;
    }

    *x = s_k210_info.target_x;
    *y = s_k210_info.target_y;
    *valid = s_k210_info.target_valid;

    s_k210_info.new_target = 0U;

    return BSP_OK;
}

BSP_Status_t K210_Comm_GetNewLaser(uint16_t *x,
                                   uint16_t *y,
                                   uint8_t *valid)
{
    if ((x == 0) ||
        (y == 0) ||
        (valid == 0)) {
        return BSP_PARAM;
    }

    if (s_k210_info.new_laser == 0U) {
        return BSP_BUSY;
    }

    *x = s_k210_info.laser_x;
    *y = s_k210_info.laser_y;
    *valid = s_k210_info.laser_valid;

    s_k210_info.new_laser = 0U;

    return BSP_OK;
}

BSP_Status_t K210_Comm_SendFrame(uint8_t command,
                                 uint8_t data1,
                                 uint8_t data2,
                                 uint8_t data3)
{
    uint8_t frame[K210_FRAME_SIZE];

    frame[0] = K210_FRAME_HEAD1;
    frame[1] = K210_FRAME_HEAD2;
    frame[2] = command;
    frame[3] = data1;
    frame[4] = data2;
    frame[5] = data3;

    frame[6] =
        K210_Comm_CalcChecksum(
            frame,
            K210_FRAME_SIZE - 1U
        );

    return BSP_UART_WriteFrame(
        K210_UART_PORT,
        frame,
        K210_FRAME_SIZE
    );
}

BSP_Status_t K210_Comm_StartDetect(uint8_t mode)
{
    return K210_Comm_SendFrame(
        K210_CMD_START_DETECT,
        mode,
        0U,
        0U
    );
}

BSP_Status_t K210_Comm_StopDetect(void)
{
    return K210_Comm_SendFrame(
        K210_CMD_STOP_DETECT,
        0U,
        0U,
        0U
    );
}

BSP_Status_t K210_Comm_SetMode(uint8_t mode)
{
    return K210_Comm_SendFrame(
        K210_CMD_SET_MODE,
        mode,
        0U,
        0U
    );
}
