#include "k210_comm.h"

#include "bsp_uart.h"
#include "bsp_systick.h"

#include <string.h>

/*
 * K210固定使用USART2。
 *
 * USART1保留给USB转TTL调试输出：
 *   PA9  -> USB转TTL RX
 *   PA10 <- USB转TTL TX
 *
 * USART2连接K210：
 *   STM32 PA2  -> K210 RX，第一阶段暂时不接
 *   STM32 PA3  <- K210 IO6 TX
 */
#define K210_UART_PORT             UART_PORT2

/*
 * 超过1秒没有收到校验正确的数据帧，
 * 则认为K210已经离线。
 */
#define K210_OFFLINE_TIMEOUT_MS    1000U

/* 串口字节接收状态机 */
typedef enum {
    K210_RX_WAIT_HEAD1 = 0,
    K210_RX_WAIT_HEAD2,
    K210_RX_RECEIVING
} K210_RxState_t;

/* K210通信状态和诊断信息 */
static K210_Comm_Info_t s_k210_info;

/* 固定7字节帧的接收状态 */
static K210_RxState_t s_rx_state;
static uint8_t s_rx_frame[K210_FRAME_SIZE];
static uint8_t s_rx_index;

/*
 * 可变数量数字快照缓存。
 *
 * s_snapshot_building：
 *   当前正在接收和组装的快照。
 *
 * s_snapshot_latest：
 *   已经完整接收并提交的最新快照。
 */
static K210_DigitSnapshot_t s_snapshot_building;
static K210_DigitSnapshot_t s_snapshot_latest;
static uint8_t s_snapshot_receiving;
static uint8_t s_snapshot_expected_count;
static uint8_t s_snapshot_received_count;
static uint8_t s_new_snapshot;

/*
 * 计算校验和。
 *
 * 校验值为指定字节累加和的低8位。
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
 * 将两个8位数据组合成一个16位无符号整数。
 * 协议使用高字节在前、低字节在后。
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
 * K210为了将横坐标装入一个字节，
 * 会把0～319压缩成0～255。
 *
 * STM32收到后再把它还原到QVGA的0～319范围。
 */
static uint16_t K210_Comm_DecodeCenterX(uint8_t encoded_x)
{
    return (uint16_t)(
        (((uint32_t)encoded_x * 319U) + 127U) / 255U
    );
}

/*
 * 放弃当前正在接收的快照。
 *
 * 出现顺序错误、数量错误、序号不一致或内容非法时调用。
 */
static void K210_Comm_AbortSnapshot(void)
{
    s_snapshot_receiving = 0U;
    s_snapshot_expected_count = 0U;
    s_snapshot_received_count = 0U;
    s_k210_info.snapshot_error_count++;
}

/* 解析一个已经通过校验的固定7字节帧 */
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
     * 只要收到一个校验正确的数据帧，
     * 就认为K210当前在线。
     */
    s_k210_info.online = 1U;
    s_k210_info.last_rx_ms = BSP_GetTickMs();
    s_k210_info.valid_frame_count++;

    switch (command) {
        /*
         * 旧版单数字识别结果：
         *
         * AA 55 01 DIGIT VALID CONFIDENCE CHECKSUM
         *
         * DIGIT：数字类别
         * VALID：0表示无效，1表示有效
         * CONFIDENCE：0～100
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
         * 目标中心坐标：
         *
         * AA 55 02 X_H X_L Y CHECKSUM
         *
         * X由高低两个字节组成。
         * Y等于0xFF时表示目标无效。
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
         * 激光点坐标：
         *
         * AA 55 03 X_H X_L Y CHECKSUM
         *
         * Y等于0xFF时表示激光点无效。
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
         * 目标有效状态：
         *
         * AA 55 04 STATE 00 00 CHECKSUM
         *
         * STATE：
         *   0：目标丢失
         *   1：目标有效
         */
        case K210_CMD_TARGET_STATE:
            if (data1 <= 1U) {
                s_k210_info.target_valid = data1;
            } else {
                s_k210_info.format_error_count++;
            }
            break;

        /*
         * 心跳帧不包含业务数据。
         * 收到有效心跳后，前面已经刷新了在线状态。
         */
        case K210_CMD_HEARTBEAT:
            break;

        /*
         * 多数字快照开始帧：
         *
         * AA 55 10 SEQUENCE COUNT STATUS CHECKSUM
         *
         * SEQUENCE：快照序号
         * COUNT：本次数字数量
         * STATUS：识别状态
         */
        case K210_CMD_DIGIT_SNAPSHOT_BEGIN:
            /*
             * 检查以下非法情况：
             *
             * 1. 数量超过STM32缓存容量；
             * 2. 状态值不在定义范围内；
             * 3. NORMAL状态下数量为0；
             * 4. 非NORMAL状态下数量却不为0。
             */
            if ((data2 > K210_MAX_DIGITS) ||
                (data3 > K210_RESULT_OVERFLOW) ||
                ((data3 == K210_RESULT_NORMAL) &&
                 (data2 == 0U)) ||
                ((data3 != K210_RESULT_NORMAL) &&
                 (data2 != 0U))) {
                K210_Comm_AbortSnapshot();
                break;
            }

            memset(
                &s_snapshot_building,
                0,
                sizeof(s_snapshot_building)
            );

            s_snapshot_building.sequence = data1;
            s_snapshot_building.count = data2;
            s_snapshot_building.status = data3;

            s_snapshot_expected_count = data2;
            s_snapshot_received_count = 0U;
            s_snapshot_receiving = 1U;
            break;

        /*
         * 多数字快照项目帧：
         *
         * AA 55 11 INDEX_DIGIT CONFIDENCE CENTER_X_8 CHECKSUM
         *
         * INDEX_DIGIT高4位：项目序号
         * INDEX_DIGIT低4位：数字类别
         * CONFIDENCE：0～100
         * CENTER_X_8：压缩后的横坐标0～255
         */
        case K210_CMD_DIGIT_SNAPSHOT_ITEM:
        {
            uint8_t index;
            uint8_t digit;

            index = (uint8_t)((data1 >> 4U) & 0x0FU);
            digit = (uint8_t)(data1 & 0x0FU);

            /*
             * 必须按照0、1、2……的顺序接收ITEM。
             * 任意一个条件错误都放弃整次快照。
             */
            if ((s_snapshot_receiving == 0U) ||
                (s_snapshot_building.status !=
                 K210_RESULT_NORMAL) ||
                (index != s_snapshot_received_count) ||
                (index >= s_snapshot_expected_count) ||
                (index >= K210_MAX_DIGITS) ||
                (digit < 1U) ||
                (digit > 8U) ||
                (data2 > 100U)) {
                K210_Comm_AbortSnapshot();
                break;
            }

            s_snapshot_building.items[index].digit = digit;
            s_snapshot_building.items[index].confidence = data2;
            s_snapshot_building.items[index].center_x =
                K210_Comm_DecodeCenterX(data3);

            s_snapshot_received_count++;
            break;
        }

        /*
         * 多数字快照结束帧：
         *
         * AA 55 12 SEQUENCE COUNT 00 CHECKSUM
         *
         * BEGIN和END的序号、数量必须一致，
         * 实际收到的ITEM数量也必须与COUNT一致。
         */
        case K210_CMD_DIGIT_SNAPSHOT_END:
            if ((s_snapshot_receiving == 0U) ||
                (data1 != s_snapshot_building.sequence) ||
                (data2 != s_snapshot_expected_count) ||
                (data3 != 0U) ||
                (s_snapshot_received_count !=
                 s_snapshot_expected_count)) {
                K210_Comm_AbortSnapshot();
                break;
            }

            /*
             * 如果上一份快照还没有被应用层读取，
             * 新快照会覆盖旧快照，并记录覆盖次数。
             */
            if (s_new_snapshot != 0U) {
                s_k210_info.snapshot_overwrite_count++;
            }

            s_snapshot_latest = s_snapshot_building;
            s_new_snapshot = 1U;
            s_k210_info.snapshot_count++;

            s_snapshot_receiving = 0U;
            s_snapshot_expected_count = 0U;
            s_snapshot_received_count = 0U;
            break;

        /* 未定义的命令 */
        default:
            s_k210_info.format_error_count++;
            break;
    }
}

/*
 * 将USART2收到的一个字节送入协议状态机。
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
        /* 等待第一个帧头0xAA */
        case K210_RX_WAIT_HEAD1:
            if (byte == K210_FRAME_HEAD1) {
                s_rx_frame[0] = byte;
                s_rx_state = K210_RX_WAIT_HEAD2;
            }
            break;

        /* 已经收到0xAA，等待第二个帧头0x55 */
        case K210_RX_WAIT_HEAD2:
            if (byte == K210_FRAME_HEAD2) {
                s_rx_frame[1] = byte;
                s_rx_index = 2U;
                s_rx_state = K210_RX_RECEIVING;
            } else if (byte == K210_FRAME_HEAD1) {
                /*
                 * 如果收到AA AA，
                 * 第二个AA仍可能是新帧的第一个字节。
                 */
                s_rx_frame[0] = byte;
            } else {
                s_rx_index = 0U;
                s_rx_state = K210_RX_WAIT_HEAD1;
            }
            break;

        /* 接收固定帧剩余的5个字节 */
        case K210_RX_RECEIVING:
            if (s_rx_index < K210_FRAME_SIZE) {
                s_rx_frame[s_rx_index] = byte;
                s_rx_index++;
            } else {
                /*
                 * 异常保护，防止数组越界。
                 */
                s_rx_index = 0U;
                s_rx_state = K210_RX_WAIT_HEAD1;
                break;
            }

            /* 收满7个字节后进行校验 */
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

        /* 异常状态恢复 */
        default:
            s_rx_index = 0U;
            s_rx_state = K210_RX_WAIT_HEAD1;
            break;
    }
}

void K210_Comm_Init(void)
{
    /* 清空通信状态和帧缓存 */
    memset(&s_k210_info, 0, sizeof(s_k210_info));
    memset(s_rx_frame, 0, sizeof(s_rx_frame));
    memset(
        &s_snapshot_building,
        0,
        sizeof(s_snapshot_building)
    );
    memset(
        &s_snapshot_latest,
        0,
        sizeof(s_snapshot_latest)
    );

    /* 初始化固定帧接收状态机 */
    s_rx_state = K210_RX_WAIT_HEAD1;
    s_rx_index = 0U;

    /* 初始化多数字快照接收状态 */
    s_snapshot_receiving = 0U;
    s_snapshot_expected_count = 0U;
    s_snapshot_received_count = 0U;
    s_new_snapshot = 0U;

    /*
     * 清空USART2底层接收缓存。
     * USART2硬件已经由BSP_InitAll()初始化。
     */
    BSP_UART_FlushRx(K210_UART_PORT);
}

void K210_Comm_Update(void)
{
    uint8_t byte;
    uint32_t now_ms;

    /*
     * 推进USART2底层接收任务。
     * 如果USART2使用DMA，这一步会更新软件接收缓存。
     */
    BSP_UART_Task(K210_UART_PORT);

    /*
     * 将USART2软件缓冲区中的所有字节，
     * 逐个送入协议状态机。
     */
    while (BSP_UART_GetChar(
               K210_UART_PORT,
               &byte
           ) != 0U) {
        K210_Comm_InputByte(byte);
    }

    /* 检查K210是否通信超时 */
    now_ms = BSP_GetTickMs();

    if (s_k210_info.online != 0U) {
        if ((uint32_t)(
                now_ms -
                s_k210_info.last_rx_ms
            ) > K210_OFFLINE_TIMEOUT_MS) {
            /* 超过1秒未收到有效帧，判定K210离线 */
            s_k210_info.online = 0U;

            /*
             * 离线后，将旧版目标状态标记为无效。
             * 已经提交的快照仍可由应用层决定是否清除。
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

    if (s_k210_info.new_digit == 0U) {
        return BSP_BUSY;
    }

    *digit = s_k210_info.digit;
    *valid = s_k210_info.digit_valid;
    *confidence = s_k210_info.digit_confidence;

    /* 读取完成后清除新数据标志 */
    s_k210_info.new_digit = 0U;

    return BSP_OK;
}

BSP_Status_t K210_Comm_GetNewSnapshot(
    K210_DigitSnapshot_t *snapshot
)
{
    if (snapshot == 0) {
        return BSP_PARAM;
    }

    if (s_new_snapshot == 0U) {
        return BSP_BUSY;
    }

    /* 复制最新完整快照，并清除新快照标志 */
    *snapshot = s_snapshot_latest;
    s_new_snapshot = 0U;

    return BSP_OK;
}

uint8_t K210_Comm_ReadDigits(
    uint8_t digits[K210_MAX_DIGITS]
)
{
    K210_DigitSnapshot_t snapshot;
    uint8_t i;

    if (digits == 0) {
        return 0U;
    }

    /* 没有有效结果时，调用者不会残留上一次读取的数字 */
    memset(digits, 0, K210_MAX_DIGITS * sizeof(digits[0]));

    if (s_k210_info.online == 0U) {
        return 0U;
    }

    /* 直接读取最新结果，不消耗测试任务使用的新快照标志 */
    snapshot = s_snapshot_latest;

    if ((snapshot.status != K210_RESULT_NORMAL) ||
        (snapshot.count == 0U) ||
        (snapshot.count > K210_MAX_DIGITS)) {
        return 0U;
    }

    /* 只向业务层返回从左到右排列的数字值 */
    for (i = 0U; i < snapshot.count; i++) {
        digits[i] = snapshot.items[i].digit;
    }

    return snapshot.count;
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
