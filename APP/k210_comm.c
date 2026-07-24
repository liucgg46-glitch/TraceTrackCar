#include "k210_comm.h"

#include "bsp_uart.h"
#include "bsp_systick.h"

#include <string.h>

#define K210_UART_PORT             UART_PORT_K210
#define K210_OFFLINE_TIMEOUT_MS    1000U
#define K210_ROAD_PROFILE_TX_INTERVAL_MS  200U
#define K210_ROAD_PROFILE_TX_REPEAT_COUNT 10U

#define K210_VISION_FIELD_INFO     0x01U
#define K210_VISION_FIELD_X        0x02U
#define K210_VISION_FIELD_Y        0x04U
#define K210_VISION_FIELD_SIZE     0x08U
#define K210_VISION_FIELD_AREA     0x10U
#define K210_VISION_REQUIRED_FIELDS \
    (K210_VISION_FIELD_INFO | K210_VISION_FIELD_X | K210_VISION_FIELD_Y)

typedef enum {
    K210_RX_WAIT_HEAD1 = 0,
    K210_RX_WAIT_HEAD2,
    K210_RX_RECEIVING
} K210_RxState_t;

static K210_Comm_Info_t s_k210_info;
static K210_RxState_t s_rx_state;
static uint8_t s_rx_frame[K210_FRAME_SIZE];
static uint8_t s_rx_index;

static K210_DigitSnapshot_t s_snapshot_building;
static K210_DigitSnapshot_t s_snapshot_latest;
static uint8_t s_snapshot_receiving;
static uint8_t s_snapshot_expected_count;
static uint8_t s_snapshot_received_count;
static uint8_t s_new_snapshot;

static K210_VisionResult_t s_vision_building;
static K210_VisionResult_t s_vision_latest;
static uint8_t s_vision_receiving;
static uint8_t s_vision_expected_count;
static uint8_t s_vision_field_mask[K210_MAX_VISION_TARGETS];
static uint8_t s_new_vision_result;
static uint8_t s_have_vision_result;
static uint8_t s_selected_road_profile;
static uint8_t s_road_profile_tx_remaining;
static uint32_t s_road_profile_last_tx_ms;

static uint8_t K210_Comm_RoadProfileValid(uint8_t profile_id)
{
    return (profile_id == K210_ROAD_PROFILE_CURRENT ||
            profile_id == K210_ROAD_PROFILE_OLD) ? 1U : 0U;
}

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

static uint16_t K210_Comm_MakeU16(uint8_t high,
                                  uint8_t low)
{
    return (uint16_t)(
        ((uint16_t)high << 8U) |
        (uint16_t)low
    );
}

static uint16_t K210_Comm_DecodeCenterX(uint8_t encoded_x)
{
    return (uint16_t)(
        (((uint32_t)encoded_x * 319U) + 127U) / 255U
    );
}

static void K210_Comm_AbortSnapshot(void)
{
    s_snapshot_receiving = 0U;
    s_snapshot_expected_count = 0U;
    s_snapshot_received_count = 0U;
    s_k210_info.snapshot_error_count++;
}

static void K210_Comm_ResetVisionBuilding(void)
{
    memset(&s_vision_building, 0, sizeof(s_vision_building));
    memset(s_vision_field_mask, 0, sizeof(s_vision_field_mask));
    s_vision_receiving = 0U;
    s_vision_expected_count = 0U;
}

static void K210_Comm_AbortVisionResult(void)
{
    K210_Comm_ResetVisionBuilding();
    s_k210_info.vision_error_count++;
}

static uint8_t K210_Comm_VisionIndexValid(uint8_t index)
{
    if (s_vision_receiving == 0U) {
        return 0U;
    }

    if (index >= s_vision_expected_count) {
        return 0U;
    }

    if (index >= K210_MAX_VISION_TARGETS) {
        return 0U;
    }

    return 1U;
}

static uint8_t K210_Comm_VisionFieldMissing(uint8_t index,
                                             uint8_t field)
{
    if ((s_vision_field_mask[index] & field) != 0U) {
        return 0U;
    }

    return 1U;
}

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

    s_k210_info.online = 1U;
    s_k210_info.last_rx_ms = BSP_GetTickMs();
    s_k210_info.valid_frame_count++;

    switch (command) {
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

        case K210_CMD_TARGET_STATE:
            if (data1 <= 1U) {
                s_k210_info.target_valid = data1;
            } else {
                s_k210_info.format_error_count++;
            }
            break;

        case K210_CMD_HEARTBEAT:
            break;

        case K210_CMD_DIGIT_SNAPSHOT_BEGIN:
            if ((data2 > K210_MAX_DIGITS) ||
                (data3 > K210_RESULT_OVERFLOW) ||
                ((data3 == K210_RESULT_NORMAL) &&
                 (data2 == 0U)) ||
                ((data3 != K210_RESULT_NORMAL) &&
                 (data2 != 0U))) {
                K210_Comm_AbortSnapshot();
                break;
            }

            memset(&s_snapshot_building,
                   0,
                   sizeof(s_snapshot_building));

            s_snapshot_building.sequence = data1;
            s_snapshot_building.count = data2;
            s_snapshot_building.status = data3;

            s_snapshot_expected_count = data2;
            s_snapshot_received_count = 0U;
            s_snapshot_receiving = 1U;
            break;

        case K210_CMD_DIGIT_SNAPSHOT_ITEM:
        {
            uint8_t index;
            uint8_t digit;

            index = (uint8_t)((data1 >> 4U) & 0x0FU);
            digit = (uint8_t)(data1 & 0x0FU);

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

        case K210_CMD_VISION_BEGIN:
            if ((data2 == K210_VISION_MODE_NONE) ||
                (data2 > K210_VISION_MODE_CUSTOM) ||
                (data3 > K210_MAX_VISION_TARGETS)) {
                K210_Comm_AbortVisionResult();
                break;
            }

            if (s_vision_receiving != 0U) {
                K210_Comm_AbortVisionResult();
            }

            K210_Comm_ResetVisionBuilding();

            s_vision_building.sequence = data1;
            s_vision_building.mode = data2;
            s_vision_building.target_count = data3;
            s_vision_expected_count = data3;
            s_vision_receiving = 1U;
            break;

        case K210_CMD_VISION_TARGET_INFO:
        {
            uint8_t index;
            uint8_t class_id;

            index = (uint8_t)((data1 >> 4U) & 0x0FU);
            class_id = (uint8_t)(data1 & 0x0FU);

            if ((K210_Comm_VisionIndexValid(index) == 0U) ||
                (K210_Comm_VisionFieldMissing(
                     index,
                     K210_VISION_FIELD_INFO) == 0U) ||
                (class_id == K210_TARGET_CLASS_NONE) ||
                (data3 > 100U)) {
                K210_Comm_AbortVisionResult();
                break;
            }

            s_vision_building.targets[index].class_id = class_id;
            s_vision_building.targets[index].value = data2;
            s_vision_building.targets[index].confidence = data3;
            s_vision_field_mask[index] |= K210_VISION_FIELD_INFO;
            break;
        }

        case K210_CMD_VISION_TARGET_X:
        {
            uint8_t index;

            index = data1;

            if ((K210_Comm_VisionIndexValid(index) == 0U) ||
                (K210_Comm_VisionFieldMissing(
                     index,
                     K210_VISION_FIELD_X) == 0U)) {
                K210_Comm_AbortVisionResult();
                break;
            }

            s_vision_building.targets[index].center_x =
                K210_Comm_MakeU16(data2, data3);
            s_vision_field_mask[index] |= K210_VISION_FIELD_X;
            break;
        }

        case K210_CMD_VISION_TARGET_Y:
        {
            uint8_t index;

            index = data1;

            if ((K210_Comm_VisionIndexValid(index) == 0U) ||
                (K210_Comm_VisionFieldMissing(
                     index,
                     K210_VISION_FIELD_Y) == 0U)) {
                K210_Comm_AbortVisionResult();
                break;
            }

            s_vision_building.targets[index].center_y =
                K210_Comm_MakeU16(data2, data3);
            s_vision_field_mask[index] |= K210_VISION_FIELD_Y;
            break;
        }

        case K210_CMD_VISION_TARGET_SIZE:
        {
            uint8_t index;

            index = data1;

            if ((K210_Comm_VisionIndexValid(index) == 0U) ||
                (K210_Comm_VisionFieldMissing(
                     index,
                     K210_VISION_FIELD_SIZE) == 0U)) {
                K210_Comm_AbortVisionResult();
                break;
            }

            s_vision_building.targets[index].width = data2;
            s_vision_building.targets[index].height = data3;
            s_vision_field_mask[index] |= K210_VISION_FIELD_SIZE;
            break;
        }

        case K210_CMD_VISION_TARGET_AREA:
        {
            uint8_t index;

            index = data1;

            if ((K210_Comm_VisionIndexValid(index) == 0U) ||
                (K210_Comm_VisionFieldMissing(
                     index,
                     K210_VISION_FIELD_AREA) == 0U)) {
                K210_Comm_AbortVisionResult();
                break;
            }

            s_vision_building.targets[index].area =
                K210_Comm_MakeU16(data2, data3);
            s_vision_field_mask[index] |= K210_VISION_FIELD_AREA;
            break;
        }

        case K210_CMD_VISION_END:
        {
            uint8_t i;
            uint8_t result_valid;

            result_valid = 1U;

            if ((s_vision_receiving == 0U) ||
                (data1 != s_vision_building.sequence) ||
                (data2 != s_vision_expected_count) ||
                (data3 > K210_VISION_RESULT_ERROR)) {
                result_valid = 0U;
            }

            if (result_valid != 0U) {
                if ((data3 == K210_VISION_RESULT_OK) &&
                    (data2 == 0U)) {
                    result_valid = 0U;
                }

                if ((data3 != K210_VISION_RESULT_OK) &&
                    (data2 != 0U)) {
                    result_valid = 0U;
                }
            }

            if ((result_valid != 0U) &&
                (data3 == K210_VISION_RESULT_OK)) {
                for (i = 0U; i < s_vision_expected_count; i++) {
                    if ((s_vision_field_mask[i] &
                         K210_VISION_REQUIRED_FIELDS) !=
                        K210_VISION_REQUIRED_FIELDS) {
                        result_valid = 0U;
                        break;
                    }
                }
            }

            if (result_valid == 0U) {
                K210_Comm_AbortVisionResult();
                break;
            }

            s_vision_building.status = data3;

            if (s_new_vision_result != 0U) {
                s_k210_info.vision_overwrite_count++;
            }

            s_vision_latest = s_vision_building;
            s_new_vision_result = 1U;
            s_have_vision_result = 1U;

            s_k210_info.vision_mode = s_vision_latest.mode;
            s_k210_info.new_vision_result = 1U;
            s_k210_info.vision_result_count++;

            K210_Comm_ResetVisionBuilding();
            break;
        }

        case K210_CMD_ROAD_LINE_STATE:
        {
            uint8_t status;
            uint8_t near_source;
            uint8_t mid_source;

            status = data3;
            near_source =
                ((status & 0x02U) != 0U) ? 1U : 0U;
            mid_source =
                ((status & 0x04U) != 0U) ? 1U : 0U;

            /*
             * bit5～bit7必须为0；
             * NEAR和MID不能同时置位。
             */
            if (((status & 0xE0U) != 0U) ||
                ((near_source != 0U) &&
                 (mid_source != 0U))) {
                s_k210_info.format_error_count++;
                break;
            }

            s_k210_info.road.line_x =
                K210_Comm_DecodeCenterX(data1);

            s_k210_info.road.line_error =
                (int16_t)data2 - 128;

            s_k210_info.road.line_valid =
                ((status & 0x01U) != 0U) ? 1U : 0U;

            if (near_source != 0U) {
                s_k210_info.road.line_source =
                    K210_LINE_SOURCE_NEAR;
            } else if (mid_source != 0U) {
                s_k210_info.road.line_source =
                    K210_LINE_SOURCE_MID;
            } else {
                s_k210_info.road.line_source =
                    K210_LINE_SOURCE_NONE;
            }

            s_k210_info.road.cross_valid =
                ((status & 0x08U) != 0U) ? 1U : 0U;

            s_k210_info.road.stop_valid =
                ((status & 0x10U) != 0U) ? 1U : 0U;

            s_k210_info.road.new_line_state = 1U;
            break;
        }

        case K210_CMD_ROAD_EVENT_STATE:
            if (data3 <= K210_ROAD_EVENT_STOP_LEAVE) {
                s_k210_info.road.cross_count = data1;
                s_k210_info.road.stop_count = data2;

                /*
                 * 周期发送的NONE帧只刷新累计数量，
                 * 不覆盖尚未被应用层读取的事件。
                 */
                if (data3 != K210_ROAD_EVENT_NONE) {
                    s_k210_info.road.road_event = data3;
                    s_k210_info.road.new_road_event = 1U;
                }
            } else {
                s_k210_info.format_error_count++;
            }
            break;

        default:
            s_k210_info.format_error_count++;
            break;
    }
}

static void K210_Comm_InputByte(uint8_t byte)
{
    uint8_t checksum;

    switch (s_rx_state) {
        case K210_RX_WAIT_HEAD1:
            if (byte == K210_FRAME_HEAD1) {
                s_rx_frame[0] = byte;
                s_rx_state = K210_RX_WAIT_HEAD2;
            }
            break;

        case K210_RX_WAIT_HEAD2:
            if (byte == K210_FRAME_HEAD2) {
                s_rx_frame[1] = byte;
                s_rx_index = 2U;
                s_rx_state = K210_RX_RECEIVING;
            } else if (byte == K210_FRAME_HEAD1) {
                s_rx_frame[0] = byte;
            } else {
                s_rx_index = 0U;
                s_rx_state = K210_RX_WAIT_HEAD1;
            }
            break;

        case K210_RX_RECEIVING:
            if (s_rx_index < K210_FRAME_SIZE) {
                s_rx_frame[s_rx_index] = byte;
                s_rx_index++;
            } else {
                s_rx_index = 0U;
                s_rx_state = K210_RX_WAIT_HEAD1;
                break;
            }

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

                s_rx_index = 0U;
                s_rx_state = K210_RX_WAIT_HEAD1;
            }
            break;

        default:
            s_rx_index = 0U;
            s_rx_state = K210_RX_WAIT_HEAD1;
            break;
    }
}

void K210_Comm_Init(void)
{
    memset(&s_k210_info, 0, sizeof(s_k210_info));
    memset(s_rx_frame, 0, sizeof(s_rx_frame));
    memset(&s_snapshot_building, 0, sizeof(s_snapshot_building));
    memset(&s_snapshot_latest, 0, sizeof(s_snapshot_latest));
    memset(&s_vision_latest, 0, sizeof(s_vision_latest));

    s_rx_state = K210_RX_WAIT_HEAD1;
    s_rx_index = 0U;

    s_snapshot_receiving = 0U;
    s_snapshot_expected_count = 0U;
    s_snapshot_received_count = 0U;
    s_new_snapshot = 0U;

    K210_Comm_ResetVisionBuilding();
    s_new_vision_result = 0U;
    s_have_vision_result = 0U;

    s_selected_road_profile = K210_ROAD_PROFILE_CURRENT;
    s_road_profile_tx_remaining = K210_ROAD_PROFILE_TX_REPEAT_COUNT;
    s_road_profile_last_tx_ms = BSP_GetTickMs() - K210_ROAD_PROFILE_TX_INTERVAL_MS;
    s_k210_info.selected_road_profile = s_selected_road_profile;
    s_k210_info.road_profile_tx_remaining = s_road_profile_tx_remaining;
    s_k210_info.road_profile_last_tx_ok = 0U;

    BSP_UART_FlushRx(K210_UART_PORT);
}

void K210_Comm_Update(void)
{
    uint8_t byte;
    uint32_t now_ms;

    BSP_UART_Task(K210_UART_PORT);

    while (BSP_UART_GetChar(
               K210_UART_PORT,
               &byte
           ) != 0U) {
        K210_Comm_InputByte(byte);
    }

    now_ms = BSP_GetTickMs();

    if (s_road_profile_tx_remaining > 0U &&
        (uint32_t)(now_ms - s_road_profile_last_tx_ms) >= K210_ROAD_PROFILE_TX_INTERVAL_MS) {
        BSP_Status_t tx_status = K210_Comm_SendRoadProfile(s_selected_road_profile);
        if (tx_status == BSP_OK) {
            s_road_profile_last_tx_ms = now_ms;
            s_road_profile_tx_remaining--;
            s_k210_info.road_profile_last_tx_ok = 1U;
            s_k210_info.road_profile_tx_count++;
        } else if (tx_status == BSP_BUSY) {
            s_k210_info.road_profile_last_tx_ok = 0U;
            s_k210_info.road_profile_tx_busy_count++;
        } else {
            s_k210_info.road_profile_last_tx_ok = 0U;
            s_road_profile_tx_remaining = 0U;
        }
        s_k210_info.road_profile_tx_remaining = s_road_profile_tx_remaining;
    }

    if (s_k210_info.online != 0U) {
        if ((uint32_t)(
                now_ms -
                s_k210_info.last_rx_ms
            ) > K210_OFFLINE_TIMEOUT_MS) {
            s_k210_info.online = 0U;
            s_k210_info.timeout_count++;

            s_k210_info.digit_valid = 0U;
            s_k210_info.target_valid = 0U;
            s_k210_info.laser_valid = 0U;

            s_k210_info.road.line_valid = 0U;
            s_k210_info.road.line_source =
                K210_LINE_SOURCE_NONE;
            s_k210_info.road.cross_valid = 0U;
            s_k210_info.road.stop_valid = 0U;

            if (s_snapshot_receiving != 0U) {
                K210_Comm_AbortSnapshot();
            }

            if (s_vision_receiving != 0U) {
                K210_Comm_AbortVisionResult();
            }
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

    memset(digits, 0, K210_MAX_DIGITS * sizeof(digits[0]));

    if (s_k210_info.online == 0U) {
        return 0U;
    }

    snapshot = s_snapshot_latest;

    if ((snapshot.status != K210_RESULT_NORMAL) ||
        (snapshot.count == 0U) ||
        (snapshot.count > K210_MAX_DIGITS)) {
        return 0U;
    }

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

BSP_Status_t K210_Comm_GetNewVisionResult(
    K210_VisionResult_t *result
)
{
    if (result == 0) {
        return BSP_PARAM;
    }

    if (s_new_vision_result == 0U) {
        return BSP_BUSY;
    }

    *result = s_vision_latest;
    s_new_vision_result = 0U;
    s_k210_info.new_vision_result = 0U;

    return BSP_OK;
}

BSP_Status_t K210_Comm_ReadLatestVisionResult(
    K210_VisionResult_t *result
)
{
    if (result == 0) {
        return BSP_PARAM;
    }

    if (s_have_vision_result == 0U) {
        return BSP_BUSY;
    }

    *result = s_vision_latest;
    return BSP_OK;
}

BSP_Status_t K210_Comm_ReadPrimaryTarget(
    K210_VisionTarget_t *target
)
{
    if (target == 0) {
        return BSP_PARAM;
    }

    if ((s_k210_info.online == 0U) ||
        (s_have_vision_result == 0U) ||
        (s_vision_latest.status != K210_VISION_RESULT_OK) ||
        (s_vision_latest.target_count == 0U) ||
        (s_vision_latest.target_count > K210_MAX_VISION_TARGETS)) {
        return BSP_BUSY;
    }

    *target = s_vision_latest.targets[0];
    return BSP_OK;
}

BSP_Status_t K210_Comm_ReadRoadState(
    K210_RoadState_t *road
)
{
    if (road == 0) {
        return BSP_PARAM;
    }

    if (s_k210_info.online == 0U) {
        return BSP_BUSY;
    }

    *road = s_k210_info.road;
    return BSP_OK;
}

BSP_Status_t K210_Comm_GetNewRoadLineState(
    K210_RoadState_t *road
)
{
    if (road == 0) {
        return BSP_PARAM;
    }

    if ((s_k210_info.online == 0U) ||
        (s_k210_info.road.new_line_state == 0U)) {
        return BSP_BUSY;
    }

    *road = s_k210_info.road;
    s_k210_info.road.new_line_state = 0U;

    return BSP_OK;
}

BSP_Status_t K210_Comm_GetNewRoadEvent(
    uint8_t *cross_count,
    uint8_t *stop_count,
    uint8_t *road_event
)
{
    if ((cross_count == 0) ||
        (stop_count == 0) ||
        (road_event == 0)) {
        return BSP_PARAM;
    }

    if ((s_k210_info.online == 0U) ||
        (s_k210_info.road.new_road_event == 0U)) {
        return BSP_BUSY;
    }

    *cross_count = s_k210_info.road.cross_count;
    *stop_count = s_k210_info.road.stop_count;
    *road_event = s_k210_info.road.road_event;

    s_k210_info.road.new_road_event = 0U;
    s_k210_info.road.road_event =
        K210_ROAD_EVENT_NONE;

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
    if ((mode == K210_VISION_MODE_NONE) ||
        (mode > K210_VISION_MODE_CUSTOM)) {
        return BSP_PARAM;
    }

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
    if ((mode == K210_VISION_MODE_NONE) ||
        (mode > K210_VISION_MODE_CUSTOM)) {
        return BSP_PARAM;
    }

    return K210_Comm_SendFrame(
        K210_CMD_SET_MODE,
        mode,
        0U,
        0U
    );
}
BSP_Status_t K210_Comm_SendRoadProfile(uint8_t profile_id)
{
    if (K210_Comm_RoadProfileValid(profile_id) == 0U) return BSP_PARAM;
    return K210_Comm_SendFrame(K210_CMD_SET_ROAD_PROFILE, profile_id, 0U, 0U);
}

BSP_Status_t K210_Comm_SelectRoadProfile(uint8_t profile_id)
{
    if (K210_Comm_RoadProfileValid(profile_id) == 0U) return BSP_PARAM;
    s_selected_road_profile = profile_id;
    s_road_profile_tx_remaining = K210_ROAD_PROFILE_TX_REPEAT_COUNT;
    s_road_profile_last_tx_ms = BSP_GetTickMs() - K210_ROAD_PROFILE_TX_INTERVAL_MS;
    s_k210_info.selected_road_profile = profile_id;
    s_k210_info.road_profile_tx_remaining = s_road_profile_tx_remaining;
    s_k210_info.road_profile_last_tx_ok = 0U;
    return BSP_OK;
}

void K210_Comm_RestartRoadProfileSync(void)
{
    s_road_profile_tx_remaining = K210_ROAD_PROFILE_TX_REPEAT_COUNT;
    s_road_profile_last_tx_ms = BSP_GetTickMs() - K210_ROAD_PROFILE_TX_INTERVAL_MS;
    s_k210_info.road_profile_tx_remaining = s_road_profile_tx_remaining;
    s_k210_info.road_profile_last_tx_ok = 0U;
}