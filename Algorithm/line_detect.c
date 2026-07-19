#include "line_detect.h"
#include "control_config.h"

static LineDetect_Result_t s_line;
static uint16_t s_white[LINE_DETECT_SENSOR_NUM];
static uint16_t s_black[LINE_DETECT_SENSOR_NUM];
static uint8_t s_has_white;
static uint8_t s_has_black;

static const int16_t s_weight[LINE_DETECT_SENSOR_NUM] = {
    -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
};

/*
 * 方向开关只规范化算法层的左右含义，不重排各物理通道的阈值。
 * 这样旋转传感器后，原有逐通道标定值仍与对应探头保持一致。
 */
static uint8_t Line_NormalizeMask(uint8_t mask)
{
#if CONTROL_LINE_DIRECTION_REVERSE
    uint8_t i;
    uint8_t reversed = 0U;

    for (i = 0U; i < LINE_DETECT_SENSOR_NUM; i++) {
        if ((mask & (uint8_t)(1U << i)) != 0U) {
            reversed |= (uint8_t)(1U <<
                                  (LINE_DETECT_SENSOR_NUM - 1U - i));
        }
    }
    return reversed;
#else
    return mask;
#endif
}

static int16_t Line_NormalizeError(int16_t error)
{
#if CONTROL_LINE_DIRECTION_REVERSE
    return (int16_t)(-error);
#else
    return error;
#endif
}

static uint8_t Line_IsBlack(uint16_t raw, uint16_t threshold)
{
#if LINE_DETECT_BLACK_IS_HIGH
    return (raw > threshold) ? 1U : 0U;
#else
    return (raw < threshold) ? 1U : 0U;
#endif
}

static uint16_t Line_MakeOneThreshold(uint16_t white, uint16_t black, uint16_t default_threshold)
{
    uint16_t high;
    uint16_t low;

    if (white > black) {
        high = white;
        low = black;
    } else {
        high = black;
        low = white;
    }

    if ((uint16_t)(high - low) < LINE_DETECT_MIN_DIFF) {
        return default_threshold;
    }

    return (uint16_t)(((uint32_t)white + (uint32_t)black) / 2U);
}

static LineType_t Line_Classify(uint8_t mask, uint8_t count)
{
    uint8_t left_count = 0U;
    uint8_t mid_count = 0U;
    uint8_t right_count = 0U;

    if (count == 0U) return LINE_TYPE_LOST;
    if (count >= 8U) return LINE_TYPE_FULL_BLACK;
    if (count >= 6U) return LINE_TYPE_CROSS;

    if (mask & 0x01U) left_count++;
    if (mask & 0x02U) left_count++;
    if (mask & 0x04U) left_count++;

    if (mask & 0x08U) mid_count++;
    if (mask & 0x10U) mid_count++;

    if (mask & 0x20U) right_count++;
    if (mask & 0x40U) right_count++;
    if (mask & 0x80U) right_count++;

    if ((left_count >= 2U) && (mid_count >= 1U)) return LINE_TYPE_LEFT_BRANCH;
    if ((right_count >= 2U) && (mid_count >= 1U)) return LINE_TYPE_RIGHT_BRANCH;

    return LINE_TYPE_SINGLE;
}

void LineDetect_Init(void)
{
    uint8_t i;
    static const uint16_t default_threshold[LINE_DETECT_SENSOR_NUM] = LINE_DETECT_DEFAULT_THRESHOLD_ARRAY;

    for (i = 0U; i < LINE_DETECT_SENSOR_NUM; i++) {
        s_line.raw[i] = 0U;
        s_line.threshold[i] = default_threshold[i];
        s_line.black[i] = 0U;
        s_white[i] = 0U;
        s_black[i] = 0U;
    }

    s_line.black_mask = 0U;
    s_line.black_count = 0U;
    s_line.error_x1000 = 0;
    s_line.last_error_x1000 = 0;
    s_line.type = LINE_TYPE_LOST;
    s_has_white = 0U;
    s_has_black = 0U;
}

void LineDetect_Update(const uint16_t raw[LINE_DETECT_SENSOR_NUM])
{
    uint8_t i;
    uint8_t is_black;
    int32_t weighted_sum = 0;
    int32_t count = 0;
    uint8_t mask = 0U;
    uint8_t normalized_mask;
    int16_t normalized_error;

    if (raw == 0) return;

    for (i = 0U; i < LINE_DETECT_SENSOR_NUM; i++) {
        s_line.raw[i] = raw[i];
        is_black = Line_IsBlack(raw[i], s_line.threshold[i]);
        s_line.black[i] = is_black;
        if (is_black) {
            mask |= (uint8_t)(1U << i);
            weighted_sum += s_weight[i];
            count++;
        }
    }

    normalized_mask = Line_NormalizeMask(mask);
    s_line.black_mask = normalized_mask;
    s_line.black_count = (uint8_t)count;

    if (count > 0) {
        normalized_error = Line_NormalizeError(
            (int16_t)(weighted_sum / count));
        s_line.last_error_x1000 = s_line.error_x1000;
        s_line.error_x1000 = normalized_error;
    } else {
        /* 丢线时保持最后方向，便于 line_track 做找线。 */
        s_line.last_error_x1000 = s_line.error_x1000;
    }

    s_line.type = Line_Classify(normalized_mask, (uint8_t)count);
}

BSP_Status_t LineDetect_GetResult(LineDetect_Result_t *out_result)
{
    if (out_result == 0) return BSP_PARAM;
    *out_result = s_line;
    return BSP_OK;
}

const LineDetect_Result_t *LineDetect_GetResultPtr(void)
{
    return &s_line;
}

void LineDetect_SetThreshold(uint8_t index, uint16_t threshold)
{
    if (index >= LINE_DETECT_SENSOR_NUM) return;
    s_line.threshold[index] = threshold;
}

void LineDetect_SetAllThreshold(uint16_t threshold)
{
    uint8_t i;
    for (i = 0U; i < LINE_DETECT_SENSOR_NUM; i++) {
        s_line.threshold[i] = threshold;
    }
}

BSP_Status_t LineDetect_GetThresholdArray(uint16_t *out_threshold, uint8_t max_count)
{
    uint8_t i;
    uint8_t n;

    if (out_threshold == 0) return BSP_PARAM;
    n = (max_count < LINE_DETECT_SENSOR_NUM) ? max_count : LINE_DETECT_SENSOR_NUM;
    for (i = 0U; i < n; i++) {
        out_threshold[i] = s_line.threshold[i];
    }
    return BSP_OK;
}

void LineDetect_CaptureWhite(const uint16_t raw[LINE_DETECT_SENSOR_NUM])
{
    uint8_t i;
    if (raw == 0) return;
    for (i = 0U; i < LINE_DETECT_SENSOR_NUM; i++) {
        s_white[i] = raw[i];
    }
    s_has_white = 1U;
}

void LineDetect_CaptureBlack(const uint16_t raw[LINE_DETECT_SENSOR_NUM])
{
    uint8_t i;
    if (raw == 0) return;
    for (i = 0U; i < LINE_DETECT_SENSOR_NUM; i++) {
        s_black[i] = raw[i];
    }
    s_has_black = 1U;
}

void LineDetect_MakeThresholdFromWhiteBlack(void)
{
    uint8_t i;
    if ((s_has_white == 0U) || (s_has_black == 0U)) return;

    for (i = 0U; i < LINE_DETECT_SENSOR_NUM; i++) {
        s_line.threshold[i] = Line_MakeOneThreshold(s_white[i], s_black[i], s_line.threshold[i]);
    }
}
