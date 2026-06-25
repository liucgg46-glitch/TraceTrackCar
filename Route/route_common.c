#include "route_common.h"

uint8_t Route_CountBlackBits(uint8_t mask)
{
    uint8_t cnt = 0U;

    while (mask != 0U) {
        if ((mask & 0x01U) != 0U) {
            cnt++;
        }
        mask >>= 1;
    }

    return cnt;
}

uint8_t Route_IsCrossLike(const LineDetect_Result_t *line)
{
    uint8_t cnt;

    if (line == 0) return 0U;

    cnt = Route_CountBlackBits(line->black_mask);

    if ((line->type == LINE_TYPE_CROSS) ||
        (line->type == LINE_TYPE_FULL_BLACK) ||
        (line->type == LINE_TYPE_LEFT_BRANCH) ||
        (line->type == LINE_TYPE_RIGHT_BRANCH)) {
        return 1U;
    }

    if (cnt >= 5U) {
        return 1U;
    }

    return 0U;
}

uint8_t Route_IsStableSingleLine(const LineDetect_Result_t *line)
{
    uint8_t cnt;

    if (line == 0) return 0U;

    cnt = Route_CountBlackBits(line->black_mask);

    if ((line->type == LINE_TYPE_NORMAL) &&
        (cnt >= 1U) &&
        (cnt <= 4U)) {
        return 1U;
    }

    return 0U;
}

uint8_t Route_IsLeftEdge(const LineDetect_Result_t *line)
{
    if (line == 0) return 0U;

    if ((line->black_mask & 0x03U) != 0U) {
        return 1U;
    }

    return 0U;
}

uint8_t Route_IsRightEdge(const LineDetect_Result_t *line)
{
    if (line == 0) return 0U;

    if ((line->black_mask & 0xC0U) != 0U) {
        return 1U;
    }

    return 0U;
}

uint8_t Route_IsMiddleOnLine(const LineDetect_Result_t *line)
{
    if (line == 0) return 0U;

    /* 中间 2、3、4、5 路压线，认为重新抓到主线 */
    if ((line->black_mask & 0x3CU) != 0U) {
        return 1U;
    }

    return 0U;
}