#include "route_common.h"

/*
 * 统计 mask 中二进制 1 的数量。
 * 例如：mask = 0x19 = 00011001b，其中有 3 个 1，函数返回 3。
 *
 * 算法过程：
 *   1. 检查最低位是否为 1；
 *   2. mask 右移一位；
 *   3. 一直重复到 mask 变成 0。
 */
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

/*
 * 判断当前灰度图案是否属于“路口/大面积黑线”一类。
 *
 * 满足下面任意条件就返回 1：
 *   1. line_detect 已分类为 CROSS、FULL_BLACK、LEFT_BRANCH 或 RIGHT_BRANCH；
 *   2. 即使分类没有命中，但 8 路中至少 5 路为黑，也按路口处理。
 *
 * 第二条属于兜底判断，可降低分类边界附近漏判路口的概率。
 */
uint8_t Route_IsCrossLike(const LineDetect_Result_t *line)
{
    uint8_t cnt;

    /* 防止空指针解引用。 */
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

/*
 * 判断是否为“稳定的普通单线”。
 * 要求 line_detect 的类型为 LINE_TYPE_SINGLE，且黑色探头数量为 1~4。
 * 该函数适合用在“确认已经离开路口、重新回到普通线路”之类的状态切换中。
 */
uint8_t Route_IsStableSingleLine(const LineDetect_Result_t *line)
{
    uint8_t cnt;

    if (line == 0) return 0U;

    cnt = Route_CountBlackBits(line->black_mask);

    if ((line->type == LINE_TYPE_SINGLE) &&
        (cnt >= 1U) &&
        (cnt <= 4U)) {
        return 1U;
    }

    return 0U;
}

/*
 * 检查左侧边缘探头。
 * 0x03 = 00000011b，对应 black_mask 的 bit0 和 bit1。
 * 只要其中任意一位为 1，就说明最左侧区域检测到黑线。
 */
uint8_t Route_IsLeftEdge(const LineDetect_Result_t *line)
{
    if (line == 0) return 0U;

    if ((line->black_mask & 0x03U) != 0U) {
        return 1U;
    }

    return 0U;
}

/*
 * 检查右侧边缘探头。
 * 0xC0 = 11000000b，对应 black_mask 的 bit6 和 bit7。
 */
uint8_t Route_IsRightEdge(const LineDetect_Result_t *line)
{
    if (line == 0) return 0U;

    if ((line->black_mask & 0xC0U) != 0U) {
        return 1U;
    }

    return 0U;
}

/*
 * 检查中间区域是否仍压在线上。
 * 0x3C = 00111100b，对应第 2、3、4、5 路探头。
 * 只要中间四路中任意一路为黑，就返回 1。
 */
uint8_t Route_IsMiddleOnLine(const LineDetect_Result_t *line)
{
    if (line == 0) return 0U;

    /* 原文件此处注释编码损坏；实际判断的是中间第 2~5 路探头。 */
    if ((line->black_mask & 0x3CU) != 0U) {
        return 1U;
    }

    return 0U;
}
