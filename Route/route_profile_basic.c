#include "route_profile_basic.h"
#include "line_track.h"

/* 初始化时直接执行一次复位，便于以后给 Basic profile 增加状态。 */
void BasicRoute_Init(void)
{
    BasicRoute_Reset();
}

/* 当前 Basic profile 没有静态状态变量，因此这里暂时不需要做任何事情。 */
void BasicRoute_Reset(void)
{
}

/*
 * 最简单的路线方案：
 *   1. 检查输入/输出指针；
 *   2. 把线路识别结果交给普通循迹算法；
 *   3. 根据 out->valid 告诉上层应循迹还是停止。
 */
Route_ControlMode_t BasicRoute_Update(const LineDetect_Result_t *line,
                                      LineTrack_Output_t *out)
{
    if ((line == 0) || (out == 0)) {
        return ROUTE_CONTROL_ERROR;
    }

    LineTrack_Compute(line, out);
    return (out->valid != 0U) ? ROUTE_CONTROL_LINE_TRACK : ROUTE_CONTROL_STOP;
}
