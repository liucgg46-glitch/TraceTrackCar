#ifndef __ROUTE_PROFILE_SELECT_H
#define __ROUTE_PROFILE_SELECT_H

#include "route_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 赛道实现适配层。
 * RouteManager 只调用 RouteProfile_* 统一接口，不直接依赖具体赛道文件。
 * 新增赛道时在 route_config.h 分配编号，并在对应 .c 文件增加选择分支。
 */
void RouteProfile_Init(void);
void RouteProfile_Reset(void);
Route_ControlMode_t RouteProfile_Update(
    const LineDetect_Result_t *line,
    const Route_ActionFeedback_t *feedback,
    LineTrack_Output_t *out,
    Route_ActionRequest_t *request);
BSP_Status_t RouteProfile_GetInfo(RouteProfile_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __ROUTE_PROFILE_SELECT_H */
