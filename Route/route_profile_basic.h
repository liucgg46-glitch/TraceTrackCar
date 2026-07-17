#ifndef __ROUTE_PROFILE_BASIC_H
#define __ROUTE_PROFILE_BASIC_H

#include "route_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 基础路线方案初始化。 */
void BasicRoute_Init(void);

/* 基础路线方案复位；当前方案没有内部状态，因此函数体为空。 */
void BasicRoute_Reset(void);

/*
 * 基础路线周期更新：不识别特殊路口，只调用普通 LineTrack_Compute()。
 */
Route_ControlMode_t BasicRoute_Update(const LineDetect_Result_t *line,
                                      LineTrack_Output_t *out);

#ifdef __cplusplus
}
#endif

#endif
