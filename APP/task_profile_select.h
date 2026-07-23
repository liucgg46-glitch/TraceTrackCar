#ifndef __TASK_PROFILE_SELECT_H
#define __TASK_PROFILE_SELECT_H

#include "bsp_common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 总任务状态机适配层。
 * App_Init和调度任务只调用TaskProfile_*统一接口，不直接依赖具体状态机。
 * 新增状态机时在task_profile_config.h分配编号，并在对应.c文件增加选择分支。
 */
void TaskProfile_Init(void);
void TaskProfile_Reset(void);
void TaskProfile_Update(void);
uint8_t TaskProfile_GetSelected(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_PROFILE_SELECT_H */
