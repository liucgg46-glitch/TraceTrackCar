#include "task_profile_select.h"
#include "task_profile_config.h"
#include "route_config.h"

#if (TASK_PROFILE_SELECT == TASK_PROFILE_NONE)
/* 不启用总任务状态机时无需包含具体实现。 */
#elif (TASK_PROFILE_SELECT == TASK_PROFILE_MEDICINE)
#include "task_fsm.h"
#if (ROUTE_PROFILE_SELECT != ROUTE_PROFILE_MEDICINE)
#error "TASK_PROFILE_MEDICINE requires ROUTE_PROFILE_MEDICINE"
#endif
#else
#error "Invalid TASK_PROFILE_SELECT: add the selected adapter in task_profile_select.c"
#endif

void TaskProfile_Init(void)
{
#if (TASK_PROFILE_SELECT == TASK_PROFILE_NONE)
    /* 保持底盘初始化后的安全停止状态。 */
#elif (TASK_PROFILE_SELECT == TASK_PROFILE_MEDICINE)
    TaskFSM_Init();
#endif
}

void TaskProfile_Reset(void)
{
#if (TASK_PROFILE_SELECT == TASK_PROFILE_NONE)
    /* 未选择状态机时没有内部状态需要复位。 */
#elif (TASK_PROFILE_SELECT == TASK_PROFILE_MEDICINE)
    TaskFSM_Reset();
#endif
}

void TaskProfile_Update(void)
{
#if (TASK_PROFILE_SELECT == TASK_PROFILE_NONE)
    /* 未选择状态机时周期任务为空操作。 */
#elif (TASK_PROFILE_SELECT == TASK_PROFILE_MEDICINE)
    TaskFSM_Update();
#endif
}

uint8_t TaskProfile_GetSelected(void)
{
    return (uint8_t)TASK_PROFILE_SELECT;
}
