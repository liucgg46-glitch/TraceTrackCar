#include "scheduler.h"

#include "project_build_config.h"

#if (PROJECT_TEST_TASKS_ENABLE != 0U)
#include "test_task_config.h"
#else
#include "app_task_config.h"
#endif

/*
 * 正式任务表只由APP配置生成；专项测试任务表只存在于Test目录。
 * 调度器本身不承载具体业务任务选择。
 */
#if (PROJECT_TEST_TASKS_ENABLE != 0U)
TEST_SCHEDULER_TASK_LIST_DEFINE();
#else
APP_SCHEDULER_TASK_LIST_DEFINE();
#endif

void Scheduler_Init(void)
{
    uint8_t i;
    uint32_t now = BSP_GetTickMs();

    for (i = 0U; i < TASK_NUM; i++) {
        task_list[i].last_run_ms = now;
    }
}

void Scheduler_Run(void)
{
    uint8_t i;
    uint32_t now = BSP_GetTickMs();

    for (i = 0U; i < TASK_NUM; i++) {
        if ((task_list[i].task_func == 0) || (task_list[i].period_ms == 0U)) {
            continue;
        }

        if ((uint32_t)(now - task_list[i].last_run_ms) >= task_list[i].period_ms) {
            task_list[i].last_run_ms = now;
            task_list[i].task_func();
        }
    }
}

uint8_t Scheduler_GetTaskCount(void)
{
    return TASK_NUM;
}

void Scheduler_ResetTaskTime(uint8_t index)
{
    if (index >= TASK_NUM) {
        return;
    }

    task_list[index].last_run_ms = BSP_GetTickMs();
}
