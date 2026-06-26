#include "scheduler.h"
#include "app_task_config.h"

/*
 * Generate task_list[] and TASK_NUM from app_task_config.h.
 * Keep scheduler logic here generic; edit the task table in app_task_config.h.
 */
APP_SCHEDULER_TASK_LIST_DEFINE();

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
