#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include <stdint.h>
#include "bsp_systick.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 简易非阻塞静态任务调度器
 * ============================================================================
 * 定位：按周期调用各模块 Update，不负责具体业务逻辑。
 *
 * 为什么改成静态 task_list[]：
 *   1. 和赛前流程文档中的 Task_t task_list[] 模板一致；
 *   2. A/B/C 三个人的 Update 接口固定，先定接口再写内部实现；
 *   3. main.c 不再到处 Scheduler_AddTask，避免对接时漏加任务；
 *   4. 换项目时只改 app_task_config.h，不改 scheduler.c。
 *
 * 注意：所有 task_func 内部必须非阻塞，不能 delay，不能 while 等待事件。
 */

typedef void (*Scheduler_TaskFunc_t)(void);

typedef struct {
    Scheduler_TaskFunc_t task_func;   /* 任务函数，一般命名为 Module_Update */
    uint32_t period_ms;               /* 调度周期，单位 ms */
    uint32_t last_run_ms;             /* 上次运行时刻，由调度器维护，初始化填 0 */
} Task_t;

/*
 * task_list[] 和 TASK_NUM 在 app_task_config.h 中通过宏生成。
 * 用户需要改调度周期/任务顺序时，只改 app_task_config.h。
 */
extern Task_t task_list[];
extern const uint8_t TASK_NUM;

void     Scheduler_Init(void);
void     Scheduler_Run(void);
uint8_t  Scheduler_GetTaskCount(void);
void     Scheduler_ResetTaskTime(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif /* __SCHEDULER_H */
