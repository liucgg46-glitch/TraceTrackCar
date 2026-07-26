#ifndef __TASK_PROFILE_CONFIG_H
#define __TASK_PROFILE_CONFIG_H

/*
 * 总任务状态机方案编号和编译期选择入口。
 * 新增方案时先分配唯一编号，再在task_profile_select.c中接入实现。
 */
#define TASK_PROFILE_NONE                         0U
#define TASK_PROFILE_MEDICINE                     1U
#define TASK_PROFILE_B_BASIC                      2U

/* 当前使用2026通信系电赛模拟赛B题基础巡线正式任务状态机。 */
#define TASK_PROFILE_SELECT                       TASK_PROFILE_B_BASIC

#endif /* __TASK_PROFILE_CONFIG_H */