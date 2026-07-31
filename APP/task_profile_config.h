#ifndef __TASK_PROFILE_CONFIG_H
#define __TASK_PROFILE_CONFIG_H

/*
 * 总任务状态机方案编号和编译期选择入口。
 * 新增方案时先分配唯一编号，再在task_profile_select.c中接入实现。
 */
#define TASK_PROFILE_NONE                         0U
#define TASK_PROFILE_MEDICINE                     1U
#define TASK_PROFILE_B_BASIC                      2U
#define TASK_PROFILE_H2_ROUND_STOP                3U

/*
 * H题使用同一个顶层状态机，比赛项目只在该状态机内部选择子流程。
 * 当前分支用于第3项静止滚球顺序，禁止再注册第二个并行比赛状态机。
 */
#define H_COMPETITION_ITEM_ROUND_STOP              2U
#define H_COMPETITION_ITEM_BALL_SEQUENCE           3U
#define H_COMPETITION_ITEM_SELECT                  \
    H_COMPETITION_ITEM_BALL_SEQUENCE

#define TASK_PROFILE_SELECT                       TASK_PROFILE_H2_ROUND_STOP

#endif /* __TASK_PROFILE_CONFIG_H */
