#ifndef __TEST_CONFIG_H
#define __TEST_CONFIG_H

/*
 * 专项测试代码编译开关。
 *
 * 正式固件保持为 0U，此时 Test/test.c 不生成任何测试任务代码，正式任务表
 * 也不依赖 Test/test.h。需要执行分项测试时改为 1U，并按测试任务文档切换
 * APP/app_task_config.h 中的任务表；测试完成后应恢复为 0U。
 */
#ifndef PROJECT_TEST_TASKS_ENABLE
#define PROJECT_TEST_TASKS_ENABLE 1U
#endif

#if ((PROJECT_TEST_TASKS_ENABLE != 0U) && \
     (PROJECT_TEST_TASKS_ENABLE != 1U))
#error "PROJECT_TEST_TASKS_ENABLE must be 0U or 1U"
#endif

#endif /* __TEST_CONFIG_H */
