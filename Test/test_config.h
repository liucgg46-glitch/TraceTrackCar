#ifndef __TEST_CONFIG_H
#define __TEST_CONFIG_H

/*
 * 专项测试代码编译开关。
 *
 * 本次交付暂设为1U，烧录后直接进入PF8摆杆舵机安全标定。
 * 标定完成后恢复为0U，即重新使用H题第2项正式任务表。
 */
#ifndef PROJECT_TEST_TASKS_ENABLE
#define PROJECT_TEST_TASKS_ENABLE 1U
#endif

#if ((PROJECT_TEST_TASKS_ENABLE != 0U) && \
     (PROJECT_TEST_TASKS_ENABLE != 1U))
#error "PROJECT_TEST_TASKS_ENABLE must be 0U or 1U"
#endif

#endif /* __TEST_CONFIG_H */
