#ifndef __PROJECT_BUILD_CONFIG_H
#define __PROJECT_BUILD_CONFIG_H

/*
 * 全工程构建模式公共开关。
 * 正式固件必须保持为0U；专项测试可由编译命令临时覆盖为1U。
 */
#ifndef PROJECT_TEST_TASKS_ENABLE
#define PROJECT_TEST_TASKS_ENABLE 0U
#endif

#if ((PROJECT_TEST_TASKS_ENABLE != 0U) && \
     (PROJECT_TEST_TASKS_ENABLE != 1U))
#error "PROJECT_TEST_TASKS_ENABLE must be 0U or 1U"
#endif

#endif /* __PROJECT_BUILD_CONFIG_H */
