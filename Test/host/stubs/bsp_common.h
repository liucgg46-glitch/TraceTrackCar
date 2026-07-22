#ifndef __HOST_TEST_BSP_COMMON_H
#define __HOST_TEST_BSP_COMMON_H

#include <stdint.h>

/*
 * 姿态算法主机测试只需要状态码和临界区接口。
 * 这里不包含任何 STM32 头文件，保证测试编译不会依赖芯片 SDK。
 */
typedef enum {
    BSP_OK = 0,
    BSP_ERROR,
    BSP_BUSY,
    BSP_TIMEOUT,
    BSP_PARAM
} BSP_Status_t;

static __inline uint32_t BSP_EnterCritical(void)
{
    return 0U;
}

static __inline void BSP_ExitCritical(uint32_t primask)
{
    (void)primask;
}

#endif /* __HOST_TEST_BSP_COMMON_H */
