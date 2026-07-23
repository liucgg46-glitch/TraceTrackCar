#ifndef __PROJECT_CRITICAL_H
#define __PROJECT_CRITICAL_H

#include <stdint.h>

/*
 * 算法层只依赖临界区抽象，目标板实现位于BSP，主机测试提供空操作实现。
 * 返回值必须原样传给退出函数，以恢复进入前的中断状态。
 */
uint32_t Project_EnterCritical(void);
void Project_ExitCritical(uint32_t state);

#endif /* __PROJECT_CRITICAL_H */
