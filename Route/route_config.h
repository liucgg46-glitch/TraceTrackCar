#ifndef __ROUTE_CONFIG_H
#define __ROUTE_CONFIG_H

/*
 * 赛道方案编号和编译期选择入口。
 * 新增赛道时先分配唯一编号，再在 route_profile_select.c 中接入实现。
 */
#define ROUTE_PROFILE_BASIC                         0U
#define ROUTE_PROFILE_MEDICINE                      1U
#define ROUTE_PROFILE_B_BASIC                       2U

/* 当前使用2026通信系电赛模拟赛B题基础巡线赛道。 */
#define ROUTE_PROFILE_SELECT                        ROUTE_PROFILE_BASIC

#endif /* __ROUTE_CONFIG_H */
