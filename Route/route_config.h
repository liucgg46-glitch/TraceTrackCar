#ifndef __ROUTE_CONFIG_H
#define __ROUTE_CONFIG_H

/*
 * 赛道方案编号和编译期选择入口。
 * 新增赛道时先分配唯一编号，再在 route_profile_select.c 中接入实现。
 */
#define ROUTE_PROFILE_BASIC                         0U
#define ROUTE_PROFILE_MEDICINE                      1U
#define ROUTE_PROFILE_B_BASIC                       2U
#define ROUTE_PROFILE_H_OVAL                        3U

/* 当前使用2026年电赛H题顺时针椭圆赛道。 */
#define ROUTE_PROFILE_SELECT                        ROUTE_PROFILE_H_OVAL

#endif /* __ROUTE_CONFIG_H */
