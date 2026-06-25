#ifndef __ROUTE_CONFIG_H
#define __ROUTE_CONFIG_H

#define ROUTE_PROFILE_BASIC      0
#define ROUTE_PROFILE_HJDUINO    1

/* 第一次先选 BASIC，保证新架构接入后功能和原来一样 */
#define ROUTE_PROFILE_SELECT     ROUTE_PROFILE_BASIC

#endif