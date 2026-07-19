#ifndef __ROUTE_CONFIG_H
#define __ROUTE_CONFIG_H

/* Compile-time route profile selection. */
#define ROUTE_PROFILE_BASIC                         0U
#define ROUTE_PROFILE_HJDUINO                       1U
/* 当前版本固定使用基础循迹，不启用右侧环岛定角转弯。 */
#define ROUTE_PROFILE_SELECT                        ROUTE_PROFILE_BASIC

/*
 * HJduino track-specific event parameters. Vehicle control gains and speeds
 * are all in Algorithm/control_config.h.
 */
#define HJDUINO_ROUTE_START_GUARD_MS                3000U
#define HJDUINO_ROUTE_ENTRY_CONFIRM_SAMPLES         3U
#define HJDUINO_ROUTE_ENTRY_TURN_ANGLE_DEG          (-90)

#endif /* __ROUTE_CONFIG_H */
