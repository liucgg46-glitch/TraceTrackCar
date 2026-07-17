
#ifndef __ROUTE_CONFIG_H
#define __ROUTE_CONFIG_H

/*
 * 路线方案编号。
 * BASIC：不识别特殊赛道，只执行普通循迹。
 * HJDUINO：用于当前 HJduino 赛道的专用状态机。
 */
#define ROUTE_PROFILE_BASIC      0
#define ROUTE_PROFILE_HJDUINO    1

/*
 * 当前编译进工程的路线方案。
 * 这里选择 HJDUINO，因此 route_manager 会调用 HJduinoRoute_* 系列函数。
 * 改为 ROUTE_PROFILE_BASIC 后，编译器会改用 BasicRoute_* 系列函数。
 */
#define ROUTE_PROFILE_SELECT     ROUTE_PROFILE_HJDUINO

/* HJduino route parameters. Route_Update is currently scheduled every 10 ms. */
/*
 * 启动保护时间：启动后的前 3000ms 内，即使看到入口特征也不触发转弯。
 * 用途是避免小车刚放下、起跑线或初始黑块被误判为环形入口。
 */
#define HJDUINO_ROUTE_START_GUARD_MS             3000U

/*
 * 入口连续确认次数：入口特征必须连续出现 3 次才确认。
 * 如果 Route_Update 每 10ms 调用一次，则理论确认时间约为 30ms。
 */
#define HJDUINO_ROUTE_ENTRY_CONFIRM_SAMPLES      3U

/*
 * 进入环形路线时执行的相对转角。
 * motion_action 中约定 turn > 0 为左转、turn < 0 为右转，
 * 因此 -90 表示右转约 90°。
 */
#define HJDUINO_ROUTE_ENTRY_TURN_ANGLE_DEG       (-90)

/* 定角转弯时的转向速度，单位 cps。 */
#define HJDUINO_ROUTE_ENTRY_TURN_SPEED_CPS       1600

#endif
