#ifndef __APP_ALL_H
#define __APP_ALL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * App/控制层统一初始化入口。
 * 当前包含底盘、里程、姿态、动作、传感器、循迹、显示、K210和送药状态机。
 */
void App_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ALL_H */
