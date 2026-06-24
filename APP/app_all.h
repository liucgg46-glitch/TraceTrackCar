#ifndef __APP_ALL_H
#define __APP_ALL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * App/控制层统一初始化入口。
 * 当前包含：Chassis、Odometer、AngleControl、Motion。
 */
void App_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ALL_H */
