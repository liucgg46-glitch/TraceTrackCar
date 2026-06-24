#ifndef __SENSOR_MANAGER_H
#define __SENSOR_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 传感器任务胶水层。
 * 当前只调用灰度 4051 Driver 的非阻塞扫描，后续 IMU/ToF/视觉也可在这里统一调度。
 */
void SensorManager_Init(void);
void Sensor_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_MANAGER_H */
