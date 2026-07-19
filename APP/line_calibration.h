#ifndef __LINE_CALIBRATION_H
#define __LINE_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 灰度传感器专用校准任务。
 * 任务表必须先注册 Key_Update() 和 Sensor_Update()：
 *   KEY1：采集白底；
 *   KEY2：采集黑底；
 *   KEY3：计算八路阈值并显示/打印结果。
 *
 * 只有周期调用本函数时，LCD/OLED 才会进入灰度校准页面。
 */
void LineCalibration_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __LINE_CALIBRATION_H */
