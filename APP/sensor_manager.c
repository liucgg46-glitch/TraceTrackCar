#include "sensor_manager.h"

#include "drv_gray_sensor.h"
#include "drv_vl53l1x.h"

void SensorManager_Init(void)
{
    /*
     * Driver_Init() 已经完成灰度传感器和 VL53L1X 的初始化。
     * 这里不重复操作硬件，避免同一个驱动被重复初始化。
     */
}

void Sensor_Update(void)
{
    /*
     * VL53L1X 与感为灰度传感器当前共用 I2C1。
     * 两个驱动均采用非阻塞方式，并会在总线忙时等待下一轮调度。
     *
     * 先推进 VL53L1X 的短事务，再推进灰度传感器读取；如果本轮总线
     * 已被占用，相应驱动会返回 BSP_BUSY，并在下一次 1 ms 调度时重试。
     */
    (void)Drv_VL53L1X_Update();
    (void)Drv_GraySensor_Update();
}

BSP_Status_t Sensor_GetFrontDistanceMm(uint16_t *distance_mm)
{
    /* 明确检查上层传入的输出指针，避免空指针写入。 */
    if (distance_mm == 0) {
        return BSP_PARAM;
    }

    /*
     * Drv_VL53L1X_GetDistanceMm() 只在以下条件满足时返回 BSP_OK：
     *   1. VL53L1X 当前在线；
     *   2. 当前缓存中存在通过状态检查的有效测距结果。
     *
     * 该接口不会重新访问 I2C，也不会重复推进 VL53L1X 状态机。
     */
    return Drv_VL53L1X_GetDistanceMm(distance_mm);
}
