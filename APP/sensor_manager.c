#include "sensor_manager.h"

#include "drv_gray_sensor.h"
#include "drv_vl53l1x.h"
#include "drv_icm20948.h"

void SensorManager_Init(void)
{
    /*
     * Driver_Init() 已经完成灰度传感器、VL53L1X 和 ICM-20948 的初始化。
     * 这里不重复操作硬件，避免同一个驱动被重复初始化。
     */
}

void Sensor_Update(void)
{
    /*
     * ICM-20948 使用 SPI2；VL53L1X 与感为灰度传感器共用 I2C1。
     * 三个驱动都采用状态机方式推进。ICM-20948 与 LCD 共用 SPI2 时，
     * 会在 LCD DMA 占用总线时主动跳过本轮，下一次 1 ms 调度再重试。
     *
     * 上层 APP/Route 只读取缓存，不要再次直接调用各驱动的 Update()。
     */
    (void)Drv_ICM20948_Update();
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


BSP_Status_t Sensor_GetImuData(Drv_ICM20948_Data_t *data)
{
    /*
     * 只复制 ICM-20948 驱动内部已经缓存并通过有效性检查的数据。
     * 本接口不会启动 SPI 事务，也不会重复推进 IMU 状态机。
     */
    if (data == 0) {
        return BSP_PARAM;
    }

    return Drv_ICM20948_GetData(data);
}
