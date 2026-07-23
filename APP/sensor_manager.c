#include "sensor_manager.h"

#include "drv_gray_sensor.h"
#include "drv_vl53l1x.h"
#include "drv_icm20948.h"
#include "drv_hx711.h"
#include "drv_motor.h"
#include "attitude_estimator.h"

static uint8_t SensorManager_IsMotorActive(void)
{
    int16_t motor_pwm[MOTOR_COUNT];
    uint8_t motor;

    if (Motor_GetAllLastPermille(motor_pwm) != BSP_OK) {
        /* 无法确认电机状态时按活动处理，优先屏蔽可能受干扰的磁力计。 */
        return 1U;
    }

    for (motor = 0U; motor < (uint8_t)MOTOR_COUNT; motor++) {
        int16_t pwm = motor_pwm[motor];
        int16_t abs_pwm = (pwm < 0) ? (int16_t)(-pwm) : pwm;

        if (abs_pwm >= SENSOR_ATTITUDE_MOTOR_ACTIVE_MIN_PERMILLE) {
            return 1U;
        }
    }
    return 0U;
}

static BSP_Status_t SensorManager_UpdateAttitude(void)
{
    Drv_ICM20948_Data_t driver_data;
    Attitude_Input_t input;

    if (Drv_ICM20948_GetData(&driver_data) != BSP_OK) {
        Attitude_Invalidate();
        return BSP_ERROR;
    }

    input.accel_filtered_g.x = driver_data.accel_filtered_g.x;
    input.accel_filtered_g.y = driver_data.accel_filtered_g.y;
    input.accel_filtered_g.z = driver_data.accel_filtered_g.z;
    input.gyro_filtered_dps.x = driver_data.gyro_filtered_dps.x;
    input.gyro_filtered_dps.y = driver_data.gyro_filtered_dps.y;
    input.gyro_filtered_dps.z = driver_data.gyro_filtered_dps.z;
    input.mag_uT.x = driver_data.mag_uT.x;
    input.mag_uT.y = driver_data.mag_uT.y;
    input.mag_uT.z = driver_data.mag_uT.z;
    input.mag_filtered_uT.x = driver_data.mag_filtered_uT.x;
    input.mag_filtered_uT.y = driver_data.mag_filtered_uT.y;
    input.mag_filtered_uT.z = driver_data.mag_filtered_uT.z;
    input.timestamp_ms = driver_data.timestamp_ms;
    input.mag_valid = driver_data.mag_valid;
    input.mag_updated = driver_data.mag_updated;

    return Attitude_Update(&input, SensorManager_IsMotorActive());
}

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
    /*
     * 姿态层按 IMU timestamp 去重：虽然本函数每 1 ms 调用，只有约 102 Hz 的
     * 新样本会真正执行一次融合。重复样本返回 BSP_BUSY，不会重复积分。
     */
    (void)SensorManager_UpdateAttitude();
    (void)Drv_VL53L1X_Update();
    (void)Drv_GraySensor_Update();
    (void)Drv_HX711_Update();
}

uint8_t Sensor_IsImuReadyForMotion(void)
{
    Drv_ICM20948_Info_t info;

    if (Drv_ICM20948_GetInfo(&info) != BSP_OK) {
        return 0U;
    }
    if ((info.initialized == 0U) ||
        (info.running == 0U) ||
        (info.calibrating != 0U) ||
        (info.data_valid == 0U) ||
        (Attitude_IsValid() == 0U)) {
        return 0U;
    }
#if (DRV_ICM20948_GYRO_CAL_ENABLE != 0U)
    if (info.gyro_cal_samples < DRV_ICM20948_GYRO_CAL_SAMPLE_COUNT) {
        return 0U;
    }
#endif
    return 1U;
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

BSP_Status_t Sensor_GetPressureGram(float *pressure_g)
{
    if (pressure_g == 0) {
        return BSP_PARAM;
    }
    return Drv_HX711_GetGram(pressure_g);
}

BSP_Status_t Sensor_PressureTare(void)
{
    return Drv_HX711_Tare();
}

BSP_Status_t Sensor_CalibratePressure(float known_weight_g)
{
    return Drv_HX711_CalibrateKnownWeight(known_weight_g);
}


BSP_Status_t Sensor_GetAttitude(Sensor_Attitude_t *attitude)
{
    Attitude_Info_t info;
    BSP_Status_t status;

    /*
     * Attitude_GetInfo() 内部以临界区一次性复制完整缓存，三个角度和状态
     * 来自同一融合时间戳。本接口不会访问 SPI 或重复推进融合器。
     */
    if (attitude == 0) {
        return BSP_PARAM;
    }

    status = Attitude_GetInfo(&info);
    if (status != BSP_OK) {
        return status;
    }

    attitude->roll_deg = info.roll_deg;
    attitude->pitch_deg = info.pitch_deg;
    attitude->yaw_deg = info.yaw_deg;
    attitude->timestamp_ms = info.timestamp_ms;
    attitude->stationary = info.stationary;
    attitude->mag_calibrated = info.mag_calibrated;
    attitude->mag_healthy = info.mag_healthy;
    attitude->mag_used = info.mag_used;
    return BSP_OK;
}
