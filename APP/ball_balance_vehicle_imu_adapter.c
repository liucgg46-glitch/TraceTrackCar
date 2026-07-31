#include "ball_balance_vehicle_imu_adapter.h"

#include "ball_balance_app.h"
#include "bsp_systick.h"

/*
 * 当前SensorManager只公开融合姿态角，没有公开与该姿态同时间戳的
 * 去重力线性加速度。这里明确输出无效，避免把原始加速度或重力分量
 * 伪装成沿摆杆方向的扰动。后续接入可靠缓存时只需在本适配层完成
 * 轴映射、符号和低通滤波，不修改Algorithm。
 */
static BallBalance_VehicleImuAdapterInfo_t s_info;

void BallBalance_VehicleImuAdapter_Init(void)
{
    s_info.initialized = 1U;
    s_info.source_available = 0U;
    s_info.valid = 0U;
    s_info.disturbance_mm_s2 = 0.0f;
    s_info.timestamp_ms = 0U;
    s_info.invalid_count = 0U;
    BallBalance_App_SetVehicleFeedforwardEnabled(0U);
    BallBalance_App_SetVehicleDisturbanceMmS2(0.0f, 0U, 0U);
}

void BallBalance_VehicleImuAdapter_Update(void)
{
    uint32_t now_ms;

    if (s_info.initialized == 0U) {
        return;
    }

    now_ms = BSP_GetTickMs();
    s_info.valid = 0U;
    s_info.disturbance_mm_s2 = 0.0f;
    s_info.timestamp_ms = now_ms;
    if (s_info.invalid_count < 0xFFFFFFFFUL) {
        s_info.invalid_count++;
    }
    BallBalance_App_SetVehicleDisturbanceMmS2(
        0.0f,
        0U,
        now_ms
    );
}

BSP_Status_t BallBalance_VehicleImuAdapter_GetInfo(
    BallBalance_VehicleImuAdapterInfo_t *info
)
{
    if (info == 0) {
        return BSP_PARAM;
    }
    *info = s_info;
    return BSP_OK;
}
