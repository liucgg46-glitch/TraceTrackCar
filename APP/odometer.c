#include "odometer.h"
#include "drv_encoder.h"

static Odometer_Info_t s_odom;

void Odometer_Init(void)
{
    Odometer_Clear();
}

void Odometer_Clear(void)
{
    Drv_Encoder_ClearAllTotal();
    s_odom.left_mm = 0;
    s_odom.right_mm = 0;
    s_odom.distance_mm = 0;
    s_odom.delta_left_mm = 0;
    s_odom.delta_right_mm = 0;
    s_odom.delta_distance_mm = 0;
}

void Odometer_Update(void)
{
    int32_t new_left;
    int32_t new_right;
    int32_t new_dist;

    new_left  = Drv_Encoder_GetLeftTotalMm();
    new_right = Drv_Encoder_GetRightTotalMm();
    new_dist  = (new_left + new_right) / 2;

    s_odom.delta_left_mm     = new_left - s_odom.left_mm;
    s_odom.delta_right_mm    = new_right - s_odom.right_mm;
    s_odom.delta_distance_mm = new_dist - s_odom.distance_mm;

    s_odom.left_mm = new_left;
    s_odom.right_mm = new_right;
    s_odom.distance_mm = new_dist;
}

int32_t Odometer_GetLeftMm(void)
{
    return s_odom.left_mm;
}

int32_t Odometer_GetRightMm(void)
{
    return s_odom.right_mm;
}

int32_t Odometer_GetDistanceMm(void)
{
    return s_odom.distance_mm;
}

BSP_Status_t Odometer_GetInfo(Odometer_Info_t *info)
{
    if (info == 0) return BSP_PARAM;
    *info = s_odom;
    return BSP_OK;
}
