#include "odometer.h"

static Odometer_Info_t s_odom;
static int32_t s_origin_left_mm;
static int32_t s_origin_right_mm;

static int32_t Odometer_GetRelativeMm(int32_t total_mm, int32_t origin_mm)
{
    /* 使用无符号差值定义32位计数回绕后的相对距离。 */
    return (int32_t)((uint32_t)total_mm - (uint32_t)origin_mm);
}

void Odometer_Init(int32_t left_total_mm, int32_t right_total_mm)
{
    Odometer_Clear(left_total_mm, right_total_mm);
}

void Odometer_Clear(int32_t left_total_mm, int32_t right_total_mm)
{
    s_origin_left_mm = left_total_mm;
    s_origin_right_mm = right_total_mm;
    s_odom.left_mm = 0;
    s_odom.right_mm = 0;
    s_odom.distance_mm = 0;
    s_odom.delta_left_mm = 0;
    s_odom.delta_right_mm = 0;
    s_odom.delta_distance_mm = 0;
}

void Odometer_Update(int32_t left_total_mm, int32_t right_total_mm)
{
    int32_t new_left;
    int32_t new_right;
    int32_t new_dist;

    new_left = Odometer_GetRelativeMm(left_total_mm, s_origin_left_mm);
    new_right = Odometer_GetRelativeMm(right_total_mm, s_origin_right_mm);
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

Project_Status_t Odometer_GetInfo(Odometer_Info_t *info)
{
    if (info == 0) return PROJECT_PARAM;
    *info = s_odom;
    return PROJECT_OK;
}
