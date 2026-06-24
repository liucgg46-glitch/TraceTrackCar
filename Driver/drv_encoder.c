#include "drv_encoder.h"

#define DRV_ENCODER_PI 3.1415926f

typedef struct {
    BSP_Encoder_Id_t bsp_id;
} Drv_Encoder_Cfg_t;

static const Drv_Encoder_Cfg_t s_drv_enc_cfg[WHEEL_COUNT] = {
    [WHEEL_FL] = {DRV_ENCODER_FL_BSP_ID},
    [WHEEL_FR] = {DRV_ENCODER_FR_BSP_ID},
    [WHEEL_RL] = {DRV_ENCODER_RL_BSP_ID},
    [WHEEL_RR] = {DRV_ENCODER_RR_BSP_ID},
};

static int32_t Encoder_CpsToMmS(int32_t cps)
{
    float circumference = DRV_ENCODER_PI * DRV_ENCODER_WHEEL_DIAMETER_MM;
    float mm_s = ((float)cps * circumference) / DRV_ENCODER_COUNTS_PER_REV;
    return (int32_t)mm_s;
}

static int32_t Encoder_CountToMm(int32_t count)
{
    float circumference = DRV_ENCODER_PI * DRV_ENCODER_WHEEL_DIAMETER_MM;
    float mm = ((float)count * circumference) / DRV_ENCODER_COUNTS_PER_REV;
    return (int32_t)mm;
}

static int32_t Avg2(int32_t a, uint8_t use_a, int32_t b, uint8_t use_b)
{
    int32_t sum = 0;
    int32_t n = 0;
    if (use_a) { sum += a; n++; }
    if (use_b) { sum += b; n++; }
    if (n == 0) return 0;
    return sum / n;
}

void Drv_Encoder_Init(void)
{
    BSP_Encoder_InitAll();
}

void Drv_Encoder_Update(void)
{
    BSP_Encoder_UpdateAll();
}

int16_t Drv_Encoder_GetWheelDelta(Wheel_Id_t wheel)
{
    if (wheel >= WHEEL_COUNT) return 0;
    return BSP_Encoder_GetDelta(s_drv_enc_cfg[wheel].bsp_id);
}

int32_t Drv_Encoder_GetWheelSpeedCps(Wheel_Id_t wheel)
{
    if (wheel >= WHEEL_COUNT) return 0;
    return BSP_Encoder_GetSpeedCps(s_drv_enc_cfg[wheel].bsp_id);
}

int32_t Drv_Encoder_GetWheelTotalCount(Wheel_Id_t wheel)
{
    if (wheel >= WHEEL_COUNT) return 0;
    return BSP_Encoder_GetTotal(s_drv_enc_cfg[wheel].bsp_id);
}

int32_t Drv_Encoder_GetWheelSpeedMmS(Wheel_Id_t wheel)
{
    return Encoder_CpsToMmS(Drv_Encoder_GetWheelSpeedCps(wheel));
}

int32_t Drv_Encoder_GetWheelTotalMm(Wheel_Id_t wheel)
{
    return Encoder_CountToMm(Drv_Encoder_GetWheelTotalCount(wheel));
}

int32_t Drv_Encoder_GetLeftSpeedCps(void)
{
    return Avg2(Drv_Encoder_GetWheelSpeedCps(WHEEL_FL), DRV_ENCODER_LEFT_USE_FRONT,
                Drv_Encoder_GetWheelSpeedCps(WHEEL_RL), DRV_ENCODER_LEFT_USE_REAR);
}

int32_t Drv_Encoder_GetRightSpeedCps(void)
{
    return Avg2(Drv_Encoder_GetWheelSpeedCps(WHEEL_FR), DRV_ENCODER_RIGHT_USE_FRONT,
                Drv_Encoder_GetWheelSpeedCps(WHEEL_RR), DRV_ENCODER_RIGHT_USE_REAR);
}

int32_t Drv_Encoder_GetLeftSpeedMmS(void)
{
    return Avg2(Drv_Encoder_GetWheelSpeedMmS(WHEEL_FL), DRV_ENCODER_LEFT_USE_FRONT,
                Drv_Encoder_GetWheelSpeedMmS(WHEEL_RL), DRV_ENCODER_LEFT_USE_REAR);
}

int32_t Drv_Encoder_GetRightSpeedMmS(void)
{
    return Avg2(Drv_Encoder_GetWheelSpeedMmS(WHEEL_FR), DRV_ENCODER_RIGHT_USE_FRONT,
                Drv_Encoder_GetWheelSpeedMmS(WHEEL_RR), DRV_ENCODER_RIGHT_USE_REAR);
}

int32_t Drv_Encoder_GetLeftTotalMm(void)
{
    return Avg2(Drv_Encoder_GetWheelTotalMm(WHEEL_FL), DRV_ENCODER_LEFT_USE_FRONT,
                Drv_Encoder_GetWheelTotalMm(WHEEL_RL), DRV_ENCODER_LEFT_USE_REAR);
}

int32_t Drv_Encoder_GetRightTotalMm(void)
{
    return Avg2(Drv_Encoder_GetWheelTotalMm(WHEEL_FR), DRV_ENCODER_RIGHT_USE_FRONT,
                Drv_Encoder_GetWheelTotalMm(WHEEL_RR), DRV_ENCODER_RIGHT_USE_REAR);
}

void Drv_Encoder_ClearAllTotal(void)
{
    BSP_Encoder_ClearAllTotal();
}

BSP_Status_t Drv_Encoder_GetWheelInfo(Wheel_Id_t wheel, Drv_Encoder_WheelInfo_t *info)
{
    if (wheel >= WHEEL_COUNT || info == 0) return BSP_PARAM;

    info->delta_count = Drv_Encoder_GetWheelDelta(wheel);
    info->speed_cps   = Drv_Encoder_GetWheelSpeedCps(wheel);
    info->total_count = Drv_Encoder_GetWheelTotalCount(wheel);
    info->speed_mm_s  = Drv_Encoder_GetWheelSpeedMmS(wheel);
    info->total_mm    = Drv_Encoder_GetWheelTotalMm(wheel);
    return BSP_OK;
}

void Encoder_Update(void)
{
    Drv_Encoder_Update();
}
