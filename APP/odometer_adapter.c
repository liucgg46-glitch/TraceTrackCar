#include "odometer_adapter.h"

#include "drv_encoder.h"
#include "odometer.h"

void AppOdometer_Init(void)
{
    Odometer_Init(Drv_Encoder_GetLeftTotalMm(),
                  Drv_Encoder_GetRightTotalMm());
}

void AppOdometer_Clear(void)
{
    Odometer_Clear(Drv_Encoder_GetLeftTotalMm(),
                   Drv_Encoder_GetRightTotalMm());
}

void AppOdometer_Update(void)
{
    Odometer_Update(Drv_Encoder_GetLeftTotalMm(),
                    Drv_Encoder_GetRightTotalMm());
}
