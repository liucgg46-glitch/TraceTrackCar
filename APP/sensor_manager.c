#include "sensor_manager.h"
#include "drv_gray_sensor.h"

void SensorManager_Init(void)
{
    /* Driver_Init() has initialized the selected gray sensor. */
}

void Sensor_Update(void)
{
    (void)Drv_GraySensor_Update();
}
