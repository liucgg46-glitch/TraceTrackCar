#include "sensor_manager.h"
#include "drv_gray_4051.h"

void SensorManager_Init(void)
{
    /* Driver_Init() 已初始化 Drv_Gray4051，这里暂时不做额外动作。 */
}

void Sensor_Update(void)
{
    Drv_Gray4051_Update();
}
