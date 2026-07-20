#ifndef __DRV_LASER_H
#define __DRV_LASER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 激光MOS模块触发极性：
 *   1U：PG2输出高电平时激光打开；
 *   0U：PG2输出低电平时激光打开。
 *
 * 当前按MOS模块跳帽选择“高电平触发”配置。
 */
#define DRV_LASER_ACTIVE_LEVEL    1U

void Drv_Laser_Init(void);
void Drv_Laser_On(void);
void Drv_Laser_Off(void);
void Drv_Laser_Set(uint8_t enable);
void Drv_Laser_Toggle(void);
uint8_t Drv_Laser_IsOn(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_LASER_H */