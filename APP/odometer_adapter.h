#ifndef __ODOMETER_ADAPTER_H
#define __ODOMETER_ADAPTER_H

/* APP适配层负责读取编码器Driver，再把纯里程数据传给算法。 */
void AppOdometer_Init(void);
void AppOdometer_Clear(void);
void AppOdometer_Update(void);

#endif /* __ODOMETER_ADAPTER_H */
