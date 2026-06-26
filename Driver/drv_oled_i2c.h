#ifndef __DRV_OLED_I2C_H
#define __DRV_OLED_I2C_H

#include "bsp_i2c.h"
#include "drv_oled_image.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Hardware config: 4-pin SSD1306 OLED on I2C1 PB8/PB9. */
#define DRV_OLED_I2C_ENABLE          1U
#define DRV_OLED_I2C_BUS             I2C_BUS1
#define DRV_OLED_I2C_ADDR_7BIT       0x3CU
#define DRV_OLED_I2C_ADDR_ALT_7BIT   0x3DU
#define DRV_OLED_WIDTH               128U
#define DRV_OLED_HEIGHT              64U
#define DRV_OLED_PAGE_NUM            (DRV_OLED_HEIGHT / 8U)
#define DRV_OLED_COLOR_OFF           0U
#define DRV_OLED_COLOR_ON            1U

void        Drv_OledI2c_Init(void);
uint8_t     Drv_OledI2c_IsReady(void);
BSP_Status_t Drv_OledI2c_GetLastStatus(void);
uint8_t     Drv_OledI2c_GetAsyncState(void);
uint16_t    Drv_OledI2c_GetErrorCount(void);

void        Drv_OledI2c_Clear(void);
void        Drv_OledI2c_Fill(uint8_t color);
void        Drv_OledI2c_Flush(void);
void        Drv_OledI2c_FlushPage(uint8_t page);
void        Drv_OledI2c_Task(void);
uint8_t     Drv_OledI2c_IsBusy(void);

void        Drv_OledI2c_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void        Drv_OledI2c_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
void        Drv_OledI2c_DrawHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color);
void        Drv_OledI2c_DrawVLine(uint8_t x, uint8_t y, uint8_t h, uint8_t color);
void        Drv_OledI2c_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);

void        Drv_OledI2c_DrawChar5x7(uint8_t x, uint8_t y, char ch, uint8_t color);
void        Drv_OledI2c_DrawString5x7(uint8_t x, uint8_t y, const char *str, uint8_t color);
void        Drv_OledI2c_DrawMonoImage(uint8_t x, uint8_t y, const Drv_Oled_MonoImage_t *img, uint8_t color);

uint8_t    *Drv_OledI2c_GetBuffer(void);
uint16_t    Drv_OledI2c_GetBufferSize(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_OLED_I2C_H */
