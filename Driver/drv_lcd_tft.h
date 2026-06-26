#ifndef __DRV_LCD_TFT_H
#define __DRV_LCD_TFT_H

#include "bsp_common.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_LCD_TFT_SPI_BUS          SPI_BUS2
#define DRV_LCD_TFT_CS_GPIO          BSP_GPIO_LCD_CS
#define DRV_LCD_TFT_DC_GPIO          BSP_GPIO_LCD_DC
#define DRV_LCD_TFT_BL_GPIO          BSP_GPIO_LCD_BL

#define DRV_LCD_TFT_WIDTH            240U
#define DRV_LCD_TFT_HEIGHT           240U
#define DRV_LCD_TFT_X_OFFSET         0U
#define DRV_LCD_TFT_Y_OFFSET         0U
#define DRV_LCD_TFT_DEFAULT_ROTATION 0U
#define DRV_LCD_TFT_ASYNC_TEXT_MAX_CHARS 20U
#define DRV_LCD_TFT_ASYNC_FILL_MAX_ROWS 20U

#define DRV_LCD_TFT_MADCTL_BGR       0U
#define DRV_LCD_TFT_BL_ACTIVE_LEVEL  1U

#define DRV_LCD_COLOR_BLACK          0x0000U
#define DRV_LCD_COLOR_WHITE          0xFFFFU
#define DRV_LCD_COLOR_RED            0xF800U
#define DRV_LCD_COLOR_GREEN          0x07E0U
#define DRV_LCD_COLOR_BLUE           0x001FU
#define DRV_LCD_COLOR_YELLOW         0xFFE0U
#define DRV_LCD_COLOR_CYAN           0x07FFU
#define DRV_LCD_COLOR_MAGENTA        0xF81FU
#define DRV_LCD_COLOR_GRAY           0x8410U
#define DRV_LCD_COLOR_DARK           0x18C3U

void         Drv_LcdTft_Init(void);
uint8_t      Drv_LcdTft_IsReady(void);
uint8_t      Drv_LcdTft_IsBusy(void);
BSP_Status_t Drv_LcdTft_GetLastStatus(void);
uint8_t      Drv_LcdTft_GetAsyncStage(void);
uint16_t     Drv_LcdTft_GetErrorCount(void);
void         Drv_LcdTft_Task(void);
void         Drv_LcdTft_Backlight(uint8_t on);
uint16_t     Drv_LcdTft_Rgb888To565(uint32_t rgb888);

BSP_Status_t Drv_LcdTft_TryClear(uint16_t color);
BSP_Status_t Drv_LcdTft_TryFill(uint16_t color);
BSP_Status_t Drv_LcdTft_TryFillRect(uint16_t x, uint16_t y,
                                      uint16_t w, uint16_t h,
                                      uint16_t color);
BSP_Status_t Drv_LcdTft_TryDrawHLine(uint16_t x, uint16_t y,
                                      uint16_t w, uint16_t color);
BSP_Status_t Drv_LcdTft_TryDrawVLine(uint16_t x, uint16_t y,
                                      uint16_t h, uint16_t color);
BSP_Status_t Drv_LcdTft_TryDrawRect(uint16_t x, uint16_t y,
                                     uint16_t w, uint16_t h,
                                     uint16_t color);
BSP_Status_t Drv_LcdTft_TryDrawStringLine(uint16_t x, uint16_t y,
                                           const char *str,
                                           uint16_t fg,
                                           uint16_t bg);
BSP_Status_t Drv_LcdTft_TryDrawString5x7(uint16_t x, uint16_t y,
                                          const char *str,
                                          uint16_t fg,
                                          uint16_t bg);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_LCD_TFT_H */
