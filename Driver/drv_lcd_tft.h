#ifndef __DRV_LCD_TFT_H
#define __DRV_LCD_TFT_H

#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "drv_lcd_image.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==================== Portable hardware config ====================*/

#define DRV_LCD_TFT_SPI_BUS          SPI_BUS2
#define DRV_LCD_TFT_CS_GPIO          BSP_GPIO_LCD_CS
#define DRV_LCD_TFT_DC_GPIO          BSP_GPIO_LCD_DC
#define DRV_LCD_TFT_BL_GPIO          BSP_GPIO_LCD_BL

#define DRV_LCD_TFT_WIDTH            240U
#define DRV_LCD_TFT_HEIGHT           240U
#define DRV_LCD_TFT_X_OFFSET         0U
#define DRV_LCD_TFT_Y_OFFSET         0U
#define DRV_LCD_TFT_DEFAULT_ROTATION 0U

/* Change to 1 if red/blue colors are swapped on your LCD module. */
#define DRV_LCD_TFT_MADCTL_BGR       0U

/* Backlight polarity: most modules turn BL on with high level. */
#define DRV_LCD_TFT_BL_ACTIVE_LEVEL  1U

/*==================== RGB565 colors ====================*/

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

void     Drv_LcdTft_Init(void); // TFT屏幕硬件初始化（SPI、复位、DC、背光引脚，屏幕寄存器配置，上电唤醒屏幕）
uint8_t  Drv_LcdTft_IsReady(void);// 查询屏幕是否初始化就绪、通信正常，返回1就绪/0忙或异常
void     Drv_LcdTft_Backlight(uint8_t on);// 屏幕背光控制：on=1打开背光，on=0关闭背光

uint16_t Drv_LcdTft_Rgb888To565(uint32_t rgb888);// RGB888(24位颜色) 转换成 TFT通用 RGB565(16位颜色)，返回转换后的16位色值
void     Drv_LcdTft_SetRotation(uint8_t rotation);// 设置屏幕显示旋转方向（0/1/2/3对应0°、90°、180°、270°横竖屏翻转）
void     Drv_LcdTft_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);// 设置绘图窗口区域：限定后续填充/写像素仅在 x0,y0 ~ x1,y1 矩形内生效

void     Drv_LcdTft_DrawPixel(uint16_t x, uint16_t y, uint16_t color);// 在指定坐标(x,y)绘制单个像素点，color为RGB565颜色
void     Drv_LcdTft_FillScreen(uint16_t color);// 全屏填充统一颜色，快速清屏/底色填充
void     Drv_LcdTft_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);// 填充实心矩形：起点(x,y)，宽度w、高度h，整体填充指定颜色
void     Drv_LcdTft_DrawHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color);// 绘制水平直线：起点(x,y)，长度w，整条线为指定颜色
void     Drv_LcdTft_DrawVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color);// 绘制垂直直线：起点(x,y)，高度h，整条线为指定颜色
void     Drv_LcdTft_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);// 绘制空心矩形边框：起点(x,y)，宽w高h，边框为指定颜色，内部不填充

void     Drv_LcdTft_DrawChar(uint16_t x, uint16_t y, char ch, uint16_t fg, uint16_t bg);// 在(x,y)位置打印单个ASCII字符，fg字体颜色，bg背景底色
void     Drv_LcdTft_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg);// 在(x,y)位置打印字符串，fg字体颜色，bg背景底色，自动逐字符排版

// 绘制单色位图图片：img为单色图片结构体，fg前景色、bg背景色
void     Drv_LcdTft_DrawMonoImage(uint16_t x, uint16_t y, const Drv_Lcd_MonoImage_t *img,
                                  uint16_t fg, uint16_t bg);

// 绘制RGB565彩色位图图片：img为16位彩色图片结构体，直接在(x,y)位置贴图
void     Drv_LcdTft_DrawRgb565Image(uint16_t x, uint16_t y, const Drv_Lcd_Rgb565Image_t *img);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_LCD_TFT_H */
