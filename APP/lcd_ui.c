#include "lcd_ui.h"
#include "drv_lcd_tft.h"
#include "bsp_systick.h"
#include <stdio.h>

static uint32_t s_lcd_ui_counter = 0U;

void LcdUi_ShowBoot(void)
{
#if LCD_UI_ENABLE
    Drv_LcdTft_FillScreen(DRV_LCD_COLOR_BLACK);
    Drv_LcdTft_DrawRect(4U, 4U, 232U, 232U, DRV_LCD_COLOR_BLUE);
    Drv_LcdTft_DrawString(16U, 18U, "TRACETRACK CAR", DRV_LCD_COLOR_WHITE, DRV_LCD_COLOR_BLACK);
    Drv_LcdTft_DrawString(16U, 36U, "LCD: SPI2 ST7789", DRV_LCD_COLOR_CYAN, DRV_LCD_COLOR_BLACK);
    Drv_LcdTft_DrawString(16U, 54U, "SCL PB13 SDA PB15", DRV_LCD_COLOR_GREEN, DRV_LCD_COLOR_BLACK);
    Drv_LcdTft_DrawString(16U, 72U, "CS PB12 DC PC10", DRV_LCD_COLOR_GREEN, DRV_LCD_COLOR_BLACK);
    Drv_LcdTft_DrawString(16U, 90U, "BL PC11", DRV_LCD_COLOR_GREEN, DRV_LCD_COLOR_BLACK);
#endif
}

void LcdUi_Init(void)
{
#if LCD_UI_ENABLE
    Drv_LcdTft_Init();
    LcdUi_ShowBoot();
#endif
}

void LcdUi_ShowStatus(const char *line1, const char *line2, const char *line3)
{
#if LCD_UI_ENABLE
    Drv_LcdTft_FillRect(10U, 118U, 220U, 80U, DRV_LCD_COLOR_BLACK);
    if (line1 != 0) {
        Drv_LcdTft_DrawString(16U, 124U, line1, DRV_LCD_COLOR_WHITE, DRV_LCD_COLOR_BLACK);
    }
    if (line2 != 0) {
        Drv_LcdTft_DrawString(16U, 142U, line2, DRV_LCD_COLOR_WHITE, DRV_LCD_COLOR_BLACK);
    }
    if (line3 != 0) {
        Drv_LcdTft_DrawString(16U, 160U, line3, DRV_LCD_COLOR_WHITE, DRV_LCD_COLOR_BLACK);
    }
#else
    (void)line1;
    (void)line2;
    (void)line3;
#endif
}

void LcdUi_Update(void)
{
#if LCD_UI_ENABLE
    static uint32_t last_ms = 0U;
    char buf[24];

    if (Drv_LcdTft_IsReady() == 0U) {
        return;
    }

    if (BSP_TimeElapsed(&last_ms, LCD_UI_UPDATE_PERIOD_MS) == 0U) {
        return;
    }

    s_lcd_ui_counter++;
    (void)sprintf(buf, "RUN: %lu", (unsigned long)s_lcd_ui_counter);
    Drv_LcdTft_FillRect(16U, 208U, 110U, 10U, DRV_LCD_COLOR_BLACK);
    Drv_LcdTft_DrawString(16U, 208U, buf, DRV_LCD_COLOR_YELLOW, DRV_LCD_COLOR_BLACK);
#endif
}
