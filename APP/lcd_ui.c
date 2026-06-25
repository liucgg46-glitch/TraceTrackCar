#include "lcd_ui.h"
#include "drv_lcd_tft.h"
#include "bsp_systick.h"
#include <stdio.h>
#include "line_follow_app.h"
#include "route_config.h"
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
    static uint32_t last_fast_ms = 0U;
    static uint32_t last_slow_ms = 0U;

    LineFollow_Info_t info;
    char buf[32];
    char bits[9];
    uint8_t i;

    if (Drv_LcdTft_IsReady() == 0U) {
        return;
    }

    /*
     * 快速区域：50ms 刷新一次，只刷新八路01值
     */
    if (BSP_TimeElapsed(&last_fast_ms, 50U) == 0U) {
        return;
    }

    if (LineFollow_GetInfo(&info) != BSP_OK) {
        return;
    }

    for (i = 0U; i < 8U; i++) {
        bits[i] = info.detect.black[i] ? '1' : '0';
    }
    bits[8] = '\0';

    /*
     * 不要 FillRect。
     * 只覆盖 D 这一行。
     */
    (void)sprintf(buf, "D:%s        ", bits);
    Drv_LcdTft_DrawString(12U, 104U, buf,
                          DRV_LCD_COLOR_WHITE,
                          DRV_LCD_COLOR_BLACK);

    /*
     * 慢速区域：200ms 刷新一次，显示其他调参状态
     */
    if (BSP_TimeElapsed(&last_slow_ms, 200U) == 0U) {
        return;
    }

    (void)sprintf(buf, "M:%02X C:%d        ",
                  info.detect.black_mask,
                  info.detect.black_count);
    Drv_LcdTft_DrawString(12U, 124U, buf,
                          DRV_LCD_COLOR_CYAN,
                          DRV_LCD_COLOR_BLACK);

    (void)sprintf(buf, "T:%d E:%d        ",
                  info.detect.type,
                  info.detect.error_x1000);
    Drv_LcdTft_DrawString(12U, 144U, buf,
                          DRV_LCD_COLOR_GREEN,
                          DRV_LCD_COLOR_BLACK);

    (void)sprintf(buf, "V:%d W:%d        ",
                  info.output.linear_cps,
                  info.output.turn_cps);
    Drv_LcdTft_DrawString(12U, 164U, buf,
                          DRV_LCD_COLOR_YELLOW,
                          DRV_LCD_COLOR_BLACK);

    (void)sprintf(buf, "RUN:%d OK:%d      ",
                  info.state,
                  info.output.valid);
    Drv_LcdTft_DrawString(12U, 184U, buf,
                          DRV_LCD_COLOR_WHITE,
                          DRV_LCD_COLOR_BLACK);

#if ROUTE_PROFILE_SELECT == ROUTE_PROFILE_BASIC
    Drv_LcdTft_DrawString(12U, 204U, "P:BASIC        ",
                          DRV_LCD_COLOR_YELLOW,
                          DRV_LCD_COLOR_BLACK);
#else
    Drv_LcdTft_DrawString(12U, 204U, "P:HJDUINO      ",
                          DRV_LCD_COLOR_YELLOW,
                          DRV_LCD_COLOR_BLACK);
#endif

#endif
}