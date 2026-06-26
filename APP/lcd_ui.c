#include "lcd_ui.h"
#include "drv_lcd_tft.h"
#include "bsp_systick.h"
#include <stdio.h>

typedef enum {
    LCD_UI_JOB_NONE = 0,
    LCD_UI_JOB_BOOT,
    LCD_UI_JOB_RUN_BASE,
    LCD_UI_JOB_STATUS
} LcdUi_Job_t;

static LcdUi_Job_t s_lcd_ui_job = LCD_UI_JOB_BOOT;
static uint8_t s_lcd_ui_step = 0U;
static uint8_t s_lcd_boot_visible = 0U;
static uint32_t s_lcd_boot_done_ms = 0U;
static uint32_t s_lcd_ui_counter = 0U;
static char s_lcd_status_line[3][DRV_LCD_TFT_ASYNC_TEXT_MAX_CHARS + 1U];

static void LcdUi_CopyLine(uint8_t index, const char *line)
{
    uint8_t i;

    if (index >= 3U) {
        return;
    }

    for (i = 0U; i < DRV_LCD_TFT_ASYNC_TEXT_MAX_CHARS; i++) {
        if ((line == 0) || (line[i] == '\0')) {
            break;
        }
        s_lcd_status_line[index][i] = line[i];
    }
    s_lcd_status_line[index][i] = '\0';
}

static BSP_Status_t LcdUi_DrawBaseStep(uint8_t step)
{
    switch (step) {
        case 0U:
            return Drv_LcdTft_TryClear(DRV_LCD_COLOR_BLACK);

        case 1U:
            return Drv_LcdTft_TryDrawRect(4U, 4U, 232U, 232U, DRV_LCD_COLOR_BLUE);

        case 2U:
            return Drv_LcdTft_TryDrawString5x7(16U, 18U, "TRACETRACK CAR",
                                               DRV_LCD_COLOR_WHITE,
                                               DRV_LCD_COLOR_BLACK);

        case 3U:
            return Drv_LcdTft_TryDrawString5x7(16U, 36U, "LCD SPI2 ST7789",
                                               DRV_LCD_COLOR_CYAN,
                                               DRV_LCD_COLOR_BLACK);

        case 4U:
            return Drv_LcdTft_TryDrawString5x7(16U, 54U, "ASYNC DMA MODE",
                                               DRV_LCD_COLOR_YELLOW,
                                               DRV_LCD_COLOR_BLACK);

        case 5U:
            return Drv_LcdTft_TryDrawString5x7(16U, 72U, "PB13 PB15 0xSPI2",
                                               DRV_LCD_COLOR_GREEN,
                                               DRV_LCD_COLOR_BLACK);

        default:
            return BSP_PARAM;
    }
}

static BSP_Status_t LcdUi_DrawRunBaseStep(uint8_t step)
{
    switch (step) {
        case 0U:
            return Drv_LcdTft_TryClear(DRV_LCD_COLOR_BLACK);

        case 1U:
            return Drv_LcdTft_TryDrawRect(4U, 4U, 232U, 232U, DRV_LCD_COLOR_BLUE);

        default:
            return BSP_PARAM;
    }
}

static BSP_Status_t LcdUi_DrawStatusStep(uint8_t step)
{
    switch (step) {
        case 0U:
            return Drv_LcdTft_TryClear(DRV_LCD_COLOR_BLACK);

        case 1U:
            return Drv_LcdTft_TryDrawRect(4U, 4U, 232U, 232U, DRV_LCD_COLOR_BLUE);

        case 2U:
            if (s_lcd_status_line[0][0] == '\0') {
                return BSP_OK;
            }
            return Drv_LcdTft_TryDrawString5x7(16U, 18U, s_lcd_status_line[0],
                                               DRV_LCD_COLOR_WHITE,
                                               DRV_LCD_COLOR_BLACK);

        case 3U:
            if (s_lcd_status_line[1][0] == '\0') {
                return BSP_OK;
            }
            return Drv_LcdTft_TryDrawString5x7(16U, 36U, s_lcd_status_line[1],
                                               DRV_LCD_COLOR_WHITE,
                                               DRV_LCD_COLOR_BLACK);

        case 4U:
            if (s_lcd_status_line[2][0] == '\0') {
                return BSP_OK;
            }
            return Drv_LcdTft_TryDrawString5x7(16U, 54U, s_lcd_status_line[2],
                                               DRV_LCD_COLOR_WHITE,
                                               DRV_LCD_COLOR_BLACK);

        default:
            return BSP_PARAM;
    }
}

static void LcdUi_RunJob(void)
{
    BSP_Status_t ret;

    if ((s_lcd_ui_job == LCD_UI_JOB_NONE) ||
        (Drv_LcdTft_IsReady() == 0U) ||
        (Drv_LcdTft_IsBusy() != 0U)) {
        return;
    }

    if (s_lcd_ui_job == LCD_UI_JOB_BOOT) {
        ret = LcdUi_DrawBaseStep(s_lcd_ui_step);
        if (ret == BSP_OK) {
            s_lcd_ui_step++;
        } else if (ret == BSP_PARAM) {
            s_lcd_ui_job = LCD_UI_JOB_NONE;
            s_lcd_ui_step = 0U;
            s_lcd_boot_visible = 1U;
            s_lcd_boot_done_ms = BSP_GET_TICK();
        }
        return;
    }

    if (s_lcd_ui_job == LCD_UI_JOB_RUN_BASE) {
        ret = LcdUi_DrawRunBaseStep(s_lcd_ui_step);
        if (ret == BSP_OK) {
            s_lcd_ui_step++;
        } else if (ret == BSP_PARAM) {
            s_lcd_ui_job = LCD_UI_JOB_NONE;
            s_lcd_ui_step = 0U;
        }
        return;
    }

    if (s_lcd_ui_job == LCD_UI_JOB_STATUS) {
        ret = LcdUi_DrawStatusStep(s_lcd_ui_step);
        if (ret == BSP_OK) {
            s_lcd_ui_step++;
        } else if (ret == BSP_PARAM) {
            s_lcd_ui_job = LCD_UI_JOB_NONE;
            s_lcd_ui_step = 0U;
        }
    }
}

void LcdUi_ShowBoot(void)
{
#if LCD_UI_ENABLE
    s_lcd_ui_job = LCD_UI_JOB_BOOT;
    s_lcd_ui_step = 0U;
    s_lcd_boot_visible = 0U;
#endif
}

void LcdUi_Init(void)
{
#if LCD_UI_ENABLE
    LcdUi_ShowBoot();
#endif
}

void LcdUi_ShowStatus(const char *line1, const char *line2, const char *line3)
{
#if LCD_UI_ENABLE
    LcdUi_CopyLine(0U, line1);
    LcdUi_CopyLine(1U, line2);
    LcdUi_CopyLine(2U, line3);
    s_lcd_ui_job = LCD_UI_JOB_STATUS;
    s_lcd_ui_step = 0U;
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
    BSP_Status_t ret;

    LcdUi_RunJob();

    if ((s_lcd_ui_job == LCD_UI_JOB_NONE) &&
        (s_lcd_boot_visible != 0U) &&
        ((uint32_t)(BSP_GET_TICK() - s_lcd_boot_done_ms) >= LCD_UI_BOOT_HOLD_MS)) {
        s_lcd_boot_visible = 0U;
        s_lcd_ui_job = LCD_UI_JOB_RUN_BASE;
        s_lcd_ui_step = 0U;
        return;
    }

    if ((s_lcd_ui_job != LCD_UI_JOB_NONE) ||
        (Drv_LcdTft_IsReady() == 0U) ||
        (Drv_LcdTft_IsBusy() != 0U)) {
        return;
    }

    if (BSP_TimeElapsed(&last_ms, LCD_UI_UPDATE_PERIOD_MS) == 0U) {
        return;
    }

    s_lcd_ui_counter++;
    (void)sprintf(buf, "RUN:%lu     ", (unsigned long)s_lcd_ui_counter);
    ret = Drv_LcdTft_TryDrawString5x7(16U, 214U, buf,
                                      DRV_LCD_COLOR_WHITE,
                                      DRV_LCD_COLOR_BLACK);
    if (ret != BSP_OK) {
        s_lcd_ui_counter--;
    }
#endif
}

void LCD_Update(void)
{
    LcdUi_Update();
}
