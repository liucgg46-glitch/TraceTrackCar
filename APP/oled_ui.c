#include "oled_ui.h"
#include "drv_oled_i2c.h"
#include "bsp_systick.h"
#include <stdio.h>

static uint32_t s_oled_ui_counter = 0U;
static uint8_t s_oled_boot_visible = 0U;
static uint32_t s_oled_boot_done_ms = 0U;

static void OledUi_DrawBase(void)
{
    Drv_OledI2c_Clear();
    Drv_OledI2c_DrawRect(0U, 0U, 128U, 64U, DRV_OLED_COLOR_ON);
    Drv_OledI2c_DrawString5x7(6U, 6U,  "TRACE CAR", DRV_OLED_COLOR_ON);
    Drv_OledI2c_DrawString5x7(6U, 18U, "OLED I2C1", DRV_OLED_COLOR_ON);
    Drv_OledI2c_DrawString5x7(6U, 30U, "ASYNC DMA", DRV_OLED_COLOR_ON);
    Drv_OledI2c_DrawString5x7(6U, 42U, "PB8 PB9 0x3C", DRV_OLED_COLOR_ON);
}

static void OledUi_DrawRunBase(void)
{
    Drv_OledI2c_Clear();
    Drv_OledI2c_DrawRect(0U, 0U, 128U, 64U, DRV_OLED_COLOR_ON);
    Drv_OledI2c_Flush();
}

void OledUi_ShowBoot(void)
{
#if OLED_UI_ENABLE
    /*
     * This function only writes the OLED RAM buffer and marks pages dirty.
     * The real I2C transfer is pushed by Drv_OledI2c_Task(), so it is safe
     * to call this before the SSD1306 async init has fully finished.
     */
    OledUi_DrawBase();
    Drv_OledI2c_Flush();
    s_oled_boot_visible = 1U;
    s_oled_boot_done_ms = 0U;
#endif
}

void OledUi_Init(void)
{
#if OLED_UI_ENABLE
    OledUi_ShowBoot();
#endif
}

void OledUi_ShowStatus(const char *line1, const char *line2, const char *line3)
{
#if OLED_UI_ENABLE
    Drv_OledI2c_Clear();
    if (line1 != 0) {
        Drv_OledI2c_DrawString5x7(0U, 0U, line1, DRV_OLED_COLOR_ON);
    }
    if (line2 != 0) {
        Drv_OledI2c_DrawString5x7(0U, 12U, line2, DRV_OLED_COLOR_ON);
    }
    if (line3 != 0) {
        Drv_OledI2c_DrawString5x7(0U, 24U, line3, DRV_OLED_COLOR_ON);
    }
    Drv_OledI2c_Flush();
#else
    (void)line1;
    (void)line2;
    (void)line3;
#endif
}

void OledUi_Update(void)
{
#if OLED_UI_ENABLE
    static uint32_t last_ms = 0U;
    char buf[24];

    if (Drv_OledI2c_IsReady() == 0U) {
        return;
    }

    if (Drv_OledI2c_IsBusy() != 0U) {
        return;
    }

    if (s_oled_boot_visible != 0U) {
        if (s_oled_boot_done_ms == 0U) {
            s_oled_boot_done_ms = BSP_GET_TICK();
            return;
        }

        if ((uint32_t)(BSP_GET_TICK() - s_oled_boot_done_ms) >= OLED_UI_BOOT_HOLD_MS) {
            s_oled_boot_visible = 0U;
            OledUi_DrawRunBase();
        }
        return;
    }

    if (BSP_TimeElapsed(&last_ms, OLED_UI_UPDATE_PERIOD_MS) == 0U) {
        return;
    }

    s_oled_ui_counter++;
    (void)sprintf(buf, "RUN:%lu", (unsigned long)s_oled_ui_counter);
    Drv_OledI2c_FillRect(6U, 52U, 70U, 8U, DRV_OLED_COLOR_OFF);
    Drv_OledI2c_DrawString5x7(6U, 52U, buf, DRV_OLED_COLOR_ON);
    Drv_OledI2c_FlushPage(6U);
    Drv_OledI2c_FlushPage(7U);
#endif
}

void OLED_Update(void)
{
    OledUi_Update();
}
