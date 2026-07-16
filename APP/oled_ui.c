#include "oled_ui.h"
#include "drv_oled_i2c.h"
#include "bsp_systick.h"
#include "sensor_manager.h"
#include "line_follow_app.h"
#include "chassis.h"
#include <stdio.h>

static uint8_t s_oled_boot_visible = 0U;
static uint32_t s_oled_boot_done_ms = 0U;

static const char *OledUi_LineTypeName(LineType_t type)
{
    switch (type) {
        case LINE_TYPE_SINGLE:       return "SINGLE";
        case LINE_TYPE_LEFT_BRANCH:  return "LEFT";
        case LINE_TYPE_RIGHT_BRANCH: return "RIGHT";
        case LINE_TYPE_CROSS:        return "CROSS";
        case LINE_TYPE_FULL_BLACK:   return "FULL";
        case LINE_TYPE_LOST:
        default:                     return "LOST";
    }
}

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

void OledUi_ShowDashboard(void)
{
#if OLED_UI_ENABLE
    Sensor_Attitude_t attitude;
    LineFollow_Info_t line;
    Chassis_Info_t chassis;
    uint16_t distance_mm;
    int32_t roll_x10 = 0;
    int32_t pitch_x10 = 0;
    int32_t yaw_x10 = 0;
    int32_t roll_abs_x10;
    int32_t pitch_abs_x10;
    int32_t yaw_abs_x10;
    uint8_t attitude_ok;
    uint8_t line_ok;
    uint8_t chassis_ok;
    uint8_t distance_ok;
    char text[32];

    if ((Drv_OledI2c_IsReady() == 0U) ||
        (Drv_OledI2c_IsBusy() != 0U) ||
        (s_oled_boot_visible != 0U)) {
        return;
    }

    attitude_ok = (uint8_t)(Sensor_GetAttitude(&attitude) == BSP_OK);
    line_ok = (uint8_t)(LineFollow_GetInfo(&line) == BSP_OK);
    chassis_ok = (uint8_t)(Chassis_GetInfo(&chassis) == BSP_OK);
    distance_ok = (uint8_t)(Sensor_GetFrontDistanceMm(&distance_mm) == BSP_OK);

    if (attitude_ok != 0U) {
        roll_x10 = (int32_t)(attitude.roll_deg * 10.0f);
        pitch_x10 = (int32_t)(attitude.pitch_deg * 10.0f);
        yaw_x10 = (int32_t)(attitude.yaw_deg * 10.0f);
    }
    roll_abs_x10 = (roll_x10 < 0) ? -roll_x10 : roll_x10;
    pitch_abs_x10 = (pitch_x10 < 0) ? -pitch_x10 : pitch_x10;
    yaw_abs_x10 = (yaw_x10 < 0) ? -yaw_x10 : yaw_x10;

    Drv_OledI2c_Clear();
    Drv_OledI2c_DrawRect(0U, 0U, 128U, 64U, DRV_OLED_COLOR_ON);

    if (attitude_ok != 0U) {
        (void)snprintf(text, sizeof(text), "R:%c%ld.%ld P:%c%ld.%ld",
                       (roll_x10 < 0) ? '-' : '+',
                       (long)(roll_abs_x10 / 10), (long)(roll_abs_x10 % 10),
                       (pitch_x10 < 0) ? '-' : '+',
                       (long)(pitch_abs_x10 / 10), (long)(pitch_abs_x10 % 10));
    } else {
        (void)snprintf(text, sizeof(text), "R:WAIT P:WAIT");
    }
    Drv_OledI2c_DrawString5x7(4U, 2U, text, DRV_OLED_COLOR_ON);

    if (attitude_ok != 0U) {
        (void)snprintf(text, sizeof(text), "Y:%c%ld.%ld M:%c%c",
                       (yaw_x10 < 0) ? '-' : '+',
                       (long)(yaw_abs_x10 / 10), (long)(yaw_abs_x10 % 10),
                       attitude.mag_healthy ? 'H' : '-',
                       attitude.mag_used ? 'U' : '-');
    } else {
        (void)snprintf(text, sizeof(text), "Y:WAIT M:--");
    }
    Drv_OledI2c_DrawString5x7(4U, 12U, text, DRV_OLED_COLOR_ON);

    if (line_ok != 0U) {
        (void)snprintf(text, sizeof(text), "LINE:%s %s",
                       (line.state == LINE_FOLLOW_RUN) ? "RUN" : "STOP",
                       OledUi_LineTypeName(line.detect.type));
        Drv_OledI2c_DrawString5x7(4U, 22U, text, DRV_OLED_COLOR_ON);
        (void)snprintf(text, sizeof(text), "M:%02X E:%d",
                       (unsigned int)line.detect.black_mask,
                       (int)line.detect.error_x1000);
    } else {
        Drv_OledI2c_DrawString5x7(4U, 22U, "LINE:WAIT", DRV_OLED_COLOR_ON);
        (void)snprintf(text, sizeof(text), "M:-- E:----");
    }
    Drv_OledI2c_DrawString5x7(4U, 32U, text, DRV_OLED_COLOR_ON);

    if (chassis_ok != 0U) {
        (void)snprintf(text, sizeof(text), "T:%d/%d F:%ld/%ld",
                       (int)chassis.left_target_cps,
                       (int)chassis.right_target_cps,
                       (long)chassis.fl_feedback_cps,
                       (long)chassis.fr_feedback_cps);
    } else {
        (void)snprintf(text, sizeof(text), "T:--/-- F:--/--");
    }
    Drv_OledI2c_DrawString5x7(4U, 42U, text, DRV_OLED_COLOR_ON);

    if ((chassis_ok != 0U) && (distance_ok != 0U)) {
        (void)snprintf(text, sizeof(text), "P:%d/%d D:%u",
                       (int)chassis.fl_output,
                       (int)chassis.fr_output,
                       (unsigned int)distance_mm);
    } else if (chassis_ok != 0U) {
        (void)snprintf(text, sizeof(text), "P:%d/%d D:----",
                       (int)chassis.fl_output,
                       (int)chassis.fr_output);
    } else {
        (void)snprintf(text, sizeof(text), "P:--/-- D:----");
    }
    Drv_OledI2c_DrawString5x7(4U, 52U, text, DRV_OLED_COLOR_ON);

    Drv_OledI2c_Flush();
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

    OledUi_ShowDashboard();
#endif
}

void OLED_Update(void)
{
    OledUi_Update();
}
