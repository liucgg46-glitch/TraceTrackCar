#include "oled_ui.h"
#include "drv_oled_i2c.h"
#include "bsp_systick.h"
#include "sensor_manager.h"
#include "line_follow_app.h"
#include "chassis.h"
#include "motion_action.h"
#include "route_manager.h"
#include "line_detect.h"
#include <stdio.h>

typedef enum {
    OLED_UI_LINE_CAL_READY = 0,
    OLED_UI_LINE_CAL_WHITE_CAPTURED,
    OLED_UI_LINE_CAL_BLACK_CAPTURED,
    OLED_UI_LINE_CAL_RESULT
} OledUi_LineCalibrationPage_t;

static uint8_t s_oled_boot_visible = 0U;
static uint32_t s_oled_boot_done_ms = 0U;
static uint8_t s_oled_route_test_active = 0U;
static uint8_t s_oled_route_test_page = 0U;
static uint8_t s_oled_line_cal_active = 0U;
static OledUi_LineCalibrationPage_t s_oled_line_cal_page = OLED_UI_LINE_CAL_READY;
static uint16_t s_oled_line_cal_threshold[LINE_DETECT_SENSOR_NUM];

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

static const char *OledUi_LineTrackModeName(LineTrack_Mode_t mode)
{
    switch (mode) {
        case LINE_TRACK_MODE_TRACK:        return "TRACK";
        case LINE_TRACK_MODE_WIDE_LINE:    return "WIDE";
        case LINE_TRACK_MODE_LOST_CONFIRM: return "LOST";
        case LINE_TRACK_MODE_SEARCH:       return "SEARCH";
        case LINE_TRACK_MODE_FAILSAFE:
        default:                           return "SAFE";
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
     * 本函数只更新 OLED RAM 缓冲并标记脏页，实际 I2C 传输由
     * Drv_OledI2c_Task() 推进，因此可在 SSD1306 异步初始化完成前调用。
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
    uint8_t i;

    s_oled_route_test_active = 0U;
    s_oled_route_test_page = 0U;
    s_oled_line_cal_active = 0U;
    s_oled_line_cal_page = OLED_UI_LINE_CAL_READY;
    for (i = 0U; i < LINE_DETECT_SENSOR_NUM; i++) {
        s_oled_line_cal_threshold[i] = 0U;
    }
    OledUi_ShowBoot();
#endif
}

static void OledUi_ShowLineCalibration(void)
{
    uint8_t i;
    char text[32];

    Drv_OledI2c_Clear();
    Drv_OledI2c_DrawRect(0U, 0U, 128U, 64U, DRV_OLED_COLOR_ON);

    switch (s_oled_line_cal_page) {
        case OLED_UI_LINE_CAL_WHITE_CAPTURED:
            Drv_OledI2c_DrawString5x7(4U, 2U, "WHITE CAPTURED", DRV_OLED_COLOR_ON);
            Drv_OledI2c_DrawString5x7(4U, 14U, "MOVE TO BLACK", DRV_OLED_COLOR_ON);
            Drv_OledI2c_DrawString5x7(4U, 26U, "WAIT STABLE", DRV_OLED_COLOR_ON);
            Drv_OledI2c_DrawString5x7(4U, 38U, "PRESS KEY2", DRV_OLED_COLOR_ON);
            break;

        case OLED_UI_LINE_CAL_BLACK_CAPTURED:
            Drv_OledI2c_DrawString5x7(4U, 2U, "BLACK CAPTURED", DRV_OLED_COLOR_ON);
            Drv_OledI2c_DrawString5x7(4U, 14U, "PRESS KEY3", DRV_OLED_COLOR_ON);
            Drv_OledI2c_DrawString5x7(4U, 26U, "MAKE THRESHOLDS", DRV_OLED_COLOR_ON);
            break;

        case OLED_UI_LINE_CAL_RESULT:
            Drv_OledI2c_DrawString5x7(4U, 2U, "CALIBRATION DONE", DRV_OLED_COLOR_ON);
            for (i = 0U; i < LINE_DETECT_SENSOR_NUM; i = (uint8_t)(i + 2U)) {
                (void)snprintf(text, sizeof(text), "T%u:%u T%u:%u",
                               (unsigned int)i,
                               (unsigned int)s_oled_line_cal_threshold[i],
                               (unsigned int)(i + 1U),
                               (unsigned int)s_oled_line_cal_threshold[i + 1U]);
                Drv_OledI2c_DrawString5x7(4U,
                                         (uint8_t)(12U + ((i / 2U) * 10U)),
                                         text,
                                         DRV_OLED_COLOR_ON);
            }
            Drv_OledI2c_DrawString5x7(4U, 52U, "RAM ONLY UART SAVE", DRV_OLED_COLOR_ON);
            break;

        case OLED_UI_LINE_CAL_READY:
        default:
            Drv_OledI2c_DrawString5x7(4U, 2U, "GRAY CALIBRATION", DRV_OLED_COLOR_ON);
            Drv_OledI2c_DrawString5x7(4U, 14U, "KEY1: WHITE", DRV_OLED_COLOR_ON);
            Drv_OledI2c_DrawString5x7(4U, 26U, "KEY2: BLACK", DRV_OLED_COLOR_ON);
            Drv_OledI2c_DrawString5x7(4U, 38U, "KEY3: CALCULATE", DRV_OLED_COLOR_ON);
            break;
    }

    Drv_OledI2c_Flush();
}

static void OledUi_ShowRouteTest(void)
{
    RouteManager_Info_t route;
    LineFollow_Info_t line;
    Motion_Info_t motion;
    char text[32];

    Drv_OledI2c_Clear();
    Drv_OledI2c_DrawRect(0U, 0U, 128U, 64U, DRV_OLED_COLOR_ON);

    if ((RouteManager_GetInfo(&route) != BSP_OK) ||
        (LineFollow_GetInfo(&line) != BSP_OK) ||
        (Motion_GetInfo(&motion) != BSP_OK)) {
        Drv_OledI2c_DrawString5x7(4U, 2U, "ROUTE TEST", DRV_OLED_COLOR_ON);
        Drv_OledI2c_DrawString5x7(4U, 14U, "ROUTE INFO ERROR", DRV_OLED_COLOR_ON);
        Drv_OledI2c_Flush();
        return;
    }

    if (s_oled_route_test_page == 0U) {
        Drv_OledI2c_DrawString5x7(4U, 2U, "ROUTE TEST A", DRV_OLED_COLOR_ON);
        (void)snprintf(text, sizeof(text), "P:%u S:%u C:%u",
                       (unsigned int)route.profile,
                       (unsigned int)route.profile_state,
                       (unsigned int)route.control_mode);
        Drv_OledI2c_DrawString5x7(4U, 12U, text, DRV_OLED_COLOR_ON);
        (void)snprintf(text, sizeof(text), "LF:%u M:%u CF:%u",
                       (unsigned int)line.state,
                       (unsigned int)route.motion_state,
                       (unsigned int)route.event_confirm_samples);
        Drv_OledI2c_DrawString5x7(4U, 22U, text, DRV_OLED_COLOR_ON);
        (void)snprintf(text, sizeof(text), "RUN:%lus TR:%lu",
                       (unsigned long)(route.running_ms / 1000U),
                       (unsigned long)route.transition_count);
        Drv_OledI2c_DrawString5x7(4U, 32U, text, DRV_OLED_COLOR_ON);
        (void)snprintf(text, sizeof(text), "T:%s MK:%02X",
                       OledUi_LineTypeName(line.detect.type),
                       (unsigned int)line.detect.black_mask);
        Drv_OledI2c_DrawString5x7(4U, 42U, text, DRV_OLED_COLOR_ON);
        (void)snprintf(text, sizeof(text), "E:%d/%d",
                       (int)line.detect.error_x1000,
                       (int)route.line_filtered_error);
        Drv_OledI2c_DrawString5x7(4U, 52U, text, DRV_OLED_COLOR_ON);
        s_oled_route_test_page = 1U;
    } else {
        Drv_OledI2c_DrawString5x7(4U, 2U, "ROUTE TEST B", DRV_OLED_COLOR_ON);
        (void)snprintf(text, sizeof(text), "MD:%s L:%lu",
                       OledUi_LineTrackModeName(route.line_track_mode),
                       (unsigned long)route.line_lost_ms);
        Drv_OledI2c_DrawString5x7(4U, 12U, text, DRV_OLED_COLOR_ON);
        (void)snprintf(text, sizeof(text), "DE:%d SAT:%u",
                       (int)route.line_derivative_error,
                       (unsigned int)route.line_turn_saturated);
        Drv_OledI2c_DrawString5x7(4U, 22U, text, DRV_OLED_COLOR_ON);
        (void)snprintf(text, sizeof(text), "AD:%d U:%ld",
                       (int)route.line_adaptive_linear_cps,
                       (long)route.line_turn_unclamped_cps);
        Drv_OledI2c_DrawString5x7(4U, 32U, text, DRV_OLED_COLOR_ON);
        (void)snprintf(text, sizeof(text), "OUT:%d/%d",
                       (int)line.output.linear_cps,
                       (int)line.output.turn_cps);
        Drv_OledI2c_DrawString5x7(4U, 42U, text, DRV_OLED_COLOR_ON);
        (void)snprintf(text, sizeof(text), "YAW:%d",
                       (int)motion.current_yaw_deg);
        Drv_OledI2c_DrawString5x7(4U, 52U, text, DRV_OLED_COLOR_ON);
        s_oled_route_test_page = 0U;
    }

    Drv_OledI2c_Flush();
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

    if (s_oled_line_cal_active != 0U) {
        OledUi_ShowLineCalibration();
        return;
    }
    if (s_oled_route_test_active != 0U) {
        OledUi_ShowRouteTest();
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

void OledUi_RouteTestBegin(void)
{
#if OLED_UI_ENABLE
    if (s_oled_route_test_active == 0U) {
        s_oled_route_test_page = 0U;
    }
    s_oled_route_test_active = 1U;
    s_oled_line_cal_active = 0U;
#endif
}

void OledUi_LineCalibrationBegin(void)
{
#if OLED_UI_ENABLE
    s_oled_route_test_active = 0U;
    s_oled_line_cal_active = 1U;
    s_oled_line_cal_page = OLED_UI_LINE_CAL_READY;
#endif
}

void OledUi_LineCalibrationWhiteCaptured(void)
{
#if OLED_UI_ENABLE
    s_oled_route_test_active = 0U;
    s_oled_line_cal_active = 1U;
    s_oled_line_cal_page = OLED_UI_LINE_CAL_WHITE_CAPTURED;
#endif
}

void OledUi_LineCalibrationBlackCaptured(void)
{
#if OLED_UI_ENABLE
    s_oled_route_test_active = 0U;
    s_oled_line_cal_active = 1U;
    s_oled_line_cal_page = OLED_UI_LINE_CAL_BLACK_CAPTURED;
#endif
}

void OledUi_LineCalibrationShowResult(const uint16_t *threshold,
                                      uint8_t count)
{
#if OLED_UI_ENABLE
    uint8_t i;

    if ((threshold == 0) || (count < LINE_DETECT_SENSOR_NUM)) {
        return;
    }
    for (i = 0U; i < LINE_DETECT_SENSOR_NUM; i++) {
        s_oled_line_cal_threshold[i] = threshold[i];
    }
    s_oled_route_test_active = 0U;
    s_oled_line_cal_active = 1U;
    s_oled_line_cal_page = OLED_UI_LINE_CAL_RESULT;
#else
    (void)threshold;
    (void)count;
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
