#include "lcd_ui.h"
#include "drv_lcd_tft.h"
#include "bsp_systick.h"
#include "sensor_manager.h"
#include "line_follow_app.h"
#include "chassis.h"
#include "motion_action.h"
#include "route_manager.h"
#include "line_detect.h"
#include "vehicle_config.h"
#include <stdio.h>

#define LCD_UI_DASHBOARD_LINE_COUNT  10U
#define LCD_UI_DASHBOARD_X           12U
#define LCD_UI_DASHBOARD_Y           14U
#define LCD_UI_DASHBOARD_LINE_GAP    20U

typedef enum {
    LCD_UI_JOB_NONE = 0,
    LCD_UI_JOB_BOOT,
    LCD_UI_JOB_RUN_BASE,
    LCD_UI_JOB_DASHBOARD,
    LCD_UI_JOB_STATUS
} LcdUi_Job_t;

typedef enum {
    LCD_UI_LINE_CAL_READY = 0,
    LCD_UI_LINE_CAL_WHITE_CAPTURED,
    LCD_UI_LINE_CAL_BLACK_CAPTURED,
    LCD_UI_LINE_CAL_RESULT
} LcdUi_LineCalibrationPage_t;

static LcdUi_Job_t s_lcd_ui_job = LCD_UI_JOB_BOOT;
static uint8_t s_lcd_ui_step = 0U;
static uint8_t s_lcd_boot_visible = 0U;
static uint32_t s_lcd_boot_done_ms = 0U;
static uint8_t s_lcd_dashboard_dirty = 1U;
static uint8_t s_lcd_chassis_test_active = 0U;
static uint8_t s_lcd_route_test_active = 0U;
static uint8_t s_lcd_line_cal_active = 0U;
static uint8_t s_lcd_status_active = 0U;
static uint8_t s_lcd_status_base_drawn = 0U;
static LcdUi_LineCalibrationPage_t s_lcd_line_cal_page = LCD_UI_LINE_CAL_READY;
static uint16_t s_lcd_line_cal_threshold[LINE_DETECT_SENSOR_NUM];
static char s_lcd_status_line[3][DRV_LCD_TFT_ASYNC_TEXT_MAX_CHARS + 1U];
static char s_lcd_dashboard_line[LCD_UI_DASHBOARD_LINE_COUNT]
                                [DRV_LCD_TFT_ASYNC_TEXT_MAX_CHARS + 1U];

static const char *LcdUi_LineTypeName(LineType_t type)
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

static const char *LcdUi_LineTrackModeName(LineTrack_Mode_t mode)
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

static void LcdUi_CopyDashboardLine(uint8_t index, const char *line)
{
    uint8_t i = 0U;

    if (index >= LCD_UI_DASHBOARD_LINE_COUNT) {
        return;
    }

    while ((i < DRV_LCD_TFT_ASYNC_TEXT_MAX_CHARS) &&
           (line != 0) && (line[i] != '\0')) {
        s_lcd_dashboard_line[index][i] = line[i];
        i++;
    }
    while (i < DRV_LCD_TFT_ASYNC_TEXT_MAX_CHARS) {
        s_lcd_dashboard_line[index][i++] = ' ';
    }
    s_lcd_dashboard_line[index][i] = '\0';
}

static void LcdUi_CopyLine(uint8_t index, const char *line)
{
    uint8_t i = 0U;

    if (index >= 3U) {
        return;
    }

    while ((i < DRV_LCD_TFT_ASYNC_TEXT_MAX_CHARS) &&
           (line != 0) && (line[i] != '\0')) {
        s_lcd_status_line[index][i] = line[i];
        i++;
    }
    /*
     * 状态页复用同一显示区域，短文本必须用背景色覆盖上一帧较长文本，
     * 否则 "CODE 4" 后会残留旧的 "PHASE CURVE1" 字符。
     */
    while (i < DRV_LCD_TFT_ASYNC_TEXT_MAX_CHARS) {
        s_lcd_status_line[index][i++] = ' ';
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
            return Drv_LcdTft_TryDrawString5x7(16U, 36U, "LCD SPI1 ST7789",
                                               DRV_LCD_COLOR_CYAN,
                                               DRV_LCD_COLOR_BLACK);

        case 4U:
            return Drv_LcdTft_TryDrawString5x7(16U, 54U, "ASYNC DMA MODE",
                                               DRV_LCD_COLOR_YELLOW,
                                               DRV_LCD_COLOR_BLACK);

        case 5U:
            return Drv_LcdTft_TryDrawString5x7(16U, 72U, "PA5 SCK PA7 MOSI",
                                               DRV_LCD_COLOR_GREEN,
                                               DRV_LCD_COLOR_BLACK);

        default:
            return BSP_PARAM;
    }
}

static BSP_Status_t LcdUi_DrawDashboardStep(uint8_t step)
{
    uint16_t color = DRV_LCD_COLOR_WHITE;

    if (step >= LCD_UI_DASHBOARD_LINE_COUNT) {
        return BSP_PARAM;
    }
    if (step == 0U) {
        color = DRV_LCD_COLOR_CYAN;
    } else if (s_lcd_line_cal_active != 0U) {
        if ((s_lcd_line_cal_page == LCD_UI_LINE_CAL_RESULT) &&
            (step >= 1U) && (step <= LINE_DETECT_SENSOR_NUM)) {
            color = DRV_LCD_COLOR_GREEN;
        } else {
            color = DRV_LCD_COLOR_YELLOW;
        }
    } else if (s_lcd_chassis_test_active != 0U) {
        if ((step == 2U) || (step == 3U)) {
            color = DRV_LCD_COLOR_CYAN;
        } else if ((step == 4U) || (step == 5U)) {
            color = DRV_LCD_COLOR_YELLOW;
        } else if (step >= 6U) {
            color = DRV_LCD_COLOR_GREEN;
        }
    } else if ((step == 4U) || (step == 5U)) {
        color = DRV_LCD_COLOR_YELLOW;
    } else if (step >= 6U) {
        color = DRV_LCD_COLOR_GREEN;
    } else {
        color = DRV_LCD_COLOR_WHITE;
    }

    return Drv_LcdTft_TryDrawString5x7(
        LCD_UI_DASHBOARD_X,
        (uint16_t)(LCD_UI_DASHBOARD_Y + ((uint16_t)step * LCD_UI_DASHBOARD_LINE_GAP)),
        s_lcd_dashboard_line[step],
        color,
        DRV_LCD_COLOR_BLACK);
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
            if (s_lcd_status_base_drawn != 0U) {
                return BSP_OK;
            }
            return Drv_LcdTft_TryClear(DRV_LCD_COLOR_BLACK);

        case 1U:
            if (s_lcd_status_base_drawn != 0U) {
                return BSP_OK;
            }
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

    if (s_lcd_ui_job == LCD_UI_JOB_DASHBOARD) {
        ret = LcdUi_DrawDashboardStep(s_lcd_ui_step);
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
            s_lcd_status_base_drawn = 1U;
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

static void LcdUi_BuildNormalDashboard(void)
{
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
    char text[48];

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

    LcdUi_CopyDashboardLine(0U, "TRACETRACK DASHBOARD");

    if (attitude_ok != 0U) {
        (void)snprintf(text, sizeof(text), "IMU:OK MAG:%c%c",
                       attitude.mag_healthy ? 'H' : '-',
                       attitude.mag_used ? 'U' : '-');
    } else {
        (void)snprintf(text, sizeof(text), "IMU:WAIT MAG:--");
    }
    LcdUi_CopyDashboardLine(1U, text);

    (void)snprintf(text, sizeof(text), "R:%c%ld.%01ld P:%c%ld.%01ld",
                   (roll_x10 < 0) ? '-' : '+',
                   (long)(roll_abs_x10 / 10), (long)(roll_abs_x10 % 10),
                   (pitch_x10 < 0) ? '-' : '+',
                   (long)(pitch_abs_x10 / 10), (long)(pitch_abs_x10 % 10));
    LcdUi_CopyDashboardLine(2U, text);

    (void)snprintf(text, sizeof(text), "YAW:%c%ld.%01ld DEG",
                   (yaw_x10 < 0) ? '-' : '+',
                   (long)(yaw_abs_x10 / 10), (long)(yaw_abs_x10 % 10));
    LcdUi_CopyDashboardLine(3U, text);

    if (line_ok != 0U) {
        (void)snprintf(text, sizeof(text), "LINE:%s %s",
                       (line.state == LINE_FOLLOW_RUN) ? "RUN" : "STOP",
                       LcdUi_LineTypeName(line.detect.type));
        LcdUi_CopyDashboardLine(4U, text);
        (void)snprintf(text, sizeof(text), "MASK:%02X ERR:%d",
                       (unsigned int)line.detect.black_mask,
                       (int)line.detect.error_x1000);
        LcdUi_CopyDashboardLine(5U, text);
    } else {
        LcdUi_CopyDashboardLine(4U, "LINE:WAIT");
        LcdUi_CopyDashboardLine(5U, "MASK:-- ERR:----");
    }

    if (chassis_ok != 0U) {
        (void)snprintf(text, sizeof(text), "TGT L:%d R:%d",
                       (int)chassis.left_target_cps,
                       (int)chassis.right_target_cps);
        LcdUi_CopyDashboardLine(6U, text);
        (void)snprintf(text, sizeof(text), "FB L:%ld R:%ld",
                       (long)chassis.fl_feedback_cps,
                       (long)chassis.fr_feedback_cps);
        LcdUi_CopyDashboardLine(7U, text);
        (void)snprintf(text, sizeof(text), "PWM L:%d R:%d",
                       (int)chassis.fl_output,
                       (int)chassis.fr_output);
        LcdUi_CopyDashboardLine(8U, text);
    } else {
        LcdUi_CopyDashboardLine(6U, "TGT L:---- R:----");
        LcdUi_CopyDashboardLine(7U, "FB  L:---- R:----");
        LcdUi_CopyDashboardLine(8U, "PWM L:---- R:----");
    }

    if (distance_ok != 0U) {
        (void)snprintf(text, sizeof(text), "TOF:%u MM",
                       (unsigned int)distance_mm);
        LcdUi_CopyDashboardLine(9U, text);
    } else {
        LcdUi_CopyDashboardLine(9U, "TOF:---- MM");
    }
}

static void LcdUi_BuildLineCalibrationDashboard(void)
{
    uint8_t i;
    char text[24];

    for (i = 0U; i < LCD_UI_DASHBOARD_LINE_COUNT; i++) {
        LcdUi_CopyDashboardLine(i, "");
    }

    switch (s_lcd_line_cal_page) {
        case LCD_UI_LINE_CAL_WHITE_CAPTURED:
            LcdUi_CopyDashboardLine(0U, "WHITE CAPTURED");
            LcdUi_CopyDashboardLine(1U, "MOVE TO BLACK");
            LcdUi_CopyDashboardLine(2U, "WAIT VALUE STABLE");
            LcdUi_CopyDashboardLine(3U, "PRESS KEY2");
            break;

        case LCD_UI_LINE_CAL_BLACK_CAPTURED:
            LcdUi_CopyDashboardLine(0U, "BLACK CAPTURED");
            LcdUi_CopyDashboardLine(1U, "PRESS KEY3");
            LcdUi_CopyDashboardLine(2U, "TO MAKE THRESHOLDS");
            break;

        case LCD_UI_LINE_CAL_RESULT:
            LcdUi_CopyDashboardLine(0U, "CALIBRATION DONE");
            for (i = 0U; i < LINE_DETECT_SENSOR_NUM; i++) {
                (void)snprintf(text, sizeof(text), "TH%u: %u",
                               (unsigned int)i,
                               (unsigned int)s_lcd_line_cal_threshold[i]);
                LcdUi_CopyDashboardLine((uint8_t)(i + 1U), text);
            }
            LcdUi_CopyDashboardLine(9U, "RAM ONLY COPY UART");
            break;

        case LCD_UI_LINE_CAL_READY:
        default:
            LcdUi_CopyDashboardLine(0U, "GRAY CALIBRATION");
            LcdUi_CopyDashboardLine(1U, "KEY1: WHITE");
            LcdUi_CopyDashboardLine(2U, "KEY2: BLACK");
            LcdUi_CopyDashboardLine(3U, "KEY3: CALCULATE");
            break;
    }
}

static void LcdUi_BuildChassisTestDashboard(void)
{
    Chassis_Info_t chassis;
    int32_t fl_error;
    int32_t fr_error;
    char text[48];

    if (Chassis_GetInfo(&chassis) != BSP_OK) {
        LcdUi_CopyDashboardLine(0U, "CHASSIS PWM TEST");
        LcdUi_CopyDashboardLine(1U, "CHASSIS INFO ERROR");
        LcdUi_CopyDashboardLine(2U, "");
        LcdUi_CopyDashboardLine(3U, "");
        LcdUi_CopyDashboardLine(4U, "");
        LcdUi_CopyDashboardLine(5U, "");
        LcdUi_CopyDashboardLine(6U, "");
        LcdUi_CopyDashboardLine(7U, "");
        LcdUi_CopyDashboardLine(8U, "");
        LcdUi_CopyDashboardLine(9U, "");
        return;
    }

    fl_error = (int32_t)chassis.left_target_cps - chassis.fl_feedback_cps;
    fr_error = (int32_t)chassis.right_target_cps - chassis.fr_feedback_cps;

    LcdUi_CopyDashboardLine(0U, "CHASSIS PWM TEST");
    (void)snprintf(text, sizeof(text), "MODE:%s",
                   (chassis.mode == CHASSIS_MODE_SPEED) ? "SPEED" : "STOP");
    LcdUi_CopyDashboardLine(1U, text);
    (void)snprintf(text, sizeof(text), "LIN:%d TURN:%d",
                   (int)chassis.linear_target_cps,
                   (int)chassis.turn_target_cps);
    LcdUi_CopyDashboardLine(2U, text);
    (void)snprintf(text, sizeof(text), "TGT L:%d R:%d",
                   (int)chassis.left_target_cps,
                   (int)chassis.right_target_cps);
    LcdUi_CopyDashboardLine(3U, text);
    (void)snprintf(text, sizeof(text), "FB1 L:%ld R:%ld",
                   (long)chassis.fl_feedback_cps,
                   (long)chassis.fr_feedback_cps);
    LcdUi_CopyDashboardLine(4U, text);
    (void)snprintf(text, sizeof(text), "ERR1 L:%ld R:%ld",
                   (long)fl_error,
                   (long)fr_error);
    LcdUi_CopyDashboardLine(5U, text);
    (void)snprintf(text, sizeof(text), "PWM1 L:%d R:%d",
                   (int)chassis.fl_output,
                   (int)chassis.fr_output);
    LcdUi_CopyDashboardLine(6U, text);

#if (VEHICLE_REAR_DRIVE_ENABLE != 0U)
    (void)snprintf(text, sizeof(text), "FB2 L:%ld R:%ld",
                   (long)chassis.rl_feedback_cps,
                   (long)chassis.rr_feedback_cps);
    LcdUi_CopyDashboardLine(7U, text);
    (void)snprintf(text, sizeof(text), "ERR2 L:%ld R:%ld",
                   (long)((int32_t)chassis.left_target_cps - chassis.rl_feedback_cps),
                   (long)((int32_t)chassis.right_target_cps - chassis.rr_feedback_cps));
    LcdUi_CopyDashboardLine(8U, text);
    (void)snprintf(text, sizeof(text), "PWM2 L:%d R:%d",
                   (int)chassis.rl_output,
                   (int)chassis.rr_output);
    LcdUi_CopyDashboardLine(9U, text);
#else
    LcdUi_CopyDashboardLine(7U, "K1 FWD K2 +200");
    LcdUi_CopyDashboardLine(8U, "K3 LEFT K4 RIGHT");
    LcdUi_CopyDashboardLine(9U, "K5 STOP");
#endif
}

static void LcdUi_BuildRouteTestDashboard(void)
{
    RouteManager_Info_t route;
    LineFollow_Info_t line;
    Motion_Info_t motion;
    char text[48];

    if ((RouteManager_GetInfo(&route) != BSP_OK) ||
        (LineFollow_GetInfo(&line) != BSP_OK) ||
        (Motion_GetInfo(&motion) != BSP_OK)) {
        LcdUi_CopyDashboardLine(0U, "ROUTE TEST");
        LcdUi_CopyDashboardLine(1U, "ROUTE INFO ERROR");
        LcdUi_CopyDashboardLine(2U, "");
        LcdUi_CopyDashboardLine(3U, "");
        LcdUi_CopyDashboardLine(4U, "");
        LcdUi_CopyDashboardLine(5U, "");
        LcdUi_CopyDashboardLine(6U, "");
        LcdUi_CopyDashboardLine(7U, "");
        LcdUi_CopyDashboardLine(8U, "");
        LcdUi_CopyDashboardLine(9U, "");
        return;
    }

    LcdUi_CopyDashboardLine(0U, "ROUTE TEST");
    (void)snprintf(text, sizeof(text), "P:%u PS:%u CTL:%u",
                   (unsigned int)route.profile,
                   (unsigned int)route.profile_state,
                   (unsigned int)route.control_mode);
    LcdUi_CopyDashboardLine(1U, text);
    (void)snprintf(text, sizeof(text), "LF:%u MOT:%u CF:%u",
                   (unsigned int)line.state,
                   (unsigned int)route.action_state,
                   (unsigned int)route.event_confirm_samples);
    LcdUi_CopyDashboardLine(2U, text);
    (void)snprintf(text, sizeof(text), "RUN:%lus TR:%lu",
                   (unsigned long)(route.running_ms / 1000U),
                   (unsigned long)route.transition_count);
    LcdUi_CopyDashboardLine(3U, text);
    (void)snprintf(text, sizeof(text), "TYPE:%s M:%02X",
                   LcdUi_LineTypeName(line.detect.type),
                   (unsigned int)line.detect.black_mask);
    LcdUi_CopyDashboardLine(4U, text);
    (void)snprintf(text, sizeof(text), "RAW:%d FIL:%d",
                   (int)line.detect.error_x1000,
                   (int)route.line_filtered_error);
    LcdUi_CopyDashboardLine(5U, text);
    (void)snprintf(text, sizeof(text), "MODE:%s L:%lu",
                   LcdUi_LineTrackModeName(route.line_track_mode),
                   (unsigned long)route.line_lost_ms);
    LcdUi_CopyDashboardLine(6U, text);
    (void)snprintf(text, sizeof(text), "SCAN P:%u D:%d R:%u",
                   (unsigned int)route.line_search_phase,
                   (int)route.line_search_direction,
                   (unsigned int)route.line_reacquire_samples);
    LcdUi_CopyDashboardLine(7U, text);
    (void)snprintf(text, sizeof(text), "OUT L:%d T:%d",
                   (int)line.output.linear_cps,
                   (int)line.output.turn_cps);
    LcdUi_CopyDashboardLine(8U, text);
    (void)snprintf(text, sizeof(text), "YAW:%d",
                   (int)motion.current_yaw_deg);
    LcdUi_CopyDashboardLine(9U, text);
}

void LcdUi_Init(void)
{
#if LCD_UI_ENABLE
    uint8_t i;

    s_lcd_line_cal_active = 0U;
    s_lcd_chassis_test_active = 0U;
    s_lcd_route_test_active = 0U;
    s_lcd_status_active = 0U;
    s_lcd_status_base_drawn = 0U;
    s_lcd_line_cal_page = LCD_UI_LINE_CAL_READY;
    s_lcd_dashboard_dirty = 1U;
    for (i = 0U; i < LINE_DETECT_SENSOR_NUM; i++) {
        s_lcd_line_cal_threshold[i] = 0U;
    }
    LcdUi_ShowBoot();
#endif
}

void LcdUi_ShowDashboard(void)
{
#if LCD_UI_ENABLE
    s_lcd_status_active = 0U;
    s_lcd_status_base_drawn = 0U;
    if (s_lcd_ui_job != LCD_UI_JOB_NONE) {
        return;
    }

    if (s_lcd_line_cal_active != 0U) {
        LcdUi_BuildLineCalibrationDashboard();
    } else if (s_lcd_chassis_test_active != 0U) {
        LcdUi_BuildChassisTestDashboard();
    } else if (s_lcd_route_test_active != 0U) {
        LcdUi_BuildRouteTestDashboard();
    } else {
        LcdUi_BuildNormalDashboard();
    }

    s_lcd_dashboard_dirty = 0U;
    s_lcd_ui_job = LCD_UI_JOB_DASHBOARD;
    s_lcd_ui_step = 0U;
#endif
}

void LcdUi_ChassisTestBegin(void)
{
#if LCD_UI_ENABLE
    s_lcd_status_active = 0U;
    s_lcd_status_base_drawn = 0U;
    s_lcd_chassis_test_active = 1U;
    s_lcd_route_test_active = 0U;
    s_lcd_line_cal_active = 0U;
    s_lcd_dashboard_dirty = 1U;
#endif
}

void LcdUi_RouteTestBegin(void)
{
#if LCD_UI_ENABLE
    s_lcd_status_active = 0U;
    s_lcd_status_base_drawn = 0U;
    if (s_lcd_route_test_active == 0U) {
        s_lcd_chassis_test_active = 0U;
        s_lcd_route_test_active = 1U;
        s_lcd_line_cal_active = 0U;
        s_lcd_dashboard_dirty = 1U;
    }
#endif
}

void LcdUi_LineCalibrationBegin(void)
{
#if LCD_UI_ENABLE
    s_lcd_status_active = 0U;
    s_lcd_status_base_drawn = 0U;
    s_lcd_chassis_test_active = 0U;
    s_lcd_route_test_active = 0U;
    s_lcd_line_cal_active = 1U;
    s_lcd_line_cal_page = LCD_UI_LINE_CAL_READY;
    s_lcd_dashboard_dirty = 1U;
#endif
}

void LcdUi_LineCalibrationWhiteCaptured(void)
{
#if LCD_UI_ENABLE
    s_lcd_status_active = 0U;
    s_lcd_status_base_drawn = 0U;
    s_lcd_chassis_test_active = 0U;
    s_lcd_route_test_active = 0U;
    s_lcd_line_cal_active = 1U;
    s_lcd_line_cal_page = LCD_UI_LINE_CAL_WHITE_CAPTURED;
    s_lcd_dashboard_dirty = 1U;
#endif
}

void LcdUi_LineCalibrationBlackCaptured(void)
{
#if LCD_UI_ENABLE
    s_lcd_status_active = 0U;
    s_lcd_status_base_drawn = 0U;
    s_lcd_chassis_test_active = 0U;
    s_lcd_route_test_active = 0U;
    s_lcd_line_cal_active = 1U;
    s_lcd_line_cal_page = LCD_UI_LINE_CAL_BLACK_CAPTURED;
    s_lcd_dashboard_dirty = 1U;
#endif
}

void LcdUi_LineCalibrationShowResult(const uint16_t *threshold,
                                     uint8_t count)
{
#if LCD_UI_ENABLE
    uint8_t i;

    if ((threshold == 0) || (count < LINE_DETECT_SENSOR_NUM)) {
        return;
    }

    for (i = 0U; i < LINE_DETECT_SENSOR_NUM; i++) {
        s_lcd_line_cal_threshold[i] = threshold[i];
    }
    s_lcd_status_active = 0U;
    s_lcd_status_base_drawn = 0U;
    s_lcd_chassis_test_active = 0U;
    s_lcd_route_test_active = 0U;
    s_lcd_line_cal_active = 1U;
    s_lcd_line_cal_page = LCD_UI_LINE_CAL_RESULT;
    s_lcd_dashboard_dirty = 1U;
#else
    (void)threshold;
    (void)count;
#endif
}

void LcdUi_ShowStatus(const char *line1, const char *line2, const char *line3)
{
#if LCD_UI_ENABLE
    LcdUi_CopyLine(0U, line1);
    LcdUi_CopyLine(1U, line2);
    LcdUi_CopyLine(2U, line3);
    s_lcd_status_active = 1U;
    s_lcd_boot_visible = 0U;
    if (s_lcd_ui_job != LCD_UI_JOB_STATUS) {
        s_lcd_ui_job = LCD_UI_JOB_STATUS;
        s_lcd_ui_step = 0U;
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

    LcdUi_RunJob();

    if ((s_lcd_ui_job == LCD_UI_JOB_NONE) &&
        (s_lcd_boot_visible != 0U)) {
        if ((uint32_t)(BSP_GET_TICK() - s_lcd_boot_done_ms) >= LCD_UI_BOOT_HOLD_MS) {
            s_lcd_boot_visible = 0U;
            s_lcd_ui_job = LCD_UI_JOB_RUN_BASE;
            s_lcd_ui_step = 0U;
        }
        /* Keep the boot page exclusive until its hold time has elapsed. */
        return;
    }

    if ((s_lcd_ui_job != LCD_UI_JOB_NONE) ||
        (Drv_LcdTft_IsReady() == 0U) ||
        (Drv_LcdTft_IsBusy() != 0U)) {
        return;
    }

    if (s_lcd_dashboard_dirty != 0U) {
        last_ms = BSP_GET_TICK();
    } else {
        if (BSP_TimeElapsed(&last_ms, LCD_UI_UPDATE_PERIOD_MS) == 0U) {
            return;
        }
    }

    if (s_lcd_status_active != 0U) {
        s_lcd_ui_job = LCD_UI_JOB_STATUS;
        s_lcd_ui_step = 0U;
    } else {
        LcdUi_ShowDashboard();
    }
#endif
}

void LCD_Update(void)
{
    LcdUi_Update();
}
