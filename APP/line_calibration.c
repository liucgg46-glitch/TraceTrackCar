#include "line_calibration.h"
#include "bsp_key.h"
#include "bsp_uart.h"
#include "drv_gray_sensor.h"
#include "line_detect.h"
#include "lcd_ui.h"
#include "oled_ui.h"
#include <stdio.h>

static uint8_t s_line_cal_started;
static uint8_t s_line_cal_white_ready;
static uint8_t s_line_cal_black_ready;

static void LineCalibration_Send(const char *message, uint16_t length)
{
    (void)BSP_UART_WriteFrame(UART_PORT1,
                              (const uint8_t *)message,
                              length);
}

static void LineCalibration_PrintSample(const char *name,
                                        const uint16_t sample[LINE_DETECT_SENSOR_NUM])
{
    char buf[128];
    int n;

    n = snprintf(buf,
                 sizeof(buf),
                 "%s %u %u %u %u %u %u %u %u\r\n",
                 name,
                 (unsigned int)sample[0],
                 (unsigned int)sample[1],
                 (unsigned int)sample[2],
                 (unsigned int)sample[3],
                 (unsigned int)sample[4],
                 (unsigned int)sample[5],
                 (unsigned int)sample[6],
                 (unsigned int)sample[7]);
    if ((n > 0) && (n < (int)sizeof(buf))) {
        LineCalibration_Send(buf, (uint16_t)n);
    }
}

static void LineCalibration_PrintThresholdDefines(
    const uint16_t threshold[LINE_DETECT_SENSOR_NUM])
{
    char buf[80];
    int n;
    uint8_t i;

    for (i = 0U; i < LINE_DETECT_SENSOR_NUM; i++) {
        n = snprintf(buf,
                     sizeof(buf),
                     "#define LINE_DETECT_DEFAULT_THRESHOLD_%u %uU\r\n",
                     (unsigned int)i,
                     (unsigned int)threshold[i]);
        if ((n > 0) && (n < (int)sizeof(buf))) {
            LineCalibration_Send(buf, (uint16_t)n);
        }
    }
}

static uint8_t LineCalibration_ReadGray(
    uint16_t sample[LINE_DETECT_SENSOR_NUM])
{
    if (Drv_GraySensor_IsOnline() == 0U) {
        return 0U;
    }
    return (uint8_t)(Drv_GraySensor_GetFiltArray(
                         sample,
                         LINE_DETECT_SENSOR_NUM) == BSP_OK);
}

void LineCalibration_Update(void)
{
    uint16_t sample[LINE_DETECT_SENSOR_NUM];
    uint16_t threshold[LINE_DETECT_SENSOR_NUM];

    if (s_line_cal_started == 0U) {
        static const char banner[] =
            "GRAY CAL READY: KEY1=WHITE KEY2=BLACK KEY3=MAKE\r\n";
        s_line_cal_started = 1U;
        s_line_cal_white_ready = 0U;
        s_line_cal_black_ready = 0U;
        LcdUi_LineCalibrationBegin();
        OledUi_LineCalibrationBegin();
        LineCalibration_Send(banner, (uint16_t)(sizeof(banner) - 1U));
    }

#if BSP_KEY1_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY1)) {
        if (LineCalibration_ReadGray(sample) == 0U) {
            static const char message[] =
                "GRAY CAL WHITE FAILED: SENSOR OFFLINE\r\n";
            LineCalibration_Send(message, (uint16_t)(sizeof(message) - 1U));
        } else {
            LineDetect_CaptureWhite(sample);
            s_line_cal_white_ready = 1U;
            s_line_cal_black_ready = 0U;
            LineCalibration_PrintSample("WHITE", sample);
            LcdUi_LineCalibrationWhiteCaptured();
            OledUi_LineCalibrationWhiteCaptured();
        }
    }
#endif

#if BSP_KEY2_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY2)) {
        if (LineCalibration_ReadGray(sample) == 0U) {
            static const char message[] =
                "GRAY CAL BLACK FAILED: SENSOR OFFLINE\r\n";
            LineCalibration_Send(message, (uint16_t)(sizeof(message) - 1U));
        } else {
            LineDetect_CaptureBlack(sample);
            s_line_cal_black_ready = 1U;
            LineCalibration_PrintSample("BLACK", sample);
            LcdUi_LineCalibrationBlackCaptured();
            OledUi_LineCalibrationBlackCaptured();
        }
    }
#endif

#if BSP_KEY3_ENABLE
    if (BSP_Key_WasPressed(BSP_KEY3)) {
        if ((s_line_cal_white_ready == 0U) ||
            (s_line_cal_black_ready == 0U)) {
            static const char message[] =
                "GRAY CAL WAIT: PRESS KEY1 AND KEY2 FIRST\r\n";
            LineCalibration_Send(message, (uint16_t)(sizeof(message) - 1U));
        } else {
            static const char message[] = "GRAY CAL THRESHOLDS READY\r\n";

            LineDetect_MakeThresholdFromWhiteBlack();
            if (LineDetect_GetThresholdArray(threshold,
                                             LINE_DETECT_SENSOR_NUM) != BSP_OK) {
                static const char error[] = "GRAY CAL THRESHOLD READ FAILED\r\n";
                LineCalibration_Send(error, (uint16_t)(sizeof(error) - 1U));
                return;
            }

            LineCalibration_Send(message, (uint16_t)(sizeof(message) - 1U));
            LineCalibration_PrintThresholdDefines(threshold);
            LcdUi_LineCalibrationShowResult(threshold,
                                            LINE_DETECT_SENSOR_NUM);
            OledUi_LineCalibrationShowResult(threshold,
                                             LINE_DETECT_SENSOR_NUM);
            s_line_cal_white_ready = 0U;
            s_line_cal_black_ready = 0U;
        }
    }
#endif
}
