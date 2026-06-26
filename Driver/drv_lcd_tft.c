#include "drv_lcd_tft.h"
#include "drv_lcd_font.h"
#include "bsp_systick.h"
#include <string.h>

#define LCD_CMD_CASET       0x2AU
#define LCD_CMD_RASET       0x2BU
#define LCD_CMD_RAMWR       0x2CU
#define LCD_CMD_MADCTL      0x36U
#define LCD_CMD_COLMOD      0x3AU

#define LCD_ASYNC_TEXT_PIXEL_BUF_LEN  ((DRV_LCD_FONT_5X7_WIDTH + DRV_LCD_FONT_5X7_X_SPACE) * \
                                       DRV_LCD_FONT_5X7_HEIGHT * \
                                       DRV_LCD_TFT_ASYNC_TEXT_MAX_CHARS * 2U)
#define LCD_ASYNC_FILL_PIXEL_BUF_LEN  (DRV_LCD_TFT_WIDTH * \
                                       DRV_LCD_TFT_ASYNC_FILL_MAX_ROWS * 2U)
#if (LCD_ASYNC_FILL_PIXEL_BUF_LEN > LCD_ASYNC_TEXT_PIXEL_BUF_LEN)
#define LCD_ASYNC_PIXEL_BUF_LEN       LCD_ASYNC_FILL_PIXEL_BUF_LEN
#else
#define LCD_ASYNC_PIXEL_BUF_LEN       LCD_ASYNC_TEXT_PIXEL_BUF_LEN
#endif
#define LCD_INIT_RETRY_DELAY_MS  100U

typedef struct {
    uint8_t cmd;
    const uint8_t *data;
    uint8_t data_len;
    uint16_t delay_after_ms;
} Lcd_InitOp_t;

typedef enum {
    LCD_ASYNC_IDLE = 0,
    LCD_ASYNC_INIT_CMD,
    LCD_ASYNC_INIT_DATA,
    LCD_ASYNC_INIT_DELAY,
    LCD_ASYNC_CASET_CMD,
    LCD_ASYNC_CASET_DATA,
    LCD_ASYNC_RASET_CMD,
    LCD_ASYNC_RASET_DATA,
    LCD_ASYNC_RAMWR_CMD,
    LCD_ASYNC_PIXEL_DATA
} Lcd_AsyncStage_t;

static const uint8_t s_lcd_colmod_data[] = {0x55U};
static const uint8_t s_lcd_b2_data[] = {0x0CU, 0x0CU, 0x00U, 0x33U, 0x33U};
static const uint8_t s_lcd_b7_data[] = {0x35U};
static const uint8_t s_lcd_bb_data[] = {0x19U};
static const uint8_t s_lcd_c0_data[] = {0x2CU};
static const uint8_t s_lcd_c2_data[] = {0x01U};
static const uint8_t s_lcd_c3_data[] = {0x12U};
static const uint8_t s_lcd_c4_data[] = {0x20U};
static const uint8_t s_lcd_c6_data[] = {0x0FU};
static const uint8_t s_lcd_d0_data[] = {0xA4U, 0xA1U};
static const uint8_t s_lcd_e0_data[] = {0xD0U, 0x04U, 0x0DU, 0x11U, 0x13U, 0x2BU, 0x3FU,
                                        0x54U, 0x4CU, 0x18U, 0x0DU, 0x0BU, 0x1FU, 0x23U};
static const uint8_t s_lcd_e1_data[] = {0xD0U, 0x04U, 0x0CU, 0x11U, 0x13U, 0x2CU, 0x3FU,
                                        0x44U, 0x51U, 0x2FU, 0x1FU, 0x1FU, 0x20U, 0x23U};

static uint8_t s_lcd_madctl_data[1];

static const Lcd_InitOp_t s_lcd_init_ops[] = {
    {0x11U, 0, 0U, 120U},
    {LCD_CMD_COLMOD, s_lcd_colmod_data, sizeof(s_lcd_colmod_data), 0U},
    {LCD_CMD_MADCTL, s_lcd_madctl_data, sizeof(s_lcd_madctl_data), 0U},
    {0xB2U, s_lcd_b2_data, sizeof(s_lcd_b2_data), 0U},
    {0xB7U, s_lcd_b7_data, sizeof(s_lcd_b7_data), 0U},
    {0xBBU, s_lcd_bb_data, sizeof(s_lcd_bb_data), 0U},
    {0xC0U, s_lcd_c0_data, sizeof(s_lcd_c0_data), 0U},
    {0xC2U, s_lcd_c2_data, sizeof(s_lcd_c2_data), 0U},
    {0xC3U, s_lcd_c3_data, sizeof(s_lcd_c3_data), 0U},
    {0xC4U, s_lcd_c4_data, sizeof(s_lcd_c4_data), 0U},
    {0xC6U, s_lcd_c6_data, sizeof(s_lcd_c6_data), 0U},
    {0xD0U, s_lcd_d0_data, sizeof(s_lcd_d0_data), 0U},
    {0xE0U, s_lcd_e0_data, sizeof(s_lcd_e0_data), 0U},
    {0xE1U, s_lcd_e1_data, sizeof(s_lcd_e1_data), 0U},
    {0x21U, 0, 0U, 0U},
    {0x29U, 0, 0U, 20U}
};

static uint8_t s_lcd_ready = 0U;
static uint8_t s_lcd_rotation = DRV_LCD_TFT_DEFAULT_ROTATION;
static volatile uint8_t s_lcd_async_segment_busy = 0U;
static volatile uint8_t s_lcd_async_segment_done = 0U;
static volatile BSP_Status_t s_lcd_async_status = BSP_OK;
static BSP_Status_t s_lcd_last_status = BSP_OK;
static Lcd_AsyncStage_t s_lcd_async_stage = LCD_ASYNC_IDLE;
static uint8_t s_lcd_init_index = 0U;
static uint8_t s_lcd_delay_advance_index = 0U;
static uint16_t s_lcd_error_count = 0U;
static uint32_t s_lcd_delay_start_ms = 0U;
static uint16_t s_lcd_delay_ms = 0U;
static uint8_t s_lcd_async_tx[16];
static uint8_t s_lcd_async_rx[LCD_ASYNC_PIXEL_BUF_LEN];
static uint8_t s_lcd_async_window[4];
static uint8_t s_lcd_async_pixels[LCD_ASYNC_PIXEL_BUF_LEN];
static uint16_t s_lcd_async_pixel_len = 0U;
static uint16_t s_lcd_async_x0 = 0U;
static uint16_t s_lcd_async_y0 = 0U;
static uint16_t s_lcd_async_x1 = 0U;
static uint16_t s_lcd_async_y1 = 0U;
static uint8_t s_lcd_fill_active = 0U;
static uint16_t s_lcd_fill_x = 0U;
static uint16_t s_lcd_fill_y = 0U;
static uint16_t s_lcd_fill_w = 0U;
static uint16_t s_lcd_fill_h = 0U;
static uint16_t s_lcd_fill_next_y = 0U;
static uint16_t s_lcd_fill_chunk_rows = 0U;
static uint16_t s_lcd_fill_color = 0U;
static uint8_t s_lcd_rect_active = 0U;
static uint8_t s_lcd_rect_step = 0U;
static uint16_t s_lcd_rect_x = 0U;
static uint16_t s_lcd_rect_y = 0U;
static uint16_t s_lcd_rect_w = 0U;
static uint16_t s_lcd_rect_h = 0U;
static uint16_t s_lcd_rect_color = 0U;

static void Lcd_Select(void)
{
    BSP_GPIO_Write(DRV_LCD_TFT_CS_GPIO, 0U);
}

static void Lcd_Unselect(void)
{
    BSP_GPIO_Write(DRV_LCD_TFT_CS_GPIO, 1U);
}

static void Lcd_CommandMode(void)
{
    BSP_GPIO_Write(DRV_LCD_TFT_DC_GPIO, 0U);
}

static void Lcd_DataMode(void)
{
    BSP_GPIO_Write(DRV_LCD_TFT_DC_GPIO, 1U);
}

static uint8_t Lcd_GetMadctl(uint8_t rotation)
{
    uint8_t madctl;

    switch (rotation & 0x03U) {
        case 1U:  madctl = 0x60U; break;
        case 2U:  madctl = 0xC0U; break;
        case 3U:  madctl = 0xA0U; break;
        default:  madctl = 0x00U; break;
    }

#if DRV_LCD_TFT_MADCTL_BGR
    madctl |= 0x08U;
#endif

    return madctl;
}

static void Lcd_AsyncCallback(SPI_Bus_t bus, void *ctx, BSP_Status_t status)
{
    (void)bus;
    (void)ctx;
    Lcd_Unselect();
    s_lcd_async_status = status;
    s_lcd_async_segment_busy = 0U;
    s_lcd_async_segment_done = 1U;
}

static uint8_t Lcd_IsInitStage(Lcd_AsyncStage_t stage)
{
    return (uint8_t)((stage == LCD_ASYNC_INIT_CMD) ||
                     (stage == LCD_ASYNC_INIT_DATA) ||
                     (stage == LCD_ASYNC_INIT_DELAY));
}

static void Lcd_RestartInitAfterError(void)
{
    s_lcd_ready = 0U;
    s_lcd_init_index = 0U;
    s_lcd_delay_advance_index = 0U;
    s_lcd_delay_start_ms = BSP_GET_TICK();
    s_lcd_delay_ms = LCD_INIT_RETRY_DELAY_MS;
    s_lcd_async_stage = LCD_ASYNC_INIT_DELAY;
    s_lcd_async_segment_busy = 0U;
    s_lcd_async_segment_done = 0U;
    Drv_LcdTft_Backlight(0U);
    Lcd_Unselect();
}

static BSP_Status_t Lcd_StartAsyncSegment(uint8_t data_mode, uint8_t *tx, uint16_t len)
{
    BSP_Status_t ret;

    if ((tx == 0) || (len == 0U) || (len > sizeof(s_lcd_async_rx))) {
        return BSP_PARAM;
    }

    Lcd_Select();
    if (data_mode) {
        Lcd_DataMode();
    } else {
        Lcd_CommandMode();
    }

    s_lcd_async_segment_busy = 1U;
    s_lcd_async_segment_done = 0U;
    s_lcd_async_status = BSP_BUSY;
    ret = BSP_SPI_TransferAsync_DMA(DRV_LCD_TFT_SPI_BUS,
                                    tx,
                                    s_lcd_async_rx,
                                    len,
                                    Lcd_AsyncCallback,
                                    0);
    if (ret != BSP_OK) {
        s_lcd_async_segment_busy = 0U;
        s_lcd_async_segment_done = 0U;
        s_lcd_async_status = ret;
        s_lcd_last_status = ret;
        Lcd_Unselect();
    }

    return ret;
}

static BSP_Status_t Lcd_PrepareFillChunk(void)
{
    uint32_t max_pixels;
    uint16_t max_rows;
    uint16_t remain_rows;
    uint16_t rows;
    uint32_t pixel_count;
    uint32_t i;
    uint16_t idx = 0U;

    if ((s_lcd_fill_active == 0U) || (s_lcd_fill_w == 0U) || (s_lcd_fill_h == 0U)) {
        return BSP_PARAM;
    }

    max_pixels = (uint32_t)sizeof(s_lcd_async_pixels) / 2U;
    max_rows = (uint16_t)(max_pixels / s_lcd_fill_w);
    if (max_rows == 0U) {
        return BSP_PARAM;
    }

    remain_rows = (uint16_t)((s_lcd_fill_y + s_lcd_fill_h) - s_lcd_fill_next_y);
    rows = (remain_rows > max_rows) ? max_rows : remain_rows;
    pixel_count = (uint32_t)s_lcd_fill_w * rows;

    for (i = 0U; i < pixel_count; i++) {
        s_lcd_async_pixels[idx++] = (uint8_t)(s_lcd_fill_color >> 8);
        s_lcd_async_pixels[idx++] = (uint8_t)s_lcd_fill_color;
    }

    s_lcd_async_x0 = s_lcd_fill_x;
    s_lcd_async_y0 = s_lcd_fill_next_y;
    s_lcd_async_x1 = (uint16_t)(s_lcd_fill_x + s_lcd_fill_w - 1U);
    s_lcd_async_y1 = (uint16_t)(s_lcd_fill_next_y + rows - 1U);
    s_lcd_async_pixel_len = idx;
    s_lcd_fill_chunk_rows = rows;
    s_lcd_async_stage = LCD_ASYNC_CASET_CMD;
    s_lcd_async_segment_busy = 0U;
    s_lcd_async_segment_done = 0U;
    s_lcd_async_status = BSP_OK;

    return BSP_OK;
}

static BSP_Status_t Lcd_BeginFillJob(uint16_t x, uint16_t y,
                                      uint16_t w, uint16_t h,
                                      uint16_t color)
{
    BSP_Status_t ret;

    if ((w == 0U) || (h == 0U) ||
        (x >= DRV_LCD_TFT_WIDTH) || (y >= DRV_LCD_TFT_HEIGHT)) {
        return BSP_PARAM;
    }

    if ((x + w) > DRV_LCD_TFT_WIDTH) {
        w = (uint16_t)(DRV_LCD_TFT_WIDTH - x);
    }
    if ((y + h) > DRV_LCD_TFT_HEIGHT) {
        h = (uint16_t)(DRV_LCD_TFT_HEIGHT - y);
    }

    if (((uint32_t)w * h) == 0U) {
        return BSP_PARAM;
    }

    s_lcd_fill_active = 1U;
    s_lcd_fill_x = x;
    s_lcd_fill_y = y;
    s_lcd_fill_w = w;
    s_lcd_fill_h = h;
    s_lcd_fill_next_y = y;
    s_lcd_fill_chunk_rows = 0U;
    s_lcd_fill_color = color;

    ret = Lcd_PrepareFillChunk();
    if (ret != BSP_OK) {
        s_lcd_fill_active = 0U;
        s_lcd_last_status = ret;
    }

    return ret;
}

static BSP_Status_t Lcd_StartRectSegment(void)
{
    while (s_lcd_rect_step < 4U) {
        switch (s_lcd_rect_step++) {
            case 0U:
                return Lcd_BeginFillJob(s_lcd_rect_x,
                                        s_lcd_rect_y,
                                        s_lcd_rect_w,
                                        1U,
                                        s_lcd_rect_color);

            case 1U:
                if (s_lcd_rect_h > 1U) {
                    return Lcd_BeginFillJob(s_lcd_rect_x,
                                            (uint16_t)(s_lcd_rect_y + s_lcd_rect_h - 1U),
                                            s_lcd_rect_w,
                                            1U,
                                            s_lcd_rect_color);
                }
                break;

            case 2U:
                if (s_lcd_rect_h > 2U) {
                    return Lcd_BeginFillJob(s_lcd_rect_x,
                                            (uint16_t)(s_lcd_rect_y + 1U),
                                            1U,
                                            (uint16_t)(s_lcd_rect_h - 2U),
                                            s_lcd_rect_color);
                }
                break;

            case 3U:
                if ((s_lcd_rect_w > 1U) && (s_lcd_rect_h > 2U)) {
                    return Lcd_BeginFillJob((uint16_t)(s_lcd_rect_x + s_lcd_rect_w - 1U),
                                            (uint16_t)(s_lcd_rect_y + 1U),
                                            1U,
                                            (uint16_t)(s_lcd_rect_h - 2U),
                                            s_lcd_rect_color);
                }
                break;

            default:
                break;
        }
    }

    s_lcd_rect_active = 0U;
    return BSP_OK;
}

static void Lcd_AdvanceInitAfterTransfer(void)
{
    const Lcd_InitOp_t *op;

    if (s_lcd_init_index >= (uint8_t)(sizeof(s_lcd_init_ops) / sizeof(s_lcd_init_ops[0]))) {
        s_lcd_async_stage = LCD_ASYNC_IDLE;
        s_lcd_ready = 1U;
        Drv_LcdTft_Backlight(1U);
        return;
    }

    op = &s_lcd_init_ops[s_lcd_init_index];
    if ((s_lcd_async_stage == LCD_ASYNC_INIT_CMD) && (op->data_len != 0U)) {
        s_lcd_async_stage = LCD_ASYNC_INIT_DATA;
        return;
    }

    if (op->delay_after_ms != 0U) {
        s_lcd_delay_start_ms = BSP_GET_TICK();
        s_lcd_delay_ms = op->delay_after_ms;
        s_lcd_delay_advance_index = 1U;
        s_lcd_async_stage = LCD_ASYNC_INIT_DELAY;
        return;
    }

    s_lcd_init_index++;
    if (s_lcd_init_index >= (uint8_t)(sizeof(s_lcd_init_ops) / sizeof(s_lcd_init_ops[0]))) {
        s_lcd_async_stage = LCD_ASYNC_IDLE;
        s_lcd_ready = 1U;
        Drv_LcdTft_Backlight(1U);
    } else {
        s_lcd_async_stage = LCD_ASYNC_INIT_CMD;
    }
}

static void Lcd_CompleteDelayIfElapsed(void)
{
    if ((uint32_t)(BSP_GET_TICK() - s_lcd_delay_start_ms) < s_lcd_delay_ms) {
        return;
    }

    if (s_lcd_delay_advance_index != 0U) {
        s_lcd_init_index++;
        s_lcd_delay_advance_index = 0U;
    }

    if (s_lcd_init_index >= (uint8_t)(sizeof(s_lcd_init_ops) / sizeof(s_lcd_init_ops[0]))) {
        s_lcd_async_stage = LCD_ASYNC_IDLE;
        s_lcd_ready = 1U;
        Drv_LcdTft_Backlight(1U);
    } else {
        s_lcd_async_stage = LCD_ASYNC_INIT_CMD;
    }
}

void Drv_LcdTft_Init(void)
{
    BSP_SPI_Init(DRV_LCD_TFT_SPI_BUS);
    BSP_GPIO_Init(DRV_LCD_TFT_CS_GPIO);
    BSP_GPIO_Init(DRV_LCD_TFT_DC_GPIO);
    BSP_GPIO_Init(DRV_LCD_TFT_BL_GPIO);

    Lcd_Unselect();
    Lcd_DataMode();
    Drv_LcdTft_Backlight(0U);

    s_lcd_ready = 0U;
    s_lcd_rotation = DRV_LCD_TFT_DEFAULT_ROTATION;
    s_lcd_madctl_data[0] = Lcd_GetMadctl(s_lcd_rotation);
    s_lcd_init_index = 0U;
    s_lcd_delay_advance_index = 0U;
    s_lcd_delay_start_ms = BSP_GET_TICK();
    s_lcd_delay_ms = 20U;
    s_lcd_async_stage = LCD_ASYNC_INIT_DELAY;
    s_lcd_async_segment_busy = 0U;
    s_lcd_async_segment_done = 0U;
    s_lcd_async_status = BSP_OK;
    s_lcd_last_status = BSP_OK;
    s_lcd_error_count = 0U;
}

uint8_t Drv_LcdTft_IsReady(void)
{
    return s_lcd_ready;
}

uint8_t Drv_LcdTft_IsBusy(void)
{
    return (uint8_t)((s_lcd_async_stage != LCD_ASYNC_IDLE) || (s_lcd_async_segment_busy != 0U));
}

BSP_Status_t Drv_LcdTft_GetLastStatus(void)
{
    return s_lcd_last_status;
}

uint8_t Drv_LcdTft_GetAsyncStage(void)
{
    return (uint8_t)s_lcd_async_stage;
}

uint16_t Drv_LcdTft_GetErrorCount(void)
{
    return s_lcd_error_count;
}

void Drv_LcdTft_Task(void)
{
    uint16_t x_start;
    uint16_t x_end;
    uint16_t y_start;
    uint16_t y_end;
    const Lcd_InitOp_t *op;

    if (s_lcd_async_stage == LCD_ASYNC_IDLE) {
        return;
    }

    if (s_lcd_async_segment_busy) {
        return;
    }

    if (s_lcd_async_segment_done) {
        s_lcd_async_segment_done = 0U;
        if (s_lcd_async_status != BSP_OK) {
            s_lcd_last_status = s_lcd_async_status;
            s_lcd_error_count++;
            s_lcd_fill_active = 0U;
            s_lcd_rect_active = 0U;
            if (Lcd_IsInitStage(s_lcd_async_stage) != 0U) {
                Lcd_RestartInitAfterError();
            } else {
                s_lcd_async_stage = LCD_ASYNC_IDLE;
                Lcd_Unselect();
            }
            return;
        }
        s_lcd_last_status = BSP_OK;

        if ((s_lcd_async_stage == LCD_ASYNC_INIT_CMD) ||
            (s_lcd_async_stage == LCD_ASYNC_INIT_DATA)) {
            Lcd_AdvanceInitAfterTransfer();
        } else if (s_lcd_async_stage == LCD_ASYNC_PIXEL_DATA) {
            if (s_lcd_fill_active != 0U) {
                s_lcd_fill_next_y = (uint16_t)(s_lcd_fill_next_y + s_lcd_fill_chunk_rows);
                if (s_lcd_fill_next_y < (uint16_t)(s_lcd_fill_y + s_lcd_fill_h)) {
                    if (Lcd_PrepareFillChunk() != BSP_OK) {
                        s_lcd_fill_active = 0U;
                        s_lcd_last_status = BSP_PARAM;
                        s_lcd_error_count++;
                        s_lcd_async_stage = LCD_ASYNC_IDLE;
                    }
                } else {
                    s_lcd_fill_active = 0U;
                    if (s_lcd_rect_active != 0U) {
                        BSP_Status_t rect_ret = Lcd_StartRectSegment();

                        if (rect_ret != BSP_OK) {
                            s_lcd_rect_active = 0U;
                            s_lcd_last_status = rect_ret;
                            s_lcd_error_count++;
                            s_lcd_async_stage = LCD_ASYNC_IDLE;
                        } else if (s_lcd_rect_active == 0U) {
                            s_lcd_async_stage = LCD_ASYNC_IDLE;
                        }
                    } else {
                        s_lcd_async_stage = LCD_ASYNC_IDLE;
                    }
                }
            } else {
                s_lcd_async_stage = LCD_ASYNC_IDLE;
            }
        } else {
            s_lcd_async_stage = (Lcd_AsyncStage_t)((uint8_t)s_lcd_async_stage + 1U);
        }
    }

    switch (s_lcd_async_stage) {
        case LCD_ASYNC_INIT_DELAY:
            Lcd_CompleteDelayIfElapsed();
            break;

        case LCD_ASYNC_INIT_CMD:
            if (s_lcd_init_index >= (uint8_t)(sizeof(s_lcd_init_ops) / sizeof(s_lcd_init_ops[0]))) {
                s_lcd_async_stage = LCD_ASYNC_IDLE;
                s_lcd_ready = 1U;
                Drv_LcdTft_Backlight(1U);
                break;
            }
            op = &s_lcd_init_ops[s_lcd_init_index];
            s_lcd_async_tx[0] = op->cmd;
            (void)Lcd_StartAsyncSegment(0U, s_lcd_async_tx, 1U);
            break;

        case LCD_ASYNC_INIT_DATA:
            op = &s_lcd_init_ops[s_lcd_init_index];
            if ((op->data == 0) || (op->data_len == 0U) || (op->data_len > sizeof(s_lcd_async_tx))) {
                s_lcd_last_status = BSP_PARAM;
                s_lcd_error_count++;
                s_lcd_async_stage = LCD_ASYNC_IDLE;
                break;
            }
            memcpy(s_lcd_async_tx, op->data, op->data_len);
            (void)Lcd_StartAsyncSegment(1U, s_lcd_async_tx, op->data_len);
            break;

        case LCD_ASYNC_CASET_CMD:
            s_lcd_async_tx[0] = LCD_CMD_CASET;
            (void)Lcd_StartAsyncSegment(0U, s_lcd_async_tx, 1U);
            break;

        case LCD_ASYNC_CASET_DATA:
            x_start = (uint16_t)(s_lcd_async_x0 + DRV_LCD_TFT_X_OFFSET);
            x_end = (uint16_t)(s_lcd_async_x1 + DRV_LCD_TFT_X_OFFSET);
            s_lcd_async_window[0] = (uint8_t)(x_start >> 8);
            s_lcd_async_window[1] = (uint8_t)x_start;
            s_lcd_async_window[2] = (uint8_t)(x_end >> 8);
            s_lcd_async_window[3] = (uint8_t)x_end;
            (void)Lcd_StartAsyncSegment(1U, s_lcd_async_window, 4U);
            break;

        case LCD_ASYNC_RASET_CMD:
            s_lcd_async_tx[0] = LCD_CMD_RASET;
            (void)Lcd_StartAsyncSegment(0U, s_lcd_async_tx, 1U);
            break;

        case LCD_ASYNC_RASET_DATA:
            y_start = (uint16_t)(s_lcd_async_y0 + DRV_LCD_TFT_Y_OFFSET);
            y_end = (uint16_t)(s_lcd_async_y1 + DRV_LCD_TFT_Y_OFFSET);
            s_lcd_async_window[0] = (uint8_t)(y_start >> 8);
            s_lcd_async_window[1] = (uint8_t)y_start;
            s_lcd_async_window[2] = (uint8_t)(y_end >> 8);
            s_lcd_async_window[3] = (uint8_t)y_end;
            (void)Lcd_StartAsyncSegment(1U, s_lcd_async_window, 4U);
            break;

        case LCD_ASYNC_RAMWR_CMD:
            s_lcd_async_tx[0] = LCD_CMD_RAMWR;
            (void)Lcd_StartAsyncSegment(0U, s_lcd_async_tx, 1U);
            break;

        case LCD_ASYNC_PIXEL_DATA:
            (void)Lcd_StartAsyncSegment(1U, s_lcd_async_pixels, s_lcd_async_pixel_len);
            break;

        default:
            s_lcd_async_stage = LCD_ASYNC_IDLE;
            break;
    }
}

void Drv_LcdTft_Backlight(uint8_t on)
{
    uint8_t level = on ? DRV_LCD_TFT_BL_ACTIVE_LEVEL : (uint8_t)(1U - DRV_LCD_TFT_BL_ACTIVE_LEVEL);

    BSP_GPIO_Write(DRV_LCD_TFT_BL_GPIO, level);
}

uint16_t Drv_LcdTft_Rgb888To565(uint32_t rgb888)
{
    uint16_t r = (uint16_t)((rgb888 >> 16) & 0xFFU);
    uint16_t g = (uint16_t)((rgb888 >> 8) & 0xFFU);
    uint16_t b = (uint16_t)(rgb888 & 0xFFU);

    return (uint16_t)(((r & 0xF8U) << 8) | ((g & 0xFCU) << 3) | (b >> 3));
}

BSP_Status_t Drv_LcdTft_TryClear(uint16_t color)
{
    return Drv_LcdTft_TryFillRect(0U, 0U,
                                  DRV_LCD_TFT_WIDTH,
                                  DRV_LCD_TFT_HEIGHT,
                                  color);
}

BSP_Status_t Drv_LcdTft_TryFill(uint16_t color)
{
    return Drv_LcdTft_TryClear(color);
}

BSP_Status_t Drv_LcdTft_TryFillRect(uint16_t x, uint16_t y,
                                      uint16_t w, uint16_t h,
                                      uint16_t color)
{
    BSP_Status_t ret;

    if ((s_lcd_ready == 0U) || (Drv_LcdTft_IsBusy() != 0U)) {
        return BSP_BUSY;
    }

    s_lcd_rect_active = 0U;
    ret = Lcd_BeginFillJob(x, y, w, h, color);
    if (ret != BSP_OK) {
        return ret;
    }

    Drv_LcdTft_Task();

    return BSP_OK;
}

BSP_Status_t Drv_LcdTft_TryDrawHLine(uint16_t x, uint16_t y,
                                      uint16_t w, uint16_t color)
{
    return Drv_LcdTft_TryFillRect(x, y, w, 1U, color);
}

BSP_Status_t Drv_LcdTft_TryDrawVLine(uint16_t x, uint16_t y,
                                      uint16_t h, uint16_t color)
{
    return Drv_LcdTft_TryFillRect(x, y, 1U, h, color);
}

BSP_Status_t Drv_LcdTft_TryDrawRect(uint16_t x, uint16_t y,
                                     uint16_t w, uint16_t h,
                                     uint16_t color)
{
    BSP_Status_t ret;

    if ((s_lcd_ready == 0U) || (Drv_LcdTft_IsBusy() != 0U)) {
        return BSP_BUSY;
    }

    if ((w == 0U) || (h == 0U) ||
        (x >= DRV_LCD_TFT_WIDTH) || (y >= DRV_LCD_TFT_HEIGHT)) {
        return BSP_PARAM;
    }

    if ((x + w) > DRV_LCD_TFT_WIDTH) {
        w = (uint16_t)(DRV_LCD_TFT_WIDTH - x);
    }
    if ((y + h) > DRV_LCD_TFT_HEIGHT) {
        h = (uint16_t)(DRV_LCD_TFT_HEIGHT - y);
    }

    s_lcd_rect_active = 1U;
    s_lcd_rect_step = 0U;
    s_lcd_rect_x = x;
    s_lcd_rect_y = y;
    s_lcd_rect_w = w;
    s_lcd_rect_h = h;
    s_lcd_rect_color = color;

    ret = Lcd_StartRectSegment();
    if (ret != BSP_OK) {
        s_lcd_rect_active = 0U;
        s_lcd_last_status = ret;
        return ret;
    }

    Drv_LcdTft_Task();

    return BSP_OK;
}

BSP_Status_t Drv_LcdTft_TryDrawStringLine(uint16_t x, uint16_t y, const char *str,
                                           uint16_t fg, uint16_t bg)
{
    uint8_t glyph[DRV_LCD_FONT_5X7_WIDTH];
    uint16_t idx = 0U;
    uint16_t cur_x = x;
    uint8_t char_w = (uint8_t)(DRV_LCD_FONT_5X7_WIDTH + DRV_LCD_FONT_5X7_X_SPACE);
    uint8_t char_count = 0U;
    uint8_t col;
    uint8_t row;

    if ((s_lcd_ready == 0U) || (str == 0)) {
        return BSP_BUSY;
    }

    if ((x >= DRV_LCD_TFT_WIDTH) || (y >= DRV_LCD_TFT_HEIGHT)) {
        return BSP_PARAM;
    }

    if (Drv_LcdTft_IsBusy() != 0U) {
        return BSP_BUSY;
    }

    while ((str[char_count] != '\0') &&
           (str[char_count] != '\n') &&
           (char_count < DRV_LCD_TFT_ASYNC_TEXT_MAX_CHARS) &&
           ((cur_x + char_w) <= DRV_LCD_TFT_WIDTH)) {
        cur_x = (uint16_t)(cur_x + char_w);
        char_count++;
    }

    if ((char_count == 0U) ||
        ((y + DRV_LCD_FONT_5X7_HEIGHT) > DRV_LCD_TFT_HEIGHT)) {
        return BSP_PARAM;
    }

    for (row = 0U; row < DRV_LCD_FONT_5X7_HEIGHT; row++) {
        uint8_t ch_idx;

        for (ch_idx = 0U; ch_idx < char_count; ch_idx++) {
            Drv_LcdFont_GetGlyph5x7(str[ch_idx], glyph);
            for (col = 0U; col < char_w; col++) {
                uint16_t color = bg;

                if ((col < DRV_LCD_FONT_5X7_WIDTH) && ((glyph[col] & (1U << row)) != 0U)) {
                    color = fg;
                }

                s_lcd_async_pixels[idx++] = (uint8_t)(color >> 8);
                s_lcd_async_pixels[idx++] = (uint8_t)color;
            }
        }
    }

    s_lcd_async_x0 = x;
    s_lcd_async_y0 = y;
    s_lcd_async_x1 = (uint16_t)(x + ((uint16_t)char_count * char_w) - 1U);
    s_lcd_async_y1 = (uint16_t)(y + DRV_LCD_FONT_5X7_HEIGHT - 1U);
    s_lcd_async_pixel_len = idx;
    s_lcd_fill_active = 0U;
    s_lcd_rect_active = 0U;
    s_lcd_async_stage = LCD_ASYNC_CASET_CMD;
    s_lcd_async_segment_busy = 0U;
    s_lcd_async_segment_done = 0U;
    s_lcd_async_status = BSP_OK;
    Drv_LcdTft_Task();

    return BSP_OK;
}

BSP_Status_t Drv_LcdTft_TryDrawString5x7(uint16_t x, uint16_t y,
                                          const char *str,
                                          uint16_t fg,
                                          uint16_t bg)
{
    return Drv_LcdTft_TryDrawStringLine(x, y, str, fg, bg);
}
