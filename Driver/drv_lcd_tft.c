#include "drv_lcd_tft.h"
#include "drv_lcd_font.h"
#include "bsp_systick.h"

#define LCD_CMD_CASET       0x2AU
#define LCD_CMD_RASET       0x2BU
#define LCD_CMD_RAMWR       0x2CU
#define LCD_CMD_MADCTL      0x36U
#define LCD_CMD_COLMOD      0x3AU

static uint8_t s_lcd_ready = 0U;
static uint8_t s_lcd_rotation = DRV_LCD_TFT_DEFAULT_ROTATION;

static void Lcd_WaitMs(uint32_t ms)
{
    uint32_t start = BSP_GET_TICK();
    uint32_t guard = ms * 30000UL + 1UL;

    while ((uint32_t)(BSP_GET_TICK() - start) < ms) {
        if (guard-- == 0U) {
            break;
        }
    }
}

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

static void Lcd_WriteBytes(const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len == 0U)) {
        return;
    }

    (void)BSP_SPI_Transfer(DRV_LCD_TFT_SPI_BUS, data, 0, len, 0x00U);
}

static void Lcd_WriteCommand(uint8_t cmd)
{
    Lcd_Select();
    Lcd_CommandMode();
    Lcd_WriteBytes(&cmd, 1U);
    Lcd_Unselect();
}

static void Lcd_WriteData8(uint8_t data)
{
    Lcd_Select();
    Lcd_DataMode();
    Lcd_WriteBytes(&data, 1U);
    Lcd_Unselect();
}

static void Lcd_WriteData(const uint8_t *data, uint16_t len)
{
    Lcd_Select();
    Lcd_DataMode();
    Lcd_WriteBytes(data, len);
    Lcd_Unselect();
}

static void Lcd_WriteData16(uint16_t data)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(data >> 8);
    buf[1] = (uint8_t)data;
    Lcd_WriteData(buf, 2U);
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

void Drv_LcdTft_Init(void)
{
    static const uint8_t b2[] = {0x0CU, 0x0CU, 0x00U, 0x33U, 0x33U};
    static const uint8_t d0[] = {0xA4U, 0xA1U};
    static const uint8_t e0[] = {0xD0U, 0x04U, 0x0DU, 0x11U, 0x13U, 0x2BU, 0x3FU,
                                 0x54U, 0x4CU, 0x18U, 0x0DU, 0x0BU, 0x1FU, 0x23U};
    static const uint8_t e1[] = {0xD0U, 0x04U, 0x0CU, 0x11U, 0x13U, 0x2CU, 0x3FU,
                                 0x44U, 0x51U, 0x2FU, 0x1FU, 0x1FU, 0x20U, 0x23U};

    BSP_SPI_Init(DRV_LCD_TFT_SPI_BUS);
    BSP_GPIO_Init(DRV_LCD_TFT_CS_GPIO);
    BSP_GPIO_Init(DRV_LCD_TFT_DC_GPIO);
    BSP_GPIO_Init(DRV_LCD_TFT_BL_GPIO);

    Lcd_Unselect();
    Lcd_DataMode();
    Drv_LcdTft_Backlight(0U);
    Lcd_WaitMs(20U);

    Lcd_WriteCommand(0x11U);
    Lcd_WaitMs(120U);

    Lcd_WriteCommand(LCD_CMD_COLMOD);
    Lcd_WriteData8(0x55U);

    Lcd_WriteCommand(LCD_CMD_MADCTL);
    Lcd_WriteData8(Lcd_GetMadctl(s_lcd_rotation));

    Lcd_WriteCommand(0xB2U);
    Lcd_WriteData(b2, (uint16_t)sizeof(b2));
    Lcd_WriteCommand(0xB7U);
    Lcd_WriteData8(0x35U);
    Lcd_WriteCommand(0xBBU);
    Lcd_WriteData8(0x19U);
    Lcd_WriteCommand(0xC0U);
    Lcd_WriteData8(0x2CU);
    Lcd_WriteCommand(0xC2U);
    Lcd_WriteData8(0x01U);
    Lcd_WriteCommand(0xC3U);
    Lcd_WriteData8(0x12U);
    Lcd_WriteCommand(0xC4U);
    Lcd_WriteData8(0x20U);
    Lcd_WriteCommand(0xC6U);
    Lcd_WriteData8(0x0FU);
    Lcd_WriteCommand(0xD0U);
    Lcd_WriteData(d0, (uint16_t)sizeof(d0));
    Lcd_WriteCommand(0xE0U);
    Lcd_WriteData(e0, (uint16_t)sizeof(e0));
    Lcd_WriteCommand(0xE1U);
    Lcd_WriteData(e1, (uint16_t)sizeof(e1));
    Lcd_WriteCommand(0x21U);
    Lcd_WriteCommand(0x29U);
    Lcd_WaitMs(20U);

    s_lcd_ready = 1U;
    Drv_LcdTft_FillScreen(DRV_LCD_COLOR_BLACK);
    Drv_LcdTft_Backlight(1U);
}

uint8_t Drv_LcdTft_IsReady(void)
{
    return s_lcd_ready;
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

void Drv_LcdTft_SetRotation(uint8_t rotation)
{
    s_lcd_rotation = (uint8_t)(rotation & 0x03U);
    Lcd_WriteCommand(LCD_CMD_MADCTL);
    Lcd_WriteData8(Lcd_GetMadctl(s_lcd_rotation));
}

void Drv_LcdTft_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint16_t x_start = (uint16_t)(x0 + DRV_LCD_TFT_X_OFFSET);
    uint16_t x_end = (uint16_t)(x1 + DRV_LCD_TFT_X_OFFSET);
    uint16_t y_start = (uint16_t)(y0 + DRV_LCD_TFT_Y_OFFSET);
    uint16_t y_end = (uint16_t)(y1 + DRV_LCD_TFT_Y_OFFSET);

    Lcd_WriteCommand(LCD_CMD_CASET);
    Lcd_WriteData16(x_start);
    Lcd_WriteData16(x_end);

    Lcd_WriteCommand(LCD_CMD_RASET);
    Lcd_WriteData16(y_start);
    Lcd_WriteData16(y_end);

    Lcd_WriteCommand(LCD_CMD_RAMWR);
}

void Drv_LcdTft_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if ((x >= DRV_LCD_TFT_WIDTH) || (y >= DRV_LCD_TFT_HEIGHT) || (s_lcd_ready == 0U)) {
        return;
    }

    Drv_LcdTft_SetWindow(x, y, x, y);
    Lcd_WriteData16(color);
}

void Drv_LcdTft_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    uint32_t pixels;
    uint8_t buf[64U * 2U];
    uint16_t i;

    if ((s_lcd_ready == 0U) || (w == 0U) || (h == 0U) ||
        (x >= DRV_LCD_TFT_WIDTH) || (y >= DRV_LCD_TFT_HEIGHT)) {
        return;
    }

    if ((uint32_t)x + w > DRV_LCD_TFT_WIDTH) {
        w = (uint16_t)(DRV_LCD_TFT_WIDTH - x);
    }
    if ((uint32_t)y + h > DRV_LCD_TFT_HEIGHT) {
        h = (uint16_t)(DRV_LCD_TFT_HEIGHT - y);
    }

    for (i = 0U; i < 64U; i++) {
        buf[i * 2U] = (uint8_t)(color >> 8);
        buf[i * 2U + 1U] = (uint8_t)color;
    }

    Drv_LcdTft_SetWindow(x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U));
    pixels = (uint32_t)w * h;

    Lcd_Select();
    Lcd_DataMode();
    while (pixels >= 64U) {
        Lcd_WriteBytes(buf, (uint16_t)sizeof(buf));
        pixels -= 64U;
    }
    if (pixels > 0U) {
        Lcd_WriteBytes(buf, (uint16_t)(pixels * 2U));
    }
    Lcd_Unselect();
}

void Drv_LcdTft_FillScreen(uint16_t color)
{
    Drv_LcdTft_FillRect(0U, 0U, DRV_LCD_TFT_WIDTH, DRV_LCD_TFT_HEIGHT, color);
}

void Drv_LcdTft_DrawHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color)
{
    Drv_LcdTft_FillRect(x, y, w, 1U, color);
}

void Drv_LcdTft_DrawVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color)
{
    Drv_LcdTft_FillRect(x, y, 1U, h, color);
}

void Drv_LcdTft_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if ((w == 0U) || (h == 0U)) {
        return;
    }

    Drv_LcdTft_DrawHLine(x, y, w, color);
    Drv_LcdTft_DrawHLine(x, (uint16_t)(y + h - 1U), w, color);
    Drv_LcdTft_DrawVLine(x, y, h, color);
    Drv_LcdTft_DrawVLine((uint16_t)(x + w - 1U), y, h, color);
}

void Drv_LcdTft_DrawChar(uint16_t x, uint16_t y, char ch, uint16_t fg, uint16_t bg)
{
    uint8_t glyph[DRV_LCD_FONT_5X7_WIDTH];
    uint8_t buf[(DRV_LCD_FONT_5X7_WIDTH + DRV_LCD_FONT_5X7_X_SPACE) *
                DRV_LCD_FONT_5X7_HEIGHT * 2U];
    uint16_t idx = 0U;
    uint8_t char_w = (uint8_t)(DRV_LCD_FONT_5X7_WIDTH + DRV_LCD_FONT_5X7_X_SPACE);
    uint8_t col;
    uint8_t row;

    if ((x + char_w > DRV_LCD_TFT_WIDTH) ||
        (y + DRV_LCD_FONT_5X7_HEIGHT > DRV_LCD_TFT_HEIGHT)) {
        return;
    }

    Drv_LcdFont_GetGlyph5x7(ch, glyph);
    for (row = 0U; row < DRV_LCD_FONT_5X7_HEIGHT; row++) {
        for (col = 0U; col < char_w; col++) {
            uint16_t color = bg;

            if ((col < DRV_LCD_FONT_5X7_WIDTH) && ((glyph[col] & (1U << row)) != 0U)) {
                color = fg;
            }

            buf[idx++] = (uint8_t)(color >> 8);
            buf[idx++] = (uint8_t)color;
        }
    }

    Drv_LcdTft_SetWindow(x, y,
                         (uint16_t)(x + char_w - 1U),
                         (uint16_t)(y + DRV_LCD_FONT_5X7_HEIGHT - 1U));
    Lcd_WriteData(buf, idx);
}

void Drv_LcdTft_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg)
{
    uint16_t cur_x = x;
    uint16_t cur_y = y;
    uint16_t step = (uint16_t)(DRV_LCD_FONT_5X7_WIDTH + DRV_LCD_FONT_5X7_X_SPACE);

    if (str == 0) {
        return;
    }

    while (*str != '\0') {
        if (*str == '\n') {
            cur_x = x;
            cur_y = (uint16_t)(cur_y + DRV_LCD_FONT_5X7_HEIGHT + DRV_LCD_FONT_5X7_Y_SPACE);
        } else {
            if ((cur_x + DRV_LCD_FONT_5X7_WIDTH > DRV_LCD_TFT_WIDTH) ||
                (cur_y + DRV_LCD_FONT_5X7_HEIGHT > DRV_LCD_TFT_HEIGHT)) {
                break;
            }
            Drv_LcdTft_DrawChar(cur_x, cur_y, *str, fg, bg);
            cur_x = (uint16_t)(cur_x + step);
        }
        str++;
    }
}

void Drv_LcdTft_DrawMonoImage(uint16_t x, uint16_t y, const Drv_Lcd_MonoImage_t *img,
                              uint16_t fg, uint16_t bg)
{
    uint16_t ix;
    uint16_t iy;
    uint32_t bit_index;
    uint8_t byte;

    if ((img == 0) || (img->data == 0)) {
        return;
    }

    for (iy = 0U; iy < img->height; iy++) {
        for (ix = 0U; ix < img->width; ix++) {
            bit_index = (uint32_t)iy * img->width + ix;
            byte = img->data[bit_index / 8U];
            Drv_LcdTft_DrawPixel((uint16_t)(x + ix), (uint16_t)(y + iy),
                                 (byte & (0x80U >> (bit_index & 0x07U))) ? fg : bg);
        }
    }
}

void Drv_LcdTft_DrawRgb565Image(uint16_t x, uint16_t y, const Drv_Lcd_Rgb565Image_t *img)
{
    uint32_t count;
    uint32_t i;

    if ((img == 0) || (img->data == 0) || (img->width == 0U) || (img->height == 0U) ||
        (x >= DRV_LCD_TFT_WIDTH) || (y >= DRV_LCD_TFT_HEIGHT)) {
        return;
    }

    if (((uint32_t)x + img->width > DRV_LCD_TFT_WIDTH) ||
        ((uint32_t)y + img->height > DRV_LCD_TFT_HEIGHT)) {
        return;
    }

    Drv_LcdTft_SetWindow(x, y, (uint16_t)(x + img->width - 1U), (uint16_t)(y + img->height - 1U));
    count = (uint32_t)img->width * img->height;

    Lcd_Select();
    Lcd_DataMode();
    for (i = 0U; i < count; i++) {
        uint8_t buf[2];

        buf[0] = (uint8_t)(img->data[i] >> 8);
        buf[1] = (uint8_t)img->data[i];
        Lcd_WriteBytes(buf, 2U);
    }
    Lcd_Unselect();
}
