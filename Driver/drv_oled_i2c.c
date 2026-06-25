#include "drv_oled_i2c.h"
#include "drv_oled_font.h"
#include "bsp_systick.h"
#include <string.h>

#define OLED_CTRL_CMD                0x00U
#define OLED_CTRL_DATA               0x40U
#define OLED_BUFFER_SIZE             (DRV_OLED_WIDTH * DRV_OLED_PAGE_NUM)

static uint8_t s_oled_buf[OLED_BUFFER_SIZE];
static uint8_t s_oled_ready = 0U;

static void Oled_WaitMs(uint32_t ms)
{
    uint32_t start = BSP_GET_TICK();
    uint32_t guard = ms * 30000UL + 1UL;

    while ((uint32_t)(BSP_GET_TICK() - start) < ms) {
        if (guard-- == 0U) {
            break;
        }
    }
}

static BSP_Status_t Oled_WriteCommand(uint8_t cmd)
{
    uint8_t buf[2];

    buf[0] = OLED_CTRL_CMD;
    buf[1] = cmd;
    return BSP_I2C_MasterWrite(DRV_OLED_I2C_BUS, DRV_OLED_I2C_ADDR_7BIT, buf, 2U);
}

static BSP_Status_t Oled_WriteData(const uint8_t *data, uint16_t len)
{
    uint8_t tx[DRV_OLED_WIDTH + 1U];

    if ((data == 0) || (len == 0U) || (len > DRV_OLED_WIDTH)) {
        return BSP_PARAM;
    }

    tx[0] = OLED_CTRL_DATA;
    memcpy(&tx[1], data, len);
    return BSP_I2C_MasterWrite(DRV_OLED_I2C_BUS, DRV_OLED_I2C_ADDR_7BIT, tx, (uint16_t)(len + 1U));
}

static void Oled_SetPageColumn(uint8_t page, uint8_t col)
{
    (void)Oled_WriteCommand((uint8_t)(0xB0U | (page & 0x07U)));
    (void)Oled_WriteCommand((uint8_t)(0x00U | (col & 0x0FU)));
    (void)Oled_WriteCommand((uint8_t)(0x10U | ((col >> 4) & 0x0FU)));
}

void Drv_OledI2c_Init(void)
{
    BSP_Status_t ret = BSP_OK;

#define OLED_INIT_CMD(cmd) do { if (Oled_WriteCommand((cmd)) != BSP_OK) { ret = BSP_ERROR; } } while (0)

    BSP_I2C_Init(DRV_OLED_I2C_BUS);
    Oled_WaitMs(50U);

    OLED_INIT_CMD(0xAEU);
    OLED_INIT_CMD(0xD5U);
    OLED_INIT_CMD(0x80U);
    OLED_INIT_CMD(0xA8U);
    OLED_INIT_CMD(0x3FU);
    OLED_INIT_CMD(0xD3U);
    OLED_INIT_CMD(0x00U);
    OLED_INIT_CMD(0x40U);
    OLED_INIT_CMD(0xA1U);
    OLED_INIT_CMD(0xC8U);
    OLED_INIT_CMD(0xDAU);
    OLED_INIT_CMD(0x12U);
    OLED_INIT_CMD(0x81U);
    OLED_INIT_CMD(0xCFU);
    OLED_INIT_CMD(0xD9U);
    OLED_INIT_CMD(0xF1U);
    OLED_INIT_CMD(0xDBU);
    OLED_INIT_CMD(0x30U);
    OLED_INIT_CMD(0xA4U);
    OLED_INIT_CMD(0xA6U);
    OLED_INIT_CMD(0x8DU);
    OLED_INIT_CMD(0x14U);
    OLED_INIT_CMD(0xAFU);

    s_oled_ready = (ret == BSP_OK) ? 1U : 0U;
    if (s_oled_ready) {
        Drv_OledI2c_Clear();
    }

#undef OLED_INIT_CMD
}

uint8_t Drv_OledI2c_IsReady(void)
{
    return s_oled_ready;
}

void Drv_OledI2c_DisplayOn(uint8_t on)
{
    (void)Oled_WriteCommand(on ? 0xAFU : 0xAEU);
}

void Drv_OledI2c_Clear(void)
{
    Drv_OledI2c_Fill(DRV_OLED_COLOR_OFF);
}

void Drv_OledI2c_Fill(uint8_t color)
{
    memset(s_oled_buf, color ? 0xFF : 0x00, sizeof(s_oled_buf));
    Drv_OledI2c_Flush();
}

void Drv_OledI2c_Flush(void)
{
    uint8_t page;

    if (s_oled_ready == 0U) {
        return;
    }

    for (page = 0U; page < DRV_OLED_PAGE_NUM; page++) {
        Drv_OledI2c_FlushPage(page);
    }
}

void Drv_OledI2c_FlushPage(uint8_t page)
{
    if ((s_oled_ready == 0U) || (page >= DRV_OLED_PAGE_NUM)) {
        return;
    }

    Oled_SetPageColumn(page, 0U);
    (void)Oled_WriteData(&s_oled_buf[(uint16_t)page * DRV_OLED_WIDTH], DRV_OLED_WIDTH);
}

void Drv_OledI2c_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    uint16_t index;
    uint8_t mask;

    if ((x >= DRV_OLED_WIDTH) || (y >= DRV_OLED_HEIGHT)) {
        return;
    }

    index = (uint16_t)(y / 8U) * DRV_OLED_WIDTH + x;
    mask = (uint8_t)(1U << (y & 0x07U));
    if (color) {
        s_oled_buf[index] |= mask;
    } else {
        s_oled_buf[index] &= (uint8_t)(~mask);
    }
}

void Drv_OledI2c_DrawHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color)
{
    uint8_t i;

    for (i = 0U; i < w; i++) {
        Drv_OledI2c_DrawPixel((uint8_t)(x + i), y, color);
    }
}

void Drv_OledI2c_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
    uint8_t ix;
    uint8_t iy;

    for (iy = 0U; iy < h; iy++) {
        for (ix = 0U; ix < w; ix++) {
            Drv_OledI2c_DrawPixel((uint8_t)(x + ix), (uint8_t)(y + iy), color);
        }
    }
}

void Drv_OledI2c_DrawVLine(uint8_t x, uint8_t y, uint8_t h, uint8_t color)
{
    uint8_t i;

    for (i = 0U; i < h; i++) {
        Drv_OledI2c_DrawPixel(x, (uint8_t)(y + i), color);
    }
}

void Drv_OledI2c_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
    if ((w == 0U) || (h == 0U)) {
        return;
    }

    Drv_OledI2c_DrawHLine(x, y, w, color);
    Drv_OledI2c_DrawHLine(x, (uint8_t)(y + h - 1U), w, color);
    Drv_OledI2c_DrawVLine(x, y, h, color);
    Drv_OledI2c_DrawVLine((uint8_t)(x + w - 1U), y, h, color);
}

void Drv_OledI2c_DrawChar5x7(uint8_t x, uint8_t y, char ch, uint8_t color)
{
    uint8_t glyph[DRV_OLED_FONT_5X7_WIDTH];
    uint8_t col;
    uint8_t row;

    Drv_OledFont_GetGlyph5x7(ch, glyph);
    for (col = 0U; col < DRV_OLED_FONT_5X7_WIDTH; col++) {
        for (row = 0U; row < DRV_OLED_FONT_5X7_HEIGHT; row++) {
            if ((glyph[col] & (1U << row)) != 0U) {
                Drv_OledI2c_DrawPixel((uint8_t)(x + col), (uint8_t)(y + row), color);
            } else {
                Drv_OledI2c_DrawPixel((uint8_t)(x + col), (uint8_t)(y + row), (uint8_t)!color);
            }
        }
    }

    for (row = 0U; row < DRV_OLED_FONT_5X7_HEIGHT; row++) {
        Drv_OledI2c_DrawPixel((uint8_t)(x + DRV_OLED_FONT_5X7_WIDTH),
                              (uint8_t)(y + row),
                              (uint8_t)!color);
    }
}

void Drv_OledI2c_DrawString5x7(uint8_t x, uint8_t y, const char *str, uint8_t color)
{
    uint8_t cur_x = x;

    if (str == 0) {
        return;
    }

    while (*str != '\0') {
        if (cur_x + DRV_OLED_FONT_5X7_WIDTH > DRV_OLED_WIDTH) {
            break;
        }
        Drv_OledI2c_DrawChar5x7(cur_x, y, *str, color);
        cur_x = (uint8_t)(cur_x + DRV_OLED_FONT_5X7_WIDTH + DRV_OLED_FONT_5X7_X_SPACE);
        str++;
    }
}

void Drv_OledI2c_DrawMonoImage(uint8_t x, uint8_t y, const Drv_Oled_MonoImage_t *img, uint8_t color)
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
            Drv_OledI2c_DrawPixel((uint8_t)(x + ix), (uint8_t)(y + iy),
                                  (byte & (0x80U >> (bit_index & 0x07U))) ? color : (uint8_t)!color);
        }
    }
}

uint8_t *Drv_OledI2c_GetBuffer(void)
{
    return s_oled_buf;
}

uint16_t Drv_OledI2c_GetBufferSize(void)
{
    return (uint16_t)sizeof(s_oled_buf);
}
