#include "drv_oled_i2c.h"
#include "drv_oled_font.h"
#include "bsp_systick.h"
#include <string.h>

#define OLED_CTRL_CMD                0x00U
#define OLED_CTRL_DATA               0x40U
#define OLED_BUFFER_SIZE             (DRV_OLED_WIDTH * DRV_OLED_PAGE_NUM)
#define OLED_ASYNC_TX_BUF_LEN        32U
#define OLED_ASYNC_DATA_CHUNK        (OLED_ASYNC_TX_BUF_LEN - 1U)
#define OLED_INIT_POWER_ON_DELAY_MS  50U
#define OLED_INIT_RETRY_DELAY_MS     100U

typedef enum {
    OLED_ASYNC_IDLE = 0,
    OLED_ASYNC_INIT_WAIT,
    OLED_ASYNC_INIT_CMD_BUSY,
    OLED_ASYNC_FLUSH_CMD_BUSY,
    OLED_ASYNC_FLUSH_DATA_BUSY,
    OLED_ASYNC_ERROR
} Oled_AsyncState_t;

static const uint8_t s_oled_init_cmds[] = {
    0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
    0xA1U, 0xC8U, 0xDAU, 0x12U, 0x81U, 0xCFU, 0xD9U, 0xF1U,
    0xDBU, 0x30U, 0xA4U, 0xA6U, 0x8DU, 0x14U, 0xAFU
};

static uint8_t s_oled_buf[OLED_BUFFER_SIZE];
static uint8_t s_oled_ready = 0U;
static uint8_t s_oled_dirty_pages = 0U;
static uint8_t s_oled_init_index = 0U;
static uint8_t s_oled_async_page = 0U;
static uint8_t s_oled_async_col = 0U;
static uint8_t s_oled_async_tx[OLED_ASYNC_TX_BUF_LEN];
static uint8_t s_oled_active_addr = DRV_OLED_I2C_ADDR_7BIT;
static uint32_t s_oled_delay_start_ms = 0U;
static uint16_t s_oled_delay_ms = OLED_INIT_POWER_ON_DELAY_MS;
static uint16_t s_oled_error_count = 0U;
static volatile Oled_AsyncState_t s_oled_async_state = OLED_ASYNC_IDLE;
static volatile uint8_t s_oled_async_done = 0U;
static volatile BSP_Status_t s_oled_async_status = BSP_OK;
static BSP_Status_t s_oled_last_status = BSP_OK;

static void Oled_AsyncCallback(I2C_Bus_t bus, int result)
{
    (void)bus;
    s_oled_async_status = (result == 0) ? BSP_OK : BSP_ERROR;
    s_oled_async_done = 1U;
}

static void Oled_DetectAddressOnInit(void)
{
    uint8_t addr[8];
    uint8_t found = 0U;
    uint8_t i;

    s_oled_active_addr = DRV_OLED_I2C_ADDR_7BIT;

    if (BSP_I2C_ScanBus(DRV_OLED_I2C_BUS, addr, (uint8_t)sizeof(addr), &found) != BSP_OK) {
        return;
    }

    for (i = 0U; (i < found) && (i < (uint8_t)sizeof(addr)); i++) {
        if (addr[i] == DRV_OLED_I2C_ADDR_7BIT) {
            s_oled_active_addr = DRV_OLED_I2C_ADDR_7BIT;
            return;
        }
        if (addr[i] == DRV_OLED_I2C_ADDR_ALT_7BIT) {
            s_oled_active_addr = DRV_OLED_I2C_ADDR_ALT_7BIT;
            return;
        }
    }
}

static uint8_t Oled_FindNextDirtyPage(void)
{
    uint8_t page;

    for (page = 0U; page < DRV_OLED_PAGE_NUM; page++) {
        if ((s_oled_dirty_pages & (uint8_t)(1U << page)) != 0U) {
            return page;
        }
    }

    return DRV_OLED_PAGE_NUM;
}

static BSP_Status_t Oled_StartAsyncWrite(const uint8_t *data, uint16_t len)
{
    BSP_Status_t ret;

    if ((data == 0) || (len == 0U) || (len > OLED_ASYNC_TX_BUF_LEN)) {
        return BSP_PARAM;
    }

    memcpy(s_oled_async_tx, data, len);
    s_oled_async_done = 0U;
    s_oled_async_status = BSP_BUSY;
    ret = BSP_I2C_MasterWrite_DMA_Async(DRV_OLED_I2C_BUS,
                                        s_oled_active_addr,
                                        s_oled_async_tx,
                                        len,
                                        Oled_AsyncCallback);
    if (ret != BSP_OK) {
        s_oled_async_done = 0U;
        s_oled_async_status = ret;
        s_oled_last_status = ret;
    }

    return ret;
}

static void Oled_RestartInitAfterError(void)
{
    s_oled_ready = 0U;
    s_oled_init_index = 0U;
    s_oled_async_done = 0U;
    s_oled_delay_start_ms = BSP_GET_TICK();
    s_oled_delay_ms = OLED_INIT_RETRY_DELAY_MS;
    s_oled_async_state = OLED_ASYNC_INIT_WAIT;
}

static void Oled_StartInitCommand(void)
{
    uint8_t tx[2];
    BSP_Status_t ret;

    if (s_oled_init_index >= (uint8_t)sizeof(s_oled_init_cmds)) {
        s_oled_ready = 1U;
        s_oled_async_state = OLED_ASYNC_IDLE;
        s_oled_dirty_pages = (uint8_t)((1U << DRV_OLED_PAGE_NUM) - 1U);
        return;
    }

    tx[0] = OLED_CTRL_CMD;
    tx[1] = s_oled_init_cmds[s_oled_init_index];
    ret = Oled_StartAsyncWrite(tx, 2U);
    if (ret == BSP_OK) {
        s_oled_async_state = OLED_ASYNC_INIT_CMD_BUSY;
    } else if (ret == BSP_BUSY) {
        s_oled_delay_start_ms = BSP_GET_TICK();
        s_oled_delay_ms = 1U;
        s_oled_async_state = OLED_ASYNC_INIT_WAIT;
    } else {
        s_oled_async_state = OLED_ASYNC_ERROR;
    }
}

static void Oled_StartPageCommand(uint8_t page)
{
    uint8_t tx[4];

    tx[0] = OLED_CTRL_CMD;
    tx[1] = (uint8_t)(0xB0U | (page & 0x07U));
    tx[2] = 0x00U;
    tx[3] = 0x10U;

    s_oled_async_page = page;
    s_oled_async_col = 0U;
    s_oled_dirty_pages &= (uint8_t)(~(uint8_t)(1U << page));

    if (Oled_StartAsyncWrite(tx, 4U) == BSP_OK) {
        s_oled_async_state = OLED_ASYNC_FLUSH_CMD_BUSY;
    } else {
        s_oled_dirty_pages |= (uint8_t)(1U << page);
        s_oled_async_state = OLED_ASYNC_ERROR;
    }
}

static void Oled_StartPageDataChunk(void)
{
    uint8_t tx[OLED_ASYNC_TX_BUF_LEN];
    uint8_t remain;
    uint8_t chunk;

    if (s_oled_async_col >= DRV_OLED_WIDTH) {
        s_oled_async_state = OLED_ASYNC_IDLE;
        return;
    }

    remain = (uint8_t)(DRV_OLED_WIDTH - s_oled_async_col);
    chunk = (remain > OLED_ASYNC_DATA_CHUNK) ? OLED_ASYNC_DATA_CHUNK : remain;

    tx[0] = OLED_CTRL_DATA;
    memcpy(&tx[1],
           &s_oled_buf[((uint16_t)s_oled_async_page * DRV_OLED_WIDTH) + s_oled_async_col],
           chunk);

    if (Oled_StartAsyncWrite(tx, (uint16_t)(chunk + 1U)) == BSP_OK) {
        s_oled_async_col = (uint8_t)(s_oled_async_col + chunk);
        s_oled_async_state = OLED_ASYNC_FLUSH_DATA_BUSY;
    } else {
        s_oled_dirty_pages |= (uint8_t)(1U << s_oled_async_page);
        s_oled_async_state = OLED_ASYNC_ERROR;
    }
}

void Drv_OledI2c_Init(void)
{
#if (DRV_OLED_I2C_ENABLE == 0U)
    memset(s_oled_buf, 0, sizeof(s_oled_buf));
    s_oled_ready = 0U;
    s_oled_dirty_pages = 0U;
    s_oled_init_index = 0U;
    s_oled_async_done = 0U;
    s_oled_async_status = BSP_OK;
    s_oled_last_status = BSP_OK;
    s_oled_error_count = 0U;
    s_oled_async_state = OLED_ASYNC_IDLE;
    return;
#else
    BSP_I2C_Init(DRV_OLED_I2C_BUS);
    Oled_DetectAddressOnInit();
    memset(s_oled_buf, 0, sizeof(s_oled_buf));

    s_oled_ready = 0U;
    s_oled_dirty_pages = 0U;
    s_oled_init_index = 0U;
    s_oled_async_done = 0U;
    s_oled_async_status = BSP_OK;
    s_oled_last_status = BSP_OK;
    s_oled_error_count = 0U;
    s_oled_delay_start_ms = BSP_GET_TICK();
    s_oled_delay_ms = OLED_INIT_POWER_ON_DELAY_MS;
    s_oled_async_state = OLED_ASYNC_INIT_WAIT;
#endif
}

uint8_t Drv_OledI2c_IsReady(void)
{
    return s_oled_ready;
}

BSP_Status_t Drv_OledI2c_GetLastStatus(void)
{
    return s_oled_last_status;
}

uint8_t Drv_OledI2c_GetAsyncState(void)
{
    return (uint8_t)s_oled_async_state;
}

uint16_t Drv_OledI2c_GetErrorCount(void)
{
    return s_oled_error_count;
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
    /*
     * Mark pages dirty even before SSD1306 async init is finished.
     * Drv_OledI2c_Task() will flush them after s_oled_ready becomes 1.
     */
    s_oled_dirty_pages = (uint8_t)((1U << DRV_OLED_PAGE_NUM) - 1U);
}

void Drv_OledI2c_FlushPage(uint8_t page)
{
    if (page >= DRV_OLED_PAGE_NUM) {
        return;
    }

    s_oled_dirty_pages |= (uint8_t)(1U << page);
}

void Drv_OledI2c_Task(void)
{
    uint8_t page;

#if (DRV_OLED_I2C_ENABLE == 0U)
    return;
#endif

    if (s_oled_async_state == OLED_ASYNC_INIT_WAIT) {
        if ((uint32_t)(BSP_GET_TICK() - s_oled_delay_start_ms) >= s_oled_delay_ms) {
            if (BSP_I2C_IsBusy(DRV_OLED_I2C_BUS) == 0U) {
                Oled_StartInitCommand();
            }
        }
        return;
    }

    if ((s_oled_async_state == OLED_ASYNC_INIT_CMD_BUSY) ||
        (s_oled_async_state == OLED_ASYNC_FLUSH_CMD_BUSY) ||
        (s_oled_async_state == OLED_ASYNC_FLUSH_DATA_BUSY)) {
        if (s_oled_async_done == 0U) {
            return;
        }

        s_oled_async_done = 0U;
        if (s_oled_async_status != BSP_OK) {
            s_oled_last_status = s_oled_async_status;
            s_oled_error_count++;
            if ((s_oled_async_state == OLED_ASYNC_FLUSH_CMD_BUSY) ||
                (s_oled_async_state == OLED_ASYNC_FLUSH_DATA_BUSY)) {
                s_oled_dirty_pages |= (uint8_t)(1U << s_oled_async_page);
            }
            s_oled_async_state = OLED_ASYNC_ERROR;
            return;
        }

        if (s_oled_async_state == OLED_ASYNC_INIT_CMD_BUSY) {
            s_oled_last_status = BSP_OK;
            s_oled_init_index++;
            Oled_StartInitCommand();
            return;
        }

        if (s_oled_async_state == OLED_ASYNC_FLUSH_CMD_BUSY) {
            Oled_StartPageDataChunk();
            return;
        }

        if (s_oled_async_state == OLED_ASYNC_FLUSH_DATA_BUSY) {
            Oled_StartPageDataChunk();
            return;
        }
    }

    if (s_oled_async_state == OLED_ASYNC_ERROR) {
        if (s_oled_ready == 0U) {
            Oled_RestartInitAfterError();
        } else {
            s_oled_async_state = OLED_ASYNC_IDLE;
        }
        return;
    }

    if ((s_oled_ready == 0U) ||
        (s_oled_async_state != OLED_ASYNC_IDLE) ||
        (BSP_I2C_IsBusy(DRV_OLED_I2C_BUS) != 0U)) {
        return;
    }

    page = Oled_FindNextDirtyPage();
    if (page < DRV_OLED_PAGE_NUM) {
        Oled_StartPageCommand(page);
    }
}

uint8_t Drv_OledI2c_IsBusy(void)
{
    return (uint8_t)(s_oled_async_state != OLED_ASYNC_IDLE);
}

void Drv_OledI2c_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    uint16_t index;
    uint8_t mask;
    uint8_t page;

    if ((x >= DRV_OLED_WIDTH) || (y >= DRV_OLED_HEIGHT)) {
        return;
    }

    page = (uint8_t)(y / 8U);
    index = (uint16_t)page * DRV_OLED_WIDTH + x;
    mask = (uint8_t)(1U << (y & 0x07U));
    if (color) {
        s_oled_buf[index] |= mask;
    } else {
        s_oled_buf[index] &= (uint8_t)(~mask);
    }

    s_oled_dirty_pages |= (uint8_t)(1U << page);
}

void Drv_OledI2c_DrawHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color)
{
    uint16_t i;
    uint16_t xx;

    if ((w == 0U) || (y >= DRV_OLED_HEIGHT) || (x >= DRV_OLED_WIDTH)) {
        return;
    }

    for (i = 0U; i < w; i++) {
        xx = (uint16_t)x + i;
        if (xx >= DRV_OLED_WIDTH) {
            break;
        }
        Drv_OledI2c_DrawPixel((uint8_t)xx, y, color);
    }
}

void Drv_OledI2c_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
    uint16_t ix;
    uint16_t iy;
    uint16_t xx;
    uint16_t yy;

    if ((w == 0U) || (h == 0U) || (x >= DRV_OLED_WIDTH) || (y >= DRV_OLED_HEIGHT)) {
        return;
    }

    for (iy = 0U; iy < h; iy++) {
        yy = (uint16_t)y + iy;
        if (yy >= DRV_OLED_HEIGHT) {
            break;
        }
        for (ix = 0U; ix < w; ix++) {
            xx = (uint16_t)x + ix;
            if (xx >= DRV_OLED_WIDTH) {
                break;
            }
            Drv_OledI2c_DrawPixel((uint8_t)xx, (uint8_t)yy, color);
        }
    }
}

void Drv_OledI2c_DrawVLine(uint8_t x, uint8_t y, uint8_t h, uint8_t color)
{
    uint16_t i;
    uint16_t yy;

    if ((h == 0U) || (x >= DRV_OLED_WIDTH) || (y >= DRV_OLED_HEIGHT)) {
        return;
    }

    for (i = 0U; i < h; i++) {
        yy = (uint16_t)y + i;
        if (yy >= DRV_OLED_HEIGHT) {
            break;
        }
        Drv_OledI2c_DrawPixel(x, (uint8_t)yy, color);
    }
}

void Drv_OledI2c_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
    uint16_t x2;
    uint16_t y2;

    if ((w == 0U) || (h == 0U) || (x >= DRV_OLED_WIDTH) || (y >= DRV_OLED_HEIGHT)) {
        return;
    }

    x2 = (uint16_t)x + w - 1U;
    y2 = (uint16_t)y + h - 1U;
    if (x2 >= DRV_OLED_WIDTH) {
        x2 = DRV_OLED_WIDTH - 1U;
    }
    if (y2 >= DRV_OLED_HEIGHT) {
        y2 = DRV_OLED_HEIGHT - 1U;
    }

    Drv_OledI2c_DrawHLine(x, y, (uint8_t)(x2 - x + 1U), color);
    Drv_OledI2c_DrawHLine(x, (uint8_t)y2, (uint8_t)(x2 - x + 1U), color);
    Drv_OledI2c_DrawVLine(x, y, (uint8_t)(y2 - y + 1U), color);
    Drv_OledI2c_DrawVLine((uint8_t)x2, y, (uint8_t)(y2 - y + 1U), color);
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
}

void Drv_OledI2c_DrawString5x7(uint8_t x, uint8_t y, const char *str, uint8_t color)
{
    uint16_t cur_x = x;

    if ((str == 0) || (x >= DRV_OLED_WIDTH) || (y >= DRV_OLED_HEIGHT)) {
        return;
    }

    while (*str != '\0') {
        if ((cur_x + DRV_OLED_FONT_5X7_WIDTH) > DRV_OLED_WIDTH) {
            break;
        }
        Drv_OledI2c_DrawChar5x7((uint8_t)cur_x, y, *str, color);
        cur_x = (uint16_t)(cur_x + DRV_OLED_FONT_5X7_WIDTH + DRV_OLED_FONT_5X7_X_SPACE);
        str++;
    }
}

void Drv_OledI2c_DrawMonoImage(uint8_t x, uint8_t y, const Drv_Oled_MonoImage_t *img, uint8_t color)
{
    uint32_t bit_index;
    uint16_t byte_index;
    uint16_t ix;
    uint16_t iy;
    uint16_t xx;
    uint16_t yy;
    uint8_t bit_mask;
    uint8_t pixel_on;

    if ((img == 0) || (img->data == 0) ||
        (x >= DRV_OLED_WIDTH) || (y >= DRV_OLED_HEIGHT)) {
        return;
    }

    for (iy = 0U; iy < img->height; iy++) {
        yy = (uint16_t)y + iy;
        if (yy >= DRV_OLED_HEIGHT) {
            break;
        }
        for (ix = 0U; ix < img->width; ix++) {
            xx = (uint16_t)x + ix;
            if (xx >= DRV_OLED_WIDTH) {
                break;
            }
            bit_index = (uint32_t)iy * img->width + ix;
            byte_index = (uint16_t)(bit_index / 8U);
            bit_mask = (uint8_t)(0x80U >> (bit_index & 0x07U));
            pixel_on = ((img->data[byte_index] & bit_mask) != 0U) ? color : (uint8_t)!color;
            Drv_OledI2c_DrawPixel((uint8_t)xx, (uint8_t)yy, pixel_on);
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
