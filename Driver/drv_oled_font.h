#ifndef __DRV_OLED_FONT_H
#define __DRV_OLED_FONT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_OLED_FONT_5X7_WIDTH      5U
#define DRV_OLED_FONT_5X7_HEIGHT     7U
#define DRV_OLED_FONT_5X7_X_SPACE    1U

/*
 * Font data note:
 * - Current file keeps only a small 5x7 ASCII font for debug text.
 * - Add larger 8x16 ASCII or Chinese font arrays here later, then expose a
 *   getter instead of placing font arrays in the OLED driver.
 */
void Drv_OledFont_GetGlyph5x7(char ch, uint8_t glyph[DRV_OLED_FONT_5X7_WIDTH]);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_OLED_FONT_H */
