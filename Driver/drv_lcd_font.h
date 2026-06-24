#ifndef __DRV_LCD_FONT_H
#define __DRV_LCD_FONT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_LCD_FONT_5X7_WIDTH       5U
#define DRV_LCD_FONT_5X7_HEIGHT      7U
#define DRV_LCD_FONT_5X7_X_SPACE     1U
#define DRV_LCD_FONT_5X7_Y_SPACE     1U

void Drv_LcdFont_GetGlyph5x7(char ch, uint8_t glyph[DRV_LCD_FONT_5X7_WIDTH]);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_LCD_FONT_H */
