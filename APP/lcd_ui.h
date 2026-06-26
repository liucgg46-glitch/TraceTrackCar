#ifndef __LCD_UI_H
#define __LCD_UI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LCD_UI_ENABLE                1U
#define LCD_UI_UPDATE_PERIOD_MS      500U
#define LCD_UI_BOOT_HOLD_MS          1000U

void LcdUi_Init(void);
void LcdUi_Update(void);
void LcdUi_ShowBoot(void);
void LcdUi_ShowStatus(const char *line1, const char *line2, const char *line3);
void LCD_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __LCD_UI_H */
