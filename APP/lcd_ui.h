#ifndef __LCD_UI_H
#define __LCD_UI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LCD_UI_ENABLE                1U
#define LCD_UI_UPDATE_PERIOD_MS      50U

void LcdUi_Init(void);
void LcdUi_Update(void);
void LcdUi_ShowBoot(void);
void LcdUi_ShowStatus(const char *line1, const char *line2, const char *line3);

#ifdef __cplusplus
}
#endif

#endif /* __LCD_UI_H */
