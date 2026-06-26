#ifndef __OLED_UI_H
#define __OLED_UI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OLED_UI_ENABLE               1U
#define OLED_UI_UPDATE_PERIOD_MS     500U
#define OLED_UI_BOOT_HOLD_MS         1000U

void OledUi_Init(void);
void OledUi_ShowBoot(void);
void OledUi_Update(void);
void OledUi_ShowStatus(const char *line1, const char *line2, const char *line3);
void OLED_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_UI_H */
