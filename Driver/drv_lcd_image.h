#ifndef __DRV_LCD_IMAGE_H
#define __DRV_LCD_IMAGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t width;
    uint16_t height;
    const uint8_t *data;
} Drv_Lcd_MonoImage_t;

typedef struct {
    uint16_t width;
    uint16_t height;
    const uint16_t *data;
} Drv_Lcd_Rgb565Image_t;

#ifdef __cplusplus
}
#endif

#endif /* __DRV_LCD_IMAGE_H */
