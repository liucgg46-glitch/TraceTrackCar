#ifndef __DRV_OLED_IMAGE_H
#define __DRV_OLED_IMAGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t width;
    uint16_t height;
    const uint8_t *data;
} Drv_Oled_MonoImage_t;

/*
 * Image data note:
 * - Keep large image arrays in drv_oled_image.c, not in this header.
 * - Current OLED draw code expects 1 bit per pixel, row-major, bit7 first.
 * - Later you can add extern declarations here, like:
 *   extern const Drv_Oled_MonoImage_t g_oled_logo_32x32;
 */

#ifdef __cplusplus
}
#endif

#endif /* __DRV_OLED_IMAGE_H */
