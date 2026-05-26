#pragma once
// Force-included compatibility shims for Arduino_GFX on IDF 4.4.
// Arduino_ESP32RGBPanel uses esp_rgb_panel_t and __containerof which are
// IDF 5.x private APIs; we stub them here so the file compiles.
// The RGB panel path is never called — we use the SPI bus.
#include <stddef.h>

#ifndef __containerof
#define __containerof(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S3
#include "esp_lcd_panel_rgb.h"
typedef struct {
    esp_lcd_panel_t base;
    uint16_t *fb;
} esp_rgb_panel_t;
#endif
