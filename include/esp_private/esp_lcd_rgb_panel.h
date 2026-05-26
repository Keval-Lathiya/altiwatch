#pragma once
// Stub: esp_private/esp_lcd_rgb_panel.h is an IDF 5.x private header.
// Arduino_GFX's RGB panel driver includes it; we only use the SPI driver
// so this struct is never instantiated at runtime.
#include "esp_lcd_panel_rgb.h"
#include <stddef.h>

typedef struct {
    esp_lcd_panel_t base;
    uint16_t *fb;
} esp_rgb_panel_t;

#ifndef __containerof
#define __containerof(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))
#endif
