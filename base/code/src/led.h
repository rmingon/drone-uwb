/*
 * Minimal WS2812 driver, the ESP-IDF replacement for LiteLED.
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LED_RED   0xff0000
#define LED_GREEN 0x00ff00
#define LED_BLUE  0x0000ff
#define LED_WHITE 0xe0e0e0

/**
 * Drives `count` WS2812 pixels on `gpio` over RMT.
 */
esp_err_t led_init(int gpio, uint16_t count);

/**
 * Global scale applied to every pixel, 0 (off) to 255 (full).
 */
void led_set_brightness(uint8_t brightness);

/**
 * Sets one pixel from a 0xRRGGBB value and pushes the strip out.
 */
esp_err_t led_set_pixel(uint16_t index, uint32_t rgb);

#ifdef __cplusplus
}
#endif
