#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool ws2812_init(void);

#define WS2812_NUM_LEDS 8

// Set all LEDs to one color; r, g, b and brightness are 0.0 - 1.0
void ws2812_fill(float r, float g, float b, float brightness);

// Set each LED from 0xRRGGBB values; LEDs past count are turned off
void ws2812_set_rgb(const uint32_t *rgb, size_t count);
