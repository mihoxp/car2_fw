#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "ws2812.h"
#include "ws2812.pio.h"

#define WS2812_PIN      6   // same as YAHBOOM's pico_car.py

static PIO pio;
static uint sm;
static bool ready;

bool ws2812_init(void)
{
    uint offset;
    // The CYW43 WiFi driver also uses PIO, so claim whatever is free
    if (!pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &pio, &sm, &offset,
                                                          WS2812_PIN, 1, true)) {
        return false;
    }
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000);
    ready = true;
    ws2812_fill(0, 0, 0, 0);
    return true;
}

static uint8_t to_u8(float v, float brightness)
{
    float x = v * brightness;
    if (x < 0) x = 0;
    if (x > 1) x = 1;
    return (uint8_t)(x * 255.0f + 0.5f);
}

void ws2812_fill(float r, float g, float b, float brightness)
{
    if (!ready) {
        return;
    }
    uint32_t grb = ((uint32_t)to_u8(g, brightness) << 16) |
                   ((uint32_t)to_u8(r, brightness) << 8) |
                   to_u8(b, brightness);
    for (int i = 0; i < WS2812_NUM_LEDS; i++) {
        pio_sm_put_blocking(pio, sm, grb << 8u);
    }
}

void ws2812_set_rgb(const uint32_t *rgb, size_t count)
{
    if (!ready) {
        return;
    }
    for (size_t i = 0; i < WS2812_NUM_LEDS; i++) {
        uint32_t c = i < count ? rgb[i] : 0;
        uint32_t r = (c >> 16) & 0xff, g = (c >> 8) & 0xff, b = c & 0xff;
        pio_sm_put_blocking(pio, sm, ((g << 16) | (r << 8) | b) << 8u);
    }
}
