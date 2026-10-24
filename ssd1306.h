#pragma once

#include <stdbool.h>

bool ssd1306_init(void);
void ssd1306_clear(void);

// Draw text with the 8x8 font; line is 0-3, up to 16 characters per line
void ssd1306_text(int line, const char *s);

// Send the framebuffer to the display
void ssd1306_show(void);

// Invert the whole display (white background)
void ssd1306_invert(bool on);
