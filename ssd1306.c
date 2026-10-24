#include <string.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "ssd1306.h"
#include "font8x8.h"

// Same wiring as YAHBOOM's OLED example: I2C1, SCL=GP15, SDA=GP14, 128x32
#define OLED_I2C    i2c1
#define OLED_SDA    14
#define OLED_SCL    15
#define OLED_ADDR   0x3c
#define OLED_W      128
#define OLED_H      32
#define OLED_PAGES  (OLED_H / 8)

static uint8_t buf[1 + OLED_W * OLED_PAGES];   // buf[0] is the 0x40 data prefix
static bool ready;

static bool write_cmd(uint8_t cmd)
{
    uint8_t b[2] = {0x80, cmd};
    return i2c_write_blocking(OLED_I2C, OLED_ADDR, b, 2, false) == 2;
}

bool ssd1306_init(void)
{
    i2c_init(OLED_I2C, 400 * 1000);
    gpio_set_function(OLED_SDA, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA);
    gpio_pull_up(OLED_SCL);

    // Init sequence from YAHBOOM's SSD1306 class (internal charge pump)
    static const uint8_t init_seq[] = {
        0xae,             // display off
        0x20, 0x00,       // horizontal addressing
        0x40,             // start line 0
        0xa1,             // segment remap
        0xa8, OLED_H - 1, // multiplex ratio
        0xc8,             // COM scan from COM[N] to COM0
        0xd3, 0x00,       // display offset
        0xda, 0x02,       // COM pins config for 32 rows
        0xd5, 0x80,       // clock divide
        0xd9, 0xf1,       // precharge
        0xdb, 0x30,       // VCOM deselect
        0x81, 0xff,       // contrast
        0xa4,             // follow RAM
        0xa6,             // not inverted
        0x8d, 0x14,       // charge pump on
        0xaf,             // display on
    };
    for (size_t i = 0; i < sizeof(init_seq); i++) {
        if (!write_cmd(init_seq[i])) {
            return false;
        }
    }
    ready = true;
    ssd1306_clear();
    ssd1306_show();
    return true;
}

void ssd1306_clear(void)
{
    memset(buf + 1, 0, sizeof(buf) - 1);
}

void ssd1306_text(int line, const char *s)
{
    if (line < 0 || line >= OLED_PAGES) {
        return;
    }
    uint8_t *row = buf + 1 + line * OLED_W;
    for (int col = 0; col < OLED_W / 8 && s[col]; col++) {
        char c = s[col];
        if (c < 0x20 || c > 0x7e) {
            c = '?';
        }
        memcpy(row + col * 8, font8x8[c - 0x20], 8);
    }
}

void ssd1306_invert(bool on)
{
    if (ready) {
        write_cmd(on ? 0xa7 : 0xa6);
    }
}

void ssd1306_show(void)
{
    if (!ready) {
        return;
    }
    static const uint8_t window[] = {0x21, 0, OLED_W - 1, 0x22, 0, OLED_PAGES - 1};
    for (size_t i = 0; i < sizeof(window); i++) {
        write_cmd(window[i]);
    }
    buf[0] = 0x40;
    i2c_write_blocking(OLED_I2C, OLED_ADDR, buf, sizeof(buf), false);
}
