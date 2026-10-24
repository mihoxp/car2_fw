#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#include "gps.h"

#define GPS_UART        uart0
#define GPS_TX_PIN      16      // YAHBOOM's Bluetooth UART(0, 9600, tx=Pin(16), rx=Pin(17))
#define GPS_RX_PIN      17
#define GPS_BAUD        9600
#define RX_BUF_SIZE     512     // power of two; ~0.5 s of NMEA at 9600 baud
#define LINE_MAX        100     // NMEA sentences are at most 82 characters

// The UART FIFO holds only 32 bytes (~33 ms), less than a slow main loop
// iteration, so an interrupt moves bytes into this ring buffer
static volatile uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint32_t rx_head, rx_tail;
static char line[LINE_MAX];
static int line_len;
static uint32_t sentences, bad_checksums;
static volatile uint32_t rx_bytes;
static gps_sentence_cb_t sentence_cb;

static void uart_rx_irq(void)
{
    while (uart_is_readable(GPS_UART)) {
        uint8_t c = uart_getc(GPS_UART);
        rx_bytes++;
        uint32_t next = (rx_head + 1) & (RX_BUF_SIZE - 1);
        if (next != rx_tail) {
            rx_buf[rx_head] = c;
            rx_head = next;
        }
    }
}

void gps_init(void)
{
    uart_init(GPS_UART, GPS_BAUD);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(GPS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART, true);
    irq_set_exclusive_handler(UART0_IRQ, uart_rx_irq);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(GPS_UART, true, false);
}

static int hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// "$...*hh": XOR of the characters between '$' and '*'
static bool checksum_ok(const char *s)
{
    uint8_t sum = 0;
    const char *p = s + 1;
    while (*p && *p != '*') {
        sum ^= (uint8_t)*p++;
    }
    if (*p != '*') return false;
    int h = hex_digit(p[1]), l = hex_digit(p[2]);
    return h >= 0 && l >= 0 && sum == (h << 4 | l);
}

// NMEA ddmm.mmmm (or dddmm.mmmm) to degrees
static double nmea_degrees(const char *s, char hemi)
{
    double v = strtod(s, NULL);
    int deg = (int)(v / 100);
    double d = deg + (v - deg * 100) / 60.0;
    return (hemi == 'S' || hemi == 'W') ? -d : d;
}

// $GPGGA,time,lat,N,lon,E,quality,sats,hdop,alt,M,...*hh (also $GNGGA etc.)
static bool parse_gga(char *s, gps_fix_t *fix)
{
    if (strlen(s) < 6 || strncmp(s + 3, "GGA", 3) != 0) {
        return false;
    }
    char *star = strchr(s, '*');
    if (star) *star = '\0';
    // Split in place; empty fields stay as ""
    char *field[15] = {0};
    int n = 0;
    char *p = s;
    while (n < 15) {
        field[n++] = p;
        p = strchr(p, ',');
        if (!p) break;
        *p++ = '\0';
    }
    if (n < 10) {
        return false;
    }
    fix->quality = atoi(field[6]);
    fix->satellites = atoi(field[7]);
    fix->hdop = field[8][0] ? strtof(field[8], NULL) : 0.0f;
    if (fix->quality > 0 && field[2][0] && field[4][0]) {
        fix->latitude = nmea_degrees(field[2], field[3][0]);
        fix->longitude = nmea_degrees(field[4], field[5][0]);
        fix->altitude = strtod(field[9], NULL);
    } else {
        fix->latitude = fix->longitude = fix->altitude = 0.0;
    }
    return true;
}

bool gps_poll(gps_fix_t *fix)
{
    bool got = false;
    while (rx_tail != rx_head) {
        char c = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) & (RX_BUF_SIZE - 1);
        if (c == '$') {
            line_len = 0;
        }
        if (c == '\r' || c == '\n') {
            if (line_len > 0 && line[0] == '$') {
                line[line_len] = '\0';
                if (!checksum_ok(line)) {
                    bad_checksums++;
                } else {
                    sentences++;
                    if (sentence_cb) {
                        sentence_cb(line);
                    }
                    if (parse_gga(line, fix)) {
                        got = true;
                    }
                }
            }
            line_len = 0;
        } else if (line_len < LINE_MAX - 1) {
            line[line_len++] = c;
        }
    }
    return got;
}

uint32_t gps_sentences(void)
{
    return sentences;
}

uint32_t gps_bad_checksums(void)
{
    return bad_checksums;
}

uint32_t gps_rx_bytes(void)
{
    return rx_bytes;
}

void gps_set_sentence_cb(gps_sentence_cb_t cb)
{
    sentence_cb = cb;
}
