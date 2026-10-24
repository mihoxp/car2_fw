#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

#include "sbus.h"

// SBUS receiver plugged into servo header S4 (GP21 = UART1 RX); S4 is no
// longer a servo
#define SBUS_UART       uart1
#define SBUS_RX_PIN     21
#define SBUS_BAUD       100000
#define SBUS_FRAME_LEN  25
#define SBUS_HEADER     0x0f
#define SBUS_GAP_US     2000    // frames come every 7 or 14 ms with a gap between them

#define FLAG_FRAME_LOST 0x04
#define FLAG_FAILSAFE   0x08

static uint8_t buf[SBUS_FRAME_LEN];
static int buf_len;
static uint32_t last_byte_us;
static volatile uint16_t channels[SBUS_CHANNELS];
static volatile bool failsafe;
static volatile bool frame_ready;
static volatile uint32_t frames, errors;

static void decode_frame(void)
{
    // 16 channels x 11 bits, LSB first, starting at byte 1
    uint32_t acc = 0;
    int acc_bits = 0, b = 1;
    for (int i = 0; i < SBUS_CHANNELS; i++) {
        while (acc_bits < 11) {
            acc |= (uint32_t)buf[b++] << acc_bits;
            acc_bits += 8;
        }
        channels[i] = acc & 0x7ff;
        acc >>= 11;
        acc_bits -= 11;
    }
    failsafe = (buf[23] & FLAG_FAILSAFE) != 0;
    frames++;
    frame_ready = true;
}

static void uart_rx_irq(void)
{
    while (uart_is_readable(SBUS_UART)) {
        // Read the data register directly to see parity / framing errors
        uint32_t dr = uart_get_hw(SBUS_UART)->dr;
        uint32_t now = time_us_32();
        if (now - last_byte_us > SBUS_GAP_US) {
            buf_len = 0;
        }
        last_byte_us = now;
        if (dr & (UART_UARTDR_FE_BITS | UART_UARTDR_PE_BITS | UART_UARTDR_BE_BITS | UART_UARTDR_OE_BITS)) {
            errors++;
            buf_len = SBUS_FRAME_LEN;   // drop the rest of this frame
            continue;
        }
        uint8_t c = dr & 0xff;
        if (buf_len >= SBUS_FRAME_LEN) {
            continue;
        }
        if (buf_len == 0 && c != SBUS_HEADER) {
            continue;
        }
        buf[buf_len++] = c;
        if (buf_len == SBUS_FRAME_LEN) {
            decode_frame();
        }
    }
}

void sbus_init(void)
{
    uart_init(SBUS_UART, SBUS_BAUD);
    uart_set_format(SBUS_UART, 8, 2, UART_PARITY_EVEN);
    uart_set_hw_flow(SBUS_UART, false, false);
    uart_set_fifo_enabled(SBUS_UART, true);
    gpio_set_function(SBUS_RX_PIN, GPIO_FUNC_UART);
    // SBUS idles low (inverted UART); the RP2040 can invert the pin itself
    gpio_set_inover(SBUS_RX_PIN, GPIO_OVERRIDE_INVERT);
    gpio_pull_down(SBUS_RX_PIN);    // no receiver = idle, not noise
    irq_set_exclusive_handler(UART1_IRQ, uart_rx_irq);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(SBUS_UART, true, false);
}

bool sbus_get(uint16_t ch[SBUS_CHANNELS], bool *fs)
{
    if (!frame_ready) {
        return false;
    }
    uint32_t irq = save_and_disable_interrupts();
    for (int i = 0; i < SBUS_CHANNELS; i++) {
        ch[i] = channels[i];
    }
    *fs = failsafe;
    frame_ready = false;
    restore_interrupts(irq);
    return true;
}

uint32_t sbus_frames(void)
{
    return frames;
}

uint32_t sbus_errors(void)
{
    return errors;
}
