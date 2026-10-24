#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "ir.h"

#define IR_PIN          7   // same as YAHBOOM's pico_car.py ir class

// NEC timing between falling edges (the receiver output is active low)
#define LEADER_MIN_US   12000   // 9 ms mark + 4.5 ms space = 13.5 ms
#define LEADER_MAX_US   15000
#define REPEAT_MIN_US   10500   // repeat frame: 9 ms mark + 2.25 ms space = 11.25 ms
#define REPEAT_MAX_US   12000
#define BIT0_MIN_US     800     // 1.125 ms
#define BIT_SPLIT_US    1700
#define BIT1_MAX_US     2700    // 2.25 ms

static volatile uint32_t last_edge_us;
static volatile int bit_count = -1;     // -1 = waiting for the leader
static volatile uint32_t bits;
static volatile bool frame_ready;
static volatile uint32_t frame;
static volatile bool repeat_ready;

static void ir_irq(void)
{
    if (!(gpio_get_irq_event_mask(IR_PIN) & GPIO_IRQ_EDGE_FALL)) {
        return;
    }
    gpio_acknowledge_irq(IR_PIN, GPIO_IRQ_EDGE_FALL);

    uint32_t now = time_us_32();
    uint32_t dt = now - last_edge_us;
    last_edge_us = now;

    if (dt >= LEADER_MIN_US && dt <= LEADER_MAX_US) {
        bit_count = 0;
        bits = 0;
        return;
    }
    if (dt >= REPEAT_MIN_US && dt <= REPEAT_MAX_US) {
        // Sent every ~108 ms while a key is held
        repeat_ready = true;
        bit_count = -1;
        return;
    }
    if (bit_count < 0) {
        return;
    }
    if (dt < BIT0_MIN_US || dt > BIT1_MAX_US) {
        bit_count = -1;
        return;
    }
    if (dt > BIT_SPLIT_US) {
        bits |= 1u << bit_count;    // LSB first
    }
    if (++bit_count == 32) {
        frame = bits;
        frame_ready = true;
        bit_count = -1;
    }
}

void ir_init(void)
{
    gpio_init(IR_PIN);
    gpio_set_dir(IR_PIN, GPIO_IN);
    gpio_pull_up(IR_PIN);
    // Raw handler so it coexists with the cyw43 driver's GPIO interrupt
    gpio_add_raw_irq_handler(IR_PIN, ir_irq);
    gpio_set_irq_enabled(IR_PIN, GPIO_IRQ_EDGE_FALL, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

bool ir_get(uint8_t *cmd)
{
    if (!frame_ready) {
        return false;
    }
    frame_ready = false;
    uint32_t f = frame;
    uint8_t addr = f & 0xff, addr_inv = (f >> 8) & 0xff;
    uint8_t c = (f >> 16) & 0xff, c_inv = (f >> 24) & 0xff;
    // Same check as YAHBOOM: data[0]+data[1] == 0xFF and data[2]+data[3] == 0xFF
    if ((uint8_t)(addr + addr_inv) != 0xff || (uint8_t)(c + c_inv) != 0xff) {
        return false;
    }
    *cmd = c;
    return true;
}

bool ir_repeat(void)
{
    if (!repeat_ready) {
        return false;
    }
    repeat_ready = false;
    return true;
}
