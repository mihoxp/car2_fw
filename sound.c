#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/sync.h"

#include "sound.h"

#define SOUND_ADC_PIN       27      // YAHBOOM: Sound = machine.ADC(27)
#define SOUND_SAMPLE_US     500     // 2 kHz: a clap lasts only a few ms

static repeating_timer_t timer;
static volatile uint16_t lo = 0xffff, hi;

static bool sample_cb(repeating_timer_t *t)
{
    (void)t;
    // Main-loop ADC reads disable interrupts, so the ADC is free here
    adc_select_input(SOUND_ADC_PIN - 26);
    uint16_t v = adc_read();
    if (v < lo) lo = v;
    if (v > hi) hi = v;
    return true;
}

void sound_init(void)
{
    adc_gpio_init(SOUND_ADC_PIN);
    add_repeating_timer_us(-SOUND_SAMPLE_US, sample_cb, NULL, &timer);
}

uint16_t sound_take_level(void)
{
    uint32_t irq = save_and_disable_interrupts();
    uint16_t p2p = hi > lo ? hi - lo : 0;
    lo = 0xffff;
    hi = 0;
    restore_interrupts(irq);
    return (p2p << 4) | (p2p >> 8);     // 12-bit to the 0-65535 scale used elsewhere
}
