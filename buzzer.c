#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "buzzer.h"

#define BUZZER_PIN      22  // YAHBOOM: BZ = PWM(Pin(22))
#define BUZZER_MIN_HZ   50
#define BUZZER_MAX_HZ   10000

static uint slice;
static bool on;
static absolute_time_t off_at;

void buzzer_init(void)
{
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_gpio_level(BUZZER_PIN, 0);
    pwm_set_enabled(slice, false);
}

static void buzzer_off(void)
{
    pwm_set_gpio_level(BUZZER_PIN, 0);
    pwm_set_enabled(slice, false);
    on = false;
}

void buzzer_beep(uint16_t freq_hz, uint16_t duration_ms)
{
    if (freq_hz == 0 || duration_ms == 0) {
        buzzer_off();
        return;
    }
    if (freq_hz < BUZZER_MIN_HZ) freq_hz = BUZZER_MIN_HZ;
    if (freq_hz > BUZZER_MAX_HZ) freq_hz = BUZZER_MAX_HZ;
    // 1 MHz PWM counter, so wrap = period in microseconds
    float div = (float)clock_get_hz(clk_sys) / 1000000.0f;
    uint32_t wrap = 1000000u / freq_hz - 1;
    pwm_set_clkdiv(slice, div);
    pwm_set_wrap(slice, wrap);
    pwm_set_gpio_level(BUZZER_PIN, (wrap + 1) / 2);   // 50% duty
    pwm_set_enabled(slice, true);
    on = true;
    off_at = make_timeout_time_ms(duration_ms);
}

void buzzer_update(void)
{
    if (on && time_reached(off_at)) {
        buzzer_off();
    }
}
