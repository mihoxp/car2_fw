#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "servo.h"

// S1..S3 from YAHBOOM's pico_car.py: PWM(Pin(18..20)), freq 100 Hz;
// S4 (GP21) is the SBUS receiver input now
static const uint SERVO_PINS[SERVO_COUNT] = {18, 19, 20};
#define SERVO_PERIOD_US 10000   // 100 Hz
// servo180(): duty_u16 = angle * 72.2222 + 3535 of a 10 ms period,
// i.e. 539 us at 0 degrees to 2523 us at 180 degrees
#define SERVO_MIN_US    539
#define SERVO_MAX_US    2523
#define SERVO_MAX_DEG   180

static uint16_t angle[SERVO_COUNT] = {90, 90, 90};

void servo_init(void)
{
    // 1 MHz PWM counter, so levels are in microseconds
    float div = (float)clock_get_hz(clk_sys) / 1000000.0f;
    for (int i = 0; i < SERVO_COUNT; i++) {
        uint pin = SERVO_PINS[i];
        uint slice = pwm_gpio_to_slice_num(pin);
        gpio_set_function(pin, GPIO_FUNC_PWM);
        pwm_set_clkdiv(slice, div);
        pwm_set_wrap(slice, SERVO_PERIOD_US - 1);
        pwm_set_gpio_level(pin, 0);
        pwm_set_enabled(slice, true);
    }
}

void servo_set(int num, uint16_t angle_deg)
{
    if (num < 0 || num >= SERVO_COUNT) {
        return;
    }
    if (angle_deg > SERVO_MAX_DEG) {
        angle_deg = SERVO_MAX_DEG;
    }
    angle[num] = angle_deg;
    uint32_t us = SERVO_MIN_US + (uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * angle_deg / SERVO_MAX_DEG;
    pwm_set_gpio_level(SERVO_PINS[num], us);
}

uint16_t servo_get(int num)
{
    return (num >= 0 && num < SERVO_COUNT) ? angle[num] : 0;
}
