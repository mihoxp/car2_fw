#include "pico/stdlib.h"

#include "ultrasonic.h"

#define TRIG_PIN            0   // same as YAHBOOM's pico_car.py
#define ECHO_PIN            1
#define ECHO_START_TIMEOUT_US 5000
#define ECHO_MAX_US         25000   // ~4.3 m round trip
#define SOUND_SPEED_MPS     343.0f

void ultrasonic_init(void)
{
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
}

float ultrasonic_read_m(void)
{
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    absolute_time_t start_deadline = make_timeout_time_us(ECHO_START_TIMEOUT_US);
    while (!gpio_get(ECHO_PIN)) {
        if (time_reached(start_deadline)) {
            return -1.0f;
        }
    }
    uint64_t rise = time_us_64();
    while (gpio_get(ECHO_PIN)) {
        if (time_us_64() - rise > ECHO_MAX_US) {
            return -1.0f;
        }
    }
    uint64_t width = time_us_64() - rise;
    return width * 1e-6f * SOUND_SPEED_MPS / 2.0f;
}
