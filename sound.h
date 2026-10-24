#pragma once

#include <stdint.h>

// Sound sensor on ADC GP27 (needs the jumper cap on the sound header; shares
// the pin with light1). Samples at 2 kHz from a timer interrupt.
void sound_init(void);

// Peak-to-peak level (0-65535 scale) since the last call
uint16_t sound_take_level(void);
