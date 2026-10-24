#pragma once

#include <stdbool.h>
#include <stdint.h>

#define SBUS_CHANNELS 16

void sbus_init(void);

// Returns true once per new frame and copies the raw 11-bit channel values
// (0..2047, about 1000 at center) and the receiver's failsafe flag
bool sbus_get(uint16_t ch[SBUS_CHANNELS], bool *failsafe);

// Debug counters: good frames, bytes with UART errors
uint32_t sbus_frames(void);
uint32_t sbus_errors(void);
