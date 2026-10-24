#pragma once

#include <stdint.h>

void buzzer_init(void);

// Passive buzzer tone; freq_hz = 0 or duration_ms = 0 turns it off
void buzzer_beep(uint16_t freq_hz, uint16_t duration_ms);

// Call from the main loop: ends a beep when its time is up
void buzzer_update(void);
