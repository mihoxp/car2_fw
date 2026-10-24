#pragma once

#include <stdbool.h>
#include <stdint.h>

void ir_init(void);

// Returns true once per decoded NEC frame (not for repeat frames) and
// stores the command byte, the same value YAHBOOM's ir.Getir() returns.
bool ir_get(uint8_t *cmd);

// Returns true once per NEC repeat frame (key still held down).
bool ir_repeat(void);
