#pragma once

#include <stdint.h>

#define SERVO_COUNT 3     // S4 is used by the SBUS receiver

void servo_init(void);

// Angle 0..180 degrees on servo S1..S3 (num 0..2), same pulse range as
// YAHBOOM's servo180(); a servo gets no pulse until its first command
void servo_set(int num, uint16_t angle_deg);

// Last commanded angle (90 before the first command)
uint16_t servo_get(int num);
