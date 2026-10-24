#pragma once

void ultrasonic_init(void);

// Distance in meters, or a negative value when no echo came back
float ultrasonic_read_m(void);
