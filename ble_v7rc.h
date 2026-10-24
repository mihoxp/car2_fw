#pragma once

#include <stdbool.h>
#include <stdint.h>

// BLE "Nordic UART" peripheral named pico_car2 for the V7RC phone app,
// same service as YAHBOOM's ble_uart_v7rc.py. Call after cyw43_arch_init().
void ble_v7rc_init(void);

// Returns true once per received frame ("SRT" or "SRV" + four 4-digit
// channels + "#"), channels are 1000..2000 with 1500 = center.
// ch[0] = steering (right > 1500), ch[1] = throttle (forward < 1500),
// ch[2] / ch[3] = extra channels, used for servos S1 / S2.
#define V7RC_CHANNELS 4
bool ble_v7rc_get(int ch[V7RC_CHANNELS]);

bool ble_v7rc_connected(void);

// BTstack HCI state, HCI_STATE_WORKING (2) once advertising is possible
int ble_v7rc_state(void);
