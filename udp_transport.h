#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <uxr/client/profile/transport/custom/custom_transport.h>

bool wifi_connect(void);

// Leave the current network so the next wifi_connect() tries the other one
void wifi_switch_network(void);

// SSID in use (or being tried)
const char *wifi_ssid(void);

// Current IPv4 address as text, or NULL when WiFi is down
const char *wifi_ip_str(void);

bool pico_udp_transport_open(struct uxrCustomTransport *transport);
bool pico_udp_transport_close(struct uxrCustomTransport *transport);
size_t pico_udp_transport_write(struct uxrCustomTransport *transport,
                                const uint8_t *buf, size_t len, uint8_t *errcode);
size_t pico_udp_transport_read(struct uxrCustomTransport *transport,
                               uint8_t *buf, size_t len, int timeout, uint8_t *errcode);
