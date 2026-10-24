#pragma once

#include <stdbool.h>
#include <stdint.h>

// NMEA GPS (e.g. GY-NEO6MV2) on the old Bluetooth header: UART0 9600 baud,
// GPS TX -> TXD = GP17 (Pico RX), GPS RX -> RXD = GP16 (Pico TX, unused)
typedef struct {
    int quality;        // GGA fix quality: 0 = no fix, 1 = GPS, 2 = DGPS
    int satellites;
    double latitude;    // degrees, north positive
    double longitude;   // degrees, east positive
    double altitude;    // meters above mean sea level
    float hdop;
} gps_fix_t;

void gps_init(void);

// Call from the main loop; returns true when a new GGA sentence was parsed
bool gps_poll(gps_fix_t *fix);

// Counters for debugging: valid sentences and checksum errors seen so far
uint32_t gps_sentences(void);
uint32_t gps_bad_checksums(void);
uint32_t gps_rx_bytes(void);

// Called with every NMEA sentence that passed its checksum (before parsing)
typedef void (*gps_sentence_cb_t)(const char *sentence);
void gps_set_sentence_cb(gps_sentence_cb_t cb);
