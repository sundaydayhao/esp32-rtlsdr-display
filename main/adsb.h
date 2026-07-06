#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADSB_DEFAULT_CENTER_HZ       1090000000U
#define ADSB_DEFAULT_SAMPLE_RATE_HZ  2000000U
#define ADSB_MAX_AIRCRAFT           24

typedef struct adsb_receiver adsb_receiver_t;

typedef struct {
    char icao[7];
    char callsign[9];
    bool has_altitude;
    int altitude_feet;
    bool has_position;
    double latitude;
    double longitude;
    bool has_speed;
    float speed_knots;
    bool has_heading;
    float heading_degrees;
    uint32_t messages;
    int64_t last_seen_ms;
} adsb_aircraft_t;

typedef struct {
    uint64_t candidates;
    uint64_t valid_messages;
    uint32_t aircraft_count;
} adsb_status_t;

typedef void (*adsb_aircraft_cb_t)(const adsb_aircraft_t *aircraft, void *ctx);

typedef struct {
    uint32_t sample_rate_hz;
    adsb_aircraft_cb_t aircraft_cb;
    void *aircraft_cb_ctx;
} adsb_receiver_config_t;

esp_err_t adsb_receiver_create(const adsb_receiver_config_t *config, adsb_receiver_t **out);
void adsb_receiver_destroy(adsb_receiver_t *receiver);
void adsb_receiver_reset(adsb_receiver_t *receiver);
void adsb_receiver_push_iq(adsb_receiver_t *receiver, const uint8_t *iq, size_t len);
void adsb_receiver_get_status(adsb_receiver_t *receiver, adsb_status_t *out);

#ifdef __cplusplus
}
#endif
