#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct app_display app_display_t;
typedef void (*app_center_freq_cb_t)(uint32_t center_hz, void *ctx);

typedef enum {
    APP_DISPLAY_PAGE_HOME = 0,
    APP_DISPLAY_PAGE_WATERFALL,
    APP_DISPLAY_PAGE_FM,
    APP_DISPLAY_PAGE_ADSB,
    APP_DISPLAY_PAGE_SCANNER,
    APP_DISPLAY_PAGE_SENSORS,
    APP_DISPLAY_PAGE_RECORD,
} app_display_page_t;

typedef void (*app_page_cb_t)(app_display_page_t page, void *ctx);

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
} app_display_adsb_aircraft_t;

typedef struct {
    char model[16];
    char id[17];
    char data[64];
    uint32_t frequency_hz;
    uint8_t signal_level;
    uint64_t count;
    int64_t last_seen_ms;
} app_display_sensor_event_t;

typedef struct {
    int h_res;
    int v_res;
    uint32_t pclk_hz;
    int hsync;
    int hbp;
    int hfp;
    int vsync;
    int vbp;
    int vfp;
    bool pclk_active_neg;
} app_display_config_t;

esp_err_t app_display_create(const app_display_config_t *config, app_display_t **out);
void app_display_destroy(app_display_t *display);
void app_display_set_status(app_display_t *display, const char *status);
void app_display_set_radio_info(app_display_t *display, uint32_t center_hz, uint32_t sample_rate_hz);
void app_display_set_sample_rate(app_display_t *display, uint32_t sample_rate_hz);
void app_display_set_adsb_info(app_display_t *display,
                               uint32_t center_hz,
                               uint32_t sample_rate_hz,
                               bool active,
                               uint64_t candidates,
                               uint64_t valid_messages,
                               uint32_t aircraft_count);
void app_display_set_scanner_info(app_display_t *display,
                                  uint32_t start_hz,
                                  uint32_t stop_hz,
                                  bool active,
                                  uint32_t center_hz,
                                  uint32_t hop_index,
                                  uint32_t hop_count);
void app_display_set_sensor_info(app_display_t *display,
                                 uint32_t center_hz,
                                 uint32_t sample_rate_hz,
                                 bool active,
                                 uint64_t total_frames,
                                 uint64_t decoded_messages,
                                 uint32_t sensor_count,
                                 uint8_t signal_level);
void app_display_update_sensor_event(app_display_t *display, const app_display_sensor_event_t *event);
void app_display_clear_sensor_events(app_display_t *display);
void app_display_set_record_info(app_display_t *display,
                                 uint32_t center_hz,
                                 uint32_t sample_rate_hz,
                                 bool active,
                                 bool storage_ready,
                                 uint64_t bytes_written,
                                 uint64_t max_bytes,
                                 uint32_t elapsed_seconds,
                                 const char *path);
void app_display_update_adsb_aircraft(app_display_t *display, const app_display_adsb_aircraft_t *aircraft);
void app_display_clear_adsb(app_display_t *display);
void app_display_set_fm_info(app_display_t *display,
                                   const char *mode,
                                   uint32_t audio_rate_hz,
                                   bool audio_output_enabled,
                                   bool audio_output_ready,
                                   uint8_t audio_level);
void app_display_push_fm_waveform(app_display_t *display, const int16_t *samples, size_t sample_count);
void app_display_push_fm_spectrum(app_display_t *display, const uint8_t *bins, size_t bin_count);
void app_display_set_center_freq_cb(app_display_t *display,
                                          app_center_freq_cb_t cb,
                                          void *ctx);
void app_display_set_page_cb(app_display_t *display,
                             app_page_cb_t cb,
                             void *ctx);
app_display_page_t app_display_get_active_page(app_display_t *display);
void app_display_push_scanner_hop(app_display_t *display,
                                  const uint8_t *bins,
                                  size_t bin_count,
                                  uint32_t segment_start_hz,
                                  uint32_t segment_stop_hz,
                                  uint32_t scan_start_hz,
                                  uint32_t scan_stop_hz,
                                  uint32_t hop_index,
                                  uint32_t hop_count);
void app_display_push_line(app_display_t *display, const uint8_t *bins, size_t bin_count);
void app_display_render_idle(app_display_t *display);

#ifdef __cplusplus
}
#endif
