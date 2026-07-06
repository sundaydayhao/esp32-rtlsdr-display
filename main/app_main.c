#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "adsb.h"
#include "display.h"
#include "dsp.h"
#include "fm_receiver.h"
#include "rtlsdr.h"
#include "usb_host_service.h"

static const char *TAG = "app";

#ifndef CONFIG_FM_RECEIVER_NFM_MODE
#define CONFIG_FM_RECEIVER_NFM_MODE 0
#endif
#ifndef CONFIG_FM_RECEIVER_AUDIO_OUTPUT
#define CONFIG_FM_RECEIVER_AUDIO_OUTPUT 1
#endif
#ifndef CONFIG_FM_RECEIVER_AUDIO_RATE_HZ
#define CONFIG_FM_RECEIVER_AUDIO_RATE_HZ 48000
#endif
#ifndef CONFIG_FM_RECEIVER_VOLUME_PERCENT
#define CONFIG_FM_RECEIVER_VOLUME_PERCENT 35
#endif
#ifndef CONFIG_FM_RECEIVER_AUDIO_I2C_PORT
#define CONFIG_FM_RECEIVER_AUDIO_I2C_PORT 0
#endif
#ifndef CONFIG_FM_RECEIVER_AUDIO_I2C_SDA_GPIO
#define CONFIG_FM_RECEIVER_AUDIO_I2C_SDA_GPIO 0
#endif
#ifndef CONFIG_FM_RECEIVER_AUDIO_I2C_SCL_GPIO
#define CONFIG_FM_RECEIVER_AUDIO_I2C_SCL_GPIO 1
#endif
#ifndef CONFIG_FM_RECEIVER_AUDIO_I2S_PORT
#define CONFIG_FM_RECEIVER_AUDIO_I2S_PORT 1
#endif
#ifndef CONFIG_FM_RECEIVER_AUDIO_I2S_MCLK_GPIO
#define CONFIG_FM_RECEIVER_AUDIO_I2S_MCLK_GPIO -1
#endif
#ifndef CONFIG_FM_RECEIVER_AUDIO_I2S_BCLK_GPIO
#define CONFIG_FM_RECEIVER_AUDIO_I2S_BCLK_GPIO 3
#endif
#ifndef CONFIG_FM_RECEIVER_AUDIO_I2S_WS_GPIO
#define CONFIG_FM_RECEIVER_AUDIO_I2S_WS_GPIO 4
#endif
#ifndef CONFIG_FM_RECEIVER_AUDIO_I2S_DOUT_GPIO
#define CONFIG_FM_RECEIVER_AUDIO_I2S_DOUT_GPIO 5
#endif
#ifndef CONFIG_FM_RECEIVER_AUDIO_I2S_DIN_GPIO
#define CONFIG_FM_RECEIVER_AUDIO_I2S_DIN_GPIO 6
#endif
#ifndef CONFIG_FM_RECEIVER_AUDIO_PA_GPIO
#define CONFIG_FM_RECEIVER_AUDIO_PA_GPIO 7
#endif

#define STARTUP_TEST_WATERFALL_DELAY_MS  6000
#define RTLSDR_SAMPLE_RATE_MIN_HZ         225001U
#define RTLSDR_SAMPLE_RATE_MAX_HZ         2400000U
#define RTLSDR_CENTER_FREQ_MIN_HZ         24000000U
#define RTLSDR_CENTER_FREQ_MAX_HZ         1766000000U
#define SCANNER_DEFAULT_START_HZ          88000000U
#define SCANNER_DEFAULT_STOP_HZ           108000000U
#define SCANNER_TUNE_SETTLE_MS            160U
#define SENSOR_DEFAULT_CENTER_HZ          433920000U
#define SENSOR_SAMPLE_RATE_HZ             250000U
#define SENSOR_FRAME_RESET_US             12000
#define SENSOR_MIN_PULSE_US               40
#define SENSOR_MAX_PULSE_US               30000
#define SENSOR_MAX_FRAME_PULSES           900
#define SENSOR_UNIQUE_MAX                 16
#define IQ_RECORD_MAX_BYTES               (512U * 1024U)
#define IQ_RECORD_PATH_MAX                80

typedef struct {
    bool high;
    uint16_t duration_us;
} sensor_pulse_t;

typedef struct {
    app_display_t *display;
    dsp_waterfall_t *dsp;
    fm_receiver_t *fm;
    adsb_receiver_t *adsb;
    rtlsdr_t *radio;
    RingbufHandle_t iq_ring;
    volatile uint32_t requested_center_hz;
    volatile uint32_t requested_sample_rate_hz;
    uint32_t base_sample_rate_hz;
    volatile bool have_samples;
    volatile bool reset_fm_demod;
    volatile bool reset_adsb_demod;
    volatile bool fm_page_active;
    volatile bool adsb_page_active;
    volatile bool scanner_page_active;
    volatile bool sensor_page_active;
    volatile bool record_page_active;
    volatile uint32_t iq_ring_frames;
    volatile uint32_t iq_ring_drops;
    uint32_t scanner_start_hz;
    uint32_t scanner_stop_hz;
    uint32_t scanner_hop_index;
    uint32_t scanner_hop_count;
    uint32_t scanner_current_center_hz;
    TickType_t scanner_ready_tick;
    uint32_t sensor_center_hz;
    bool sensor_has_state;
    bool sensor_current_high;
    uint32_t sensor_current_samples;
    uint32_t sensor_noise_floor;
    uint32_t sensor_signal_peak;
    uint8_t sensor_signal_level;
    sensor_pulse_t sensor_frame[SENSOR_MAX_FRAME_PULSES];
    uint16_t sensor_frame_pulses;
    uint64_t sensor_total_frames;
    uint64_t sensor_decoded_messages;
    uint32_t sensor_hashes[SENSOR_UNIQUE_MAX];
    uint32_t sensor_hash_count;
    FILE *record_file;
    bool record_storage_ready;
    bool record_finished;
    uint64_t record_bytes_written;
    uint32_t record_elapsed_seconds;
    int64_t record_start_us;
    char record_path[IQ_RECORD_PATH_MAX];
} app_ctx_t;

static void samples_cb(const uint8_t *iq, size_t len, void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    if (!app->iq_ring) {
        return;
    }
    if (xRingbufferSend(app->iq_ring, iq, len, 0) == pdTRUE) {
        app->have_samples = true;
        app->iq_ring_frames++;
    } else {
        app->iq_ring_drops++;
    }
}

static void status_cb(const char *status, void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    app_display_set_status(app->display, status);
}

static uint32_t scanner_sample_rate_hz(const app_ctx_t *app)
{
    uint32_t sample_rate_hz = app->requested_sample_rate_hz ? app->requested_sample_rate_hz : app->base_sample_rate_hz;
    if (sample_rate_hz == 0) {
        sample_rate_hz = CONFIG_RTLSDR_SAMPLE_RATE;
    }
    return sample_rate_hz;
}

static uint32_t scanner_calc_hop_count(const app_ctx_t *app)
{
    uint32_t sample_rate_hz = scanner_sample_rate_hz(app);
    uint32_t start_hz = app->scanner_start_hz;
    uint32_t stop_hz = app->scanner_stop_hz;
    if (stop_hz <= start_hz) {
        return 1;
    }
    uint32_t span_hz = stop_hz - start_hz;
    return (span_hz + sample_rate_hz - 1U) / sample_rate_hz;
}

static void scanner_segment_for_hop(const app_ctx_t *app,
                                    uint32_t hop_index,
                                    uint32_t *segment_start_hz,
                                    uint32_t *segment_stop_hz,
                                    uint32_t *center_hz)
{
    uint32_t sample_rate_hz = scanner_sample_rate_hz(app);
    uint32_t start_hz = app->scanner_start_hz + hop_index * sample_rate_hz;
    uint32_t stop_hz = start_hz + sample_rate_hz;
    if (stop_hz > app->scanner_stop_hz || stop_hz < start_hz) {
        stop_hz = app->scanner_stop_hz;
    }
    if (stop_hz <= start_hz) {
        stop_hz = start_hz + 1U;
    }

    uint64_t center = (uint64_t)start_hz + sample_rate_hz / 2U;
    if (center < RTLSDR_CENTER_FREQ_MIN_HZ) {
        center = RTLSDR_CENTER_FREQ_MIN_HZ;
    } else if (center > RTLSDR_CENTER_FREQ_MAX_HZ) {
        center = RTLSDR_CENTER_FREQ_MAX_HZ;
    }

    *segment_start_hz = start_hz;
    *segment_stop_hz = stop_hz;
    *center_hz = (uint32_t)center;
}

static void scanner_queue_current_hop(app_ctx_t *app)
{
    if (!app->scanner_page_active) {
        return;
    }
    app->scanner_hop_count = scanner_calc_hop_count(app);
    if (app->scanner_hop_count == 0) {
        app->scanner_hop_count = 1;
    }
    if (app->scanner_hop_index >= app->scanner_hop_count) {
        app->scanner_hop_index = 0;
    }

    uint32_t segment_start_hz = 0;
    uint32_t segment_stop_hz = 0;
    uint32_t center_hz = 0;
    scanner_segment_for_hop(app, app->scanner_hop_index, &segment_start_hz, &segment_stop_hz, &center_hz);
    app->scanner_current_center_hz = center_hz;
    app->requested_center_hz = center_hz;
    app->scanner_ready_tick = xTaskGetTickCount() + pdMS_TO_TICKS(SCANNER_TUNE_SETTLE_MS);

    if (app->radio) {
        esp_err_t ret = rtlsdr_set_center_freq_hz(app->radio, center_hz);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "queue scanner tune failed: %s", esp_err_to_name(ret));
        }
    }
}

static void scanner_advance_hop(app_ctx_t *app)
{
    if (app->scanner_hop_count == 0) {
        app->scanner_hop_count = scanner_calc_hop_count(app);
    }
    app->scanner_hop_index++;
    if (app->scanner_hop_index >= app->scanner_hop_count) {
        app->scanner_hop_index = 0;
    }
    scanner_queue_current_hop(app);
}

static void sensor_reset(app_ctx_t *app)
{
    app->sensor_has_state = false;
    app->sensor_current_high = false;
    app->sensor_current_samples = 0;
    app->sensor_noise_floor = 0;
    app->sensor_signal_peak = 0;
    app->sensor_signal_level = 0;
    app->sensor_frame_pulses = 0;
}

static uint32_t sensor_hash_data(const char *data)
{
    uint32_t hash = 2166136261U;
    while (*data) {
        hash ^= (uint8_t)*data++;
        hash *= 16777619U;
    }
    return hash ? hash : 1U;
}

static bool sensor_note_unique(app_ctx_t *app, uint32_t hash)
{
    for (uint32_t i = 0; i < app->sensor_hash_count; i++) {
        if (app->sensor_hashes[i] == hash) {
            return false;
        }
    }
    if (app->sensor_hash_count < SENSOR_UNIQUE_MAX) {
        app->sensor_hashes[app->sensor_hash_count++] = hash;
    }
    return true;
}

static void sensor_decode_frame(app_ctx_t *app)
{
    int high_pulses = 0;
    for (uint16_t i = 0; i < app->sensor_frame_pulses; i++) {
        if (app->sensor_frame[i].high) {
            high_pulses++;
        }
    }
    if (high_pulses < 8) {
        app->sensor_frame_pulses = 0;
        return;
    }

    app->sensor_total_frames++;

    uint16_t low_min = UINT16_MAX;
    uint16_t low_max = 0;
    uint16_t high_min = UINT16_MAX;
    uint16_t high_max = 0;
    for (uint16_t i = 0; i < app->sensor_frame_pulses; i++) {
        sensor_pulse_t pulse = app->sensor_frame[i];
        if (pulse.high) {
            if (pulse.duration_us < high_min) {
                high_min = pulse.duration_us;
            }
            if (pulse.duration_us > high_max) {
                high_max = pulse.duration_us;
            }
        } else if (pulse.duration_us < SENSOR_FRAME_RESET_US) {
            if (pulse.duration_us < low_min) {
                low_min = pulse.duration_us;
            }
            if (pulse.duration_us > low_max) {
                low_max = pulse.duration_us;
            }
        }
    }

    bool use_gap_width = low_max > low_min + 100U;
    uint16_t threshold = use_gap_width ? (uint16_t)(((uint32_t)low_min + low_max) / 2U)
                                       : (uint16_t)(((uint32_t)high_min + high_max) / 2U);
    char raw[56] = {0};
    uint32_t bit_count = 0;
    uint8_t nibble = 0;
    uint8_t nibble_bits = 0;

    for (uint16_t i = 0; i + 1 < app->sensor_frame_pulses && bit_count < 128; i++) {
        sensor_pulse_t high = app->sensor_frame[i];
        sensor_pulse_t low = app->sensor_frame[i + 1];
        if (!high.high || low.high || low.duration_us >= SENSOR_FRAME_RESET_US) {
            continue;
        }
        bool bit = use_gap_width ? (low.duration_us > threshold) : (high.duration_us > threshold);
        nibble = (uint8_t)((nibble << 1) | (bit ? 1U : 0U));
        nibble_bits++;
        bit_count++;
        if (nibble_bits == 4) {
            size_t len = strlen(raw);
            if (len + 1 < sizeof(raw)) {
                raw[len] = (char)(nibble < 10 ? ('0' + nibble) : ('A' + nibble - 10));
                raw[len + 1] = '\0';
            }
            nibble = 0;
            nibble_bits = 0;
        }
    }
    if (nibble_bits > 0) {
        nibble <<= (4 - nibble_bits);
        size_t len = strlen(raw);
        if (len + 1 < sizeof(raw)) {
            raw[len] = (char)(nibble < 10 ? ('0' + nibble) : ('A' + nibble - 10));
            raw[len + 1] = '\0';
        }
    }

    if (bit_count >= 16 && raw[0]) {
        app->sensor_decoded_messages++;
        uint32_t hash = sensor_hash_data(raw);
        (void)sensor_note_unique(app, hash);

        app_display_sensor_event_t event = {
            .frequency_hz = app->sensor_center_hz,
            .signal_level = app->sensor_signal_level,
            .count = app->sensor_decoded_messages,
            .last_seen_ms = esp_timer_get_time() / 1000,
        };
        snprintf(event.model, sizeof(event.model), "OOK/ASK");
        snprintf(event.id, sizeof(event.id), "%08lX", (unsigned long)hash);
        snprintf(event.data, sizeof(event.data), "%lub %s", (unsigned long)bit_count, raw);
        app_display_update_sensor_event(app->display, &event);
    }

    app->sensor_frame_pulses = 0;
}

static void sensor_append_pulse(app_ctx_t *app, bool high, uint32_t samples)
{
    if (samples == 0) {
        return;
    }
    uint32_t duration_us = (samples * 1000000ULL + SENSOR_SAMPLE_RATE_HZ / 2U) / SENSOR_SAMPLE_RATE_HZ;
    if (!high && duration_us >= SENSOR_FRAME_RESET_US) {
        sensor_decode_frame(app);
        return;
    }
    if (duration_us < SENSOR_MIN_PULSE_US || duration_us > SENSOR_MAX_PULSE_US) {
        return;
    }
    if (app->sensor_frame_pulses == 0 && !high) {
        return;
    }
    if (app->sensor_frame_pulses >= SENSOR_MAX_FRAME_PULSES) {
        sensor_decode_frame(app);
    }
    if (app->sensor_frame_pulses < SENSOR_MAX_FRAME_PULSES) {
        app->sensor_frame[app->sensor_frame_pulses++] = (sensor_pulse_t){
            .high = high,
            .duration_us = (uint16_t)MIN(duration_us, UINT16_MAX),
        };
    }
}

static void sensor_push_iq(app_ctx_t *app, const uint8_t *iq, size_t len)
{
    for (size_t offset = 0; offset + 1 < len; offset += 2) {
        int32_t i = (int32_t)iq[offset] - 127;
        int32_t q = (int32_t)iq[offset + 1] - 127;
        uint32_t power = (uint32_t)(i * i + q * q);

        if (!app->sensor_has_state) {
            app->sensor_noise_floor = power;
            app->sensor_signal_peak = power + 1U;
            app->sensor_current_high = false;
            app->sensor_current_samples = 1;
            app->sensor_has_state = true;
            continue;
        }

        if (!app->sensor_current_high || power < app->sensor_noise_floor) {
            app->sensor_noise_floor = (app->sensor_noise_floor * 9992U + power * 8U) / 10000U;
        }
        if (power > app->sensor_signal_peak) {
            app->sensor_signal_peak = (app->sensor_signal_peak * 94U + power * 6U) / 100U;
        } else {
            app->sensor_signal_peak = (app->sensor_signal_peak * 9994U + power * 6U) / 10000U;
        }

        uint32_t spread = (app->sensor_signal_peak > app->sensor_noise_floor)
            ? (app->sensor_signal_peak - app->sensor_noise_floor) : 1U;
        uint32_t high_threshold = app->sensor_noise_floor + (spread * 35U) / 100U;
        uint32_t low_threshold = app->sensor_noise_floor + (spread * 18U) / 100U;
        bool next_high = app->sensor_current_high ? (power > low_threshold) : (power > high_threshold);
        if (spread < 500U) {
            next_high = false;
        }

        if (next_high == app->sensor_current_high) {
            app->sensor_current_samples++;
        } else {
            sensor_append_pulse(app, app->sensor_current_high, app->sensor_current_samples);
            app->sensor_current_high = next_high;
            app->sensor_current_samples = 1;
        }

        uint32_t level = (spread * 100U) / 18000U;
        app->sensor_signal_level = (uint8_t)MIN(level, 100U);
    }
}

static esp_err_t record_mount_storage(app_ctx_t *app)
{
    if (app->record_storage_ready) {
        return ESP_OK;
    }
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 2,
        .format_if_mount_failed = true,
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret == ESP_ERR_INVALID_STATE) {
        ret = ESP_OK;
    }
    if (ret == ESP_OK) {
        app->record_storage_ready = true;
    } else {
        ESP_LOGW(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

static void record_start(app_ctx_t *app)
{
    if (app->record_file || app->record_finished) {
        return;
    }
    if (record_mount_storage(app) != ESP_OK) {
        return;
    }
    snprintf(app->record_path,
             sizeof(app->record_path),
             "/spiffs/iq_%luHz_%luSps.iq",
             (unsigned long)app->requested_center_hz,
             (unsigned long)app->requested_sample_rate_hz);
    app->record_file = fopen(app->record_path, "wb");
    if (!app->record_file) {
        ESP_LOGW(TAG, "open IQ record file failed");
        return;
    }
    app->record_bytes_written = 0;
    app->record_elapsed_seconds = 0;
    app->record_start_us = esp_timer_get_time();
    app->record_finished = false;
}

static void record_stop(app_ctx_t *app)
{
    if (app->record_file) {
        fclose(app->record_file);
        app->record_file = NULL;
    }
}

static void record_write_iq(app_ctx_t *app, const uint8_t *iq, size_t len)
{
    if (!app->record_page_active || app->record_finished) {
        return;
    }
    if (!app->record_file) {
        record_start(app);
    }
    if (!app->record_file) {
        return;
    }
    uint64_t remaining = IQ_RECORD_MAX_BYTES - app->record_bytes_written;
    if (remaining == 0) {
        app->record_finished = true;
        record_stop(app);
        return;
    }
    if (len > remaining) {
        len = (size_t)remaining;
    }
    size_t written = fwrite(iq, 1, len, app->record_file);
    app->record_bytes_written += written;
    app->record_elapsed_seconds = (uint32_t)((esp_timer_get_time() - app->record_start_us) / 1000000);
    if (written < len || app->record_bytes_written >= IQ_RECORD_MAX_BYTES) {
        app->record_finished = true;
        record_stop(app);
    }
}

static void line_cb(const uint8_t *bins, size_t bin_count, void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    if (app->scanner_page_active) {
        TickType_t now = xTaskGetTickCount();
        if ((int32_t)(now - app->scanner_ready_tick) < 0) {
            return;
        }

        uint32_t hop_index = app->scanner_hop_index;
        uint32_t hop_count = app->scanner_hop_count ? app->scanner_hop_count : scanner_calc_hop_count(app);
        uint32_t segment_start_hz = 0;
        uint32_t segment_stop_hz = 0;
        uint32_t center_hz = 0;
        scanner_segment_for_hop(app, hop_index, &segment_start_hz, &segment_stop_hz, &center_hz);
        app_display_push_scanner_hop(app->display,
                                     bins,
                                     bin_count,
                                     segment_start_hz,
                                     segment_stop_hz,
                                     app->scanner_start_hz,
                                     app->scanner_stop_hz,
                                     hop_index,
                                     hop_count);
        scanner_advance_hop(app);
        return;
    }
    if (app->sensor_page_active || app->record_page_active) {
        return;
    }
    if (app->fm_page_active) {
        app_display_push_fm_spectrum(app->display, bins, bin_count);
    } else {
        app_display_push_line(app->display, bins, bin_count);
    }
}

static void center_freq_cb(uint32_t center_hz, void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    app->requested_center_hz = center_hz;
    if (app->sensor_page_active) {
        app->sensor_center_hz = center_hz;
        sensor_reset(app);
    }
    if (!app->radio) {
        ESP_LOGI(TAG, "Center frequency queued before RTL-SDR is ready: %lu Hz", (unsigned long)center_hz);
        return;
    }

    esp_err_t ret = rtlsdr_set_center_freq_hz(app->radio, center_hz);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "queue center frequency failed: %s", esp_err_to_name(ret));
    } else {
        app->reset_fm_demod = true;
        app->reset_adsb_demod = true;
    }
}

static void page_cb(app_display_page_t page, void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    bool fm_active = page == APP_DISPLAY_PAGE_FM;
    bool adsb_active = page == APP_DISPLAY_PAGE_ADSB;
    bool scanner_active = page == APP_DISPLAY_PAGE_SCANNER;
    bool sensor_active = page == APP_DISPLAY_PAGE_SENSORS;
    bool record_active = page == APP_DISPLAY_PAGE_RECORD;
    app->fm_page_active = fm_active;
    app->adsb_page_active = adsb_active;
    app->scanner_page_active = scanner_active;
    app->sensor_page_active = sensor_active;
    app->record_page_active = record_active;
    app->reset_fm_demod = fm_active;
    app->reset_adsb_demod = adsb_active;
    app->requested_sample_rate_hz = adsb_active ? ADSB_DEFAULT_SAMPLE_RATE_HZ :
        (sensor_active ? SENSOR_SAMPLE_RATE_HZ : app->base_sample_rate_hz);
    if (app->radio) {
        esp_err_t ret = rtlsdr_set_sample_rate_hz(app->radio, app->requested_sample_rate_hz);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "queue sample rate failed: %s", esp_err_to_name(ret));
        }
    }
    if (app->fm) {
        fm_receiver_set_audio_enabled(app->fm, fm_active);
    }
    if (scanner_active) {
        app->scanner_hop_index = 0;
        app->scanner_hop_count = scanner_calc_hop_count(app);
        scanner_queue_current_hop(app);
    }
    if (sensor_active) {
        app->sensor_center_hz = app->requested_center_hz ? app->requested_center_hz : SENSOR_DEFAULT_CENTER_HZ;
        sensor_reset(app);
        if (app->radio) {
            (void)rtlsdr_set_center_freq_hz(app->radio, app->sensor_center_hz);
        }
    }
    if (record_active) {
        app->record_finished = false;
        app->record_bytes_written = 0;
        app->record_elapsed_seconds = 0;
        record_start(app);
    } else {
        record_stop(app);
    }
    if (app->radio && !scanner_active && page != APP_DISPLAY_PAGE_HOME && app->requested_center_hz != 0) {
        esp_err_t ret = rtlsdr_set_center_freq_hz(app->radio, app->requested_center_hz);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "queue page tune failed: %s", esp_err_to_name(ret));
        }
    }
}

static void adsb_aircraft_cb(const adsb_aircraft_t *aircraft, void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    if (!app || !aircraft) {
        return;
    }
    app_display_adsb_aircraft_t view = {
        .has_altitude = aircraft->has_altitude,
        .altitude_feet = aircraft->altitude_feet,
        .has_position = aircraft->has_position,
        .latitude = aircraft->latitude,
        .longitude = aircraft->longitude,
        .has_speed = aircraft->has_speed,
        .speed_knots = aircraft->speed_knots,
        .has_heading = aircraft->has_heading,
        .heading_degrees = aircraft->heading_degrees,
        .messages = aircraft->messages,
        .last_seen_ms = aircraft->last_seen_ms,
    };
    snprintf(view.icao, sizeof(view.icao), "%s", aircraft->icao);
    snprintf(view.callsign, sizeof(view.callsign), "%s", aircraft->callsign);
    app_display_update_adsb_aircraft(app->display, &view);
}

static uint32_t normalized_radio_sample_rate_hz(void)
{
    const uint32_t configured_hz = CONFIG_RTLSDR_SAMPLE_RATE;
    const uint32_t audio_hz = CONFIG_FM_RECEIVER_AUDIO_RATE_HZ;
    if (audio_hz == 0 || (configured_hz % audio_hz) == 0) {
        return configured_hz;
    }

    uint32_t decimation = (configured_hz + audio_hz / 2U) / audio_hz;
    if (decimation < 1U) {
        decimation = 1U;
    }

    while ((uint64_t)decimation * audio_hz < RTLSDR_SAMPLE_RATE_MIN_HZ) {
        decimation++;
    }
    while (decimation > 1U && (uint64_t)decimation * audio_hz > RTLSDR_SAMPLE_RATE_MAX_HZ) {
        decimation--;
    }

    uint32_t adjusted_hz = decimation * audio_hz;
    if (adjusted_hz < RTLSDR_SAMPLE_RATE_MIN_HZ || adjusted_hz > RTLSDR_SAMPLE_RATE_MAX_HZ) {
        ESP_LOGW(TAG,
                 "RTL-SDR sample rate %lu Hz is not an integer multiple of FM audio %lu Hz",
                 (unsigned long)configured_hz,
                 (unsigned long)audio_hz);
        return configured_hz;
    }

    ESP_LOGW(TAG,
             "Adjusted RTL-SDR sample rate from %lu Hz to %lu Hz for exact FM decimation to %lu Hz audio",
             (unsigned long)configured_hz,
             (unsigned long)adjusted_hz,
             (unsigned long)audio_hz);
    return adjusted_hz;
}

static void dsp_task(void *arg)
{
    app_ctx_t *app = (app_ctx_t *)arg;
    TickType_t last_test = xTaskGetTickCount();
    TickType_t test_start = last_test + pdMS_TO_TICKS(STARTUP_TEST_WATERFALL_DELAY_MS);

    while (true) {
        size_t item_size = 0;
        uint8_t *item = (uint8_t *)xRingbufferReceive(app->iq_ring, &item_size, pdMS_TO_TICKS(25));
        if (item) {
            if (app->record_page_active) {
                record_write_iq(app, item, item_size);
            }
            if (app->adsb_page_active) {
                if (app->reset_adsb_demod) {
                    adsb_receiver_reset(app->adsb);
                    app_display_clear_adsb(app->display);
                    app->reset_adsb_demod = false;
                }
                adsb_receiver_push_iq(app->adsb, item, item_size);
                vRingbufferReturnItem(app->iq_ring, item);
                continue;
            }
            if (app->sensor_page_active) {
                sensor_push_iq(app, item, item_size);
                dsp_waterfall_push_iq(app->dsp, item, item_size);
                vRingbufferReturnItem(app->iq_ring, item);
                continue;
            }
            if (app->reset_fm_demod) {
                fm_receiver_reset(app->fm);
                app->reset_fm_demod = false;
            }
            if (app->fm_page_active) {
                fm_receiver_push_iq(app->fm, item, item_size);
            }
            dsp_waterfall_push_iq(app->dsp, item, item_size);
            vRingbufferReturnItem(app->iq_ring, item);
            continue;
        }

        TickType_t now = xTaskGetTickCount();
        if (!app->have_samples && now >= test_start && now - last_test > pdMS_TO_TICKS(60)) {
            dsp_waterfall_generate_test_line(app->dsp);
            last_test = now;
        }
    }
}

void app_main(void)
{
    app_ctx_t app = {0};
    app.requested_center_hz = CONFIG_RTLSDR_CENTER_FREQ_HZ;
    const uint32_t radio_sample_rate_hz = normalized_radio_sample_rate_hz();
    app.base_sample_rate_hz = radio_sample_rate_hz;
    app.requested_sample_rate_hz = radio_sample_rate_hz;
    app.scanner_start_hz = SCANNER_DEFAULT_START_HZ;
    app.scanner_stop_hz = SCANNER_DEFAULT_STOP_HZ;
    app.scanner_hop_count = scanner_calc_hop_count(&app);
    app.sensor_center_hz = SENSOR_DEFAULT_CENTER_HZ;
    snprintf(app.record_path, sizeof(app.record_path), "/spiffs/iq_%luHz_%luSps.iq",
             (unsigned long)app.requested_center_hz,
             (unsigned long)app.requested_sample_rate_hz);
    (void)record_mount_storage(&app);

    app_display_config_t display_cfg = {
        .h_res = CONFIG_LCD_H_RES,
        .v_res = CONFIG_LCD_V_RES,
        .pclk_hz = CONFIG_LCD_PIXEL_CLOCK_HZ,
        .hsync = CONFIG_LCD_HSYNC,
        .hbp = CONFIG_LCD_HBP,
        .hfp = CONFIG_LCD_HFP,
        .vsync = CONFIG_LCD_VSYNC,
        .vbp = CONFIG_LCD_VBP,
        .vfp = CONFIG_LCD_VFP,
        .pclk_active_neg = CONFIG_LCD_PCLK_ACTIVE_NEG,
    };

    ESP_ERROR_CHECK(app_display_create(&display_cfg, &app.display));
    app_display_set_radio_info(app.display, CONFIG_RTLSDR_CENTER_FREQ_HZ, radio_sample_rate_hz);
    app_display_set_center_freq_cb(app.display, center_freq_cb, &app);

    uint32_t fft_lines_per_second = radio_sample_rate_hz / CONFIG_WATERFALL_FFT_SIZE;
    size_t fft_line_decimation = fft_lines_per_second / CONFIG_WATERFALL_TARGET_FPS;
    if (fft_line_decimation < 1) {
        fft_line_decimation = 1;
    }
    ESP_LOGI(TAG, "FFT line decimation %u: %lu raw lines/s -> about %lu display lines/s",
             (unsigned)fft_line_decimation,
             (unsigned long)fft_lines_per_second,
             (unsigned long)(fft_lines_per_second / fft_line_decimation));

    dsp_waterfall_config_t dsp_cfg = {
        .fft_size = CONFIG_WATERFALL_FFT_SIZE,
        .fft_line_decimation = fft_line_decimation,
        .average_shift = CONFIG_WATERFALL_AVERAGE_SHIFT,
        .db_min = CONFIG_WATERFALL_DB_MIN,
        .db_max = CONFIG_WATERFALL_DB_MAX,
    };
    ESP_ERROR_CHECK(dsp_waterfall_create(&dsp_cfg, line_cb, &app, &app.dsp));

    adsb_receiver_config_t adsb_cfg = {
        .sample_rate_hz = ADSB_DEFAULT_SAMPLE_RATE_HZ,
        .aircraft_cb = adsb_aircraft_cb,
        .aircraft_cb_ctx = &app,
    };
    ESP_ERROR_CHECK(adsb_receiver_create(&adsb_cfg, &app.adsb));

    fm_receiver_config_t fm_cfg = {
        .mode = CONFIG_FM_RECEIVER_NFM_MODE ? FM_RECEIVER_MODE_NFM : FM_RECEIVER_MODE_WFM,
        .sample_rate_hz = radio_sample_rate_hz,
        .audio_rate_hz = CONFIG_FM_RECEIVER_AUDIO_RATE_HZ,
        .volume_percent = CONFIG_FM_RECEIVER_VOLUME_PERCENT,
        .audio_output = CONFIG_FM_RECEIVER_AUDIO_OUTPUT ?
            FM_RECEIVER_AUDIO_OUTPUT_KORVO_SPEAKER : FM_RECEIVER_AUDIO_OUTPUT_NONE,
        .audio_i2c_port = CONFIG_FM_RECEIVER_AUDIO_I2C_PORT,
        .audio_i2c_sda_gpio = CONFIG_FM_RECEIVER_AUDIO_I2C_SDA_GPIO,
        .audio_i2c_scl_gpio = CONFIG_FM_RECEIVER_AUDIO_I2C_SCL_GPIO,
        .audio_i2s_port = CONFIG_FM_RECEIVER_AUDIO_I2S_PORT,
        .audio_i2s_mclk_gpio = CONFIG_FM_RECEIVER_AUDIO_I2S_MCLK_GPIO,
        .audio_i2s_bclk_gpio = CONFIG_FM_RECEIVER_AUDIO_I2S_BCLK_GPIO,
        .audio_i2s_ws_gpio = CONFIG_FM_RECEIVER_AUDIO_I2S_WS_GPIO,
        .audio_i2s_dout_gpio = CONFIG_FM_RECEIVER_AUDIO_I2S_DOUT_GPIO,
        .audio_i2s_din_gpio = CONFIG_FM_RECEIVER_AUDIO_I2S_DIN_GPIO,
        .audio_pa_gpio = CONFIG_FM_RECEIVER_AUDIO_PA_GPIO,
        .waveform_cb = NULL,
        .waveform_cb_ctx = NULL,
    };
    ESP_ERROR_CHECK(fm_receiver_create(&fm_cfg, &app.fm));
    app_display_set_page_cb(app.display, page_cb, &app);
    app_display_set_fm_info(app.display,
                                  fm_receiver_mode_name(fm_cfg.mode),
                                  fm_cfg.audio_rate_hz,
                                  false,
                                  false,
                                  0);
    app.iq_ring = xRingbufferCreate(CONFIG_RTLSDR_BULK_TRANSFER_SIZE * CONFIG_RTLSDR_NUM_TRANSFERS * 2,
                                    RINGBUF_TYPE_BYTEBUF);
    ESP_ERROR_CHECK(app.iq_ring ? ESP_OK : ESP_ERR_NO_MEM);

    esp_err_t ret = usb_host_service_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB host failed: %s", esp_err_to_name(ret));
        app_display_set_status(app.display, "USB HOST FAILED - TEST MODE");
        return;
    }

    rtlsdr_config_t radio_cfg = {
        .center_freq_hz = CONFIG_RTLSDR_CENTER_FREQ_HZ,
        .sample_rate_hz = radio_sample_rate_hz,
        .tuner_gain_tenths_db = CONFIG_RTLSDR_TUNER_GAIN_TENTHS_DB,
        .baseband_init = CONFIG_RTLSDR_BASEBAND_INIT,
    };

    ret = rtlsdr_start(&app.radio, &radio_cfg, samples_cb, status_cb, &app);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RTL-SDR task failed: %s", esp_err_to_name(ret));
        app_display_set_status(app.display, "RTLSDR TASK FAILED - TEST MODE");
    } else if (app.requested_center_hz != radio_cfg.center_freq_hz) {
        (void)rtlsdr_set_center_freq_hz(app.radio, app.requested_center_hz);
    }
    if (app.radio && app.requested_sample_rate_hz != radio_cfg.sample_rate_hz) {
        (void)rtlsdr_set_sample_rate_hz(app.radio, app.requested_sample_rate_hz);
    }

    BaseType_t ok = xTaskCreatePinnedToCore(dsp_task, "fft_waterfall", 8192, &app, 4, NULL, 1);
    ESP_ERROR_CHECK(ok == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM);

    uint32_t last_iq_frames = 0;
    uint32_t last_iq_drops = 0;
    uint32_t last_audio_frames = 0;
    uint32_t last_audio_drops = 0;
    uint32_t last_pcm_ring_drops = 0;
    uint32_t last_audio_write_drops = 0;

    while (true) {
        fm_receiver_status_t fm_status = {0};
        adsb_status_t adsb_status = {0};
        fm_receiver_get_status(app.fm, &fm_status);
        adsb_receiver_get_status(app.adsb, &adsb_status);
        app_display_set_sample_rate(app.display, app.requested_sample_rate_hz);
        app_display_set_scanner_info(app.display,
                                     app.scanner_start_hz,
                                     app.scanner_stop_hz,
                                     app.scanner_page_active,
                                     app.scanner_current_center_hz,
                                     app.scanner_hop_index,
                                     app.scanner_hop_count);
        app_display_set_sensor_info(app.display,
                                    app.sensor_center_hz,
                                    SENSOR_SAMPLE_RATE_HZ,
                                    app.sensor_page_active,
                                    app.sensor_total_frames,
                                    app.sensor_decoded_messages,
                                    app.sensor_hash_count,
                                    app.sensor_signal_level);
        app_display_set_record_info(app.display,
                                    app.requested_center_hz,
                                    app.requested_sample_rate_hz,
                                    app.record_page_active && app.record_file != NULL,
                                    app.record_storage_ready,
                                    app.record_bytes_written,
                                    IQ_RECORD_MAX_BYTES,
                                    app.record_elapsed_seconds,
                                    app.record_path);
        app_display_set_adsb_info(app.display,
                                  ADSB_DEFAULT_CENTER_HZ,
                                  ADSB_DEFAULT_SAMPLE_RATE_HZ,
                                  app.adsb_page_active,
                                  adsb_status.candidates,
                                  adsb_status.valid_messages,
                                  adsb_status.aircraft_count);
        app_display_set_fm_info(app.display,
                                      fm_receiver_mode_name(fm_status.mode),
                                      fm_status.audio_rate_hz,
                                      fm_status.audio_output_enabled,
                                      fm_status.audio_output_ready,
                                      fm_status.audio_level);
        uint32_t iq_frames = app.iq_ring_frames;
        uint32_t iq_drops = app.iq_ring_drops;
        ESP_LOGI(TAG,
                 "FM stream: iq=%lu/s iq_drop=%lu/s pcm=%lu/s drop=%lu/s ring_drop=%lu/s write_drop=%lu/s level=%u",
                 (unsigned long)(iq_frames - last_iq_frames),
                 (unsigned long)(iq_drops - last_iq_drops),
                 (unsigned long)(fm_status.audio_frames - last_audio_frames),
                 (unsigned long)(fm_status.dropped_audio_frames - last_audio_drops),
                 (unsigned long)(fm_status.dropped_pcm_ring_frames - last_pcm_ring_drops),
                 (unsigned long)(fm_status.dropped_audio_write_frames - last_audio_write_drops),
                 (unsigned)fm_status.audio_level);
        last_iq_frames = iq_frames;
        last_iq_drops = iq_drops;
        last_audio_frames = fm_status.audio_frames;
        last_audio_drops = fm_status.dropped_audio_frames;
        last_pcm_ring_drops = fm_status.dropped_pcm_ring_frames;
        last_audio_write_drops = fm_status.dropped_audio_write_frames;
        app_display_render_idle(app.display);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
