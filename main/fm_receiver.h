#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fm_receiver fm_receiver_t;
typedef void (*fm_receiver_waveform_cb_t)(const int16_t *samples, size_t sample_count, void *ctx);

typedef enum {
    FM_RECEIVER_MODE_WFM = 0,
    FM_RECEIVER_MODE_NFM,
} fm_receiver_mode_t;

typedef struct {
    fm_receiver_mode_t mode;
    uint32_t sample_rate_hz;
    uint32_t audio_rate_hz;
    int volume_percent;
    bool i2s_enabled;
    int i2s_bclk_gpio;
    int i2s_ws_gpio;
    int i2s_dout_gpio;
    fm_receiver_waveform_cb_t waveform_cb;
    void *waveform_cb_ctx;
} fm_receiver_config_t;

typedef struct {
    fm_receiver_mode_t mode;
    uint32_t sample_rate_hz;
    uint32_t audio_rate_hz;
    uint32_t audio_frames;
    uint32_t dropped_audio_frames;
    uint8_t audio_level;
    bool audio_output_enabled;
    bool audio_output_ready;
} fm_receiver_status_t;

esp_err_t fm_receiver_create(const fm_receiver_config_t *config, fm_receiver_t **out);
void fm_receiver_destroy(fm_receiver_t *fm);

esp_err_t fm_receiver_push_iq(fm_receiver_t *fm, const uint8_t *iq, size_t len);
void fm_receiver_reset(fm_receiver_t *fm);
void fm_receiver_get_status(fm_receiver_t *fm, fm_receiver_status_t *status);
const char *fm_receiver_mode_name(fm_receiver_mode_t mode);

#ifdef __cplusplus
}
#endif
