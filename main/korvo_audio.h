#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct korvo_audio_output korvo_audio_output_t;

typedef struct {
    uint32_t sample_rate_hz;
    int volume_percent;
    int i2c_port;
    int i2c_sda_gpio;
    int i2c_scl_gpio;
    int i2s_port;
    int i2s_bclk_gpio;
    int i2s_ws_gpio;
    int i2s_dout_gpio;
    int i2s_din_gpio;
    int i2s_mclk_gpio;
    int pa_gpio;
} korvo_audio_output_config_t;

esp_err_t korvo_audio_output_create(const korvo_audio_output_config_t *config,
                                    korvo_audio_output_t **out);
void korvo_audio_output_destroy(korvo_audio_output_t *audio);
esp_err_t korvo_audio_output_set_volume(korvo_audio_output_t *audio, int volume_percent);
esp_err_t korvo_audio_output_write_mono(korvo_audio_output_t *audio,
                                        const int16_t *samples,
                                        size_t sample_count,
                                        uint32_t timeout_ms,
                                        size_t *samples_written);

#ifdef __cplusplus
}
#endif
