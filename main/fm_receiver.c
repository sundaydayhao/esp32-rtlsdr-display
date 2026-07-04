#include "fm_receiver.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "driver/i2s_common.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"

#define FM_WFM_AUDIO_CUTOFF_HZ     16000.0f
#define FM_NFM_AUDIO_CUTOFF_HZ     5500.0f
#define FM_WFM_DEVIATION_HZ        75000.0f
#define FM_NFM_DEVIATION_HZ        5000.0f
#define FM_WFM_DEEMPHASIS_US       50.0f
#define FM_NFM_DEEMPHASIS_US       0.0f
#define FM_WFM_TAP_COUNT           101
#define FM_NFM_TAP_COUNT           81
#define FM_MAX_TAP_COUNT           FM_WFM_TAP_COUNT
#define FM_PCM_FRAME_SAMPLES       256
#define FM_PCM_RING_BYTES          (FM_PCM_FRAME_SAMPLES * sizeof(int16_t) * 12)
#define FM_WAVEFORM_SAMPLES        128
#define FM_WAVEFORM_INTERVAL_MS    45

typedef struct {
    fm_receiver_mode_t mode;
    const char *name;
    uint32_t audio_rate_hz;
    float audio_cutoff_hz;
    float deviation_hz;
    float deemphasis_us;
    int tap_count;
} fm_mode_profile_t;

struct fm_receiver {
    fm_receiver_config_t cfg;
    fm_mode_profile_t profile;
    float full_scale_radians;
    uint32_t decimation;
    uint32_t decimation_phase;
    bool has_previous_iq;
    float previous_i;
    float previous_q;
    float previous_audio_input;
    float dc_output;
    float deemphasis_alpha;
    float deemphasis_output;
    float taps[FM_MAX_TAP_COUNT];
    float ring[FM_MAX_TAP_COUNT];
    int ring_index;
    int16_t pcm_frame[FM_PCM_FRAME_SAMPLES];
    size_t pcm_frame_used;
    int16_t waveform[FM_WAVEFORM_SAMPLES];
    size_t waveform_used;
    uint32_t waveform_decimation;
    uint32_t waveform_phase;
    uint64_t level_energy_accum;
    uint32_t level_sample_count;
    fm_receiver_status_t status;
    portMUX_TYPE status_lock;
    RingbufHandle_t pcm_ring;
    TaskHandle_t audio_task;
    volatile bool audio_task_stop;
    i2s_chan_handle_t i2s_tx;
};

static const char *TAG = "fm_receiver";

static fm_mode_profile_t make_profile(fm_receiver_mode_t mode, uint32_t audio_rate_hz)
{
    fm_mode_profile_t profile = {
        .mode = mode,
        .name = "WFM",
        .audio_rate_hz = audio_rate_hz,
        .audio_cutoff_hz = FM_WFM_AUDIO_CUTOFF_HZ,
        .deviation_hz = FM_WFM_DEVIATION_HZ,
        .deemphasis_us = FM_WFM_DEEMPHASIS_US,
        .tap_count = FM_WFM_TAP_COUNT,
    };

    if (mode == FM_RECEIVER_MODE_NFM) {
        profile.name = "NFM";
        profile.audio_cutoff_hz = FM_NFM_AUDIO_CUTOFF_HZ;
        profile.deviation_hz = FM_NFM_DEVIATION_HZ;
        profile.deemphasis_us = FM_NFM_DEEMPHASIS_US;
        profile.tap_count = FM_NFM_TAP_COUNT;
    }

    float nyquist_limit = (float)audio_rate_hz * 0.45f;
    if (profile.audio_cutoff_hz > nyquist_limit) {
        profile.audio_cutoff_hz = nyquist_limit;
    }
    return profile;
}

const char *fm_receiver_mode_name(fm_receiver_mode_t mode)
{
    return (mode == FM_RECEIVER_MODE_NFM) ? "NFM" : "WFM";
}

static int16_t clamp_pcm(float sample)
{
    if (sample > 1.0f) {
        sample = 1.0f;
    } else if (sample < -1.0f) {
        sample = -1.0f;
    }
    return (int16_t)lrintf(sample * 32767.0f);
}

static void build_low_pass_taps(fm_receiver_t *fm)
{
    const int tap_count = fm->profile.tap_count;
    const int center = tap_count / 2;
    float normalized_cutoff = fm->profile.audio_cutoff_hz / (float)fm->cfg.sample_rate_hz;
    if (normalized_cutoff < 0.001f) {
        normalized_cutoff = 0.001f;
    } else if (normalized_cutoff > 0.49f) {
        normalized_cutoff = 0.49f;
    }

    float sum = 0.0f;
    for (int i = 0; i < tap_count; i++) {
        const int n = i - center;
        const float sinc = (n == 0)
            ? 2.0f * normalized_cutoff
            : sinf(2.0f * (float)M_PI * normalized_cutoff * (float)n) / ((float)M_PI * (float)n);
        const float window = 0.42f
            - 0.5f * cosf(2.0f * (float)M_PI * (float)i / (float)(tap_count - 1))
            + 0.08f * cosf(4.0f * (float)M_PI * (float)i / (float)(tap_count - 1));
        fm->taps[i] = sinc * window;
        sum += fm->taps[i];
    }

    if (fabsf(sum) > 1.0e-9f) {
        for (int i = 0; i < tap_count; i++) {
            fm->taps[i] /= sum;
        }
    }
}

static void update_audio_level(fm_receiver_t *fm, int16_t pcm)
{
    int32_t sample = pcm;
    fm->level_energy_accum += (uint64_t)(sample * sample);
    fm->level_sample_count++;

    if (fm->level_sample_count >= fm->profile.audio_rate_hz / 10U) {
        float mean = (float)fm->level_energy_accum / (float)fm->level_sample_count;
        float rms = sqrtf(mean) / 32768.0f;
        uint8_t level = (uint8_t)lrintf(fminf(1.0f, rms * 3.0f) * 100.0f);
        portENTER_CRITICAL(&fm->status_lock);
        fm->status.audio_level = level;
        portEXIT_CRITICAL(&fm->status_lock);
        fm->level_energy_accum = 0;
        fm->level_sample_count = 0;
    }
}

static void update_waveform(fm_receiver_t *fm, int16_t pcm)
{
    if (!fm->cfg.waveform_cb) {
        return;
    }

    fm->waveform_phase++;
    if (fm->waveform_phase < fm->waveform_decimation) {
        return;
    }
    fm->waveform_phase = 0;

    fm->waveform[fm->waveform_used++] = pcm;
    if (fm->waveform_used >= FM_WAVEFORM_SAMPLES) {
        fm->cfg.waveform_cb(fm->waveform, FM_WAVEFORM_SAMPLES, fm->cfg.waveform_cb_ctx);
        fm->waveform_used = 0;
    }
}

static void submit_pcm_frame(fm_receiver_t *fm)
{
    if (fm->pcm_frame_used == 0) {
        return;
    }

    bool sent = !fm->cfg.i2s_enabled;
    if (fm->pcm_ring) {
        size_t bytes = fm->pcm_frame_used * sizeof(fm->pcm_frame[0]);
        sent = xRingbufferSend(fm->pcm_ring, fm->pcm_frame, bytes, 0) == pdTRUE;
    }

    portENTER_CRITICAL(&fm->status_lock);
    fm->status.audio_frames++;
    if (!sent) {
        fm->status.dropped_audio_frames++;
    }
    portEXIT_CRITICAL(&fm->status_lock);
    fm->pcm_frame_used = 0;
}

static bool demod_sample(fm_receiver_t *fm, uint8_t raw_i, uint8_t raw_q, int16_t *pcm)
{
    const float in_phase = ((float)raw_i - 127.5f) / 128.0f;
    const float quadrature = ((float)raw_q - 127.5f) / 128.0f;

    if (!fm->has_previous_iq) {
        fm->previous_i = in_phase;
        fm->previous_q = quadrature;
        fm->has_previous_iq = true;
        return false;
    }

    const float real = in_phase * fm->previous_i + quadrature * fm->previous_q;
    const float imag = quadrature * fm->previous_i - in_phase * fm->previous_q;
    const float demodulated = atan2f(imag, real);
    fm->previous_i = in_phase;
    fm->previous_q = quadrature;

    fm->ring[fm->ring_index] = demodulated;
    fm->ring_index = (fm->ring_index + 1) % fm->profile.tap_count;

    fm->decimation_phase++;
    if (fm->decimation_phase < fm->decimation) {
        return false;
    }
    fm->decimation_phase = 0;

    float filtered = 0.0f;
    int index = fm->ring_index - 1;
    if (index < 0) {
        index = fm->profile.tap_count - 1;
    }
    for (int tap = 0; tap < fm->profile.tap_count; tap++) {
        filtered += fm->taps[tap] * fm->ring[index];
        index--;
        if (index < 0) {
            index = fm->profile.tap_count - 1;
        }
    }

    const float dc_blocked = filtered - fm->previous_audio_input + 0.995f * fm->dc_output;
    fm->previous_audio_input = filtered;
    fm->dc_output = dc_blocked;

    float audio = dc_blocked;
    if (fm->deemphasis_alpha > 0.0f) {
        fm->deemphasis_output += fm->deemphasis_alpha * (audio - fm->deemphasis_output);
        audio = fm->deemphasis_output;
    }

    const float volume = (float)fm->cfg.volume_percent / 100.0f;
    *pcm = clamp_pcm((audio / fm->full_scale_radians) * volume);
    return true;
}

static esp_err_t init_i2s(fm_receiver_t *fm)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 6;
    chan_cfg.dma_frame_num = 240;

    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &fm->i2s_tx, NULL), TAG, "new i2s tx");

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(fm->profile.audio_rate_hz),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)fm->cfg.i2s_bclk_gpio,
            .ws = (gpio_num_t)fm->cfg.i2s_ws_gpio,
            .dout = (gpio_num_t)fm->cfg.i2s_dout_gpio,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(fm->i2s_tx, &std_cfg), TAG, "init i2s std");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(fm->i2s_tx), TAG, "enable i2s tx");
    return ESP_OK;
}

static void deinit_i2s(fm_receiver_t *fm)
{
    if (!fm->i2s_tx) {
        return;
    }
    (void)i2s_channel_disable(fm->i2s_tx);
    (void)i2s_del_channel(fm->i2s_tx);
    fm->i2s_tx = NULL;
}

static void audio_task(void *arg)
{
    fm_receiver_t *fm = (fm_receiver_t *)arg;
    ESP_LOGI(TAG, "Starting FM audio task");

    while (!fm->audio_task_stop) {
        size_t item_size = 0;
        uint8_t *item = xRingbufferReceive(fm->pcm_ring, &item_size, pdMS_TO_TICKS(100));
        if (!item) {
            continue;
        }

        if (fm->i2s_tx) {
            size_t written = 0;
            esp_err_t ret = i2s_channel_write(fm->i2s_tx, item, item_size, &written, pdMS_TO_TICKS(40));
            if (ret != ESP_OK || written != item_size) {
                portENTER_CRITICAL(&fm->status_lock);
                fm->status.dropped_audio_frames++;
                portEXIT_CRITICAL(&fm->status_lock);
            }
        }

        vRingbufferReturnItem(fm->pcm_ring, item);
    }

    ESP_LOGI(TAG, "FM audio task stopped");
    fm->audio_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t fm_receiver_create(const fm_receiver_config_t *config, fm_receiver_t **out)
{
    ESP_RETURN_ON_FALSE(config && out, ESP_ERR_INVALID_ARG, TAG, "bad args");
    ESP_RETURN_ON_FALSE(config->sample_rate_hz > 0 && config->audio_rate_hz >= 8000, ESP_ERR_INVALID_ARG, TAG, "bad rates");

    fm_receiver_t *fm = heap_caps_calloc(1, sizeof(*fm), MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(fm, ESP_ERR_NO_MEM, TAG, "no memory");

    fm->cfg = *config;
    if (fm->cfg.volume_percent < 0) {
        fm->cfg.volume_percent = 0;
    } else if (fm->cfg.volume_percent > 150) {
        fm->cfg.volume_percent = 150;
    }
    fm->profile = make_profile(fm->cfg.mode, fm->cfg.audio_rate_hz);
    fm->decimation = fm->cfg.sample_rate_hz / fm->profile.audio_rate_hz;
    if (fm->decimation < 1) {
        fm->decimation = 1;
    }
    fm->full_scale_radians = fmaxf(1.0e-6f,
                                   2.0f * (float)M_PI * fm->profile.deviation_hz /
                                   (float)fm->cfg.sample_rate_hz);
    if (fm->profile.deemphasis_us > 0.0f) {
        const float dt = 1.0f / (float)fm->profile.audio_rate_hz;
        const float tau = fm->profile.deemphasis_us / 1000000.0f;
        fm->deemphasis_alpha = dt / (tau + dt);
    }
    fm->status_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    fm->status.mode = fm->cfg.mode;
    fm->status.sample_rate_hz = fm->cfg.sample_rate_hz;
    fm->status.audio_rate_hz = fm->profile.audio_rate_hz;
    fm->status.audio_output_enabled = fm->cfg.i2s_enabled;
    fm->waveform_decimation = (fm->profile.audio_rate_hz * FM_WAVEFORM_INTERVAL_MS) /
                              (1000U * FM_WAVEFORM_SAMPLES);
    if (fm->waveform_decimation < 1) {
        fm->waveform_decimation = 1;
    }

    build_low_pass_taps(fm);

    if (fm->cfg.i2s_enabled) {
        fm->pcm_ring = xRingbufferCreate(FM_PCM_RING_BYTES, RINGBUF_TYPE_BYTEBUF);
        if (!fm->pcm_ring) {
            fm_receiver_destroy(fm);
            return ESP_ERR_NO_MEM;
        }

        esp_err_t ret = init_i2s(fm);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "I2S audio disabled: %s", esp_err_to_name(ret));
            fm->status.audio_output_ready = false;
        } else {
            fm->status.audio_output_ready = true;
            BaseType_t ok = xTaskCreatePinnedToCore(audio_task, "fm_audio", 4096, fm, 5, &fm->audio_task, 1);
            if (ok != pdTRUE) {
                fm_receiver_destroy(fm);
                return ESP_ERR_NO_MEM;
            }
        }
    }

    ESP_LOGI(TAG,
             "FM receiver ready: %s, IQ %lu Hz, audio %lu Hz, decimation %lu, I2S %s",
             fm->profile.name,
             (unsigned long)fm->cfg.sample_rate_hz,
             (unsigned long)fm->profile.audio_rate_hz,
             (unsigned long)fm->decimation,
             fm->status.audio_output_ready ? "ready" : (fm->cfg.i2s_enabled ? "failed" : "off"));

    *out = fm;
    return ESP_OK;
}

void fm_receiver_destroy(fm_receiver_t *fm)
{
    if (!fm) {
        return;
    }

    fm->audio_task_stop = true;
    if (fm->audio_task) {
        for (int i = 0; i < 100 && fm->audio_task; i++) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    deinit_i2s(fm);
    if (fm->pcm_ring) {
        vRingbufferDelete(fm->pcm_ring);
    }
    heap_caps_free(fm);
}

void fm_receiver_reset(fm_receiver_t *fm)
{
    if (!fm) {
        return;
    }

    fm->has_previous_iq = false;
    fm->previous_i = 0.0f;
    fm->previous_q = 0.0f;
    fm->previous_audio_input = 0.0f;
    fm->dc_output = 0.0f;
    fm->deemphasis_output = 0.0f;
    fm->decimation_phase = 0;
    fm->ring_index = 0;
    memset(fm->ring, 0, sizeof(fm->ring));
    fm->pcm_frame_used = 0;
    fm->waveform_used = 0;
    fm->waveform_phase = 0;
    fm->level_energy_accum = 0;
    fm->level_sample_count = 0;

    portENTER_CRITICAL(&fm->status_lock);
    fm->status.audio_level = 0;
    portEXIT_CRITICAL(&fm->status_lock);
}

esp_err_t fm_receiver_push_iq(fm_receiver_t *fm, const uint8_t *iq, size_t len)
{
    ESP_RETURN_ON_FALSE(fm && iq && ((len & 1U) == 0), ESP_ERR_INVALID_ARG, TAG, "bad IQ buffer");

    for (size_t i = 0; i + 1 < len; i += 2) {
        int16_t pcm = 0;
        if (!demod_sample(fm, iq[i], iq[i + 1], &pcm)) {
            continue;
        }

        fm->pcm_frame[fm->pcm_frame_used++] = pcm;
        update_audio_level(fm, pcm);
        update_waveform(fm, pcm);
        if (fm->pcm_frame_used >= FM_PCM_FRAME_SAMPLES) {
            submit_pcm_frame(fm);
        }
    }

    return ESP_OK;
}

void fm_receiver_get_status(fm_receiver_t *fm, fm_receiver_status_t *status)
{
    if (!fm || !status) {
        return;
    }

    portENTER_CRITICAL(&fm->status_lock);
    *status = fm->status;
    portEXIT_CRITICAL(&fm->status_lock);
}
