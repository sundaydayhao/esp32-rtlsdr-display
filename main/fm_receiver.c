#include "fm_receiver.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "korvo_audio.h"

#define FM_WFM_AUDIO_CUTOFF_HZ     16000.0f
#define FM_NFM_AUDIO_CUTOFF_HZ     5500.0f
#define FM_WFM_DEVIATION_HZ        75000.0f
#define FM_NFM_DEVIATION_HZ        5000.0f
#define FM_WFM_DEEMPHASIS_US       50.0f
#define FM_NFM_DEEMPHASIS_US       0.0f
#define FM_WFM_TAP_COUNT           101
#define FM_NFM_TAP_COUNT           81
#define FM_MAX_TAP_COUNT           FM_WFM_TAP_COUNT
#define FM_PCM_FRAME_SAMPLES       512
#define FM_PCM_RING_FRAME_COUNT    40
#define FM_PCM_RING_BYTES          (FM_PCM_FRAME_SAMPLES * sizeof(int16_t) * FM_PCM_RING_FRAME_COUNT)
#define FM_PCM_SEND_TIMEOUT_MS     12
#define FM_AUDIO_PREBUFFER_FRAMES  6
#define FM_WAVEFORM_SAMPLES        128
#define FM_WAVEFORM_INTERVAL_MS    120

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
    volatile bool audio_enabled;
    volatile bool audio_prebuffering;
    volatile uint32_t pcm_queued_frames;
    korvo_audio_output_t *audio_output;
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

static float fast_atan2f_approx(float y, float x)
{
    const float one_quarter_pi = (float)M_PI * 0.25f;
    const float three_quarter_pi = (float)M_PI * 0.75f;
    float abs_y = fabsf(y) + 1.0e-12f;
    float angle;
    float r;

    if (x < 0.0f) {
        r = (x + abs_y) / (abs_y - x);
        angle = three_quarter_pi;
    } else {
        r = (x - abs_y) / (x + abs_y);
        angle = one_quarter_pi;
    }

    angle += (0.1963f * r * r - 0.9817f) * r;
    return y < 0.0f ? -angle : angle;
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

    bool sent = fm->cfg.audio_output == FM_RECEIVER_AUDIO_OUTPUT_NONE || !fm->audio_enabled;
    if (fm->audio_enabled && fm->pcm_ring) {
        size_t bytes = fm->pcm_frame_used * sizeof(fm->pcm_frame[0]);
        sent = xRingbufferSend(fm->pcm_ring,
                               fm->pcm_frame,
                               bytes,
                               pdMS_TO_TICKS(FM_PCM_SEND_TIMEOUT_MS)) == pdTRUE;
        if (sent) {
            fm->pcm_queued_frames++;
        }
    }

    portENTER_CRITICAL(&fm->status_lock);
    fm->status.audio_frames++;
    if (!sent) {
        fm->status.dropped_audio_frames++;
        fm->status.dropped_pcm_ring_frames++;
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
    const float demodulated = fast_atan2f_approx(imag, real);
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

static esp_err_t init_audio_output(fm_receiver_t *fm)
{
    if (fm->cfg.audio_output == FM_RECEIVER_AUDIO_OUTPUT_NONE) {
        return ESP_OK;
    }
    ESP_RETURN_ON_FALSE(fm->cfg.audio_output == FM_RECEIVER_AUDIO_OUTPUT_KORVO_SPEAKER,
                        ESP_ERR_NOT_SUPPORTED,
                        TAG,
                        "unsupported audio output");

    korvo_audio_output_config_t audio_cfg = {
        .sample_rate_hz = fm->profile.audio_rate_hz,
        .volume_percent = fm->cfg.volume_percent,
        .i2c_port = fm->cfg.audio_i2c_port,
        .i2c_sda_gpio = fm->cfg.audio_i2c_sda_gpio,
        .i2c_scl_gpio = fm->cfg.audio_i2c_scl_gpio,
        .i2s_port = fm->cfg.audio_i2s_port,
        .i2s_bclk_gpio = fm->cfg.audio_i2s_bclk_gpio,
        .i2s_ws_gpio = fm->cfg.audio_i2s_ws_gpio,
        .i2s_dout_gpio = fm->cfg.audio_i2s_dout_gpio,
        .i2s_din_gpio = fm->cfg.audio_i2s_din_gpio,
        .i2s_mclk_gpio = fm->cfg.audio_i2s_mclk_gpio,
        .pa_gpio = fm->cfg.audio_pa_gpio,
    };
    return korvo_audio_output_create(&audio_cfg, &fm->audio_output);
}

static void deinit_audio_output(fm_receiver_t *fm)
{
    if (fm->audio_output) {
        korvo_audio_output_destroy(fm->audio_output);
        fm->audio_output = NULL;
    }
}

static void audio_task(void *arg)
{
    fm_receiver_t *fm = (fm_receiver_t *)arg;
    ESP_LOGI(TAG, "Starting FM audio task");

    while (!fm->audio_task_stop) {
        if (fm->audio_enabled && fm->audio_prebuffering) {
            if (fm->pcm_queued_frames < FM_AUDIO_PREBUFFER_FRAMES) {
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
            fm->audio_prebuffering = false;
        }

        size_t item_size = 0;
        uint8_t *item = xRingbufferReceive(fm->pcm_ring, &item_size, pdMS_TO_TICKS(100));
        if (!item) {
            continue;
        }
        if (fm->pcm_queued_frames > 0) {
            fm->pcm_queued_frames--;
        }

        if (fm->audio_enabled && fm->audio_output) {
            size_t sample_count = item_size / sizeof(int16_t);
            size_t written = 0;
            esp_err_t ret = korvo_audio_output_write_mono(fm->audio_output,
                                                          (const int16_t *)item,
                                                          sample_count,
                                                          40,
                                                          &written);
            if (ret != ESP_OK || written != sample_count) {
                portENTER_CRITICAL(&fm->status_lock);
                fm->status.dropped_audio_frames++;
                fm->status.dropped_audio_write_frames++;
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
    fm->status.volume_percent = fm->cfg.volume_percent;
    fm->status.audio_output_enabled = false;
    fm->waveform_decimation = (fm->profile.audio_rate_hz * FM_WAVEFORM_INTERVAL_MS) /
                              (1000U * FM_WAVEFORM_SAMPLES);
    if (fm->waveform_decimation < 1) {
        fm->waveform_decimation = 1;
    }

    build_low_pass_taps(fm);

    if (fm->cfg.audio_output != FM_RECEIVER_AUDIO_OUTPUT_NONE) {
        fm->pcm_ring = xRingbufferCreate(FM_PCM_RING_BYTES, RINGBUF_TYPE_BYTEBUF);
        if (!fm->pcm_ring) {
            fm_receiver_destroy(fm);
            return ESP_ERR_NO_MEM;
        }

        esp_err_t ret = init_audio_output(fm);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "FM audio output disabled: %s", esp_err_to_name(ret));
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
             "FM receiver ready: %s, IQ %lu Hz, audio %lu Hz, decimation %lu, PCM %u x %u, I2S %s",
             fm->profile.name,
             (unsigned long)fm->cfg.sample_rate_hz,
             (unsigned long)fm->profile.audio_rate_hz,
             (unsigned long)fm->decimation,
             (unsigned)FM_PCM_FRAME_SAMPLES,
             (unsigned)FM_PCM_RING_FRAME_COUNT,
             fm->status.audio_output_ready ? "ready" :
             (fm->cfg.audio_output != FM_RECEIVER_AUDIO_OUTPUT_NONE ? "failed" : "off"));

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
    deinit_audio_output(fm);
    if (fm->pcm_ring) {
        vRingbufferDelete(fm->pcm_ring);
    }
    heap_caps_free(fm);
}

static void drain_pcm_ring(fm_receiver_t *fm)
{
    if (!fm->pcm_ring) {
        return;
    }

    size_t item_size = 0;
    uint8_t *item = NULL;
    while ((item = xRingbufferReceive(fm->pcm_ring, &item_size, 0)) != NULL) {
        vRingbufferReturnItem(fm->pcm_ring, item);
    }
}

void fm_receiver_set_audio_enabled(fm_receiver_t *fm, bool enabled)
{
    if (!fm) {
        return;
    }

    bool ready = fm->cfg.audio_output != FM_RECEIVER_AUDIO_OUTPUT_NONE && fm->status.audio_output_ready;
    fm->audio_enabled = enabled && ready;
    if (!fm->audio_enabled) {
        drain_pcm_ring(fm);
        fm->pcm_queued_frames = 0;
    } else {
        fm->audio_prebuffering = true;
    }

    portENTER_CRITICAL(&fm->status_lock);
    fm->status.audio_output_enabled = fm->audio_enabled;
    if (!fm->audio_enabled) {
        fm->status.audio_level = 0;
    }
    portEXIT_CRITICAL(&fm->status_lock);
    ESP_LOGI(TAG, "FM audio output %s", fm->audio_enabled ? "enabled" : "disabled");
}

esp_err_t fm_receiver_set_volume(fm_receiver_t *fm, int volume_percent)
{
    ESP_RETURN_ON_FALSE(fm, ESP_ERR_INVALID_ARG, TAG, "bad volume receiver");
    if (volume_percent < 0) {
        volume_percent = 0;
    } else if (volume_percent > 100) {
        volume_percent = 100;
    }

    fm->cfg.volume_percent = volume_percent;
    if (fm->audio_output) {
        ESP_RETURN_ON_ERROR(korvo_audio_output_set_volume(fm->audio_output, volume_percent),
                            TAG,
                            "set output volume");
    }

    portENTER_CRITICAL(&fm->status_lock);
    fm->status.volume_percent = volume_percent;
    portEXIT_CRITICAL(&fm->status_lock);
    ESP_LOGI(TAG, "FM volume set to %d%%", volume_percent);
    return ESP_OK;
}

int fm_receiver_get_volume(fm_receiver_t *fm)
{
    if (!fm) {
        return 0;
    }
    return fm->cfg.volume_percent;
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
    drain_pcm_ring(fm);
    fm->pcm_queued_frames = 0;
    fm->audio_prebuffering = fm->audio_enabled;
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
