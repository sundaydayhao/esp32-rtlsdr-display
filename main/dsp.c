#include "dsp.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_heap_caps.h"

struct dsp_waterfall {
    dsp_waterfall_config_t cfg;
    dsp_waterfall_line_cb_t line_cb;
    void *line_cb_ctx;
    float *window;
    float *real;
    float *imag;
    float *scratch_real;
    float *scratch_imag;
    float *avg_db;
    size_t samples_used;
    size_t fft_windows_seen;
    uint32_t test_phase;
};

static const char *TAG = "dsp";

static bool is_power_of_two(size_t value)
{
    return value && ((value & (value - 1)) == 0);
}

static void fft_in_place(float *real, float *imag, size_t n)
{
    size_t j = 0;
    for (size_t i = 1; i < n; i++) {
        size_t bit = n >> 1;
        while (j & bit) {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if (i < j) {
            float tr = real[i];
            float ti = imag[i];
            real[i] = real[j];
            imag[i] = imag[j];
            real[j] = tr;
            imag[j] = ti;
        }
    }

    for (size_t len = 2; len <= n; len <<= 1) {
        float angle = -2.0f * (float)M_PI / (float)len;
        float wlen_r = cosf(angle);
        float wlen_i = sinf(angle);
        for (size_t i = 0; i < n; i += len) {
            float wr = 1.0f;
            float wi = 0.0f;
            for (size_t k = 0; k < len / 2; k++) {
                size_t even = i + k;
                size_t odd = even + len / 2;
                float ur = real[even];
                float ui = imag[even];
                float vr = real[odd] * wr - imag[odd] * wi;
                float vi = real[odd] * wi + imag[odd] * wr;
                real[even] = ur + vr;
                imag[even] = ui + vi;
                real[odd] = ur - vr;
                imag[odd] = ui - vi;
                float next_wr = wr * wlen_r - wi * wlen_i;
                wi = wr * wlen_i + wi * wlen_r;
                wr = next_wr;
            }
        }
    }
}

static uint8_t db_to_u8(const dsp_waterfall_t *dsp, float db)
{
    const float min_db = (float)dsp->cfg.db_min;
    const float max_db = (float)dsp->cfg.db_max;
    float norm = (db - min_db) / (max_db - min_db);
    if (norm < 0.0f) {
        norm = 0.0f;
    } else if (norm > 1.0f) {
        norm = 1.0f;
    }
    return (uint8_t)(norm * 255.0f + 0.5f);
}

static void emit_fft_line(dsp_waterfall_t *dsp)
{
    const size_t n = dsp->cfg.fft_size;
    memcpy(dsp->scratch_real, dsp->real, n * sizeof(float));
    memcpy(dsp->scratch_imag, dsp->imag, n * sizeof(float));
    fft_in_place(dsp->scratch_real, dsp->scratch_imag, n);

    uint8_t *bins = heap_caps_malloc(n, MALLOC_CAP_8BIT);
    if (!bins) {
        return;
    }

    for (size_t out = 0; out < n; out++) {
        size_t idx = (out + n / 2) & (n - 1);
        float re = dsp->scratch_real[idx];
        float im = dsp->scratch_imag[idx];
        float power = (re * re + im * im) / ((float)n * (float)n);
        float db = 10.0f * log10f(power + 1.0e-12f);
        if (dsp->cfg.average_shift > 0) {
            float prev = dsp->avg_db[out];
            db = prev + (db - prev) / (float)(1 << dsp->cfg.average_shift);
            dsp->avg_db[out] = db;
        }
        bins[out] = db_to_u8(dsp, db);
    }

    dsp->line_cb(bins, n, dsp->line_cb_ctx);
    heap_caps_free(bins);
}

esp_err_t dsp_waterfall_create(const dsp_waterfall_config_t *config,
                               dsp_waterfall_line_cb_t line_cb,
                               void *line_cb_ctx,
                               dsp_waterfall_t **out)
{
    ESP_RETURN_ON_FALSE(config && line_cb && out, ESP_ERR_INVALID_ARG, TAG, "bad args");
    ESP_RETURN_ON_FALSE(is_power_of_two(config->fft_size), ESP_ERR_INVALID_ARG, TAG, "FFT size must be power of two");
    ESP_RETURN_ON_FALSE(config->db_max > config->db_min, ESP_ERR_INVALID_ARG, TAG, "invalid dB range");

    dsp_waterfall_t *dsp = calloc(1, sizeof(*dsp));
    ESP_RETURN_ON_FALSE(dsp, ESP_ERR_NO_MEM, TAG, "no memory");
    dsp->cfg = *config;
    dsp->line_cb = line_cb;
    dsp->line_cb_ctx = line_cb_ctx;

    size_t n = dsp->cfg.fft_size;
    dsp->window = heap_caps_malloc(n * sizeof(float), MALLOC_CAP_8BIT);
    dsp->real = heap_caps_calloc(n, sizeof(float), MALLOC_CAP_8BIT);
    dsp->imag = heap_caps_calloc(n, sizeof(float), MALLOC_CAP_8BIT);
    dsp->scratch_real = heap_caps_malloc(n * sizeof(float), MALLOC_CAP_8BIT);
    dsp->scratch_imag = heap_caps_malloc(n * sizeof(float), MALLOC_CAP_8BIT);
    dsp->avg_db = heap_caps_malloc(n * sizeof(float), MALLOC_CAP_8BIT);
    if (!dsp->window || !dsp->real || !dsp->imag || !dsp->scratch_real || !dsp->scratch_imag || !dsp->avg_db) {
        dsp_waterfall_destroy(dsp);
        return ESP_ERR_NO_MEM;
    }

    for (size_t i = 0; i < n; i++) {
        dsp->window[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * (float)i / (float)(n - 1));
        dsp->avg_db[i] = (float)dsp->cfg.db_min;
    }

    *out = dsp;
    return ESP_OK;
}

void dsp_waterfall_destroy(dsp_waterfall_t *dsp)
{
    if (!dsp) {
        return;
    }
    heap_caps_free(dsp->window);
    heap_caps_free(dsp->real);
    heap_caps_free(dsp->imag);
    heap_caps_free(dsp->scratch_real);
    heap_caps_free(dsp->scratch_imag);
    heap_caps_free(dsp->avg_db);
    free(dsp);
}

esp_err_t dsp_waterfall_push_iq(dsp_waterfall_t *dsp, const uint8_t *iq, size_t len)
{
    ESP_RETURN_ON_FALSE(dsp && iq && ((len & 1) == 0), ESP_ERR_INVALID_ARG, TAG, "bad IQ buffer");
    const size_t n = dsp->cfg.fft_size;
    for (size_t i = 0; i + 1 < len; i += 2) {
        size_t pos = dsp->samples_used++;
        dsp->real[pos] = (((float)iq[i] - 127.5f) / 128.0f) * dsp->window[pos];
        dsp->imag[pos] = (((float)iq[i + 1] - 127.5f) / 128.0f) * dsp->window[pos];
        if (dsp->samples_used == n) {
            dsp->fft_windows_seen++;
            size_t decimation = dsp->cfg.fft_line_decimation ? dsp->cfg.fft_line_decimation : 1;
            if ((dsp->fft_windows_seen % decimation) == 0) {
                emit_fft_line(dsp);
                vTaskDelay(1);
            } else if ((dsp->fft_windows_seen & 0x0f) == 0) {
                taskYIELD();
            }
            dsp->samples_used = 0;
        }
    }
    return ESP_OK;
}

void dsp_waterfall_generate_test_line(dsp_waterfall_t *dsp)
{
    if (!dsp) {
        return;
    }

    const size_t n = dsp->cfg.fft_size;
    uint8_t *bins = heap_caps_malloc(n, MALLOC_CAP_8BIT);
    if (!bins) {
        return;
    }

    uint32_t phase = dsp->test_phase++;
    int tone_a = (int)((phase * 3U) % n);
    int tone_b = (int)((n - ((phase * 5U) % n)) % n);
    for (size_t i = 0; i < n; i++) {
        int dist_a = abs((int)i - tone_a);
        int dist_b = abs((int)i - tone_b);
        if (dist_a > (int)n / 2) {
            dist_a = (int)n - dist_a;
        }
        if (dist_b > (int)n / 2) {
            dist_b = (int)n - dist_b;
        }
        int value = 16 + (int)((i * 37U + phase * 11U) & 0x1f);
        value += 220 / (dist_a + 1);
        value += 160 / (dist_b + 2);
        if (value > 255) {
            value = 255;
        }
        bins[i] = (uint8_t)value;
    }
    dsp->line_cb(bins, n, dsp->line_cb_ctx);
    heap_caps_free(bins);
}

size_t dsp_waterfall_fft_size(const dsp_waterfall_t *dsp)
{
    return dsp ? dsp->cfg.fft_size : 0;
}
