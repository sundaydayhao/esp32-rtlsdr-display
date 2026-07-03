#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dsp_waterfall dsp_waterfall_t;

typedef struct {
    size_t fft_size;
    size_t fft_line_decimation;
    int average_shift;
    int db_min;
    int db_max;
} dsp_waterfall_config_t;

typedef void (*dsp_waterfall_line_cb_t)(const uint8_t *bins, size_t bin_count, void *ctx);

esp_err_t dsp_waterfall_create(const dsp_waterfall_config_t *config,
                               dsp_waterfall_line_cb_t line_cb,
                               void *line_cb_ctx,
                               dsp_waterfall_t **out);

void dsp_waterfall_destroy(dsp_waterfall_t *dsp);
esp_err_t dsp_waterfall_push_iq(dsp_waterfall_t *dsp, const uint8_t *iq, size_t len);
void dsp_waterfall_generate_test_line(dsp_waterfall_t *dsp);

size_t dsp_waterfall_fft_size(const dsp_waterfall_t *dsp);

#ifdef __cplusplus
}
#endif
