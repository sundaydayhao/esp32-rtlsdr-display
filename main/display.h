#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct waterfall_display waterfall_display_t;

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
} waterfall_display_config_t;

esp_err_t waterfall_display_create(const waterfall_display_config_t *config, waterfall_display_t **out);
void waterfall_display_destroy(waterfall_display_t *display);
void waterfall_display_set_status(waterfall_display_t *display, const char *status);
void waterfall_display_set_radio_info(waterfall_display_t *display, uint32_t center_hz, uint32_t sample_rate_hz);
void waterfall_display_push_line(waterfall_display_t *display, const uint8_t *bins, size_t bin_count);
void waterfall_display_render_idle(waterfall_display_t *display);

#ifdef __cplusplus
}
#endif
