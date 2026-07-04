#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "usb/usb_host.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct rtlsdr rtlsdr_t;

typedef struct {
    uint32_t center_freq_hz;
    uint32_t sample_rate_hz;
    int tuner_gain_tenths_db;
    bool baseband_init;
} rtlsdr_config_t;

typedef void (*rtlsdr_samples_cb_t)(const uint8_t *iq, size_t len, void *ctx);
typedef void (*rtlsdr_status_cb_t)(const char *status, void *ctx);

esp_err_t rtlsdr_start(rtlsdr_t **out_dev,
                       const rtlsdr_config_t *config,
                       rtlsdr_samples_cb_t samples_cb,
                       rtlsdr_status_cb_t status_cb,
                       void *cb_ctx);

void rtlsdr_stop(rtlsdr_t *dev);

esp_err_t rtlsdr_set_center_freq_hz(rtlsdr_t *dev, uint32_t center_freq_hz);
uint32_t rtlsdr_get_center_freq_hz(const rtlsdr_t *dev);
uint32_t rtlsdr_get_sample_rate_hz(const rtlsdr_t *dev);

#ifdef __cplusplus
}
#endif
