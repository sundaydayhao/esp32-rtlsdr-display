#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "display.h"
#include "dsp.h"
#include "fm_receiver.h"
#include "rtlsdr.h"
#include "usb_host_service.h"

static const char *TAG = "app";

#ifndef CONFIG_FM_RECEIVER_NFM_MODE
#define CONFIG_FM_RECEIVER_NFM_MODE 0
#endif
#ifndef CONFIG_FM_RECEIVER_I2S_OUTPUT
#define CONFIG_FM_RECEIVER_I2S_OUTPUT 0
#endif
#ifndef CONFIG_FM_RECEIVER_AUDIO_RATE_HZ
#define CONFIG_FM_RECEIVER_AUDIO_RATE_HZ 48000
#endif
#ifndef CONFIG_FM_RECEIVER_VOLUME_PERCENT
#define CONFIG_FM_RECEIVER_VOLUME_PERCENT 80
#endif
#ifndef CONFIG_FM_RECEIVER_I2S_BCLK_GPIO
#define CONFIG_FM_RECEIVER_I2S_BCLK_GPIO -1
#endif
#ifndef CONFIG_FM_RECEIVER_I2S_WS_GPIO
#define CONFIG_FM_RECEIVER_I2S_WS_GPIO -1
#endif
#ifndef CONFIG_FM_RECEIVER_I2S_DOUT_GPIO
#define CONFIG_FM_RECEIVER_I2S_DOUT_GPIO -1
#endif

#define STARTUP_TEST_WATERFALL_DELAY_MS  6000

typedef struct {
    waterfall_display_t *display;
    dsp_waterfall_t *dsp;
    fm_receiver_t *fm;
    rtlsdr_t *radio;
    RingbufHandle_t iq_ring;
    volatile uint32_t requested_center_hz;
    volatile bool have_samples;
    volatile bool reset_fm_demod;
} app_ctx_t;

static void samples_cb(const uint8_t *iq, size_t len, void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    if (!app->iq_ring) {
        return;
    }
    if (xRingbufferSend(app->iq_ring, iq, len, 0) == pdTRUE) {
        app->have_samples = true;
    }
}

static void status_cb(const char *status, void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    waterfall_display_set_status(app->display, status);
}

static void line_cb(const uint8_t *bins, size_t bin_count, void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    waterfall_display_push_line(app->display, bins, bin_count);
}

static void fm_waveform_cb(const int16_t *samples, size_t sample_count, void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    waterfall_display_push_fm_waveform(app->display, samples, sample_count);
}

static void center_freq_cb(uint32_t center_hz, void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    app->requested_center_hz = center_hz;
    if (!app->radio) {
        ESP_LOGI(TAG, "Center frequency queued before RTL-SDR is ready: %lu Hz", (unsigned long)center_hz);
        return;
    }

    esp_err_t ret = rtlsdr_set_center_freq_hz(app->radio, center_hz);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "queue center frequency failed: %s", esp_err_to_name(ret));
    } else {
        app->reset_fm_demod = true;
    }
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
            dsp_waterfall_push_iq(app->dsp, item, item_size);
            if (app->reset_fm_demod) {
                fm_receiver_reset(app->fm);
                app->reset_fm_demod = false;
            }
            fm_receiver_push_iq(app->fm, item, item_size);
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

    waterfall_display_config_t display_cfg = {
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

    ESP_ERROR_CHECK(waterfall_display_create(&display_cfg, &app.display));
    waterfall_display_set_radio_info(app.display, CONFIG_RTLSDR_CENTER_FREQ_HZ, CONFIG_RTLSDR_SAMPLE_RATE);
    waterfall_display_set_center_freq_cb(app.display, center_freq_cb, &app);

    uint32_t fft_lines_per_second = CONFIG_RTLSDR_SAMPLE_RATE / CONFIG_WATERFALL_FFT_SIZE;
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

    fm_receiver_config_t fm_cfg = {
        .mode = CONFIG_FM_RECEIVER_NFM_MODE ? FM_RECEIVER_MODE_NFM : FM_RECEIVER_MODE_WFM,
        .sample_rate_hz = CONFIG_RTLSDR_SAMPLE_RATE,
        .audio_rate_hz = CONFIG_FM_RECEIVER_AUDIO_RATE_HZ,
        .volume_percent = CONFIG_FM_RECEIVER_VOLUME_PERCENT,
        .i2s_enabled = CONFIG_FM_RECEIVER_I2S_OUTPUT,
        .i2s_bclk_gpio = CONFIG_FM_RECEIVER_I2S_BCLK_GPIO,
        .i2s_ws_gpio = CONFIG_FM_RECEIVER_I2S_WS_GPIO,
        .i2s_dout_gpio = CONFIG_FM_RECEIVER_I2S_DOUT_GPIO,
        .waveform_cb = fm_waveform_cb,
        .waveform_cb_ctx = &app,
    };
    ESP_ERROR_CHECK(fm_receiver_create(&fm_cfg, &app.fm));
    waterfall_display_set_fm_info(app.display,
                                  fm_receiver_mode_name(fm_cfg.mode),
                                  fm_cfg.audio_rate_hz,
                                  fm_cfg.i2s_enabled,
                                  false,
                                  0);

    app.iq_ring = xRingbufferCreate(CONFIG_RTLSDR_BULK_TRANSFER_SIZE * CONFIG_RTLSDR_NUM_TRANSFERS * 2,
                                    RINGBUF_TYPE_BYTEBUF);
    ESP_ERROR_CHECK(app.iq_ring ? ESP_OK : ESP_ERR_NO_MEM);

    esp_err_t ret = usb_host_service_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB host failed: %s", esp_err_to_name(ret));
        waterfall_display_set_status(app.display, "USB HOST FAILED - TEST MODE");
        return;
    }

    rtlsdr_config_t radio_cfg = {
        .center_freq_hz = CONFIG_RTLSDR_CENTER_FREQ_HZ,
        .sample_rate_hz = CONFIG_RTLSDR_SAMPLE_RATE,
        .tuner_gain_tenths_db = CONFIG_RTLSDR_TUNER_GAIN_TENTHS_DB,
        .baseband_init = CONFIG_RTLSDR_BASEBAND_INIT,
    };

    ret = rtlsdr_start(&app.radio, &radio_cfg, samples_cb, status_cb, &app);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RTL-SDR task failed: %s", esp_err_to_name(ret));
        waterfall_display_set_status(app.display, "RTLSDR TASK FAILED - TEST MODE");
    } else if (app.requested_center_hz != radio_cfg.center_freq_hz) {
        (void)rtlsdr_set_center_freq_hz(app.radio, app.requested_center_hz);
    }

    BaseType_t ok = xTaskCreatePinnedToCore(dsp_task, "fft_waterfall", 8192, &app, 4, NULL, 1);
    ESP_ERROR_CHECK(ok == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM);

    while (true) {
        fm_receiver_status_t fm_status = {0};
        fm_receiver_get_status(app.fm, &fm_status);
        waterfall_display_set_fm_info(app.display,
                                      fm_receiver_mode_name(fm_status.mode),
                                      fm_status.audio_rate_hz,
                                      fm_status.audio_output_enabled,
                                      fm_status.audio_output_ready,
                                      fm_status.audio_level);
        waterfall_display_render_idle(app.display);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
