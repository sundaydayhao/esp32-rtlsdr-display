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
#include "rtlsdr.h"
#include "usb_host_service.h"

static const char *TAG = "app";

typedef struct {
    waterfall_display_t *display;
    dsp_waterfall_t *dsp;
    RingbufHandle_t iq_ring;
    volatile bool have_samples;
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

static void dsp_task(void *arg)
{
    app_ctx_t *app = (app_ctx_t *)arg;
    TickType_t last_test = xTaskGetTickCount();

    while (true) {
        size_t item_size = 0;
        uint8_t *item = (uint8_t *)xRingbufferReceive(app->iq_ring, &item_size, pdMS_TO_TICKS(25));
        if (item) {
            dsp_waterfall_push_iq(app->dsp, item, item_size);
            vRingbufferReturnItem(app->iq_ring, item);
            continue;
        }

        if (!app->have_samples && xTaskGetTickCount() - last_test > pdMS_TO_TICKS(60)) {
            dsp_waterfall_generate_test_line(app->dsp);
            last_test = xTaskGetTickCount();
        }
    }
}

void app_main(void)
{
    app_ctx_t app = {0};

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

    app.iq_ring = xRingbufferCreate(CONFIG_RTLSDR_BULK_TRANSFER_SIZE * CONFIG_RTLSDR_NUM_TRANSFERS * 2,
                                    RINGBUF_TYPE_BYTEBUF);
    ESP_ERROR_CHECK(app.iq_ring ? ESP_OK : ESP_ERR_NO_MEM);

    BaseType_t ok = xTaskCreatePinnedToCore(dsp_task, "fft_waterfall", 8192, &app, 4, NULL, 1);
    ESP_ERROR_CHECK(ok == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM);

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

    rtlsdr_t *radio = NULL;
    ret = rtlsdr_start(&radio, &radio_cfg, samples_cb, status_cb, &app);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RTL-SDR task failed: %s", esp_err_to_name(ret));
        waterfall_display_set_status(app.display, "RTLSDR TASK FAILED - TEST MODE");
    }

    while (true) {
        waterfall_display_render_idle(app.display);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
