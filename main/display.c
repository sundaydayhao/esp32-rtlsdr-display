#include "display.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/lock.h>
#include <sys/param.h>
#include <unistd.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hal/lcd_types.h"
#include "lvgl.h"
#include "sdkconfig.h"

#define LVGL_TICK_PERIOD_MS         2
#define LVGL_TASK_MAX_DELAY_MS      500
#define LVGL_TASK_MIN_DELAY_MS      (1000 / CONFIG_FREERTOS_HZ)
#define WATERFALL_IMAGE_ALIGN       4
#define TOUCH_TIMEOUT_MS            20
#define WATERFALL_SWIPE_MIN_PX      40
#define WATERFALL_SWIPE_MIN_HZ      1000
#define CENTER_FREQ_MIN_HZ          24000000U
#define CENTER_FREQ_MAX_HZ          1766000000U

#define GT911_PRODUCT_ID_REG        0x8140
#define GT911_STATUS_REG            0x814e
#define GT911_POINT1_REG            0x8150
#define GT911_STATUS_READY          0x80
#define GT911_STATUS_TOUCH_MASK     0x0f

#define FT5X06_VENDOR_ID_REG        0xa8
#define FT5X06_TD_STATUS_REG        0x02
#define FT5X06_P1_XH_REG            0x03

typedef enum {
    PAGE_HOME = 0,
    PAGE_WATERFALL,
    PAGE_FM,
    PAGE_ADSB,
    PAGE_SCANNER,
    PAGE_SENSORS,
    PAGE_RECORD,
} app_page_t;

typedef enum {
    TOUCH_CHIP_NONE = 0,
    TOUCH_CHIP_GT911,
    TOUCH_CHIP_FT5X06,
} touch_chip_t;

typedef struct {
    app_page_t page;
    const char *title;
    const char *subtitle;
    const char *accent_hex;
    lv_color_t accent;
} feature_def_t;

struct waterfall_display {
    waterfall_display_config_t cfg;
    waterfall_center_freq_cb_t center_freq_cb;
    void *center_freq_cb_ctx;
    esp_lcd_panel_handle_t panel;
    lv_display_t *lv_disp;
    void *lv_draw_buf;
    lv_obj_t *root;
    lv_obj_t *content;
    lv_obj_t *footer_label;
    lv_obj_t *header_title;
    lv_obj_t *header_subtitle;
    lv_obj_t *header_button;
    lv_obj_t *waterfall_img;
    lv_obj_t *side_panel;
    lv_obj_t *freq_row;
    lv_obj_t *status_value_label;
    lv_obj_t *freq_value_label;
    lv_obj_t *rate_value_label;
    lv_obj_t *marker_value_label;
    lv_obj_t *touch_value_label;
    lv_obj_t *fm_mode_value_label;
    lv_obj_t *fm_audio_rate_value_label;
    lv_obj_t *fm_audio_output_value_label;
    lv_obj_t *fm_audio_level_bar;
    lv_obj_t *fm_audio_level_label;
    lv_obj_t *fm_center_large_label;
    lv_obj_t *fm_status_large_label;
    lv_obj_t *fm_waveform_img;
    lv_obj_t *freq_dialog;
    lv_obj_t *freq_textarea;
    lv_obj_t *freq_digit_roller;
    lv_obj_t *placeholder_label;
    lv_image_dsc_t waterfall_dsc;
    lv_image_dsc_t fm_waveform_dsc;
    uint16_t *waterfall_pixels;
    uint16_t *fm_waveform_pixels;
    int waterfall_w;
    int waterfall_h;
    int fm_waveform_w;
    int fm_waveform_h;
    int spectrum_h;
    int ruler_h;
    char status[96];
    char touch_status[64];
    char fm_mode[8];
    char footer_text[256];
    uint32_t center_hz;
    uint32_t sample_rate_hz;
    uint32_t fm_audio_rate_hz;
    uint8_t fm_audio_level;
    uint32_t marker_hz;
    bool marker_valid;
    bool fm_audio_output_enabled;
    bool fm_audio_output_ready;
    bool have_samples;
    bool waterfall_dirty;
    bool fm_waveform_dirty;
    bool status_dirty;
    bool shutting_down;
    bool waterfall_swipe_tracking;
    bool waterfall_swipe_moved;
    bool waterfall_suppress_next_click;
    int waterfall_swipe_start_x;
    int waterfall_swipe_start_y;
    app_page_t active_page;
    _lock_t lvgl_api_lock;
    TaskHandle_t lvgl_task_handle;
    esp_timer_handle_t lvgl_tick_timer;
#if CONFIG_TOUCH_ENABLED
    i2c_master_bus_handle_t touch_bus;
    i2c_master_dev_handle_t touch_dev;
    lv_indev_t *touch_indev;
    touch_chip_t touch_chip;
    bool touch_last_pressed;
    uint32_t touch_status_log_count;
#endif
};

static const char *TAG = "display";
static waterfall_display_t *s_display;
static void close_freq_dialog(waterfall_display_t *display);

#ifndef CONFIG_FM_RECEIVER_NFM_MODE
#define CONFIG_FM_RECEIVER_NFM_MODE 0
#endif
#ifndef CONFIG_FM_RECEIVER_I2S_OUTPUT
#define CONFIG_FM_RECEIVER_I2S_OUTPUT 0
#endif
#ifndef CONFIG_FM_RECEIVER_AUDIO_RATE_HZ
#define CONFIG_FM_RECEIVER_AUDIO_RATE_HZ 48000
#endif

static feature_def_t s_features[] = {
    {PAGE_ADSB, "ADS-B", "1090 MHz aircraft view", "#4cc9f0", {0}},
    {PAGE_FM, "FM Receiver", "WFM/NFM spectrum", "#80ed99", {0}},
    {PAGE_WATERFALL, "Waterfall", "Live FFT and marker", "#f77f00", {0}},
    {PAGE_SENSORS, "Sensors", "433/315 MHz packets", "#2dd4bf", {0}},
    {PAGE_RECORD, "IQ Record", "Raw IQ capture", "#ff5c8a", {0}},
    {PAGE_SCANNER, "Scanner", "Wideband heatmap", "#a855f7", {0}},
};

static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint16_t)(r & 0xf8) << 8) | ((uint16_t)(g & 0xfc) << 3) | (uint16_t)(b >> 3);
}

static uint64_t gpio_bit_mask(int gpio_num)
{
    return (gpio_num >= 0) ? (1ULL << (unsigned)gpio_num) : 0;
}

static lv_color_t color_hex(uint32_t hex)
{
    return lv_color_hex(hex);
}

static uint16_t palette(uint8_t value)
{
    uint8_t k = value >> 1;
    uint8_t r5;
    uint8_t g6;
    uint8_t b5;

    if (k < 32) {
        r5 = k / 2;
        g6 = 0;
        b5 = k;
    } else if (k < 64) {
        r5 = k / 2;
        g6 = 0;
        b5 = 63 - k;
    } else if (k < 96) {
        r5 = 31;
        g6 = (uint8_t)((k - 64) * 2);
        b5 = 0;
    } else {
        r5 = 31;
        g6 = 63;
        b5 = k - 96;
    }

    return ((uint16_t)r5 << 11) | ((uint16_t)g6 << 5) | b5;
}

static uint8_t scaled_bin_at(const uint8_t *bins, size_t bin_count, int x, int width)
{
    size_t start = ((size_t)x * bin_count) / (size_t)width;
    size_t end = ((size_t)(x + 1) * bin_count) / (size_t)width;
    if (end <= start) {
        end = start + 1;
    }
    if (end > bin_count) {
        end = bin_count;
    }

    uint8_t peak = 0;
    for (size_t i = start; i < end; i++) {
        if (bins[i] > peak) {
            peak = bins[i];
        }
    }
    return peak;
}

static const feature_def_t *feature_for_page(app_page_t page)
{
    for (size_t i = 0; i < sizeof(s_features) / sizeof(s_features[0]); i++) {
        if (s_features[i].page == page) {
            return &s_features[i];
        }
    }
    return &s_features[2];
}

static void format_freq(char *buf, size_t len, int64_t hz, int decimals)
{
    double abs_hz = (hz < 0) ? -(double)hz : (double)hz;
    const char *sign = (hz < 0) ? "-" : "";
    if (abs_hz >= 1000000000.0) {
        snprintf(buf, len, "%s%.*f GHz", sign, decimals + 1, abs_hz / 1000000000.0);
    } else {
        snprintf(buf, len, "%s%.*f MHz", sign, decimals, abs_hz / 1000000.0);
    }
}

static void format_freq_input(char *buf, size_t len, uint32_t hz)
{
    snprintf(buf, len, "%lu.%03lu", (unsigned long)(hz / 1000000U), (unsigned long)((hz % 1000000U) / 1000U));
}

static bool parse_mhz_input(const char *text, uint32_t *out_hz)
{
    if (!text || !out_hz) {
        return false;
    }

    uint64_t whole_mhz = 0;
    uint64_t frac_hz = 0;
    uint32_t frac_scale = 1;
    bool saw_digit = false;
    bool saw_dot = false;

    for (const char *p = text; *p; p++) {
        char c = *p;
        if (c == ' ' || c == '\t') {
            continue;
        }
        if (c == '.') {
            if (saw_dot) {
                return false;
            }
            saw_dot = true;
            continue;
        }
        if (c < '0' || c > '9') {
            return false;
        }

        saw_digit = true;
        uint32_t digit = (uint32_t)(c - '0');
        if (!saw_dot) {
            whole_mhz = whole_mhz * 10U + digit;
            if (whole_mhz > UINT32_MAX / 1000000ULL) {
                return false;
            }
        } else if (frac_scale < 1000000U) {
            frac_hz = frac_hz * 10U + digit;
            frac_scale *= 10U;
        }
    }

    if (!saw_digit) {
        return false;
    }
    while (frac_scale < 1000000U) {
        frac_hz *= 10U;
        frac_scale *= 10U;
    }

    uint64_t hz = whole_mhz * 1000000ULL + frac_hz;
    if (hz < CENTER_FREQ_MIN_HZ) {
        hz = CENTER_FREQ_MIN_HZ;
    }
    if (hz > CENTER_FREQ_MAX_HZ) {
        hz = CENTER_FREQ_MAX_HZ;
    }

    *out_hz = (uint32_t)hz;
    return true;
}

static void set_center_freq_locked(waterfall_display_t *display, uint32_t center_hz)
{
    if (center_hz < CENTER_FREQ_MIN_HZ) {
        center_hz = CENTER_FREQ_MIN_HZ;
    } else if (center_hz > CENTER_FREQ_MAX_HZ) {
        center_hz = CENTER_FREQ_MAX_HZ;
    }
    if (display->center_hz == center_hz) {
        return;
    }

    display->center_hz = center_hz;
    display->marker_valid = false;
    display->status_dirty = true;

    if (display->center_freq_cb) {
        display->center_freq_cb(center_hz, display->center_freq_cb_ctx);
    }

    char freq[32];
    format_freq(freq, sizeof(freq), center_hz, 3);
    ESP_LOGI(TAG, "Center frequency display set to %s", freq);
}

static void step_center_freq_locked(waterfall_display_t *display, int direction, int steps)
{
    if (direction == 0 || steps <= 0) {
        return;
    }

    uint32_t step_hz = display->sample_rate_hz / 10U;
    if (step_hz < WATERFALL_SWIPE_MIN_HZ) {
        step_hz = WATERFALL_SWIPE_MIN_HZ;
    }

    int64_t delta = (int64_t)step_hz * steps * direction;
    int64_t next = (int64_t)display->center_hz + delta;
    if (next < CENTER_FREQ_MIN_HZ) {
        next = CENTER_FREQ_MIN_HZ;
    } else if (next > CENTER_FREQ_MAX_HZ) {
        next = CENTER_FREQ_MAX_HZ;
    }
    set_center_freq_locked(display, (uint32_t)next);
}

static void fill_rect_rgb565(waterfall_display_t *display, int x, int y, int w, int h, uint16_t color)
{
    if (!display->waterfall_pixels) {
        return;
    }
    int x2 = x + w;
    int y2 = y + h;
    if (x < 0) {
        x = 0;
    }
    if (y < 0) {
        y = 0;
    }
    if (x2 > display->waterfall_w) {
        x2 = display->waterfall_w;
    }
    if (y2 > display->waterfall_h) {
        y2 = display->waterfall_h;
    }
    if (x >= x2 || y >= y2) {
        return;
    }

    for (int row = y; row < y2; row++) {
        uint16_t *dst = display->waterfall_pixels + row * display->waterfall_w + x;
        for (int col = x; col < x2; col++) {
            *dst++ = color;
        }
    }
}

static void draw_hline(waterfall_display_t *display, int x, int y, int w, uint16_t color)
{
    fill_rect_rgb565(display, x, y, w, 1, color);
}

static void fm_fill_rect(waterfall_display_t *display, int x, int y, int w, int h, uint16_t color)
{
    if (!display->fm_waveform_pixels) {
        return;
    }
    int x2 = x + w;
    int y2 = y + h;
    if (x < 0) {
        x = 0;
    }
    if (y < 0) {
        y = 0;
    }
    if (x2 > display->fm_waveform_w) {
        x2 = display->fm_waveform_w;
    }
    if (y2 > display->fm_waveform_h) {
        y2 = display->fm_waveform_h;
    }
    if (x >= x2 || y >= y2) {
        return;
    }

    for (int row = y; row < y2; row++) {
        uint16_t *dst = display->fm_waveform_pixels + row * display->fm_waveform_w + x;
        for (int col = x; col < x2; col++) {
            *dst++ = color;
        }
    }
}

static void fm_draw_line(waterfall_display_t *display, int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        fm_fill_rect(display, x0, y0, 1, 1, color);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static void draw_fm_waveform_background(waterfall_display_t *display)
{
    if (!display->fm_waveform_pixels) {
        return;
    }
    fm_fill_rect(display, 0, 0, display->fm_waveform_w, display->fm_waveform_h, rgb565(8, 13, 11));

    uint16_t grid = rgb565(28, 48, 38);
    for (int i = 0; i <= 8; i++) {
        int x = (i == 8) ? display->fm_waveform_w - 1 : (display->fm_waveform_w * i) / 8;
        fm_fill_rect(display, x, 0, 1, display->fm_waveform_h, grid);
    }
    for (int i = 0; i <= 4; i++) {
        int y = (i == 4) ? display->fm_waveform_h - 1 : (display->fm_waveform_h * i) / 4;
        fm_fill_rect(display, 0, y, display->fm_waveform_w, 1, grid);
    }
    fm_fill_rect(display, 0, display->fm_waveform_h / 2, display->fm_waveform_w, 2, rgb565(42, 75, 58));
}

static void draw_fm_waveform(waterfall_display_t *display, const int16_t *samples, size_t sample_count)
{
    if (!display->fm_waveform_pixels || !samples || sample_count == 0) {
        return;
    }

    int peak = 1;
    for (size_t i = 0; i < sample_count; i++) {
        int v = samples[i];
        if (v < 0) {
            v = -v;
        }
        if (v > peak) {
            peak = v;
        }
    }
    if (peak < 1024) {
        peak = 1024;
    }

    const int w = display->fm_waveform_w;
    const int h = display->fm_waveform_h;
    const int mid = h / 2;
    int amplitude = (h / 2) - 8;
    if (amplitude < 8) {
        amplitude = h / 3;
    }

    const uint16_t glow = rgb565(42, 110, 70);
    const uint16_t trace = rgb565(174, 232, 197);
    int prev_x = 0;
    int prev_y = mid;

    for (int x = 0; x < w; x++) {
        size_t index = (sample_count == 1 || w <= 1)
            ? 0
            : ((size_t)x * (sample_count - 1U)) / (size_t)(w - 1);
        int y = mid - (int)(((int32_t)samples[index] * amplitude) / peak);
        if (y < 3) {
            y = 3;
        } else if (y > h - 4) {
            y = h - 4;
        }

        if (x == 0) {
            fm_fill_rect(display, x, y, 1, 2, trace);
        } else {
            fm_draw_line(display, prev_x, prev_y + 1, x, y + 1, glow);
            fm_draw_line(display, prev_x, prev_y, x, y, trace);
        }
        prev_x = x;
        prev_y = y;
    }
}

static void draw_vline(waterfall_display_t *display, int x, int y, int h, uint16_t color)
{
    fill_rect_rgb565(display, x, y, 1, h, color);
}

static void set_px(waterfall_display_t *display, int x, int y, uint16_t color)
{
    if (x >= 0 && x < display->waterfall_w && y >= 0 && y < display->waterfall_h) {
        display->waterfall_pixels[y * display->waterfall_w + x] = color;
    }
}

static void draw_trace_line(waterfall_display_t *display, int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        set_px(display, x0, y0, color);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static void draw_waterfall_background(waterfall_display_t *display)
{
    uint16_t spectrum_bg = rgb565(8, 14, 20);
    uint16_t waterfall_bg = rgb565(2, 6, 12);
    uint16_t grid = rgb565(26, 43, 53);
    uint16_t center = rgb565(60, 96, 86);
    int w = display->waterfall_w;
    int h = display->waterfall_h;
    int spectrum_h = display->spectrum_h;
    int ruler_h = display->ruler_h;
    int waterfall_y = spectrum_h + ruler_h;

    fill_rect_rgb565(display, 0, 0, w, spectrum_h, spectrum_bg);
    fill_rect_rgb565(display, 0, spectrum_h, w, ruler_h, rgb565(5, 9, 14));
    fill_rect_rgb565(display, 0, waterfall_y, w, h - waterfall_y, waterfall_bg);

    for (int i = 1; i < 8; i++) {
        int x = (w * i) / 8;
        draw_vline(display, x, 0, h, (i == 4) ? center : grid);
    }
    for (int i = 1; i < 4; i++) {
        int y = (spectrum_h * i) / 4;
        draw_hline(display, 0, y, w, grid);
    }
    draw_hline(display, 0, spectrum_h - 1, w, rgb565(38, 54, 65));
    draw_hline(display, 0, waterfall_y - 1, w, rgb565(30, 45, 54));
}

static void draw_spectrum(waterfall_display_t *display, const uint8_t *bins, size_t bin_count)
{
    int w = display->waterfall_w;
    int h = display->spectrum_h;
    int bottom = h - 3;
    uint16_t trace = rgb565(236, 244, 218);
    int prev_x = 0;
    int prev_y = -1;

    fill_rect_rgb565(display, 0, 0, w, h, rgb565(8, 14, 20));
    for (int i = 1; i < 8; i++) {
        int x = (w * i) / 8;
        draw_vline(display, x, 0, h, (i == 4) ? rgb565(60, 96, 86) : rgb565(26, 43, 53));
    }
    for (int i = 1; i < 4; i++) {
        int y = (h * i) / 4;
        draw_hline(display, 0, y, w, rgb565(26, 43, 53));
    }

    for (int x = 0; x < w; x++) {
        uint8_t value = scaled_bin_at(bins, bin_count, x, w);
        int bar = 1 + ((int)value * (h - 14)) / 255;
        int y_trace = bottom - bar;
        if (y_trace < 4) {
            y_trace = 4;
        }

        uint16_t color = palette(value);
        for (int y = y_trace + 1; y <= bottom; y++) {
            display->waterfall_pixels[y * w + x] = color;
        }

        if (prev_y >= 0) {
            draw_trace_line(display, prev_x, prev_y, x, y_trace, trace);
        } else {
            set_px(display, x, y_trace, trace);
        }
        prev_x = x;
        prev_y = y_trace;
    }

    draw_vline(display, w / 2, 0, h, rgb565(72, 110, 100));
    draw_hline(display, 0, h - 1, w, rgb565(38, 54, 65));
}

static void draw_ruler(waterfall_display_t *display)
{
    int w = display->waterfall_w;
    int top = display->spectrum_h;
    int h = display->ruler_h;

    fill_rect_rgb565(display, 0, top, w, h, rgb565(5, 9, 14));
    for (int i = 0; i <= 8; i++) {
        int x = (i == 8) ? (w - 1) : (w * i) / 8;
        int tick_h = (i == 0 || i == 4 || i == 8) ? h : h / 2;
        draw_vline(display, x, top + h - tick_h, tick_h, rgb565(54, 82, 92));
    }
    draw_hline(display, 0, top + h - 1, w, rgb565(30, 45, 54));
}

static void mark_waterfall_dirty(waterfall_display_t *display)
{
    display->waterfall_dirty = true;
}

static void refresh_metrics_locked(waterfall_display_t *display)
{
    if (!display->status_dirty) {
        return;
    }

    char freq[32];
    char rate[32];
    char marker[32];
    char fm_audio_rate[32];
    char fm_audio_output[32];
    char fm_audio_level[16];
    format_freq(freq, sizeof(freq), display->center_hz, 3);
    snprintf(rate, sizeof(rate), "%.3f MSPS", (double)display->sample_rate_hz / 1000000.0);
    if (display->marker_valid) {
        format_freq(marker, sizeof(marker), display->marker_hz, 3);
    } else {
        snprintf(marker, sizeof(marker), "Tap spectrum");
    }
    snprintf(fm_audio_rate, sizeof(fm_audio_rate), "%lu Hz", (unsigned long)display->fm_audio_rate_hz);
    if (!display->fm_audio_output_enabled) {
        snprintf(fm_audio_output, sizeof(fm_audio_output), "Off");
    } else {
        snprintf(fm_audio_output, sizeof(fm_audio_output), "%s", display->fm_audio_output_ready ? "I2S ready" : "I2S failed");
    }
    snprintf(fm_audio_level, sizeof(fm_audio_level), "%u%%", (unsigned)display->fm_audio_level);

    if (display->footer_label) {
        snprintf(display->footer_text,
                 sizeof(display->footer_text),
                 "%s | %s | FM %s %u%%",
                 freq,
                 rate,
                 display->fm_mode,
                 (unsigned)display->fm_audio_level);
        lv_label_set_text(display->footer_label, display->footer_text);
    }
    if (display->status_value_label) {
        lv_label_set_text(display->status_value_label, display->status);
    }
    if (display->freq_value_label) {
        lv_label_set_text(display->freq_value_label, freq);
    }
    if (display->fm_center_large_label) {
        lv_label_set_text(display->fm_center_large_label, freq);
    }
    if (display->rate_value_label) {
        lv_label_set_text(display->rate_value_label, rate);
    }
    if (display->marker_value_label) {
        lv_label_set_text(display->marker_value_label, marker);
    }
    if (display->touch_value_label) {
        lv_label_set_text(display->touch_value_label, display->touch_status);
    }
    if (display->fm_mode_value_label) {
        lv_label_set_text(display->fm_mode_value_label, display->fm_mode);
    }
    if (display->fm_audio_rate_value_label) {
        lv_label_set_text(display->fm_audio_rate_value_label, fm_audio_rate);
    }
    if (display->fm_audio_output_value_label) {
        lv_label_set_text(display->fm_audio_output_value_label, fm_audio_output);
    }
    if (display->fm_status_large_label) {
        char fm_status[96];
        snprintf(fm_status,
                 sizeof(fm_status),
                 "%s / audio %s / level %u%%",
                 display->fm_mode,
                 display->fm_audio_output_ready ? "ready" : (display->fm_audio_output_enabled ? "unavailable" : "off"),
                 (unsigned)display->fm_audio_level);
        lv_label_set_text(display->fm_status_large_label, fm_status);
    }
    if (display->fm_audio_level_bar) {
        lv_bar_set_value(display->fm_audio_level_bar, display->fm_audio_level, LV_ANIM_OFF);
    }
    if (display->fm_audio_level_label) {
        lv_label_set_text(display->fm_audio_level_label, fm_audio_level);
    }

    display->status_dirty = false;
}

static void update_lvgl_objects_locked(waterfall_display_t *display)
{
    if (display->shutting_down) {
        return;
    }
    refresh_metrics_locked(display);
    if (display->waterfall_dirty && display->waterfall_img) {
        lv_obj_invalidate(display->waterfall_img);
        display->waterfall_dirty = false;
    }
    if (display->fm_waveform_dirty && display->fm_waveform_img) {
        lv_obj_invalidate(display->fm_waveform_img);
        display->fm_waveform_dirty = false;
    }
}

static bool notify_lvgl_flush_ready(esp_lcd_panel_handle_t panel,
                                    const esp_lcd_rgb_panel_event_data_t *event_data,
                                    void *user_ctx)
{
    (void)panel;
    (void)event_data;
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    esp_lcd_panel_draw_bitmap(panel_handle,
                              area->x1,
                              area->y1,
                              area->x2 + 1,
                              area->y2 + 1,
                              px_map);
}

static void increase_lvgl_tick(void *arg)
{
    (void)arg;
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void lvgl_task(void *arg)
{
    waterfall_display_t *display = (waterfall_display_t *)arg;
    ESP_LOGI(TAG, "Starting LVGL task");
    while (true) {
        _lock_acquire(&display->lvgl_api_lock);
        if (display->shutting_down) {
            display->lvgl_task_handle = NULL;
            _lock_release(&display->lvgl_api_lock);
            break;
        }
        update_lvgl_objects_locked(display);
        uint32_t time_till_next_ms = lv_timer_handler();
        _lock_release(&display->lvgl_api_lock);
        time_till_next_ms = MAX(time_till_next_ms, LVGL_TASK_MIN_DELAY_MS);
        time_till_next_ms = MIN(time_till_next_ms, LVGL_TASK_MAX_DELAY_MS);
        usleep(1000 * time_till_next_ms);
    }
    ESP_LOGI(TAG, "LVGL task stopped");
    vTaskDelete(NULL);
}

static void style_plain(lv_obj_t *obj)
{
    lv_obj_remove_style_all(obj);
    lv_obj_set_style_pad_all(obj, 0, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_set_style_radius(obj, 0, 0);
}

static lv_obj_t *create_label(lv_obj_t *parent, const char *text, int font_size, lv_color_t color, bool bold)
{
    (void)font_size;
    (void)bold;
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, color, 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_letter_space(label, 0, 0);
    lv_label_set_long_mode(label, LV_LABEL_LONG_MODE_DOTS);
    return label;
}

static void style_button(lv_obj_t *button, lv_color_t bg, lv_color_t border, int radius)
{
    lv_obj_set_style_bg_color(button, bg, 0);
    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(button, border, 0);
    lv_obj_set_style_border_width(button, 1, 0);
    lv_obj_set_style_radius(button, radius, 0);
    lv_obj_set_style_pad_all(button, 0, 0);
}

static void clear_content(waterfall_display_t *display)
{
    close_freq_dialog(display);
    if (display->content) {
        lv_obj_clean(display->content);
    }
    display->header_button = NULL;
    display->waterfall_img = NULL;
    display->side_panel = NULL;
    display->freq_row = NULL;
    display->status_value_label = NULL;
    display->freq_value_label = NULL;
    display->rate_value_label = NULL;
    display->marker_value_label = NULL;
    display->touch_value_label = NULL;
    display->fm_mode_value_label = NULL;
    display->fm_audio_rate_value_label = NULL;
    display->fm_audio_output_value_label = NULL;
    display->fm_audio_level_bar = NULL;
    display->fm_audio_level_label = NULL;
    display->fm_center_large_label = NULL;
    display->fm_status_large_label = NULL;
    display->fm_waveform_img = NULL;
    display->freq_dialog = NULL;
    display->freq_textarea = NULL;
    display->freq_digit_roller = NULL;
    display->placeholder_label = NULL;
}

static lv_obj_t *metric_row(lv_obj_t *parent, const char *name, const char *value, lv_obj_t **row_out)
{
    lv_obj_t *box = lv_obj_create(parent);
    if (row_out) {
        *row_out = box;
    }
    style_button(box, color_hex(0x0b0f13), color_hex(0x27313a), 6);
    lv_obj_set_width(box, LV_PCT(100));
    lv_obj_set_height(box, 48);
    lv_obj_set_flex_flow(box, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_left(box, 12, 0);
    lv_obj_set_style_pad_right(box, 12, 0);
    lv_obj_set_style_pad_top(box, 7, 0);
    lv_obj_set_style_pad_bottom(box, 7, 0);
    lv_obj_set_style_pad_row(box, 4, 0);

    lv_obj_t *name_label = create_label(box, name, 12, color_hex(0x8293a0), false);
    lv_obj_set_width(name_label, LV_PCT(100));
    lv_obj_t *value_label = create_label(box, value, 15, color_hex(0xeef6fb), true);
    lv_obj_set_width(value_label, LV_PCT(100));
    return value_label;
}

static void switch_page_locked(waterfall_display_t *display, app_page_t page);
static void freq_row_event_cb(lv_event_t *event);

static void close_freq_dialog(waterfall_display_t *display)
{
    if (display->freq_dialog) {
        lv_obj_delete(display->freq_dialog);
        display->freq_dialog = NULL;
        display->freq_textarea = NULL;
        display->freq_digit_roller = NULL;
    }
}

static lv_obj_t *dialog_button(lv_obj_t *parent,
                               const char *text,
                               lv_color_t bg,
                               lv_color_t border,
                               int x,
                               int y,
                               int w,
                               lv_event_cb_t cb,
                               waterfall_display_t *display)
{
    lv_obj_t *button = lv_button_create(parent);
    style_button(button, bg, border, 6);
    lv_obj_set_size(button, w, 40);
    lv_obj_align(button, LV_ALIGN_TOP_LEFT, x, y);
    lv_obj_add_event_cb(button, cb, LV_EVENT_CLICKED, display);

    lv_obj_t *label = create_label(button, text, 16, color_hex(0xf5f8fb), true);
    lv_obj_center(label);
    return button;
}

static void freq_digit_event_cb(lv_event_t *event)
{
    waterfall_display_t *display = lv_event_get_user_data(event);
    if (!display || !display->freq_textarea || !display->freq_digit_roller) {
        return;
    }

    char digit[2] = {0};
    lv_roller_get_selected_str(display->freq_digit_roller, digit, sizeof(digit));
    lv_textarea_add_text(display->freq_textarea, digit);
}

static void freq_dot_event_cb(lv_event_t *event)
{
    waterfall_display_t *display = lv_event_get_user_data(event);
    if (!display || !display->freq_textarea) {
        return;
    }

    const char *text = lv_textarea_get_text(display->freq_textarea);
    if (text && strchr(text, '.') == NULL) {
        lv_textarea_add_text(display->freq_textarea, ".");
    }
}

static void freq_backspace_event_cb(lv_event_t *event)
{
    waterfall_display_t *display = lv_event_get_user_data(event);
    if (display && display->freq_textarea) {
        lv_textarea_delete_char(display->freq_textarea);
    }
}

static void freq_cancel_event_cb(lv_event_t *event)
{
    waterfall_display_t *display = lv_event_get_user_data(event);
    if (display) {
        close_freq_dialog(display);
    }
}

static void freq_ok_event_cb(lv_event_t *event)
{
    waterfall_display_t *display = lv_event_get_user_data(event);
    if (!display || !display->freq_textarea) {
        return;
    }

    uint32_t next_hz = 0;
    if (parse_mhz_input(lv_textarea_get_text(display->freq_textarea), &next_hz)) {
        set_center_freq_locked(display, next_hz);
        close_freq_dialog(display);
    }
}

static void open_freq_dialog_locked(waterfall_display_t *display)
{
    if (display->freq_dialog) {
        return;
    }

    lv_obj_t *overlay = lv_obj_create(display->root);
    display->freq_dialog = overlay;
    style_button(overlay, color_hex(0x07090b), color_hex(0x26313a), 0);
    lv_obj_set_size(overlay, display->cfg.h_res, display->cfg.v_res);
    lv_obj_set_pos(overlay, 0, 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_90, 0);

    int dialog_w = MIN(display->cfg.h_res - 48, 360);
    int dialog_h = MIN(display->cfg.v_res - 40, 260);
    lv_obj_t *panel = lv_obj_create(overlay);
    style_button(panel, color_hex(0x11171c), color_hex(0x384653), 8);
    lv_obj_set_size(panel, dialog_w, dialog_h);
    lv_obj_center(panel);

    lv_obj_t *title = create_label(panel, "Center Frequency", 16, color_hex(0xf5f8fb), true);
    lv_obj_set_width(title, dialog_w - 32);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 16, 12);

    lv_obj_t *unit = create_label(panel, "MHz", 14, color_hex(0x9fb0bd), false);
    lv_obj_align(unit, LV_ALIGN_TOP_RIGHT, -18, 50);

    lv_obj_t *ta = lv_textarea_create(panel);
    display->freq_textarea = ta;
    lv_obj_set_size(ta, dialog_w - 92, 44);
    lv_obj_align(ta, LV_ALIGN_TOP_LEFT, 16, 40);
    lv_textarea_set_one_line(ta, true);
    lv_textarea_set_accepted_chars(ta, "0123456789.");
    lv_textarea_set_max_length(ta, 12);
    lv_textarea_set_password_mode(ta, false);
    lv_textarea_set_placeholder_text(ta, "100.000");
    lv_obj_set_style_text_font(ta, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ta, color_hex(0xf4f8fb), 0);
    lv_obj_set_style_bg_color(ta, color_hex(0x080c10), 0);
    lv_obj_set_style_bg_opa(ta, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(ta, color_hex(0x2e3a44), 0);
    lv_obj_set_style_border_width(ta, 1, 0);
    lv_obj_set_style_radius(ta, 6, 0);
    char input[24];
    format_freq_input(input, sizeof(input), display->center_hz);
    lv_textarea_set_text(ta, input);
    lv_textarea_set_cursor_pos(ta, LV_TEXTAREA_CURSOR_LAST);
    lv_obj_add_state(ta, LV_STATE_FOCUSED);

    static const char *digits = "0\n1\n2\n3\n4\n5\n6\n7\n8\n9";
    lv_obj_t *roller = lv_roller_create(panel);
    display->freq_digit_roller = roller;
    lv_roller_set_options(roller, digits, LV_ROLLER_MODE_INFINITE);
    lv_roller_set_visible_row_count(roller, 3);
    lv_roller_set_selected(roller, 0, LV_ANIM_OFF);
    lv_obj_set_size(roller, 76, 104);
    lv_obj_align(roller, LV_ALIGN_TOP_LEFT, 16, 98);
    lv_obj_set_style_bg_color(roller, color_hex(0x080c10), 0);
    lv_obj_set_style_bg_opa(roller, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(roller, color_hex(0xf5f8fb), 0);
    lv_obj_set_style_border_color(roller, color_hex(0x2e3a44), 0);
    lv_obj_set_style_border_width(roller, 1, 0);
    lv_obj_set_style_radius(roller, 6, 0);
    lv_obj_set_style_bg_opa(roller, LV_OPA_40, LV_PART_SELECTED);

    int x0 = 108;
    int y0 = 98;
    dialog_button(panel, "Add", color_hex(0x173022), color_hex(0x2f6f4d), x0, y0, 74, freq_digit_event_cb, display);
    dialog_button(panel, ".", color_hex(0x19212a), color_hex(0x3a4b5a), x0 + 84, y0, 48, freq_dot_event_cb, display);
    dialog_button(panel, "<", color_hex(0x2b1b1b), color_hex(0x6b3a3a), x0 + 142, y0, 54, freq_backspace_event_cb, display);
    dialog_button(panel, "Cancel", color_hex(0x171d22), color_hex(0x3b4650), x0, y0 + 56, 88, freq_cancel_event_cb, display);
    dialog_button(panel, "OK", color_hex(0x173022), color_hex(0x2f6f4d), x0 + 98, y0 + 56, 98, freq_ok_event_cb, display);

    lv_obj_move_to_index(overlay, -1);
}

static void freq_row_event_cb(lv_event_t *event)
{
    waterfall_display_t *display = lv_event_get_user_data(event);
    if (!display) {
        return;
    }
    open_freq_dialog_locked(display);
}

static void tile_event_cb(lv_event_t *event)
{
    waterfall_display_t *display = lv_event_get_user_data(event);
    app_page_t page = (app_page_t)(uintptr_t)lv_obj_get_user_data(lv_event_get_target(event));
    ESP_LOGI(TAG, "Tile clicked: %s", feature_for_page(page)->title);
    switch_page_locked(display, page);
}

static void back_event_cb(lv_event_t *event)
{
    waterfall_display_t *display = lv_event_get_user_data(event);
    ESP_LOGI(TAG, "Back to home");
    switch_page_locked(display, PAGE_HOME);
}

static void waterfall_event_cb(lv_event_t *event)
{
    waterfall_display_t *display = lv_event_get_user_data(event);
    lv_indev_t *indev = lv_indev_active();
    if (!indev) {
        return;
    }

    lv_event_code_t code = lv_event_get_code(event);
    lv_point_t point = {0};
    lv_indev_get_point(indev, &point);

    lv_area_t coords;
    lv_obj_get_coords(lv_event_get_target(event), &coords);
    int w = coords.x2 - coords.x1 + 1;
    int h = coords.y2 - coords.y1 + 1;
    if (w <= 0 || display->sample_rate_hz == 0) {
        return;
    }

    int rel_x = point.x - coords.x1;
    if (rel_x < 0) {
        rel_x = 0;
    } else if (rel_x >= w) {
        rel_x = w - 1;
    }

    int rel_y = point.y - coords.y1;
    if (code == LV_EVENT_PRESSED) {
        display->waterfall_swipe_tracking = (rel_y >= 0 && rel_y < h / 2);
        display->waterfall_swipe_moved = false;
        display->waterfall_suppress_next_click = false;
        display->waterfall_swipe_start_x = point.x;
        display->waterfall_swipe_start_y = point.y;
        return;
    }

    if (code == LV_EVENT_PRESSING && display->waterfall_swipe_tracking) {
        int dx = point.x - display->waterfall_swipe_start_x;
        int dy = point.y - display->waterfall_swipe_start_y;
        if (abs(dx) >= WATERFALL_SWIPE_MIN_PX && abs(dx) > abs(dy)) {
            int steps = abs(dx) / WATERFALL_SWIPE_MIN_PX;
            int direction = (dx < 0) ? 1 : -1;
            step_center_freq_locked(display, direction, steps);
            display->waterfall_swipe_start_x += (dx > 0) ? (steps * WATERFALL_SWIPE_MIN_PX) : -(steps * WATERFALL_SWIPE_MIN_PX);
            display->waterfall_swipe_start_y = point.y;
            display->waterfall_swipe_moved = true;
            display->waterfall_suppress_next_click = true;
        }
        return;
    }

    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        display->waterfall_swipe_tracking = false;
        return;
    }

    if (code != LV_EVENT_CLICKED) {
        return;
    }
    if (display->waterfall_suppress_next_click || display->waterfall_swipe_moved) {
        display->waterfall_suppress_next_click = false;
        display->waterfall_swipe_moved = false;
        return;
    }

    int64_t left = (int64_t)display->center_hz - (int64_t)display->sample_rate_hz / 2;
    int64_t marker_hz = left + ((int64_t)display->sample_rate_hz * rel_x) / w;
    if (marker_hz < 0) {
        marker_hz = 0;
    } else if (marker_hz > UINT32_MAX) {
        marker_hz = UINT32_MAX;
    }
    uint32_t marker = (uint32_t)marker_hz;
    ESP_LOGI(TAG, "Waterfall marker touch x=%d -> %lu Hz", rel_x, (unsigned long)marker);
    display->marker_hz = marker;
    display->marker_valid = true;
    display->status_dirty = true;
}

static void create_home_locked(waterfall_display_t *display)
{
    clear_content(display);
    display->active_page = PAGE_HOME;
    lv_label_set_text(display->header_title, "RTL-SDR Panel");
    lv_label_set_text(display->header_subtitle, "Touch a function tile to open a receiver page");

    lv_obj_t *grid = lv_obj_create(display->content);
    style_plain(grid);
    lv_obj_set_size(grid, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(grid, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_all(grid, 8, 0);
    lv_obj_set_style_pad_row(grid, 14, 0);
    lv_obj_set_style_pad_column(grid, 14, 0);

    int tile_w = (display->cfg.h_res >= 760) ? 230 : 190;
    int tile_h = (display->cfg.v_res >= 430) ? 145 : 120;

    for (size_t i = 0; i < sizeof(s_features) / sizeof(s_features[0]); i++) {
        const feature_def_t *feature = &s_features[i];
        lv_obj_t *tile = lv_obj_create(grid);
        style_button(tile, color_hex(0x171c21), feature->accent, 8);
        lv_obj_set_size(tile, tile_w, tile_h);
        lv_obj_add_flag(tile, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_user_data(tile, (void *)(uintptr_t)feature->page);
        lv_obj_add_event_cb(tile, tile_event_cb, LV_EVENT_CLICKED, display);

        lv_obj_t *bar = lv_obj_create(tile);
        style_button(bar, feature->accent, feature->accent, 4);
        lv_obj_set_size(bar, 38, 6);
        lv_obj_align(bar, LV_ALIGN_TOP_LEFT, 16, 16);

        lv_obj_t *title = create_label(tile, feature->title, 18, color_hex(0xf5f7fa), true);
        lv_obj_set_size(title, tile_w - 32, 24);
        lv_obj_align(title, LV_ALIGN_TOP_LEFT, 16, 38);

        lv_obj_t *subtitle = create_label(tile, feature->subtitle, 13, color_hex(0x96a6b4), false);
        lv_obj_set_size(subtitle, tile_w - 32, 42);
        lv_label_set_long_mode(subtitle, LV_LABEL_LONG_MODE_WRAP);
        lv_obj_align(subtitle, LV_ALIGN_TOP_LEFT, 16, 70);

        lv_obj_t *arrow = create_label(tile, ">", 16, color_hex(0xd9e2ea), true);
        lv_obj_align(arrow, LV_ALIGN_BOTTOM_RIGHT, -18, -12);
    }

    display->status_dirty = true;
}

static void create_placeholder_visual(lv_obj_t *parent, const feature_def_t *feature, const char *body, lv_color_t accent)
{
    lv_obj_t *panel = lv_obj_create(parent);
    style_button(panel, color_hex(0x0f1418), color_hex(0x2b3640), 8);
    lv_obj_set_size(panel, LV_PCT(100), LV_PCT(100));

    lv_obj_t *accent_bar = lv_obj_create(panel);
    style_button(accent_bar, accent, accent, 4);
    lv_obj_set_size(accent_bar, 96, 8);
    lv_obj_align(accent_bar, LV_ALIGN_CENTER, 0, -70);

    lv_obj_t *title = create_label(panel, feature->title, 22, color_hex(0xf6f8fb), true);
    lv_obj_set_width(title, LV_PCT(80));
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -34);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *message = create_label(panel, body, 14, color_hex(0x9fb0bd), false);
    lv_obj_set_width(message, LV_PCT(78));
    lv_label_set_long_mode(message, LV_LABEL_LONG_MODE_WRAP);
    lv_obj_set_style_text_align(message, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(message, LV_ALIGN_CENTER, 0, 18);
}

static void create_fm_visual_locked(waterfall_display_t *display, lv_obj_t *parent, const feature_def_t *feature)
{
    (void)feature;
    lv_obj_t *waveform = lv_image_create(parent);
    display->fm_waveform_img = waveform;
    lv_image_set_src(waveform, &display->fm_waveform_dsc);
    lv_obj_set_size(waveform, display->fm_waveform_w, display->fm_waveform_h);
    lv_obj_center(waveform);
    lv_obj_remove_flag(waveform, LV_OBJ_FLAG_SCROLLABLE);
    draw_fm_waveform_background(display);
    display->fm_waveform_dirty = true;
}

static void create_fm_side_panel_locked(waterfall_display_t *display, const feature_def_t *feature)
{
    lv_obj_t *side = lv_obj_create(display->content);
    display->side_panel = side;
    style_button(side, color_hex(0x101419), color_hex(0x2b3640), 8);
    lv_obj_set_width(side, (display->cfg.h_res >= 760) ? 250 : 210);
    lv_obj_set_height(side, LV_PCT(100));
    lv_obj_align(side, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_set_flex_flow(side, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(side, 14, 0);
    lv_obj_set_style_pad_row(side, 10, 0);

    lv_obj_t *panel_title = create_label(side, feature->title, 16, color_hex(0xf3f6f8), true);
    lv_obj_set_width(panel_title, LV_PCT(100));

    display->freq_value_label = metric_row(side, "Center", "", &display->freq_row);
    if (display->freq_row) {
        lv_obj_add_flag(display->freq_row, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(display->freq_row, freq_row_event_cb, LV_EVENT_CLICKED, display);
    }
    display->fm_mode_value_label = metric_row(side, "Mode", "", NULL);
    display->rate_value_label = metric_row(side, "IQ rate", "", NULL);
    display->fm_audio_rate_value_label = metric_row(side, "Audio rate", "", NULL);
    display->fm_audio_output_value_label = metric_row(side, "Audio output", "", NULL);

    lv_obj_t *level_box = lv_obj_create(side);
    style_button(level_box, color_hex(0x0b0f13), color_hex(0x27313a), 6);
    lv_obj_set_width(level_box, LV_PCT(100));
    lv_obj_set_height(level_box, 62);
    lv_obj_set_style_pad_left(level_box, 12, 0);
    lv_obj_set_style_pad_right(level_box, 12, 0);
    lv_obj_set_style_pad_top(level_box, 8, 0);
    lv_obj_set_style_pad_bottom(level_box, 8, 0);

    lv_obj_t *name_label = create_label(level_box, "Audio level", 12, color_hex(0x8293a0), false);
    lv_obj_set_width(name_label, LV_PCT(100));
    lv_obj_align(name_label, LV_ALIGN_TOP_LEFT, 0, 0);

    display->fm_audio_level_bar = lv_bar_create(level_box);
    lv_obj_set_size(display->fm_audio_level_bar, LV_PCT(100), 12);
    lv_obj_align(display->fm_audio_level_bar, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_bar_set_range(display->fm_audio_level_bar, 0, 100);
    lv_obj_set_style_bg_color(display->fm_audio_level_bar, color_hex(0x172018), 0);
    lv_obj_set_style_bg_color(display->fm_audio_level_bar, feature->accent, LV_PART_INDICATOR);

    display->fm_audio_level_label = create_label(level_box, "", 14, color_hex(0xeef6fb), true);
    lv_obj_set_width(display->fm_audio_level_label, 56);
    lv_obj_align(display->fm_audio_level_label, LV_ALIGN_TOP_RIGHT, 0, 0);
}

static void create_side_panel_locked(waterfall_display_t *display, const feature_def_t *feature, bool live_metrics)
{
    lv_obj_t *side = lv_obj_create(display->content);
    display->side_panel = side;
    style_button(side, color_hex(0x101419), color_hex(0x2b3640), 8);
    lv_obj_set_width(side, (display->cfg.h_res >= 760) ? 250 : 210);
    lv_obj_set_height(side, LV_PCT(100));
    lv_obj_align(side, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_set_flex_flow(side, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(side, 14, 0);
    lv_obj_set_style_pad_row(side, 10, 0);

    lv_obj_t *panel_title = create_label(side, feature->title, 16, color_hex(0xf3f6f8), true);
    lv_obj_set_width(panel_title, LV_PCT(100));

    if (live_metrics) {
        display->freq_value_label = metric_row(side, "Center", "", &display->freq_row);
        if (display->freq_row) {
            lv_obj_add_flag(display->freq_row, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(display->freq_row, freq_row_event_cb, LV_EVENT_CLICKED, display);
        }
        display->rate_value_label = metric_row(side, "Sample rate", "", NULL);
        display->marker_value_label = metric_row(side, "Marker", "", NULL);
    } else {
        metric_row(side, "Runtime", "UI ready", NULL);
        metric_row(side, "Driver", "Not wired", NULL);
        metric_row(side, "Next step", "Add DSP page", NULL);
    }
}

static void create_feature_locked(waterfall_display_t *display, app_page_t page)
{
    clear_content(display);
    display->active_page = page;

    const feature_def_t *feature = feature_for_page(page);
    lv_label_set_text(display->header_title, feature->title);
    lv_label_set_text(display->header_subtitle, feature->subtitle);

    lv_obj_t *back = lv_button_create(display->content);
    display->header_button = back;
    style_button(back, color_hex(0x101419), color_hex(0x313c46), 8);
    lv_obj_set_size(back, 52, 36);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 4, 4);
    lv_obj_add_event_cb(back, back_event_cb, LV_EVENT_CLICKED, display);
    lv_obj_t *back_label = create_label(back, "<", 18, color_hex(0xedf3f8), true);
    lv_obj_center(back_label);

    int top = 50;
    int gap = 14;
    int side_w = (display->cfg.h_res >= 760) ? 250 : 210;
    int visual_w = display->cfg.h_res - 32 - side_w - gap;
    int visual_h = display->cfg.v_res - 86 - top;
    if (visual_w < 260 || display->cfg.h_res < 620) {
        side_w = 0;
        visual_w = display->cfg.h_res - 32;
    }

    lv_obj_t *visual = lv_obj_create(display->content);
    style_plain(visual);
    lv_obj_set_pos(visual, 0, top);
    lv_obj_set_size(visual, visual_w, visual_h);

    if (page == PAGE_WATERFALL) {
        lv_obj_t *img = lv_image_create(visual);
        display->waterfall_img = img;
        lv_image_set_src(img, &display->waterfall_dsc);
        lv_obj_set_size(img, display->waterfall_w, display->waterfall_h);
        lv_obj_center(img);
        lv_obj_add_flag(img, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_remove_flag(img, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(img, waterfall_event_cb, LV_EVENT_PRESSED, display);
        lv_obj_add_event_cb(img, waterfall_event_cb, LV_EVENT_PRESSING, display);
        lv_obj_add_event_cb(img, waterfall_event_cb, LV_EVENT_RELEASED, display);
        lv_obj_add_event_cb(img, waterfall_event_cb, LV_EVENT_PRESS_LOST, display);
        lv_obj_add_event_cb(img, waterfall_event_cb, LV_EVENT_CLICKED, display);
        draw_waterfall_background(display);
        draw_ruler(display);
        mark_waterfall_dirty(display);
        if (side_w > 0) {
            create_side_panel_locked(display, feature, true);
        }
    } else if (page == PAGE_FM) {
        create_fm_visual_locked(display, visual, feature);
        if (side_w > 0) {
            create_fm_side_panel_locked(display, feature);
        }
    } else {
        const char *body = "This page mirrors the desktop app navigation. Decoder and tuner logic are not implemented on this ESP32 target yet.";
        if (page == PAGE_ADSB) {
            body = "ADS-B page placeholder. Mode S decoding and aircraft model from the desktop app still need an ESP32 implementation.";
        } else if (page == PAGE_SCANNER) {
            body = "Scanner page placeholder. Wideband multi-hop scanning requires runtime tuner control before it can be enabled.";
        } else if (page == PAGE_SENSORS) {
            body = "Sensor decoder placeholder. OOK/ASK demodulators are not wired into this firmware yet.";
        } else if (page == PAGE_RECORD) {
            body = "IQ recording placeholder. Storage and file playback are not configured on this board firmware yet.";
        }
        create_placeholder_visual(visual, feature, body, feature->accent);
        if (side_w > 0) {
            create_side_panel_locked(display, feature, false);
        }
    }

    display->status_dirty = true;
}

static void switch_page_locked(waterfall_display_t *display, app_page_t page)
{
    if (page == PAGE_HOME) {
        create_home_locked(display);
    } else {
        create_feature_locked(display, page);
    }
}

static esp_err_t create_ui(waterfall_display_t *display)
{
    for (size_t i = 0; i < sizeof(s_features) / sizeof(s_features[0]); i++) {
        unsigned int rgb = 0;
        sscanf(s_features[i].accent_hex + 1, "%x", &rgb);
        s_features[i].accent = color_hex(rgb);
    }

    lv_theme_default_init(display->lv_disp,
                          color_hex(0x4cc9f0),
                          color_hex(0xf77f00),
                          LV_THEME_DEFAULT_DARK,
                          &lv_font_montserrat_14);

    lv_obj_t *root = lv_display_get_screen_active(display->lv_disp);
    display->root = root;
    style_plain(root);
    lv_obj_set_style_bg_color(root, color_hex(0x101316), 0);
    lv_obj_set_style_bg_opa(root, LV_OPA_COVER, 0);

    lv_obj_t *header = lv_obj_create(root);
    style_plain(header);
    lv_obj_set_size(header, LV_PCT(100), 58);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(header, color_hex(0x15191d), 0);
    lv_obj_set_style_bg_opa(header, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_left(header, 24, 0);

    display->header_title = create_label(header, "RTL-SDR Panel", 22, color_hex(0xf6f8fb), true);
    lv_obj_set_size(display->header_title, display->cfg.h_res - 48, 24);
    lv_obj_align(display->header_title, LV_ALIGN_TOP_LEFT, 0, 9);

    display->header_subtitle = create_label(header, "LVGL receiver pages", 13, color_hex(0x9baab7), false);
    lv_obj_set_size(display->header_subtitle, display->cfg.h_res - 48, 18);
    lv_obj_align(display->header_subtitle, LV_ALIGN_TOP_LEFT, 0, 34);

    display->content = lv_obj_create(root);
    style_plain(display->content);
    lv_obj_set_pos(display->content, 16, 68);
    lv_obj_set_size(display->content, display->cfg.h_res - 32, display->cfg.v_res - 122);

    lv_obj_t *footer = lv_obj_create(root);
    style_plain(footer);
    lv_obj_set_size(footer, LV_PCT(100), 44);
    lv_obj_align(footer, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(footer, color_hex(0x0a0d10), 0);
    lv_obj_set_style_bg_opa(footer, LV_OPA_COVER, 0);

    display->footer_label = create_label(footer, "", 13, color_hex(0xc7d0d9), false);
    lv_obj_set_size(display->footer_label, display->cfg.h_res - 32, 24);
    lv_obj_align(display->footer_label, LV_ALIGN_LEFT_MID, 18, 0);

    create_home_locked(display);
    return ESP_OK;
}

static esp_err_t setup_rgb_panel(waterfall_display_t *display)
{
    const waterfall_display_config_t *config = &display->cfg;
    esp_lcd_rgb_panel_config_t panel_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .timings = {
            .pclk_hz = config->pclk_hz,
            .h_res = config->h_res,
            .v_res = config->v_res,
            .hsync_pulse_width = config->hsync,
            .hsync_back_porch = config->hbp,
            .hsync_front_porch = config->hfp,
            .vsync_pulse_width = config->vsync,
            .vsync_back_porch = config->vbp,
            .vsync_front_porch = config->vfp,
            .flags = {
                .pclk_active_neg = config->pclk_active_neg,
            },
        },
        .data_width = CONFIG_LCD_DATA_WIDTH,
        .in_color_format = LCD_COLOR_FMT_RGB565,
        .out_color_format = LCD_COLOR_FMT_RGB565,
        .num_fbs = 1,
        .bounce_buffer_size_px = 20 * CONFIG_LCD_H_RES,
        .dma_burst_size = 64,
        .hsync_gpio_num = CONFIG_LCD_HSYNC_GPIO,
        .vsync_gpio_num = CONFIG_LCD_VSYNC_GPIO,
        .de_gpio_num = CONFIG_LCD_DE_GPIO,
        .pclk_gpio_num = CONFIG_LCD_PCLK_GPIO,
        .disp_gpio_num = -1,
        .data_gpio_nums = {
            CONFIG_LCD_DATA0_GPIO,
            CONFIG_LCD_DATA1_GPIO,
            CONFIG_LCD_DATA2_GPIO,
            CONFIG_LCD_DATA3_GPIO,
            CONFIG_LCD_DATA4_GPIO,
            CONFIG_LCD_DATA5_GPIO,
            CONFIG_LCD_DATA6_GPIO,
            CONFIG_LCD_DATA7_GPIO,
            CONFIG_LCD_DATA8_GPIO,
            CONFIG_LCD_DATA9_GPIO,
            CONFIG_LCD_DATA10_GPIO,
            CONFIG_LCD_DATA11_GPIO,
            CONFIG_LCD_DATA12_GPIO,
            CONFIG_LCD_DATA13_GPIO,
            CONFIG_LCD_DATA14_GPIO,
            CONFIG_LCD_DATA15_GPIO,
        },
        .flags.fb_in_psram = true,
    };

    ESP_RETURN_ON_ERROR(esp_lcd_new_rgb_panel(&panel_config, &display->panel), TAG, "new RGB panel");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(display->panel), TAG, "panel reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(display->panel), TAG, "panel init");
    return ESP_OK;
}

#if CONFIG_TOUCH_ENABLED
static void touch_apply_transform(waterfall_display_t *display, uint16_t *x, uint16_t *y)
{
    uint16_t tx = *x;
    uint16_t ty = *y;
#ifdef CONFIG_TOUCH_SWAP_XY
    if (CONFIG_TOUCH_SWAP_XY) {
        uint16_t tmp = tx;
        tx = ty;
        ty = tmp;
    }
#endif
#ifdef CONFIG_TOUCH_MIRROR_X
    if (CONFIG_TOUCH_MIRROR_X && display->cfg.h_res > 0) {
        tx = (uint16_t)((tx < display->cfg.h_res) ? (display->cfg.h_res - 1 - tx) : 0);
    }
#endif
#ifdef CONFIG_TOUCH_MIRROR_Y
    if (CONFIG_TOUCH_MIRROR_Y && display->cfg.v_res > 0) {
        ty = (uint16_t)((ty < display->cfg.v_res) ? (display->cfg.v_res - 1 - ty) : 0);
    }
#endif
    if (tx >= display->cfg.h_res) {
        tx = display->cfg.h_res - 1;
    }
    if (ty >= display->cfg.v_res) {
        ty = display->cfg.v_res - 1;
    }
    *x = tx;
    *y = ty;
}

static esp_err_t touch_read_reg16(waterfall_display_t *display, uint16_t reg, uint8_t *data, size_t len)
{
    uint8_t reg_buf[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xff)};
    return i2c_master_transmit_receive(display->touch_dev, reg_buf, sizeof(reg_buf), data, len, TOUCH_TIMEOUT_MS);
}

static esp_err_t touch_write_reg16_u8(waterfall_display_t *display, uint16_t reg, uint8_t value)
{
    uint8_t buf[3] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xff), value};
    return i2c_master_transmit(display->touch_dev, buf, sizeof(buf), TOUCH_TIMEOUT_MS);
}

static esp_err_t touch_read_reg8(waterfall_display_t *display, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(display->touch_dev, &reg, 1, data, len, TOUCH_TIMEOUT_MS);
}

static void scan_touch_i2c_bus(waterfall_display_t *display)
{
    char found[96] = {0};
    size_t used = 0;

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(display->touch_bus, addr, TOUCH_TIMEOUT_MS) == ESP_OK) {
            int written = snprintf(found + used, sizeof(found) - used, "%s0x%02x", used ? " " : "", addr);
            if (written > 0) {
                used += MIN((size_t)written, sizeof(found) - used - 1);
            }
        }
    }

    ESP_LOGI(TAG, "Touch I2C scan: %s", used ? found : "no devices");
}

static bool gt911_get_point(waterfall_display_t *display, uint16_t *x, uint16_t *y)
{
    uint8_t status = 0;
    if (touch_read_reg16(display, GT911_STATUS_REG, &status, 1) != ESP_OK) {
        return false;
    }
    if (display->touch_status_log_count < 8 || (status & GT911_STATUS_READY)) {
        ESP_LOGI(TAG, "GT touch status=0x%02x", status);
        display->touch_status_log_count++;
    }
    if ((status & GT911_STATUS_READY) == 0) {
        return false;
    }
    if ((status & GT911_STATUS_TOUCH_MASK) == 0) {
        (void)touch_write_reg16_u8(display, GT911_STATUS_REG, 0);
        return false;
    }

    uint8_t point[8] = {0};
    bool pressed = false;
    if (touch_read_reg16(display, GT911_POINT1_REG, point, sizeof(point)) == ESP_OK) {
        uint16_t raw_x = (uint16_t)(point[1] << 8 | point[0]);
        uint16_t raw_y = (uint16_t)(point[3] << 8 | point[2]);
        *x = raw_x;
        *y = raw_y;
        touch_apply_transform(display, x, y);
        ESP_LOGI(TAG, "GT raw touch status=0x%02x raw=%u,%u mapped=%u,%u",
                 status,
                 (unsigned)raw_x,
                 (unsigned)raw_y,
                 (unsigned)*x,
                 (unsigned)*y);
        pressed = true;
    }
    (void)touch_write_reg16_u8(display, GT911_STATUS_REG, 0);
    return pressed;
}

static bool ft5x06_get_point(waterfall_display_t *display, uint16_t *x, uint16_t *y)
{
    uint8_t count = 0;
    if (touch_read_reg8(display, FT5X06_TD_STATUS_REG, &count, 1) != ESP_OK) {
        return false;
    }
    count &= 0x0f;
    if (count == 0) {
        return false;
    }

    uint8_t point[4] = {0};
    if (touch_read_reg8(display, FT5X06_P1_XH_REG, point, sizeof(point)) != ESP_OK) {
        return false;
    }
    *x = (uint16_t)(((point[0] & 0x0f) << 8) | point[1]);
    *y = (uint16_t)(((point[2] & 0x0f) << 8) | point[3]);
    touch_apply_transform(display, x, y);
    return true;
}

static void lvgl_touch_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    waterfall_display_t *display = lv_indev_get_user_data(indev);
    uint16_t x = 0;
    uint16_t y = 0;
    bool pressed = false;

    if (display->touch_chip == TOUCH_CHIP_GT911) {
        pressed = gt911_get_point(display, &x, &y);
    } else if (display->touch_chip == TOUCH_CHIP_FT5X06) {
        pressed = ft5x06_get_point(display, &x, &y);
    }

    if (pressed) {
        data->point.x = x;
        data->point.y = y;
        data->state = LV_INDEV_STATE_PRESSED;
        if (!display->touch_last_pressed) {
            ESP_LOGI(TAG, "Touch press %u,%u", (unsigned)x, (unsigned)y);
        }
        display->touch_last_pressed = true;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
        if (display->touch_last_pressed) {
            ESP_LOGI(TAG, "Touch release");
        }
        display->touch_last_pressed = false;
    }
}

static esp_err_t setup_touch(waterfall_display_t *display)
{
    ESP_LOGI(TAG,
             "Touch init: port=%d sda=%d scl=%d freq=%d gt911=0x%02x ft5x06=0x%02x int=%d rst=%d",
             CONFIG_TOUCH_I2C_PORT,
             CONFIG_TOUCH_I2C_SDA_GPIO,
             CONFIG_TOUCH_I2C_SCL_GPIO,
             CONFIG_TOUCH_I2C_FREQ_HZ,
             CONFIG_TOUCH_GT911_ADDR,
             CONFIG_TOUCH_FT5X06_ADDR,
             CONFIG_TOUCH_INT_GPIO,
             CONFIG_TOUCH_RST_GPIO);

    if (CONFIG_TOUCH_RST_GPIO >= 0) {
        gpio_config_t rst_cfg = {
            .pin_bit_mask = gpio_bit_mask(CONFIG_TOUCH_RST_GPIO),
            .mode = GPIO_MODE_OUTPUT,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&rst_cfg), TAG, "touch rst gpio");
        gpio_set_level(CONFIG_TOUCH_RST_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(CONFIG_TOUCH_RST_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(80));
    }
    if (CONFIG_TOUCH_INT_GPIO >= 0) {
        gpio_config_t int_cfg = {
            .pin_bit_mask = gpio_bit_mask(CONFIG_TOUCH_INT_GPIO),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&int_cfg), TAG, "touch int gpio");
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = CONFIG_TOUCH_I2C_PORT,
        .sda_io_num = CONFIG_TOUCH_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_TOUCH_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &display->touch_bus), TAG, "touch i2c bus");
    scan_touch_i2c_bus(display);

    if (i2c_master_probe(display->touch_bus, CONFIG_TOUCH_GT911_ADDR, TOUCH_TIMEOUT_MS) == ESP_OK) {
        ESP_LOGI(TAG, "Touch probe matched GT911 address 0x%02x", CONFIG_TOUCH_GT911_ADDR);
        i2c_device_config_t dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = CONFIG_TOUCH_GT911_ADDR,
            .scl_speed_hz = CONFIG_TOUCH_I2C_FREQ_HZ,
        };
        ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(display->touch_bus, &dev_config, &display->touch_dev),
                            TAG, "add GT911");
        uint8_t product[4] = {0};
        if (touch_read_reg16(display, GT911_PRODUCT_ID_REG, product, sizeof(product)) == ESP_OK) {
            display->touch_chip = TOUCH_CHIP_GT911;
            snprintf(display->touch_status, sizeof(display->touch_status), "GT911 %c%c%c%c",
                     product[0], product[1], product[2], product[3]);
        } else {
            ESP_LOGW(TAG, "GT911 address responded but product ID read failed");
            i2c_master_bus_rm_device(display->touch_dev);
            display->touch_dev = NULL;
        }
    }

    if (!display->touch_dev && i2c_master_probe(display->touch_bus, CONFIG_TOUCH_FT5X06_ADDR, TOUCH_TIMEOUT_MS) == ESP_OK) {
        ESP_LOGI(TAG, "Touch probe matched FT5x06 address 0x%02x", CONFIG_TOUCH_FT5X06_ADDR);
        i2c_device_config_t dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = CONFIG_TOUCH_FT5X06_ADDR,
            .scl_speed_hz = CONFIG_TOUCH_I2C_FREQ_HZ,
        };
        ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(display->touch_bus, &dev_config, &display->touch_dev),
                            TAG, "add FT5x06");
        uint8_t vendor = 0;
        (void)touch_read_reg8(display, FT5X06_VENDOR_ID_REG, &vendor, 1);
        display->touch_chip = TOUCH_CHIP_FT5X06;
        snprintf(display->touch_status, sizeof(display->touch_status), "FT5x06 0x%02x", vendor);
    }

    if (!display->touch_dev) {
        snprintf(display->touch_status, sizeof(display->touch_status), "touch not found");
        ESP_LOGW(TAG, "No supported I2C touch controller found");
        return ESP_ERR_NOT_FOUND;
    }

    display->touch_indev = lv_indev_create();
    lv_indev_set_type(display->touch_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(display->touch_indev, display->lv_disp);
    lv_indev_set_user_data(display->touch_indev, display);
    lv_indev_set_read_cb(display->touch_indev, lvgl_touch_cb);
    ESP_LOGI(TAG, "Touch input ready: %s", display->touch_status);
    return ESP_OK;
}
#endif

static esp_err_t setup_lvgl(waterfall_display_t *display)
{
    lv_init();
    display->lv_disp = lv_display_create(display->cfg.h_res, display->cfg.v_res);
    ESP_RETURN_ON_FALSE(display->lv_disp, ESP_ERR_NO_MEM, TAG, "lv display");
    lv_display_set_user_data(display->lv_disp, display->panel);
    lv_display_set_color_format(display->lv_disp, LV_COLOR_FORMAT_RGB565);

    size_t draw_buffer_sz = (size_t)display->cfg.h_res * CONFIG_LVGL_DRAW_BUF_LINES * sizeof(lv_color16_t);
    display->lv_draw_buf = esp_lcd_rgb_alloc_draw_buffer(display->panel, draw_buffer_sz, 0);
    ESP_RETURN_ON_FALSE(display->lv_draw_buf, ESP_ERR_NO_MEM, TAG, "lv draw buffer");
    lv_display_set_buffers(display->lv_disp, display->lv_draw_buf, NULL, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(display->lv_disp, lvgl_flush_cb);

    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_rgb_panel_register_event_callbacks(display->panel, &cbs, display->lv_disp),
                        TAG, "rgb callbacks");

    const esp_timer_create_args_t tick_timer_args = {
        .callback = increase_lvgl_tick,
        .name = "lvgl_tick",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&tick_timer_args, &display->lvgl_tick_timer), TAG, "lvgl tick timer");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(display->lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000),
                        TAG, "lvgl tick start");
    return ESP_OK;
}

static esp_err_t alloc_waterfall_image(waterfall_display_t *display)
{
    int side_panel = (display->cfg.h_res >= 620) ? ((display->cfg.h_res >= 760) ? 250 : 210) : 0;
    int available_w = display->cfg.h_res - 32 - side_panel - (side_panel ? 14 : 0);
    int available_h = display->cfg.v_res - 136;
    display->waterfall_w = available_w;
    display->waterfall_h = available_h;
    if (display->waterfall_w < 240) {
        display->waterfall_w = display->cfg.h_res - 32;
    }
    if (display->waterfall_h < 160) {
        display->waterfall_h = display->cfg.v_res - 122;
    }
    display->spectrum_h = (display->waterfall_h >= 320) ? 104 : 78;
    display->ruler_h = 18;
    if (display->spectrum_h + display->ruler_h >= display->waterfall_h - 24) {
        display->spectrum_h = display->waterfall_h / 3;
        display->ruler_h = 14;
    }

    size_t bytes = (size_t)display->waterfall_w * display->waterfall_h * sizeof(uint16_t);
    display->waterfall_pixels = heap_caps_aligned_alloc(WATERFALL_IMAGE_ALIGN, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!display->waterfall_pixels) {
        display->waterfall_pixels = heap_caps_aligned_alloc(WATERFALL_IMAGE_ALIGN, bytes, MALLOC_CAP_8BIT);
    }
    ESP_RETURN_ON_FALSE(display->waterfall_pixels, ESP_ERR_NO_MEM, TAG, "waterfall pixels");
    memset(display->waterfall_pixels, 0, bytes);

    display->waterfall_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    display->waterfall_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
    display->waterfall_dsc.header.w = display->waterfall_w;
    display->waterfall_dsc.header.h = display->waterfall_h;
    display->waterfall_dsc.data_size = bytes;
    display->waterfall_dsc.data = (const uint8_t *)display->waterfall_pixels;
    return ESP_OK;
}

static esp_err_t alloc_fm_waveform_image(waterfall_display_t *display)
{
    int side_panel = (display->cfg.h_res >= 620) ? ((display->cfg.h_res >= 760) ? 250 : 210) : 0;
    int available_w = display->cfg.h_res - 32 - side_panel - (side_panel ? 14 : 0);
    int available_h = display->cfg.v_res - 136;
    if (available_w < 260 || display->cfg.h_res < 620) {
        available_w = display->cfg.h_res - 32;
    }
    if (available_h < 160) {
        available_h = display->cfg.v_res - 122;
    }

    display->fm_waveform_w = available_w;
    if (display->fm_waveform_w < 120) {
        display->fm_waveform_w = 120;
    }

    display->fm_waveform_h = available_h;
    if (display->fm_waveform_h < 48) {
        display->fm_waveform_h = 48;
    }

    size_t bytes = (size_t)display->fm_waveform_w * display->fm_waveform_h * sizeof(uint16_t);
    display->fm_waveform_pixels = heap_caps_aligned_alloc(WATERFALL_IMAGE_ALIGN,
                                                          bytes,
                                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!display->fm_waveform_pixels) {
        display->fm_waveform_pixels = heap_caps_aligned_alloc(WATERFALL_IMAGE_ALIGN, bytes, MALLOC_CAP_8BIT);
    }
    ESP_RETURN_ON_FALSE(display->fm_waveform_pixels, ESP_ERR_NO_MEM, TAG, "FM waveform pixels");
    memset(display->fm_waveform_pixels, 0, bytes);

    display->fm_waveform_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    display->fm_waveform_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
    display->fm_waveform_dsc.header.w = display->fm_waveform_w;
    display->fm_waveform_dsc.header.h = display->fm_waveform_h;
    display->fm_waveform_dsc.data_size = bytes;
    display->fm_waveform_dsc.data = (const uint8_t *)display->fm_waveform_pixels;
    draw_fm_waveform_background(display);
    return ESP_OK;
}

esp_err_t waterfall_display_create(const waterfall_display_config_t *config, waterfall_display_t **out)
{
    ESP_RETURN_ON_FALSE(config && out, ESP_ERR_INVALID_ARG, TAG, "bad args");
    esp_err_t ret = ESP_OK;
    waterfall_display_t *display = calloc(1, sizeof(*display));
    ESP_RETURN_ON_FALSE(display, ESP_ERR_NO_MEM, TAG, "no memory");
    _lock_init(&display->lvgl_api_lock);

    display->cfg = *config;
    display->center_hz = CONFIG_RTLSDR_CENTER_FREQ_HZ;
    display->sample_rate_hz = CONFIG_RTLSDR_SAMPLE_RATE;
    display->fm_audio_rate_hz = CONFIG_FM_RECEIVER_AUDIO_RATE_HZ;
    display->fm_audio_output_enabled = CONFIG_FM_RECEIVER_I2S_OUTPUT;
    display->fm_audio_output_ready = false;
    snprintf(display->fm_mode, sizeof(display->fm_mode), "%s", CONFIG_FM_RECEIVER_NFM_MODE ? "NFM" : "WFM");
    strcpy(display->status, "WAITING FOR RTL2832U");
#if CONFIG_TOUCH_ENABLED
    strcpy(display->touch_status, "touch probing");
#else
    strcpy(display->touch_status, "touch disabled");
#endif
    display->status_dirty = true;

    ESP_GOTO_ON_ERROR(setup_rgb_panel(display), fail, TAG, "rgb panel");
    ESP_GOTO_ON_ERROR(alloc_waterfall_image(display), fail, TAG, "waterfall image");
    ESP_GOTO_ON_ERROR(alloc_fm_waveform_image(display), fail, TAG, "FM waveform image");
    draw_waterfall_background(display);
    draw_ruler(display);
    ESP_GOTO_ON_ERROR(setup_lvgl(display), fail, TAG, "lvgl");

    _lock_acquire(&display->lvgl_api_lock);
    ret = create_ui(display);
#if CONFIG_TOUCH_ENABLED
    if (ret == ESP_OK) {
        esp_err_t touch_ret = setup_touch(display);
        if (touch_ret != ESP_OK) {
            display->status_dirty = true;
        }
    }
#endif
    _lock_release(&display->lvgl_api_lock);
    ESP_GOTO_ON_ERROR(ret, fail, TAG, "ui");

    BaseType_t ok = xTaskCreate(lvgl_task,
                                "LVGL",
                                CONFIG_LVGL_TASK_STACK_SIZE,
                                display,
                                CONFIG_LVGL_TASK_PRIORITY,
                                &display->lvgl_task_handle);
    ESP_GOTO_ON_FALSE(ok == pdTRUE, ESP_ERR_NO_MEM, fail, TAG, "lvgl task");

    s_display = display;
    *out = display;
    return ESP_OK;

fail:
    waterfall_display_destroy(display);
    return ret;
}

void waterfall_display_destroy(waterfall_display_t *display)
{
    if (!display) {
        return;
    }
    if (display->lvgl_task_handle && xTaskGetCurrentTaskHandle() != display->lvgl_task_handle) {
        _lock_acquire(&display->lvgl_api_lock);
        display->shutting_down = true;
        _lock_release(&display->lvgl_api_lock);
        while (display->lvgl_task_handle) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    if (display->lvgl_tick_timer) {
        esp_timer_stop(display->lvgl_tick_timer);
        esp_timer_delete(display->lvgl_tick_timer);
    }
#if CONFIG_TOUCH_ENABLED
    if (display->touch_indev) {
        lv_indev_delete(display->touch_indev);
    }
#endif
    if (display->lv_disp) {
        lv_display_delete(display->lv_disp);
    }
#if CONFIG_TOUCH_ENABLED
    if (display->touch_dev) {
        i2c_master_bus_rm_device(display->touch_dev);
    }
    if (display->touch_bus) {
        i2c_del_master_bus(display->touch_bus);
    }
#endif
    if (display->panel) {
        esp_lcd_panel_del(display->panel);
    }
    heap_caps_free(display->lv_draw_buf);
    heap_caps_free(display->waterfall_pixels);
    heap_caps_free(display->fm_waveform_pixels);
    if (s_display == display) {
        s_display = NULL;
    }
    _lock_close(&display->lvgl_api_lock);
    free(display);
}

void waterfall_display_set_status(waterfall_display_t *display, const char *status)
{
    if (!display || !status) {
        return;
    }
    _lock_acquire(&display->lvgl_api_lock);
    snprintf(display->status, sizeof(display->status), "%s", status);
    display->status_dirty = true;
    _lock_release(&display->lvgl_api_lock);
}

void waterfall_display_set_radio_info(waterfall_display_t *display, uint32_t center_hz, uint32_t sample_rate_hz)
{
    if (!display) {
        return;
    }
    _lock_acquire(&display->lvgl_api_lock);
    display->center_hz = center_hz;
    display->sample_rate_hz = sample_rate_hz;
    display->status_dirty = true;
    draw_ruler(display);
    mark_waterfall_dirty(display);
    _lock_release(&display->lvgl_api_lock);
}

void waterfall_display_set_fm_info(waterfall_display_t *display,
                                   const char *mode,
                                   uint32_t audio_rate_hz,
                                   bool audio_output_enabled,
                                   bool audio_output_ready,
                                   uint8_t audio_level)
{
    if (!display) {
        return;
    }
    _lock_acquire(&display->lvgl_api_lock);
    if (mode && mode[0]) {
        snprintf(display->fm_mode, sizeof(display->fm_mode), "%s", mode);
    }
    display->fm_audio_rate_hz = audio_rate_hz;
    display->fm_audio_output_enabled = audio_output_enabled;
    display->fm_audio_output_ready = audio_output_ready;
    display->fm_audio_level = audio_level > 100 ? 100 : audio_level;
    display->status_dirty = true;
    _lock_release(&display->lvgl_api_lock);
}

void waterfall_display_push_fm_waveform(waterfall_display_t *display, const int16_t *samples, size_t sample_count)
{
    if (!display || !samples || sample_count == 0 || !display->fm_waveform_pixels) {
        return;
    }

    _lock_acquire(&display->lvgl_api_lock);
    if (display->active_page == PAGE_FM || display->fm_waveform_img) {
        draw_fm_waveform_background(display);
        draw_fm_waveform(display, samples, sample_count);
        display->fm_waveform_dirty = true;
    }
    _lock_release(&display->lvgl_api_lock);
}

void waterfall_display_set_center_freq_cb(waterfall_display_t *display,
                                          waterfall_center_freq_cb_t cb,
                                          void *ctx)
{
    if (!display) {
        return;
    }
    _lock_acquire(&display->lvgl_api_lock);
    display->center_freq_cb = cb;
    display->center_freq_cb_ctx = ctx;
    _lock_release(&display->lvgl_api_lock);
}

void waterfall_display_push_line(waterfall_display_t *display, const uint8_t *bins, size_t bin_count)
{
    if (!display || !bins || bin_count == 0 || !display->waterfall_pixels) {
        return;
    }

    _lock_acquire(&display->lvgl_api_lock);
    int w = display->waterfall_w;
    int y0 = display->spectrum_h + display->ruler_h;
    int wh = display->waterfall_h - y0;
    if (wh <= 0) {
        _lock_release(&display->lvgl_api_lock);
        return;
    }

    draw_spectrum(display, bins, bin_count);
    draw_ruler(display);

    if (wh > 1) {
        memmove(display->waterfall_pixels + (y0 + 1) * w,
                display->waterfall_pixels + y0 * w,
                (size_t)(wh - 1) * w * sizeof(uint16_t));
    }

    uint16_t *row = display->waterfall_pixels + y0 * w;
    for (int x = 0; x < w; x++) {
        row[x] = palette(scaled_bin_at(bins, bin_count, x, w));
    }

    display->have_samples = true;
    mark_waterfall_dirty(display);
    _lock_release(&display->lvgl_api_lock);
}

void waterfall_display_render_idle(waterfall_display_t *display)
{
    if (!display) {
        return;
    }
    _lock_acquire(&display->lvgl_api_lock);
    display->status_dirty = true;
    mark_waterfall_dirty(display);
    _lock_release(&display->lvgl_api_lock);
}
