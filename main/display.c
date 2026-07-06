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
#define ADSB_CENTER_HZ              1090000000U
#define ADSB_DISPLAY_MAX_AIRCRAFT   12
#define SCANNER_DEFAULT_START_HZ    88000000U
#define SCANNER_DEFAULT_STOP_HZ     108000000U
#define SCANNER_ROW_H               3
#define SCANNER_HEADER_H            26
#define SCANNER_FOOTER_H            18
#define SENSOR_CENTER_HZ            433920000U
#define SENSOR_SAMPLE_RATE_HZ       250000U
#define SENSOR_DISPLAY_MAX_EVENTS   8
#define RECORD_PATH_MAX             80

#define GT911_PRODUCT_ID_REG        0x8140
#define GT911_STATUS_REG            0x814e
#define GT911_POINT1_REG            0x8150
#define GT911_STATUS_READY          0x80
#define GT911_STATUS_TOUCH_MASK     0x0f

#define FT5X06_VENDOR_ID_REG        0xa8
#define FT5X06_TD_STATUS_REG        0x02
#define FT5X06_P1_XH_REG            0x03

typedef enum {
    TOUCH_CHIP_NONE = 0,
    TOUCH_CHIP_GT911,
    TOUCH_CHIP_FT5X06,
} touch_chip_t;

typedef struct {
    app_display_page_t page;
    const char *title;
    const char *subtitle;
    const char *accent_hex;
    lv_color_t accent;
} feature_def_t;

typedef struct {
    lv_obj_t *img;
    lv_image_dsc_t dsc;
    uint16_t *pixels;
    int w;
    int h;
    int spectrum_h;
    int ruler_h;
    uint32_t center_hz;
    uint32_t marker_hz;
    bool marker_valid;
    bool dirty;
    bool swipe_tracking;
    bool swipe_moved;
    bool suppress_next_click;
    int swipe_start_x;
    int swipe_start_y;
} waterfall_page_state_t;

typedef struct {
    lv_obj_t *mode_value_label;
    lv_obj_t *audio_rate_value_label;
    lv_obj_t *audio_output_value_label;
    lv_obj_t *audio_level_bar;
    lv_obj_t *audio_level_label;
    lv_obj_t *center_large_label;
    lv_obj_t *status_large_label;
    lv_obj_t *waveform_img;
    lv_image_dsc_t waveform_dsc;
    uint16_t *waveform_pixels;
    int waveform_w;
    int waveform_h;
    char mode[8];
    uint32_t center_hz;
    uint32_t audio_rate_hz;
    uint8_t audio_level;
    bool audio_output_enabled;
    bool audio_output_ready;
    bool waveform_dirty;
} fm_page_state_t;

typedef struct {
    lv_obj_t *status_label;
    lv_obj_t *count_value_label;
    lv_obj_t *messages_value_label;
    lv_obj_t *candidates_value_label;
    lv_obj_t *list_label;
    uint32_t center_hz;
    uint32_t sample_rate_hz;
    uint64_t candidates;
    uint64_t valid_messages;
    uint32_t aircraft_count;
    bool active;
    app_display_adsb_aircraft_t aircraft[ADSB_DISPLAY_MAX_AIRCRAFT];
    uint32_t aircraft_used;
} adsb_page_state_t;

typedef struct {
    lv_obj_t *img;
    lv_obj_t *range_value_label;
    lv_obj_t *center_value_label;
    lv_obj_t *hop_value_label;
    lv_obj_t *lines_value_label;
    lv_image_dsc_t dsc;
    uint16_t *pixels;
    int w;
    int h;
    int plot_y;
    int plot_h;
    uint32_t start_hz;
    uint32_t stop_hz;
    uint32_t center_hz;
    uint32_t hop_index;
    uint32_t hop_count;
    uint32_t completed_lines;
    bool active;
    bool dirty;
} scanner_page_state_t;

typedef struct {
    lv_obj_t *status_label;
    lv_obj_t *list_label;
    lv_obj_t *center_value_label;
    lv_obj_t *rate_value_label;
    lv_obj_t *frames_value_label;
    lv_obj_t *decoded_value_label;
    lv_obj_t *sensors_value_label;
    lv_obj_t *signal_value_label;
    lv_obj_t *signal_bar;
    uint32_t center_hz;
    uint32_t sample_rate_hz;
    uint64_t total_frames;
    uint64_t decoded_messages;
    uint32_t sensor_count;
    uint8_t signal_level;
    bool active;
    app_display_sensor_event_t events[SENSOR_DISPLAY_MAX_EVENTS];
    uint32_t event_count;
} sensor_page_state_t;

typedef struct {
    lv_obj_t *status_label;
    lv_obj_t *path_label;
    lv_obj_t *center_value_label;
    lv_obj_t *rate_value_label;
    lv_obj_t *bytes_value_label;
    lv_obj_t *elapsed_value_label;
    lv_obj_t *storage_value_label;
    lv_obj_t *progress_bar;
    uint32_t center_hz;
    uint32_t sample_rate_hz;
    uint64_t bytes_written;
    uint64_t max_bytes;
    uint32_t elapsed_seconds;
    bool active;
    bool storage_ready;
    char path[RECORD_PATH_MAX];
} record_page_state_t;

struct app_display {
    app_display_config_t cfg;
    app_center_freq_cb_t center_freq_cb;
    void *center_freq_cb_ctx;
    app_page_cb_t page_cb;
    void *page_cb_ctx;
    esp_lcd_panel_handle_t panel;
    lv_display_t *lv_disp;
    void *lv_draw_buf;
    lv_obj_t *root;
    lv_obj_t *content;
    lv_obj_t *footer_label;
    lv_obj_t *header_title;
    lv_obj_t *header_subtitle;
    lv_obj_t *header_button;
    lv_obj_t *side_panel;
    lv_obj_t *freq_row;
    lv_obj_t *status_value_label;
    lv_obj_t *freq_value_label;
    lv_obj_t *rate_value_label;
    lv_obj_t *marker_value_label;
    lv_obj_t *touch_value_label;
    lv_obj_t *freq_dialog;
    lv_obj_t *freq_textarea;
    lv_obj_t *freq_digit_roller;
    lv_obj_t *freq_error_label;
    lv_obj_t *placeholder_label;
    char status[96];
    char touch_status[64];
    char footer_text[256];
    waterfall_page_state_t waterfall;
    fm_page_state_t fm;
    adsb_page_state_t adsb;
    scanner_page_state_t scanner;
    sensor_page_state_t sensor;
    record_page_state_t record;
    uint32_t sample_rate_hz;
    bool have_samples;
    bool status_dirty;
    bool shutting_down;
    app_display_page_t active_page;
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
static app_display_t *s_display;
static void close_freq_dialog(app_display_t *display);

#ifndef CONFIG_FM_RECEIVER_NFM_MODE
#define CONFIG_FM_RECEIVER_NFM_MODE 0
#endif
#ifndef CONFIG_FM_RECEIVER_AUDIO_OUTPUT
#define CONFIG_FM_RECEIVER_AUDIO_OUTPUT 1
#endif
#ifndef CONFIG_FM_RECEIVER_AUDIO_RATE_HZ
#define CONFIG_FM_RECEIVER_AUDIO_RATE_HZ 48000
#endif

static feature_def_t s_features[] = {
    {APP_DISPLAY_PAGE_ADSB, "ADS-B", "1090 MHz aircraft view", "#4cc9f0", {0}},
    {APP_DISPLAY_PAGE_FM, "FM Receiver", "WFM/NFM spectrum", "#80ed99", {0}},
    {APP_DISPLAY_PAGE_WATERFALL, "Waterfall", "Live FFT and marker", "#f77f00", {0}},
    {APP_DISPLAY_PAGE_SENSORS, "Sensors", "433/315 MHz packets", "#2dd4bf", {0}},
    {APP_DISPLAY_PAGE_RECORD, "IQ Record", "Raw IQ capture", "#ff5c8a", {0}},
    {APP_DISPLAY_PAGE_SCANNER, "Scanner", "Wideband heatmap", "#a855f7", {0}},
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

static uint16_t scanner_palette(uint8_t value)
{
    if (value < 20) {
        return rgb565(0, 0, 0);
    }
    if (value < 46) {
        return rgb565(5, 5, 78);
    }
    if (value < 77) {
        return rgb565(0, 28, 180);
    }
    if (value < 110) {
        return rgb565(0, 140, 220);
    }
    if (value < 145) {
        return rgb565(0, 220, 130);
    }
    if (value < 179) {
        return rgb565(128, 235, 0);
    }
    if (value < 209) {
        return rgb565(245, 230, 0);
    }
    if (value < 235) {
        return rgb565(245, 128, 0);
    }
    return rgb565(235, 24, 24);
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

static const feature_def_t *feature_for_page(app_display_page_t page)
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

static bool page_has_center_freq(app_display_page_t page)
{
    return page == APP_DISPLAY_PAGE_WATERFALL ||
           page == APP_DISPLAY_PAGE_FM ||
           page == APP_DISPLAY_PAGE_ADSB ||
           page == APP_DISPLAY_PAGE_SENSORS ||
           page == APP_DISPLAY_PAGE_RECORD;
}

static uint32_t get_page_center_freq_locked(const app_display_t *display, app_display_page_t page)
{
    if (page == APP_DISPLAY_PAGE_ADSB) {
        return display->adsb.center_hz;
    }
    if (page == APP_DISPLAY_PAGE_SCANNER) {
        return display->scanner.center_hz;
    }
    if (page == APP_DISPLAY_PAGE_SENSORS) {
        return display->sensor.center_hz;
    }
    if (page == APP_DISPLAY_PAGE_RECORD) {
        return display->record.center_hz;
    }
    if (page == APP_DISPLAY_PAGE_FM) {
        return display->fm.center_hz;
    }
    return display->waterfall.center_hz;
}

static uint32_t get_active_center_freq_locked(const app_display_t *display)
{
    return get_page_center_freq_locked(display, display->active_page);
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
    if (hz < CENTER_FREQ_MIN_HZ || hz > CENTER_FREQ_MAX_HZ) {
        return false;
    }

    *out_hz = (uint32_t)hz;
    return true;
}

static void set_freq_dialog_error_locked(app_display_t *display, const char *message)
{
    if (display->freq_error_label) {
        lv_label_set_text(display->freq_error_label, message ? message : "");
    }
    if (display->freq_textarea) {
        lv_obj_set_style_border_color(display->freq_textarea,
                                      message && message[0] ? color_hex(0xc94b4b) : color_hex(0x2e3a44),
                                      0);
    }
}

static void tune_page_center_freq_locked(app_display_t *display, app_display_page_t page)
{
    if (!page_has_center_freq(page)) {
        return;
    }

    uint32_t center_hz = get_page_center_freq_locked(display, page);
    if (display->center_freq_cb) {
        display->center_freq_cb(center_hz, display->center_freq_cb_ctx);
    }

    char freq[32];
    format_freq(freq, sizeof(freq), center_hz, 3);
    ESP_LOGI(TAG, "%s center frequency applied: %s", feature_for_page(page)->title, freq);
}

static void set_active_center_freq_locked(app_display_t *display, uint32_t center_hz)
{
    if (center_hz < CENTER_FREQ_MIN_HZ) {
        center_hz = CENTER_FREQ_MIN_HZ;
    } else if (center_hz > CENTER_FREQ_MAX_HZ) {
        center_hz = CENTER_FREQ_MAX_HZ;
    }

    app_display_page_t page = display->active_page;
    if (!page_has_center_freq(page)) {
        return;
    }

    uint32_t *page_center = &display->waterfall.center_hz;
    if (page == APP_DISPLAY_PAGE_FM) {
        page_center = &display->fm.center_hz;
    } else if (page == APP_DISPLAY_PAGE_ADSB) {
        page_center = &display->adsb.center_hz;
    } else if (page == APP_DISPLAY_PAGE_SENSORS) {
        page_center = &display->sensor.center_hz;
    } else if (page == APP_DISPLAY_PAGE_RECORD) {
        page_center = &display->record.center_hz;
    }
    if (*page_center == center_hz) {
        return;
    }

    *page_center = center_hz;
    if (page == APP_DISPLAY_PAGE_WATERFALL) {
        display->waterfall.marker_valid = false;
    }
    display->status_dirty = true;

    tune_page_center_freq_locked(display, page);

    char freq[32];
    format_freq(freq, sizeof(freq), center_hz, 3);
    ESP_LOGI(TAG, "%s center frequency set to %s", feature_for_page(page)->title, freq);
}

static void step_center_freq_locked(app_display_t *display, int direction, int steps)
{
    if (direction == 0 || steps <= 0) {
        return;
    }

    uint32_t step_hz = display->sample_rate_hz / 10U;
    if (step_hz < WATERFALL_SWIPE_MIN_HZ) {
        step_hz = WATERFALL_SWIPE_MIN_HZ;
    }

    int64_t delta = (int64_t)step_hz * steps * direction;
    int64_t next = (int64_t)get_active_center_freq_locked(display) + delta;
    if (next < CENTER_FREQ_MIN_HZ) {
        next = CENTER_FREQ_MIN_HZ;
    } else if (next > CENTER_FREQ_MAX_HZ) {
        next = CENTER_FREQ_MAX_HZ;
    }
    set_active_center_freq_locked(display, (uint32_t)next);
}

static void fill_rect_rgb565(app_display_t *display, int x, int y, int w, int h, uint16_t color)
{
    if (!display->waterfall.pixels) {
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
    if (x2 > display->waterfall.w) {
        x2 = display->waterfall.w;
    }
    if (y2 > display->waterfall.h) {
        y2 = display->waterfall.h;
    }
    if (x >= x2 || y >= y2) {
        return;
    }

    for (int row = y; row < y2; row++) {
        uint16_t *dst = display->waterfall.pixels + row * display->waterfall.w + x;
        for (int col = x; col < x2; col++) {
            *dst++ = color;
        }
    }
}

static void draw_hline(app_display_t *display, int x, int y, int w, uint16_t color)
{
    fill_rect_rgb565(display, x, y, w, 1, color);
}

static void fm_fill_rect(app_display_t *display, int x, int y, int w, int h, uint16_t color)
{
    if (!display->fm.waveform_pixels) {
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
    if (x2 > display->fm.waveform_w) {
        x2 = display->fm.waveform_w;
    }
    if (y2 > display->fm.waveform_h) {
        y2 = display->fm.waveform_h;
    }
    if (x >= x2 || y >= y2) {
        return;
    }

    for (int row = y; row < y2; row++) {
        uint16_t *dst = display->fm.waveform_pixels + row * display->fm.waveform_w + x;
        for (int col = x; col < x2; col++) {
            *dst++ = color;
        }
    }
}

static void fm_draw_line(app_display_t *display, int x0, int y0, int x1, int y1, uint16_t color)
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

static void draw_fm_waveform_background(app_display_t *display)
{
    if (!display->fm.waveform_pixels) {
        return;
    }
    fm_fill_rect(display, 0, 0, display->fm.waveform_w, display->fm.waveform_h, rgb565(8, 13, 11));

    uint16_t grid = rgb565(28, 48, 38);
    for (int i = 0; i <= 8; i++) {
        int x = (i == 8) ? display->fm.waveform_w - 1 : (display->fm.waveform_w * i) / 8;
        fm_fill_rect(display, x, 0, 1, display->fm.waveform_h, grid);
    }
    for (int i = 0; i <= 4; i++) {
        int y = (i == 4) ? display->fm.waveform_h - 1 : (display->fm.waveform_h * i) / 4;
        fm_fill_rect(display, 0, y, display->fm.waveform_w, 1, grid);
    }
    fm_fill_rect(display, 0, display->fm.waveform_h / 2, display->fm.waveform_w, 2, rgb565(42, 75, 58));
}

static void draw_fm_waveform(app_display_t *display, const int16_t *samples, size_t sample_count)
{
    if (!display->fm.waveform_pixels || !samples || sample_count == 0) {
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

    const int w = display->fm.waveform_w;
    const int h = display->fm.waveform_h;
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

static void draw_fm_spectrum(app_display_t *display, const uint8_t *bins, size_t bin_count)
{
    if (!display->fm.waveform_pixels || !bins || bin_count == 0) {
        return;
    }

    const int w = display->fm.waveform_w;
    const int h = display->fm.waveform_h;
    const int bottom = h - 8;
    if (w <= 1 || h <= 16) {
        return;
    }

    uint8_t floor = 255;
    uint8_t peak = 0;
    for (int x = 0; x < w; x++) {
        uint8_t value = scaled_bin_at(bins, bin_count, x, w);
        if (value < floor) {
            floor = value;
        }
        if (value > peak) {
            peak = value;
        }
    }

    int range = (int)peak - (int)floor;
    if (range < 32) {
        range = 32;
    }

    const uint16_t fill = rgb565(24, 92, 58);
    const uint16_t fill_hot = rgb565(64, 142, 86);
    const uint16_t trace = rgb565(184, 238, 190);
    const uint16_t glow = rgb565(48, 120, 72);
    int prev_x = 0;
    int prev_y = -1;

    for (int x = 0; x < w; x++) {
        uint8_t value = scaled_bin_at(bins, bin_count, x, w);
        int norm = ((int)value - (int)floor) * 255 / range;
        if (norm < 0) {
            norm = 0;
        } else if (norm > 255) {
            norm = 255;
        }

        int bar = 2 + (norm * (h - 22)) / 255;
        int y_trace = bottom - bar;
        if (y_trace < 5) {
            y_trace = 5;
        }

        uint16_t area = norm > 180 ? fill_hot : fill;
        for (int y = y_trace + 1; y <= bottom; y++) {
            uint16_t shade = (y < bottom - 5) ? area : rgb565(10, 34, 26);
            display->fm.waveform_pixels[y * w + x] = shade;
        }

        if (prev_y >= 0) {
            fm_draw_line(display, prev_x, prev_y + 1, x, y_trace + 1, glow);
            fm_draw_line(display, prev_x, prev_y, x, y_trace, trace);
        } else {
            fm_fill_rect(display, x, y_trace, 1, 2, trace);
        }
        prev_x = x;
        prev_y = y_trace;
    }
}

static void scanner_fill_rect(app_display_t *display, int x, int y, int w, int h, uint16_t color)
{
    if (!display->scanner.pixels) {
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
    if (x2 > display->scanner.w) {
        x2 = display->scanner.w;
    }
    if (y2 > display->scanner.h) {
        y2 = display->scanner.h;
    }
    if (x >= x2 || y >= y2) {
        return;
    }

    for (int row = y; row < y2; row++) {
        uint16_t *dst = display->scanner.pixels + row * display->scanner.w + x;
        for (int col = x; col < x2; col++) {
            *dst++ = color;
        }
    }
}

static void draw_scanner_background(app_display_t *display)
{
    if (!display->scanner.pixels) {
        return;
    }

    const int w = display->scanner.w;
    const int h = display->scanner.h;
    const int plot_y = display->scanner.plot_y;
    const int plot_h = display->scanner.plot_h;

    scanner_fill_rect(display, 0, 0, w, h, rgb565(5, 8, 12));
    scanner_fill_rect(display, 0, 0, w, SCANNER_HEADER_H, rgb565(10, 13, 18));
    scanner_fill_rect(display, 0, h - SCANNER_FOOTER_H, w, SCANNER_FOOTER_H, rgb565(8, 11, 15));

    for (int i = 1; i < 8; i++) {
        int x = (w * i) / 8;
        scanner_fill_rect(display, x, plot_y, 1, plot_h, rgb565(22, 34, 45));
    }
    for (int i = 1; i < 4; i++) {
        int y = plot_y + (plot_h * i) / 4;
        scanner_fill_rect(display, 0, y, w, 1, rgb565(18, 28, 38));
    }
    scanner_fill_rect(display, 0, plot_y - 1, w, 1, rgb565(40, 52, 64));
    scanner_fill_rect(display, 0, plot_y + plot_h, w, 1, rgb565(40, 52, 64));

    int legend_x = MAX(8, w - 142);
    int legend_y = 8;
    int legend_w = MIN(112, w - legend_x - 8);
    for (int x = 0; x < legend_w; x++) {
        uint8_t value = (uint8_t)((x * 255) / MAX(1, legend_w - 1));
        scanner_fill_rect(display, legend_x + x, legend_y, 1, 10, scanner_palette(value));
    }
}

static void scanner_scroll_for_new_line(app_display_t *display)
{
    const int w = display->scanner.w;
    const int plot_y = display->scanner.plot_y;
    const int plot_h = display->scanner.plot_h;
    const int row_h = MIN(SCANNER_ROW_H, plot_h);
    if (row_h <= 0) {
        return;
    }
    if (plot_h > row_h) {
        memmove(display->scanner.pixels + (plot_y + row_h) * w,
                display->scanner.pixels + plot_y * w,
                (size_t)(plot_h - row_h) * w * sizeof(uint16_t));
    }
    scanner_fill_rect(display, 0, plot_y, w, row_h, rgb565(3, 7, 12));
}

static void draw_vline(app_display_t *display, int x, int y, int h, uint16_t color)
{
    fill_rect_rgb565(display, x, y, 1, h, color);
}

static void set_px(app_display_t *display, int x, int y, uint16_t color)
{
    if (x >= 0 && x < display->waterfall.w && y >= 0 && y < display->waterfall.h) {
        display->waterfall.pixels[y * display->waterfall.w + x] = color;
    }
}

static void draw_trace_line(app_display_t *display, int x0, int y0, int x1, int y1, uint16_t color)
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

static void draw_waterfall_background(app_display_t *display)
{
    uint16_t spectrum_bg = rgb565(8, 14, 20);
    uint16_t waterfall_bg = rgb565(2, 6, 12);
    uint16_t grid = rgb565(26, 43, 53);
    uint16_t center = rgb565(60, 96, 86);
    int w = display->waterfall.w;
    int h = display->waterfall.h;
    int spectrum_h = display->waterfall.spectrum_h;
    int ruler_h = display->waterfall.ruler_h;
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

static void draw_spectrum(app_display_t *display, const uint8_t *bins, size_t bin_count)
{
    int w = display->waterfall.w;
    int h = display->waterfall.spectrum_h;
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
            display->waterfall.pixels[y * w + x] = color;
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

static void draw_ruler(app_display_t *display)
{
    int w = display->waterfall.w;
    int top = display->waterfall.spectrum_h;
    int h = display->waterfall.ruler_h;

    fill_rect_rgb565(display, 0, top, w, h, rgb565(5, 9, 14));
    for (int i = 0; i <= 8; i++) {
        int x = (i == 8) ? (w - 1) : (w * i) / 8;
        int tick_h = (i == 0 || i == 4 || i == 8) ? h : h / 2;
        draw_vline(display, x, top + h - tick_h, tick_h, rgb565(54, 82, 92));
    }
    draw_hline(display, 0, top + h - 1, w, rgb565(30, 45, 54));
}

static void mark_waterfall_dirty(app_display_t *display)
{
    display->waterfall.dirty = true;
}

static void format_adsb_list_locked(app_display_t *display, char *buf, size_t len)
{
    int64_t now_ms = esp_timer_get_time() / 1000;
    size_t used = 0;
    if (display->adsb.aircraft_used == 0) {
        snprintf(buf, len, "Waiting for DF17/18 messages");
        return;
    }

    buf[0] = '\0';
    for (uint32_t i = 0; i < display->adsb.aircraft_used && used + 1 < len; i++) {
        const app_display_adsb_aircraft_t *a = &display->adsb.aircraft[i];
        int seen_s = 0;
        if (a->last_seen_ms > 0 && now_ms >= a->last_seen_ms) {
            seen_s = (int)((now_ms - a->last_seen_ms) / 1000);
        }

        char alt[24];
        char speed[24];
        char heading[24];
        char position[48];
        snprintf(alt, sizeof(alt), "%s", a->has_altitude ? "" : "alt --");
        if (a->has_altitude) {
            snprintf(alt, sizeof(alt), "%d ft", a->altitude_feet);
        }
        snprintf(speed, sizeof(speed), "%s", a->has_speed ? "" : "spd --");
        if (a->has_speed) {
            snprintf(speed, sizeof(speed), "%.0f kt", (double)a->speed_knots);
        }
        snprintf(heading, sizeof(heading), "%s", a->has_heading ? "" : "hdg --");
        if (a->has_heading) {
            snprintf(heading, sizeof(heading), "%.0f deg", (double)a->heading_degrees);
        }
        if (a->has_position) {
            snprintf(position, sizeof(position), "%.4f, %.4f", a->latitude, a->longitude);
        } else {
            snprintf(position, sizeof(position), "position pending");
        }

        int written = snprintf(buf + used,
                               len - used,
                               "%s%s%s%s\n%s | %s | %s\n%s | %ds | %lu msg%s",
                               (i == 0) ? "" : "\n",
                               a->callsign[0] ? a->callsign : a->icao,
                               a->callsign[0] ? "  " : "",
                               a->callsign[0] ? a->icao : "",
                               alt,
                               speed,
                               heading,
                               position,
                               seen_s,
                               (unsigned long)a->messages,
                               (i + 1 < display->adsb.aircraft_used) ? "\n" : "");
        if (written < 0) {
            break;
        }
        used += MIN((size_t)written, len - used - 1);
    }
}

static void format_sensor_list_locked(app_display_t *display, char *buf, size_t len)
{
    size_t used = 0;
    if (display->sensor.event_count == 0) {
        snprintf(buf, len, "Waiting for OOK/ASK frames");
        return;
    }

    buf[0] = '\0';
    for (uint32_t i = 0; i < display->sensor.event_count && used + 1 < len; i++) {
        const app_display_sensor_event_t *event = &display->sensor.events[i];
        char freq[24];
        format_freq(freq, sizeof(freq), event->frequency_hz, 3);
        int seen_s = 0;
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (event->last_seen_ms > 0 && now_ms >= event->last_seen_ms) {
            seen_s = (int)((now_ms - event->last_seen_ms) / 1000);
        }
        int written = snprintf(buf + used,
                               len - used,
                               "%s%s  %s  x%llu\n%s | sig %u%% | %ds\n%s%s",
                               (i == 0) ? "" : "\n",
                               event->model[0] ? event->model : "OOK/ASK",
                               event->id[0] ? event->id : "raw",
                               (unsigned long long)event->count,
                               freq,
                               (unsigned)event->signal_level,
                               seen_s,
                               event->data,
                               (i + 1 < display->sensor.event_count) ? "\n" : "");
        if (written < 0) {
            break;
        }
        used += MIN((size_t)written, len - used - 1);
    }
}

static void format_bytes(char *buf, size_t len, uint64_t bytes)
{
    if (bytes >= 1024ULL * 1024ULL) {
        snprintf(buf, len, "%.2f MB", (double)bytes / (1024.0 * 1024.0));
    } else if (bytes >= 1024ULL) {
        snprintf(buf, len, "%.1f KB", (double)bytes / 1024.0);
    } else {
        snprintf(buf, len, "%llu B", (unsigned long long)bytes);
    }
}

static void refresh_metrics_locked(app_display_t *display)
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
    char adsb_messages[32];
    char adsb_candidates[32];
    char adsb_count[32];
    char scanner_range[64];
    char scanner_center[32];
    char scanner_hop[24];
    char scanner_lines[24];
    char sensor_center[32];
    char sensor_rate[32];
    char sensor_frames[32];
    char sensor_decoded[32];
    char sensor_count[32];
    char sensor_signal[16];
    char record_center[32];
    char record_rate[32];
    char record_bytes[32];
    char record_elapsed[32];
    char record_storage[32];
    format_freq(freq, sizeof(freq), get_active_center_freq_locked(display), 3);
    snprintf(rate, sizeof(rate), "%.3f MSPS", (double)display->sample_rate_hz / 1000000.0);
    if (display->waterfall.marker_valid) {
        format_freq(marker, sizeof(marker), display->waterfall.marker_hz, 3);
    } else {
        snprintf(marker, sizeof(marker), "Tap spectrum");
    }
    snprintf(fm_audio_rate, sizeof(fm_audio_rate), "%lu Hz", (unsigned long)display->fm.audio_rate_hz);
    if (!display->fm.audio_output_enabled) {
        snprintf(fm_audio_output, sizeof(fm_audio_output), "Off");
    } else {
        snprintf(fm_audio_output, sizeof(fm_audio_output), "%s", display->fm.audio_output_ready ? "Speaker ready" : "Speaker failed");
    }
    snprintf(fm_audio_level, sizeof(fm_audio_level), "%u%%", (unsigned)display->fm.audio_level);
    snprintf(adsb_messages, sizeof(adsb_messages), "%llu valid", (unsigned long long)display->adsb.valid_messages);
    snprintf(adsb_candidates, sizeof(adsb_candidates), "%llu cand", (unsigned long long)display->adsb.candidates);
    snprintf(adsb_count, sizeof(adsb_count), "%lu tracked", (unsigned long)display->adsb.aircraft_count);
    char scanner_start[24];
    char scanner_stop[24];
    format_freq(scanner_start, sizeof(scanner_start), display->scanner.start_hz, 1);
    format_freq(scanner_stop, sizeof(scanner_stop), display->scanner.stop_hz, 1);
    snprintf(scanner_range, sizeof(scanner_range), "%s - %s", scanner_start, scanner_stop);
    format_freq(scanner_center, sizeof(scanner_center), display->scanner.center_hz, 3);
    if (display->scanner.hop_count > 0) {
        snprintf(scanner_hop,
                 sizeof(scanner_hop),
                 "%lu/%lu",
                 (unsigned long)(display->scanner.hop_index + 1U),
                 (unsigned long)display->scanner.hop_count);
    } else {
        snprintf(scanner_hop, sizeof(scanner_hop), "--");
    }
    snprintf(scanner_lines, sizeof(scanner_lines), "%lu sweeps", (unsigned long)display->scanner.completed_lines);
    format_freq(sensor_center, sizeof(sensor_center), display->sensor.center_hz, 3);
    snprintf(sensor_rate, sizeof(sensor_rate), "%.3f MSPS", (double)display->sensor.sample_rate_hz / 1000000.0);
    snprintf(sensor_frames, sizeof(sensor_frames), "%llu frames", (unsigned long long)display->sensor.total_frames);
    snprintf(sensor_decoded, sizeof(sensor_decoded), "%llu decoded", (unsigned long long)display->sensor.decoded_messages);
    snprintf(sensor_count, sizeof(sensor_count), "%lu devices", (unsigned long)display->sensor.sensor_count);
    snprintf(sensor_signal, sizeof(sensor_signal), "%u%%", (unsigned)display->sensor.signal_level);
    format_freq(record_center, sizeof(record_center), display->record.center_hz, 3);
    snprintf(record_rate, sizeof(record_rate), "%.3f MSPS", (double)display->record.sample_rate_hz / 1000000.0);
    format_bytes(record_bytes, sizeof(record_bytes), display->record.bytes_written);
    snprintf(record_elapsed, sizeof(record_elapsed), "%lus", (unsigned long)display->record.elapsed_seconds);
    snprintf(record_storage,
             sizeof(record_storage),
             "%s",
             display->record.storage_ready ? (display->record.active ? "Recording" : "Ready") : "No storage");

    if (display->footer_label) {
        if (display->active_page == APP_DISPLAY_PAGE_SCANNER) {
            snprintf(display->footer_text,
                     sizeof(display->footer_text),
                     "%s | %s | %s",
                     scanner_range,
                     rate,
                     scanner_lines);
        } else if (display->active_page == APP_DISPLAY_PAGE_SENSORS) {
            snprintf(display->footer_text,
                     sizeof(display->footer_text),
                     "%s | %s | %s",
                     sensor_center,
                     sensor_rate,
                     sensor_decoded);
        } else if (display->active_page == APP_DISPLAY_PAGE_RECORD) {
            snprintf(display->footer_text,
                     sizeof(display->footer_text),
                     "%s | %s | %s %s",
                     record_center,
                     record_rate,
                     record_storage,
                     record_bytes);
        } else {
            snprintf(display->footer_text,
                     sizeof(display->footer_text),
                     "%s | %s | FM %s %u%%",
                     freq,
                     rate,
                     display->fm.mode,
                     (unsigned)display->fm.audio_level);
        }
        lv_label_set_text(display->footer_label, display->footer_text);
    }
    if (display->status_value_label) {
        lv_label_set_text(display->status_value_label, display->status);
    }
    if (display->freq_value_label) {
        lv_label_set_text(display->freq_value_label, freq);
    }
    if (display->fm.center_large_label) {
        lv_label_set_text(display->fm.center_large_label, freq);
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
    if (display->fm.mode_value_label) {
        lv_label_set_text(display->fm.mode_value_label, display->fm.mode);
    }
    if (display->fm.audio_rate_value_label) {
        lv_label_set_text(display->fm.audio_rate_value_label, fm_audio_rate);
    }
    if (display->fm.audio_output_value_label) {
        lv_label_set_text(display->fm.audio_output_value_label, fm_audio_output);
    }
    if (display->fm.status_large_label) {
        char fm_status[96];
        snprintf(fm_status,
                 sizeof(fm_status),
                 "%s / audio %s / level %u%%",
                 display->fm.mode,
                 display->fm.audio_output_ready ? "ready" : (display->fm.audio_output_enabled ? "unavailable" : "off"),
                 (unsigned)display->fm.audio_level);
        lv_label_set_text(display->fm.status_large_label, fm_status);
    }
    if (display->fm.audio_level_bar) {
        lv_bar_set_value(display->fm.audio_level_bar, display->fm.audio_level, LV_ANIM_OFF);
    }
    if (display->fm.audio_level_label) {
        lv_label_set_text(display->fm.audio_level_label, fm_audio_level);
    }
    if (display->adsb.status_label) {
        lv_label_set_text(display->adsb.status_label, display->adsb.active ? "Listening on 1090 MHz" : "ADS-B paused");
    }
    if (display->adsb.count_value_label) {
        lv_label_set_text(display->adsb.count_value_label, adsb_count);
    }
    if (display->adsb.messages_value_label) {
        lv_label_set_text(display->adsb.messages_value_label, adsb_messages);
    }
    if (display->adsb.candidates_value_label) {
        lv_label_set_text(display->adsb.candidates_value_label, adsb_candidates);
    }
    if (display->adsb.list_label) {
        char list[1200];
        format_adsb_list_locked(display, list, sizeof(list));
        lv_label_set_text(display->adsb.list_label, list);
    }
    if (display->scanner.range_value_label) {
        lv_label_set_text(display->scanner.range_value_label, scanner_range);
    }
    if (display->scanner.center_value_label) {
        lv_label_set_text(display->scanner.center_value_label,
                          display->scanner.active ? scanner_center : "Paused");
    }
    if (display->scanner.hop_value_label) {
        lv_label_set_text(display->scanner.hop_value_label, scanner_hop);
    }
    if (display->scanner.lines_value_label) {
        lv_label_set_text(display->scanner.lines_value_label, scanner_lines);
    }
    if (display->sensor.status_label) {
        lv_label_set_text(display->sensor.status_label,
                          display->sensor.active ? "Listening for 433/315 MHz OOK/ASK frames" : "Sensor decoder paused");
    }
    if (display->sensor.center_value_label) {
        lv_label_set_text(display->sensor.center_value_label, sensor_center);
    }
    if (display->sensor.rate_value_label) {
        lv_label_set_text(display->sensor.rate_value_label, sensor_rate);
    }
    if (display->sensor.frames_value_label) {
        lv_label_set_text(display->sensor.frames_value_label, sensor_frames);
    }
    if (display->sensor.decoded_value_label) {
        lv_label_set_text(display->sensor.decoded_value_label, sensor_decoded);
    }
    if (display->sensor.sensors_value_label) {
        lv_label_set_text(display->sensor.sensors_value_label, sensor_count);
    }
    if (display->sensor.signal_value_label) {
        lv_label_set_text(display->sensor.signal_value_label, sensor_signal);
    }
    if (display->sensor.signal_bar) {
        lv_bar_set_value(display->sensor.signal_bar, display->sensor.signal_level, LV_ANIM_OFF);
    }
    if (display->sensor.list_label) {
        char sensor_list[1000];
        format_sensor_list_locked(display, sensor_list, sizeof(sensor_list));
        lv_label_set_text(display->sensor.list_label, sensor_list);
    }
    if (display->record.status_label) {
        char status[128];
        snprintf(status,
                 sizeof(status),
                 "%s / %s / %s",
                 record_storage,
                 record_bytes,
                 record_elapsed);
        lv_label_set_text(display->record.status_label, status);
    }
    if (display->record.path_label) {
        lv_label_set_text(display->record.path_label, display->record.path[0] ? display->record.path : "/spiffs unavailable");
    }
    if (display->record.center_value_label) {
        lv_label_set_text(display->record.center_value_label, record_center);
    }
    if (display->record.rate_value_label) {
        lv_label_set_text(display->record.rate_value_label, record_rate);
    }
    if (display->record.bytes_value_label) {
        lv_label_set_text(display->record.bytes_value_label, record_bytes);
    }
    if (display->record.elapsed_value_label) {
        lv_label_set_text(display->record.elapsed_value_label, record_elapsed);
    }
    if (display->record.storage_value_label) {
        lv_label_set_text(display->record.storage_value_label, record_storage);
    }
    if (display->record.progress_bar) {
        int progress = 0;
        if (display->record.max_bytes > 0) {
            progress = (int)MIN(100ULL, (display->record.bytes_written * 100ULL) / display->record.max_bytes);
        }
        lv_bar_set_value(display->record.progress_bar, progress, LV_ANIM_OFF);
    }

    display->status_dirty = false;
}

static void update_lvgl_objects_locked(app_display_t *display)
{
    if (display->shutting_down) {
        return;
    }
    refresh_metrics_locked(display);
    if (display->waterfall.dirty && display->waterfall.img) {
        lv_obj_invalidate(display->waterfall.img);
        display->waterfall.dirty = false;
    }
    if (display->fm.waveform_dirty && display->fm.waveform_img) {
        lv_obj_invalidate(display->fm.waveform_img);
        display->fm.waveform_dirty = false;
    }
    if (display->scanner.dirty && display->scanner.img) {
        lv_obj_invalidate(display->scanner.img);
        display->scanner.dirty = false;
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
    app_display_t *display = (app_display_t *)arg;
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

static void clear_content(app_display_t *display)
{
    close_freq_dialog(display);
    if (display->content) {
        lv_obj_clean(display->content);
    }
    display->header_button = NULL;
    display->waterfall.img = NULL;
    display->side_panel = NULL;
    display->freq_row = NULL;
    display->status_value_label = NULL;
    display->freq_value_label = NULL;
    display->rate_value_label = NULL;
    display->marker_value_label = NULL;
    display->touch_value_label = NULL;
    display->fm.mode_value_label = NULL;
    display->fm.audio_rate_value_label = NULL;
    display->fm.audio_output_value_label = NULL;
    display->fm.audio_level_bar = NULL;
    display->fm.audio_level_label = NULL;
    display->fm.center_large_label = NULL;
    display->fm.status_large_label = NULL;
    display->fm.waveform_img = NULL;
    display->adsb.status_label = NULL;
    display->adsb.count_value_label = NULL;
    display->adsb.messages_value_label = NULL;
    display->adsb.candidates_value_label = NULL;
    display->adsb.list_label = NULL;
    display->scanner.img = NULL;
    display->scanner.range_value_label = NULL;
    display->scanner.center_value_label = NULL;
    display->scanner.hop_value_label = NULL;
    display->scanner.lines_value_label = NULL;
    display->sensor.status_label = NULL;
    display->sensor.list_label = NULL;
    display->sensor.center_value_label = NULL;
    display->sensor.rate_value_label = NULL;
    display->sensor.frames_value_label = NULL;
    display->sensor.decoded_value_label = NULL;
    display->sensor.sensors_value_label = NULL;
    display->sensor.signal_value_label = NULL;
    display->sensor.signal_bar = NULL;
    display->record.status_label = NULL;
    display->record.path_label = NULL;
    display->record.center_value_label = NULL;
    display->record.rate_value_label = NULL;
    display->record.bytes_value_label = NULL;
    display->record.elapsed_value_label = NULL;
    display->record.storage_value_label = NULL;
    display->record.progress_bar = NULL;
    display->freq_dialog = NULL;
    display->freq_textarea = NULL;
    display->freq_digit_roller = NULL;
    display->freq_error_label = NULL;
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

static void switch_page_locked(app_display_t *display, app_display_page_t page);
static void freq_row_event_cb(lv_event_t *event);

static void close_freq_dialog(app_display_t *display)
{
    if (display->freq_dialog) {
        lv_obj_delete(display->freq_dialog);
        display->freq_dialog = NULL;
        display->freq_textarea = NULL;
        display->freq_digit_roller = NULL;
        display->freq_error_label = NULL;
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
                               app_display_t *display)
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
    app_display_t *display = lv_event_get_user_data(event);
    if (!display || !display->freq_textarea || !display->freq_digit_roller) {
        return;
    }

    char digit[2] = {0};
    lv_roller_get_selected_str(display->freq_digit_roller, digit, sizeof(digit));
    lv_textarea_add_text(display->freq_textarea, digit);
    set_freq_dialog_error_locked(display, NULL);
}

static void freq_dot_event_cb(lv_event_t *event)
{
    app_display_t *display = lv_event_get_user_data(event);
    if (!display || !display->freq_textarea) {
        return;
    }

    const char *text = lv_textarea_get_text(display->freq_textarea);
    if (text && strchr(text, '.') == NULL) {
        lv_textarea_add_text(display->freq_textarea, ".");
        set_freq_dialog_error_locked(display, NULL);
    }
}

static void freq_backspace_event_cb(lv_event_t *event)
{
    app_display_t *display = lv_event_get_user_data(event);
    if (display && display->freq_textarea) {
        lv_textarea_delete_char(display->freq_textarea);
        set_freq_dialog_error_locked(display, NULL);
    }
}

static void freq_cancel_event_cb(lv_event_t *event)
{
    app_display_t *display = lv_event_get_user_data(event);
    if (display) {
        close_freq_dialog(display);
    }
}

static void freq_ok_event_cb(lv_event_t *event)
{
    app_display_t *display = lv_event_get_user_data(event);
    if (!display || !display->freq_textarea) {
        return;
    }

    uint32_t next_hz = 0;
    if (parse_mhz_input(lv_textarea_get_text(display->freq_textarea), &next_hz)) {
        set_active_center_freq_locked(display, next_hz);
        close_freq_dialog(display);
    } else {
        set_freq_dialog_error_locked(display, "Enter a valid 24-1766 MHz value");
    }
}

static void open_freq_dialog_locked(app_display_t *display)
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
    lv_textarea_set_placeholder_text(ta, "");
    lv_obj_set_style_text_font(ta, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ta, color_hex(0xf4f8fb), 0);
    lv_obj_set_style_bg_color(ta, color_hex(0x080c10), 0);
    lv_obj_set_style_bg_opa(ta, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(ta, color_hex(0x2e3a44), 0);
    lv_obj_set_style_border_width(ta, 1, 0);
    lv_obj_set_style_radius(ta, 6, 0);
    char input[24];
    format_freq_input(input, sizeof(input), get_active_center_freq_locked(display));
    lv_textarea_set_text(ta, input);
    lv_textarea_set_cursor_pos(ta, LV_TEXTAREA_CURSOR_LAST);
    lv_obj_add_state(ta, LV_STATE_FOCUSED);

    display->freq_error_label = create_label(panel, "", 12, color_hex(0xff8a8a), false);
    lv_obj_set_width(display->freq_error_label, dialog_w - 32);
    lv_obj_align(display->freq_error_label, LV_ALIGN_TOP_LEFT, 16, 84);

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
    app_display_t *display = lv_event_get_user_data(event);
    if (!display) {
        return;
    }
    open_freq_dialog_locked(display);
}

static void tile_event_cb(lv_event_t *event)
{
    app_display_t *display = lv_event_get_user_data(event);
    app_display_page_t page = (app_display_page_t)(uintptr_t)lv_obj_get_user_data(lv_event_get_target(event));
    ESP_LOGI(TAG, "Tile clicked: %s", feature_for_page(page)->title);
    switch_page_locked(display, page);
}

static void back_event_cb(lv_event_t *event)
{
    app_display_t *display = lv_event_get_user_data(event);
    ESP_LOGI(TAG, "Back to home");
    switch_page_locked(display, APP_DISPLAY_PAGE_HOME);
}

static void waterfall_event_cb(lv_event_t *event)
{
    app_display_t *display = lv_event_get_user_data(event);
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
        display->waterfall.swipe_tracking = (rel_y >= 0 && rel_y < h / 2);
        display->waterfall.swipe_moved = false;
        display->waterfall.suppress_next_click = false;
        display->waterfall.swipe_start_x = point.x;
        display->waterfall.swipe_start_y = point.y;
        return;
    }

    if (code == LV_EVENT_PRESSING && display->waterfall.swipe_tracking) {
        int dx = point.x - display->waterfall.swipe_start_x;
        int dy = point.y - display->waterfall.swipe_start_y;
        if (abs(dx) >= WATERFALL_SWIPE_MIN_PX && abs(dx) > abs(dy)) {
            int steps = abs(dx) / WATERFALL_SWIPE_MIN_PX;
            int direction = (dx < 0) ? 1 : -1;
            step_center_freq_locked(display, direction, steps);
            display->waterfall.swipe_start_x += (dx > 0) ? (steps * WATERFALL_SWIPE_MIN_PX) : -(steps * WATERFALL_SWIPE_MIN_PX);
            display->waterfall.swipe_start_y = point.y;
            display->waterfall.swipe_moved = true;
            display->waterfall.suppress_next_click = true;
        }
        return;
    }

    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        display->waterfall.swipe_tracking = false;
        return;
    }

    if (code != LV_EVENT_CLICKED) {
        return;
    }
    if (display->waterfall.suppress_next_click || display->waterfall.swipe_moved) {
        display->waterfall.suppress_next_click = false;
        display->waterfall.swipe_moved = false;
        return;
    }

    int64_t left = (int64_t)display->waterfall.center_hz - (int64_t)display->sample_rate_hz / 2;
    int64_t marker_hz = left + ((int64_t)display->sample_rate_hz * rel_x) / w;
    if (marker_hz < 0) {
        marker_hz = 0;
    } else if (marker_hz > UINT32_MAX) {
        marker_hz = UINT32_MAX;
    }
    uint32_t marker = (uint32_t)marker_hz;
    ESP_LOGI(TAG, "Waterfall marker touch x=%d -> %lu Hz", rel_x, (unsigned long)marker);
    display->waterfall.marker_hz = marker;
    display->waterfall.marker_valid = true;
    display->status_dirty = true;
}

static void create_home_locked(app_display_t *display)
{
    clear_content(display);
    display->active_page = APP_DISPLAY_PAGE_HOME;
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

static void create_fm_visual_locked(app_display_t *display, lv_obj_t *parent, const feature_def_t *feature)
{
    (void)feature;
    lv_obj_t *waveform = lv_image_create(parent);
    display->fm.waveform_img = waveform;
    lv_image_set_src(waveform, &display->fm.waveform_dsc);
    lv_obj_set_size(waveform, display->fm.waveform_w, display->fm.waveform_h);
    lv_obj_center(waveform);
    lv_obj_remove_flag(waveform, LV_OBJ_FLAG_SCROLLABLE);
    draw_fm_waveform_background(display);
    display->fm.waveform_dirty = true;
}

static void create_fm_side_panel_locked(app_display_t *display, const feature_def_t *feature)
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
    display->fm.mode_value_label = metric_row(side, "Mode", "", NULL);
    display->rate_value_label = metric_row(side, "IQ rate", "", NULL);
    display->fm.audio_rate_value_label = metric_row(side, "Audio rate", "", NULL);
    display->fm.audio_output_value_label = metric_row(side, "Audio output", "", NULL);

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

    display->fm.audio_level_bar = lv_bar_create(level_box);
    lv_obj_set_size(display->fm.audio_level_bar, LV_PCT(100), 12);
    lv_obj_align(display->fm.audio_level_bar, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_bar_set_range(display->fm.audio_level_bar, 0, 100);
    lv_obj_set_style_bg_color(display->fm.audio_level_bar, color_hex(0x172018), 0);
    lv_obj_set_style_bg_color(display->fm.audio_level_bar, feature->accent, LV_PART_INDICATOR);

    display->fm.audio_level_label = create_label(level_box, "", 14, color_hex(0xeef6fb), true);
    lv_obj_set_width(display->fm.audio_level_label, 56);
    lv_obj_align(display->fm.audio_level_label, LV_ALIGN_TOP_RIGHT, 0, 0);
}

static void create_adsb_visual_locked(app_display_t *display, lv_obj_t *parent, const feature_def_t *feature)
{
    lv_obj_t *panel = lv_obj_create(parent);
    style_button(panel, color_hex(0x0b1115), color_hex(0x263b45), 8);
    lv_obj_set_size(panel, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_pad_all(panel, 14, 0);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(panel, 10, 0);

    lv_obj_t *accent = lv_obj_create(panel);
    style_button(accent, feature->accent, feature->accent, 3);
    lv_obj_set_size(accent, 96, 6);

    lv_obj_t *title = create_label(panel, "1090 MHz Mode S / ADS-B", 18, color_hex(0xf3f8fb), true);
    lv_obj_set_width(title, LV_PCT(100));

    display->adsb.status_label = create_label(panel, "", 14, color_hex(0x9fb2bf), false);
    lv_obj_set_width(display->adsb.status_label, LV_PCT(100));

    display->adsb.list_label = create_label(panel, "", 14, color_hex(0xe3edf4), false);
    lv_obj_set_width(display->adsb.list_label, LV_PCT(100));
    lv_obj_set_flex_grow(display->adsb.list_label, 1);
    lv_label_set_long_mode(display->adsb.list_label, LV_LABEL_LONG_MODE_WRAP);
}

static void create_adsb_side_panel_locked(app_display_t *display, const feature_def_t *feature)
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

    display->freq_value_label = metric_row(side, "Center", "", NULL);
    display->rate_value_label = metric_row(side, "Sample rate", "", NULL);
    display->adsb.count_value_label = metric_row(side, "Aircraft", "", NULL);
    display->adsb.messages_value_label = metric_row(side, "Messages", "", NULL);
    display->adsb.candidates_value_label = metric_row(side, "Candidates", "", NULL);
}

static void create_scanner_visual_locked(app_display_t *display, lv_obj_t *parent, const feature_def_t *feature)
{
    (void)feature;
    lv_obj_t *img = lv_image_create(parent);
    display->scanner.img = img;
    lv_image_set_src(img, &display->scanner.dsc);
    lv_obj_set_size(img, display->scanner.w, display->scanner.h);
    lv_obj_center(img);
    lv_obj_remove_flag(img, LV_OBJ_FLAG_SCROLLABLE);
    draw_scanner_background(display);
    display->scanner.dirty = true;
}

static void create_scanner_side_panel_locked(app_display_t *display, const feature_def_t *feature)
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

    display->scanner.range_value_label = metric_row(side, "Range", "", NULL);
    display->scanner.center_value_label = metric_row(side, "Current tune", "", NULL);
    display->rate_value_label = metric_row(side, "Sample rate", "", NULL);
    display->scanner.hop_value_label = metric_row(side, "Hop", "", NULL);
    display->scanner.lines_value_label = metric_row(side, "Completed", "", NULL);
}

static void create_sensor_visual_locked(app_display_t *display, lv_obj_t *parent, const feature_def_t *feature)
{
    lv_obj_t *panel = lv_obj_create(parent);
    style_button(panel, color_hex(0x0b1110), color_hex(0x243b35), 8);
    lv_obj_set_size(panel, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_pad_all(panel, 14, 0);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(panel, 10, 0);

    lv_obj_t *accent = lv_obj_create(panel);
    style_button(accent, feature->accent, feature->accent, 3);
    lv_obj_set_size(accent, 96, 6);

    lv_obj_t *title = create_label(panel, "OOK/ASK Sensor Frames", 18, color_hex(0xf3f8fb), true);
    lv_obj_set_width(title, LV_PCT(100));

    display->sensor.status_label = create_label(panel, "", 14, color_hex(0x9fb2bf), false);
    lv_obj_set_width(display->sensor.status_label, LV_PCT(100));

    display->sensor.list_label = create_label(panel, "", 14, color_hex(0xe3edf4), false);
    lv_obj_set_width(display->sensor.list_label, LV_PCT(100));
    lv_obj_set_flex_grow(display->sensor.list_label, 1);
    lv_label_set_long_mode(display->sensor.list_label, LV_LABEL_LONG_MODE_WRAP);
}

static void create_sensor_side_panel_locked(app_display_t *display, const feature_def_t *feature)
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

    display->sensor.center_value_label = metric_row(side, "Center", "", &display->freq_row);
    if (display->freq_row) {
        lv_obj_add_flag(display->freq_row, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(display->freq_row, freq_row_event_cb, LV_EVENT_CLICKED, display);
    }
    display->sensor.rate_value_label = metric_row(side, "Sample rate", "", NULL);
    display->sensor.frames_value_label = metric_row(side, "Frames", "", NULL);
    display->sensor.decoded_value_label = metric_row(side, "Decoded", "", NULL);
    display->sensor.sensors_value_label = metric_row(side, "Devices", "", NULL);

    lv_obj_t *level_box = lv_obj_create(side);
    style_button(level_box, color_hex(0x0b0f13), color_hex(0x27313a), 6);
    lv_obj_set_width(level_box, LV_PCT(100));
    lv_obj_set_height(level_box, 62);
    lv_obj_set_style_pad_left(level_box, 12, 0);
    lv_obj_set_style_pad_right(level_box, 12, 0);
    lv_obj_set_style_pad_top(level_box, 8, 0);
    lv_obj_set_style_pad_bottom(level_box, 8, 0);

    lv_obj_t *name_label = create_label(level_box, "Signal", 12, color_hex(0x8293a0), false);
    lv_obj_set_width(name_label, LV_PCT(100));
    lv_obj_align(name_label, LV_ALIGN_TOP_LEFT, 0, 0);

    display->sensor.signal_bar = lv_bar_create(level_box);
    lv_obj_set_size(display->sensor.signal_bar, LV_PCT(100), 12);
    lv_obj_align(display->sensor.signal_bar, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_bar_set_range(display->sensor.signal_bar, 0, 100);
    lv_obj_set_style_bg_color(display->sensor.signal_bar, color_hex(0x172018), 0);
    lv_obj_set_style_bg_color(display->sensor.signal_bar, feature->accent, LV_PART_INDICATOR);

    display->sensor.signal_value_label = create_label(level_box, "", 14, color_hex(0xeef6fb), true);
    lv_obj_set_width(display->sensor.signal_value_label, 56);
    lv_obj_align(display->sensor.signal_value_label, LV_ALIGN_TOP_RIGHT, 0, 0);
}

static void create_record_visual_locked(app_display_t *display, lv_obj_t *parent, const feature_def_t *feature)
{
    lv_obj_t *panel = lv_obj_create(parent);
    style_button(panel, color_hex(0x110c10), color_hex(0x3d2630), 8);
    lv_obj_set_size(panel, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_pad_all(panel, 14, 0);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(panel, 12, 0);

    lv_obj_t *accent = lv_obj_create(panel);
    style_button(accent, feature->accent, feature->accent, 3);
    lv_obj_set_size(accent, 96, 6);

    lv_obj_t *title = create_label(panel, "Raw I/Q Capture", 18, color_hex(0xf3f8fb), true);
    lv_obj_set_width(title, LV_PCT(100));

    display->record.status_label = create_label(panel, "", 15, color_hex(0xe9eef4), true);
    lv_obj_set_width(display->record.status_label, LV_PCT(100));

    display->record.progress_bar = lv_bar_create(panel);
    lv_obj_set_size(display->record.progress_bar, LV_PCT(100), 16);
    lv_bar_set_range(display->record.progress_bar, 0, 100);
    lv_obj_set_style_bg_color(display->record.progress_bar, color_hex(0x20131a), 0);
    lv_obj_set_style_bg_color(display->record.progress_bar, feature->accent, LV_PART_INDICATOR);

    display->record.path_label = create_label(panel, "", 13, color_hex(0xadb8c4), false);
    lv_obj_set_width(display->record.path_label, LV_PCT(100));
    lv_label_set_long_mode(display->record.path_label, LV_LABEL_LONG_MODE_WRAP);
}

static void create_record_side_panel_locked(app_display_t *display, const feature_def_t *feature)
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

    display->record.center_value_label = metric_row(side, "Center", "", &display->freq_row);
    if (display->freq_row) {
        lv_obj_add_flag(display->freq_row, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(display->freq_row, freq_row_event_cb, LV_EVENT_CLICKED, display);
    }
    display->record.rate_value_label = metric_row(side, "Sample rate", "", NULL);
    display->record.storage_value_label = metric_row(side, "Storage", "", NULL);
    display->record.bytes_value_label = metric_row(side, "Written", "", NULL);
    display->record.elapsed_value_label = metric_row(side, "Elapsed", "", NULL);
}

static void create_side_panel_locked(app_display_t *display, const feature_def_t *feature, bool live_metrics)
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

static void create_feature_locked(app_display_t *display, app_display_page_t page)
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

    if (page == APP_DISPLAY_PAGE_WATERFALL) {
        lv_obj_t *img = lv_image_create(visual);
        display->waterfall.img = img;
        lv_image_set_src(img, &display->waterfall.dsc);
        lv_obj_set_size(img, display->waterfall.w, display->waterfall.h);
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
    } else if (page == APP_DISPLAY_PAGE_FM) {
        create_fm_visual_locked(display, visual, feature);
        if (side_w > 0) {
            create_fm_side_panel_locked(display, feature);
        }
    } else if (page == APP_DISPLAY_PAGE_ADSB) {
        create_adsb_visual_locked(display, visual, feature);
        if (side_w > 0) {
            create_adsb_side_panel_locked(display, feature);
        }
    } else if (page == APP_DISPLAY_PAGE_SCANNER) {
        create_scanner_visual_locked(display, visual, feature);
        if (side_w > 0) {
            create_scanner_side_panel_locked(display, feature);
        }
    } else if (page == APP_DISPLAY_PAGE_SENSORS) {
        create_sensor_visual_locked(display, visual, feature);
        if (side_w > 0) {
            create_sensor_side_panel_locked(display, feature);
        }
    } else if (page == APP_DISPLAY_PAGE_RECORD) {
        create_record_visual_locked(display, visual, feature);
        if (side_w > 0) {
            create_record_side_panel_locked(display, feature);
        }
    } else {
        const char *body = "This page mirrors the desktop app navigation. Decoder and tuner logic are not implemented on this ESP32 target yet.";
        create_placeholder_visual(visual, feature, body, feature->accent);
        if (side_w > 0) {
            create_side_panel_locked(display, feature, false);
        }
    }

    tune_page_center_freq_locked(display, page);
    display->status_dirty = true;
}

static void switch_page_locked(app_display_t *display, app_display_page_t page)
{
    if (page == APP_DISPLAY_PAGE_HOME) {
        create_home_locked(display);
    } else {
        create_feature_locked(display, page);
    }
    if (display->page_cb) {
        display->page_cb(display->active_page, display->page_cb_ctx);
    }
}

static esp_err_t create_ui(app_display_t *display)
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

static esp_err_t setup_rgb_panel(app_display_t *display)
{
    const app_display_config_t *config = &display->cfg;
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
static void touch_apply_transform(app_display_t *display, uint16_t *x, uint16_t *y)
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

static esp_err_t touch_read_reg16(app_display_t *display, uint16_t reg, uint8_t *data, size_t len)
{
    uint8_t reg_buf[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xff)};
    return i2c_master_transmit_receive(display->touch_dev, reg_buf, sizeof(reg_buf), data, len, TOUCH_TIMEOUT_MS);
}

static esp_err_t touch_write_reg16_u8(app_display_t *display, uint16_t reg, uint8_t value)
{
    uint8_t buf[3] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xff), value};
    return i2c_master_transmit(display->touch_dev, buf, sizeof(buf), TOUCH_TIMEOUT_MS);
}

static esp_err_t touch_read_reg8(app_display_t *display, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(display->touch_dev, &reg, 1, data, len, TOUCH_TIMEOUT_MS);
}

static void scan_touch_i2c_bus(app_display_t *display)
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

static bool gt911_get_point(app_display_t *display, uint16_t *x, uint16_t *y)
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

static bool ft5x06_get_point(app_display_t *display, uint16_t *x, uint16_t *y)
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
    app_display_t *display = lv_indev_get_user_data(indev);
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

static esp_err_t setup_touch(app_display_t *display)
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

static esp_err_t setup_lvgl(app_display_t *display)
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

static esp_err_t alloc_waterfall_image(app_display_t *display)
{
    int side_panel = (display->cfg.h_res >= 620) ? ((display->cfg.h_res >= 760) ? 250 : 210) : 0;
    int available_w = display->cfg.h_res - 32 - side_panel - (side_panel ? 14 : 0);
    int available_h = display->cfg.v_res - 136;
    display->waterfall.w = available_w;
    display->waterfall.h = available_h;
    if (display->waterfall.w < 240) {
        display->waterfall.w = display->cfg.h_res - 32;
    }
    if (display->waterfall.h < 160) {
        display->waterfall.h = display->cfg.v_res - 122;
    }
    display->waterfall.spectrum_h = (display->waterfall.h >= 320) ? 104 : 78;
    display->waterfall.ruler_h = 18;
    if (display->waterfall.spectrum_h + display->waterfall.ruler_h >= display->waterfall.h - 24) {
        display->waterfall.spectrum_h = display->waterfall.h / 3;
        display->waterfall.ruler_h = 14;
    }

    size_t bytes = (size_t)display->waterfall.w * display->waterfall.h * sizeof(uint16_t);
    display->waterfall.pixels = heap_caps_aligned_alloc(WATERFALL_IMAGE_ALIGN, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!display->waterfall.pixels) {
        display->waterfall.pixels = heap_caps_aligned_alloc(WATERFALL_IMAGE_ALIGN, bytes, MALLOC_CAP_8BIT);
    }
    ESP_RETURN_ON_FALSE(display->waterfall.pixels, ESP_ERR_NO_MEM, TAG, "waterfall pixels");
    memset(display->waterfall.pixels, 0, bytes);

    display->waterfall.dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    display->waterfall.dsc.header.cf = LV_COLOR_FORMAT_RGB565;
    display->waterfall.dsc.header.w = display->waterfall.w;
    display->waterfall.dsc.header.h = display->waterfall.h;
    display->waterfall.dsc.data_size = bytes;
    display->waterfall.dsc.data = (const uint8_t *)display->waterfall.pixels;
    return ESP_OK;
}

static esp_err_t alloc_fm_waveform_image(app_display_t *display)
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

    display->fm.waveform_w = available_w;
    if (display->fm.waveform_w < 120) {
        display->fm.waveform_w = 120;
    }

    display->fm.waveform_h = available_h;
    if (display->fm.waveform_h < 48) {
        display->fm.waveform_h = 48;
    }

    size_t bytes = (size_t)display->fm.waveform_w * display->fm.waveform_h * sizeof(uint16_t);
    display->fm.waveform_pixels = heap_caps_aligned_alloc(WATERFALL_IMAGE_ALIGN,
                                                          bytes,
                                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!display->fm.waveform_pixels) {
        display->fm.waveform_pixels = heap_caps_aligned_alloc(WATERFALL_IMAGE_ALIGN, bytes, MALLOC_CAP_8BIT);
    }
    ESP_RETURN_ON_FALSE(display->fm.waveform_pixels, ESP_ERR_NO_MEM, TAG, "FM waveform pixels");
    memset(display->fm.waveform_pixels, 0, bytes);

    display->fm.waveform_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    display->fm.waveform_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
    display->fm.waveform_dsc.header.w = display->fm.waveform_w;
    display->fm.waveform_dsc.header.h = display->fm.waveform_h;
    display->fm.waveform_dsc.data_size = bytes;
    display->fm.waveform_dsc.data = (const uint8_t *)display->fm.waveform_pixels;
    draw_fm_waveform_background(display);
    return ESP_OK;
}

static esp_err_t alloc_scanner_image(app_display_t *display)
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

    display->scanner.w = available_w;
    if (display->scanner.w < 120) {
        display->scanner.w = 120;
    }
    display->scanner.h = available_h;
    if (display->scanner.h < 80) {
        display->scanner.h = 80;
    }
    display->scanner.plot_y = MIN(SCANNER_HEADER_H, display->scanner.h / 3);
    display->scanner.plot_h = display->scanner.h - display->scanner.plot_y - SCANNER_FOOTER_H;
    if (display->scanner.plot_h < 32) {
        display->scanner.plot_y = 12;
        display->scanner.plot_h = display->scanner.h - display->scanner.plot_y - 8;
    }

    size_t bytes = (size_t)display->scanner.w * display->scanner.h * sizeof(uint16_t);
    display->scanner.pixels = heap_caps_aligned_alloc(WATERFALL_IMAGE_ALIGN,
                                                      bytes,
                                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!display->scanner.pixels) {
        display->scanner.pixels = heap_caps_aligned_alloc(WATERFALL_IMAGE_ALIGN, bytes, MALLOC_CAP_8BIT);
    }
    ESP_RETURN_ON_FALSE(display->scanner.pixels, ESP_ERR_NO_MEM, TAG, "scanner pixels");
    memset(display->scanner.pixels, 0, bytes);

    display->scanner.dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    display->scanner.dsc.header.cf = LV_COLOR_FORMAT_RGB565;
    display->scanner.dsc.header.w = display->scanner.w;
    display->scanner.dsc.header.h = display->scanner.h;
    display->scanner.dsc.data_size = bytes;
    display->scanner.dsc.data = (const uint8_t *)display->scanner.pixels;
    draw_scanner_background(display);
    return ESP_OK;
}

esp_err_t app_display_create(const app_display_config_t *config, app_display_t **out)
{
    ESP_RETURN_ON_FALSE(config && out, ESP_ERR_INVALID_ARG, TAG, "bad args");
    esp_err_t ret = ESP_OK;
    app_display_t *display = calloc(1, sizeof(*display));
    ESP_RETURN_ON_FALSE(display, ESP_ERR_NO_MEM, TAG, "no memory");
    _lock_init(&display->lvgl_api_lock);

    display->cfg = *config;
    display->waterfall.center_hz = CONFIG_RTLSDR_CENTER_FREQ_HZ;
    display->fm.center_hz = CONFIG_RTLSDR_CENTER_FREQ_HZ;
    display->scanner.start_hz = SCANNER_DEFAULT_START_HZ;
    display->scanner.stop_hz = SCANNER_DEFAULT_STOP_HZ;
    display->scanner.center_hz = SCANNER_DEFAULT_START_HZ + (SCANNER_DEFAULT_STOP_HZ - SCANNER_DEFAULT_START_HZ) / 2U;
    display->sensor.center_hz = SENSOR_CENTER_HZ;
    display->sensor.sample_rate_hz = SENSOR_SAMPLE_RATE_HZ;
    display->record.center_hz = CONFIG_RTLSDR_CENTER_FREQ_HZ;
    display->record.sample_rate_hz = CONFIG_RTLSDR_SAMPLE_RATE;
    display->sample_rate_hz = CONFIG_RTLSDR_SAMPLE_RATE;
    display->fm.audio_rate_hz = CONFIG_FM_RECEIVER_AUDIO_RATE_HZ;
    display->fm.audio_output_enabled = CONFIG_FM_RECEIVER_AUDIO_OUTPUT;
    display->fm.audio_output_ready = false;
    snprintf(display->fm.mode, sizeof(display->fm.mode), "%s", CONFIG_FM_RECEIVER_NFM_MODE ? "NFM" : "WFM");
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
    ESP_GOTO_ON_ERROR(alloc_scanner_image(display), fail, TAG, "scanner image");
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
    app_display_destroy(display);
    return ret;
}

void app_display_destroy(app_display_t *display)
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
    heap_caps_free(display->waterfall.pixels);
    heap_caps_free(display->fm.waveform_pixels);
    heap_caps_free(display->scanner.pixels);
    if (s_display == display) {
        s_display = NULL;
    }
    _lock_close(&display->lvgl_api_lock);
    free(display);
}

void app_display_set_status(app_display_t *display, const char *status)
{
    if (!display || !status) {
        return;
    }
    _lock_acquire(&display->lvgl_api_lock);
    snprintf(display->status, sizeof(display->status), "%s", status);
    display->status_dirty = true;
    _lock_release(&display->lvgl_api_lock);
}

void app_display_set_radio_info(app_display_t *display, uint32_t center_hz, uint32_t sample_rate_hz)
{
    if (!display) {
        return;
    }
    _lock_acquire(&display->lvgl_api_lock);
    display->waterfall.center_hz = center_hz;
    display->fm.center_hz = center_hz;
    display->adsb.center_hz = ADSB_CENTER_HZ;
    display->scanner.center_hz = display->scanner.start_hz + (display->scanner.stop_hz - display->scanner.start_hz) / 2U;
    display->sensor.center_hz = SENSOR_CENTER_HZ;
    display->record.center_hz = center_hz;
    display->sample_rate_hz = sample_rate_hz;
    display->adsb.sample_rate_hz = sample_rate_hz;
    display->sensor.sample_rate_hz = SENSOR_SAMPLE_RATE_HZ;
    display->record.sample_rate_hz = sample_rate_hz;
    display->status_dirty = true;
    draw_ruler(display);
    mark_waterfall_dirty(display);
    _lock_release(&display->lvgl_api_lock);
}

void app_display_set_fm_info(app_display_t *display,
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
        snprintf(display->fm.mode, sizeof(display->fm.mode), "%s", mode);
    }
    display->fm.audio_rate_hz = audio_rate_hz;
    display->fm.audio_output_enabled = audio_output_enabled;
    display->fm.audio_output_ready = audio_output_ready;
    display->fm.audio_level = audio_level > 100 ? 100 : audio_level;
    display->status_dirty = true;
    _lock_release(&display->lvgl_api_lock);
}

void app_display_push_fm_waveform(app_display_t *display, const int16_t *samples, size_t sample_count)
{
    if (!display || !samples || sample_count == 0 || !display->fm.waveform_pixels) {
        return;
    }

    _lock_acquire(&display->lvgl_api_lock);
    if (display->active_page == APP_DISPLAY_PAGE_FM || display->fm.waveform_img) {
        draw_fm_waveform_background(display);
        draw_fm_waveform(display, samples, sample_count);
        display->fm.waveform_dirty = true;
    }
    _lock_release(&display->lvgl_api_lock);
}

void app_display_set_sample_rate(app_display_t *display, uint32_t sample_rate_hz)
{
    if (!display) {
        return;
    }
    _lock_acquire(&display->lvgl_api_lock);
    display->sample_rate_hz = sample_rate_hz;
    display->status_dirty = true;
    _lock_release(&display->lvgl_api_lock);
}

void app_display_set_adsb_info(app_display_t *display,
                               uint32_t center_hz,
                               uint32_t sample_rate_hz,
                               bool active,
                               uint64_t candidates,
                               uint64_t valid_messages,
                               uint32_t aircraft_count)
{
    if (!display) {
        return;
    }
    _lock_acquire(&display->lvgl_api_lock);
    display->adsb.center_hz = center_hz;
    display->adsb.sample_rate_hz = sample_rate_hz;
    display->adsb.active = active;
    display->adsb.candidates = candidates;
    display->adsb.valid_messages = valid_messages;
    display->adsb.aircraft_count = aircraft_count;
    if (active) {
        display->sample_rate_hz = sample_rate_hz;
    }
    display->status_dirty = true;
    _lock_release(&display->lvgl_api_lock);
}

void app_display_update_adsb_aircraft(app_display_t *display, const app_display_adsb_aircraft_t *aircraft)
{
    if (!display || !aircraft || !aircraft->icao[0]) {
        return;
    }
    _lock_acquire(&display->lvgl_api_lock);
    uint32_t index = display->adsb.aircraft_used;
    for (uint32_t i = 0; i < display->adsb.aircraft_used; i++) {
        if (strcmp(display->adsb.aircraft[i].icao, aircraft->icao) == 0) {
            index = i;
            break;
        }
    }
    if (index >= ADSB_DISPLAY_MAX_AIRCRAFT) {
        index = ADSB_DISPLAY_MAX_AIRCRAFT - 1;
    } else if (index == display->adsb.aircraft_used && display->adsb.aircraft_used < ADSB_DISPLAY_MAX_AIRCRAFT) {
        display->adsb.aircraft_used++;
    }

    display->adsb.aircraft[index] = *aircraft;
    while (index > 0 && display->adsb.aircraft[index].last_seen_ms > display->adsb.aircraft[index - 1].last_seen_ms) {
        app_display_adsb_aircraft_t tmp = display->adsb.aircraft[index - 1];
        display->adsb.aircraft[index - 1] = display->adsb.aircraft[index];
        display->adsb.aircraft[index] = tmp;
        index--;
    }
    display->status_dirty = true;
    _lock_release(&display->lvgl_api_lock);
}

void app_display_clear_adsb(app_display_t *display)
{
    if (!display) {
        return;
    }
    _lock_acquire(&display->lvgl_api_lock);
    display->adsb.aircraft_used = 0;
    display->adsb.aircraft_count = 0;
    display->adsb.candidates = 0;
    display->adsb.valid_messages = 0;
    memset(display->adsb.aircraft, 0, sizeof(display->adsb.aircraft));
    display->status_dirty = true;
    _lock_release(&display->lvgl_api_lock);
}

void app_display_set_scanner_info(app_display_t *display,
                                  uint32_t start_hz,
                                  uint32_t stop_hz,
                                  bool active,
                                  uint32_t center_hz,
                                  uint32_t hop_index,
                                  uint32_t hop_count)
{
    if (!display) {
        return;
    }
    _lock_acquire(&display->lvgl_api_lock);
    if (start_hz < CENTER_FREQ_MIN_HZ) {
        start_hz = CENTER_FREQ_MIN_HZ;
    }
    if (stop_hz > CENTER_FREQ_MAX_HZ) {
        stop_hz = CENTER_FREQ_MAX_HZ;
    }
    if (stop_hz <= start_hz) {
        stop_hz = start_hz + 1U;
    }
    display->scanner.start_hz = start_hz;
    display->scanner.stop_hz = stop_hz;
    display->scanner.active = active;
    display->scanner.center_hz = center_hz;
    display->scanner.hop_index = hop_index;
    display->scanner.hop_count = hop_count;
    display->status_dirty = true;
    _lock_release(&display->lvgl_api_lock);
}

void app_display_set_sensor_info(app_display_t *display,
                                 uint32_t center_hz,
                                 uint32_t sample_rate_hz,
                                 bool active,
                                 uint64_t total_frames,
                                 uint64_t decoded_messages,
                                 uint32_t sensor_count,
                                 uint8_t signal_level)
{
    if (!display) {
        return;
    }
    _lock_acquire(&display->lvgl_api_lock);
    display->sensor.center_hz = center_hz;
    display->sensor.sample_rate_hz = sample_rate_hz;
    display->sensor.active = active;
    display->sensor.total_frames = total_frames;
    display->sensor.decoded_messages = decoded_messages;
    display->sensor.sensor_count = sensor_count;
    display->sensor.signal_level = signal_level > 100 ? 100 : signal_level;
    if (active) {
        display->sample_rate_hz = sample_rate_hz;
    }
    display->status_dirty = true;
    _lock_release(&display->lvgl_api_lock);
}

void app_display_update_sensor_event(app_display_t *display, const app_display_sensor_event_t *event)
{
    if (!display || !event || !event->data[0]) {
        return;
    }
    _lock_acquire(&display->lvgl_api_lock);
    uint32_t index = display->sensor.event_count;
    for (uint32_t i = 0; i < display->sensor.event_count; i++) {
        if (strcmp(display->sensor.events[i].id, event->id) == 0 &&
            strcmp(display->sensor.events[i].data, event->data) == 0) {
            index = i;
            break;
        }
    }
    if (index >= SENSOR_DISPLAY_MAX_EVENTS) {
        index = SENSOR_DISPLAY_MAX_EVENTS - 1;
    } else if (index == display->sensor.event_count && display->sensor.event_count < SENSOR_DISPLAY_MAX_EVENTS) {
        display->sensor.event_count++;
    }

    display->sensor.events[index] = *event;
    while (index > 0 && display->sensor.events[index].last_seen_ms > display->sensor.events[index - 1].last_seen_ms) {
        app_display_sensor_event_t tmp = display->sensor.events[index - 1];
        display->sensor.events[index - 1] = display->sensor.events[index];
        display->sensor.events[index] = tmp;
        index--;
    }
    display->status_dirty = true;
    _lock_release(&display->lvgl_api_lock);
}

void app_display_clear_sensor_events(app_display_t *display)
{
    if (!display) {
        return;
    }
    _lock_acquire(&display->lvgl_api_lock);
    display->sensor.event_count = 0;
    display->sensor.total_frames = 0;
    display->sensor.decoded_messages = 0;
    display->sensor.sensor_count = 0;
    memset(display->sensor.events, 0, sizeof(display->sensor.events));
    display->status_dirty = true;
    _lock_release(&display->lvgl_api_lock);
}

void app_display_set_record_info(app_display_t *display,
                                 uint32_t center_hz,
                                 uint32_t sample_rate_hz,
                                 bool active,
                                 bool storage_ready,
                                 uint64_t bytes_written,
                                 uint64_t max_bytes,
                                 uint32_t elapsed_seconds,
                                 const char *path)
{
    if (!display) {
        return;
    }
    _lock_acquire(&display->lvgl_api_lock);
    display->record.center_hz = center_hz;
    display->record.sample_rate_hz = sample_rate_hz;
    display->record.active = active;
    display->record.storage_ready = storage_ready;
    display->record.bytes_written = bytes_written;
    display->record.max_bytes = max_bytes;
    display->record.elapsed_seconds = elapsed_seconds;
    if (path) {
        snprintf(display->record.path, sizeof(display->record.path), "%s", path);
    }
    if (active) {
        display->sample_rate_hz = sample_rate_hz;
    }
    display->status_dirty = true;
    _lock_release(&display->lvgl_api_lock);
}

void app_display_push_fm_spectrum(app_display_t *display, const uint8_t *bins, size_t bin_count)
{
    if (!display || !bins || bin_count == 0 || !display->fm.waveform_pixels) {
        return;
    }

    _lock_acquire(&display->lvgl_api_lock);
    if (display->active_page == APP_DISPLAY_PAGE_FM && display->fm.waveform_img) {
        draw_fm_waveform_background(display);
        draw_fm_spectrum(display, bins, bin_count);
        display->fm.waveform_dirty = true;
    }
    _lock_release(&display->lvgl_api_lock);
}

void app_display_push_scanner_hop(app_display_t *display,
                                  const uint8_t *bins,
                                  size_t bin_count,
                                  uint32_t segment_start_hz,
                                  uint32_t segment_stop_hz,
                                  uint32_t scan_start_hz,
                                  uint32_t scan_stop_hz,
                                  uint32_t hop_index,
                                  uint32_t hop_count)
{
    if (!display || !bins || bin_count == 0 || !display->scanner.pixels) {
        return;
    }
    if (scan_stop_hz <= scan_start_hz || segment_stop_hz <= segment_start_hz) {
        return;
    }

    _lock_acquire(&display->lvgl_api_lock);
    display->scanner.start_hz = scan_start_hz;
    display->scanner.stop_hz = scan_stop_hz;
    display->scanner.hop_index = hop_index;
    display->scanner.hop_count = hop_count;
    display->scanner.center_hz = segment_start_hz + (segment_stop_hz - segment_start_hz) / 2U;
    display->scanner.active = true;

    if (display->active_page != APP_DISPLAY_PAGE_SCANNER || !display->scanner.img) {
        display->status_dirty = true;
        _lock_release(&display->lvgl_api_lock);
        return;
    }

    const int w = display->scanner.w;
    const int row_h = MIN(SCANNER_ROW_H, display->scanner.plot_h);
    const uint64_t span_hz = (uint64_t)scan_stop_hz - scan_start_hz;
    int x0 = (int)((((uint64_t)segment_start_hz - scan_start_hz) * (uint64_t)w) / span_hz);
    int x1 = (int)((((uint64_t)segment_stop_hz - scan_start_hz) * (uint64_t)w + span_hz - 1U) / span_hz);
    if (x0 < 0) {
        x0 = 0;
    }
    if (x1 > w) {
        x1 = w;
    }
    if (x1 <= x0) {
        x1 = MIN(w, x0 + 1);
    }

    if (hop_index == 0) {
        scanner_scroll_for_new_line(display);
    }

    uint16_t *row = display->scanner.pixels + display->scanner.plot_y * w;
    int seg_w = x1 - x0;
    for (int x = x0; x < x1; x++) {
        uint8_t value = scaled_bin_at(bins, bin_count, x - x0, seg_w);
        uint16_t color = scanner_palette(value);
        for (int y = 0; y < row_h; y++) {
            row[y * w + x] = color;
        }
    }

    if (hop_count > 0 && hop_index + 1U >= hop_count) {
        display->scanner.completed_lines++;
    }
    display->status_dirty = true;
    display->scanner.dirty = true;
    _lock_release(&display->lvgl_api_lock);
}

void app_display_set_center_freq_cb(app_display_t *display,
                                          app_center_freq_cb_t cb,
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

void app_display_set_page_cb(app_display_t *display,
                             app_page_cb_t cb,
                             void *ctx)
{
    if (!display) {
        return;
    }
    _lock_acquire(&display->lvgl_api_lock);
    display->page_cb = cb;
    display->page_cb_ctx = ctx;
    if (display->page_cb) {
        display->page_cb(display->active_page, display->page_cb_ctx);
    }
    _lock_release(&display->lvgl_api_lock);
}

app_display_page_t app_display_get_active_page(app_display_t *display)
{
    if (!display) {
        return APP_DISPLAY_PAGE_HOME;
    }
    _lock_acquire(&display->lvgl_api_lock);
    app_display_page_t page = display->active_page;
    _lock_release(&display->lvgl_api_lock);
    return page;
}

void app_display_push_line(app_display_t *display, const uint8_t *bins, size_t bin_count)
{
    if (!display || !bins || bin_count == 0 || !display->waterfall.pixels) {
        return;
    }

    _lock_acquire(&display->lvgl_api_lock);
    int w = display->waterfall.w;
    int y0 = display->waterfall.spectrum_h + display->waterfall.ruler_h;
    int wh = display->waterfall.h - y0;
    if (wh <= 0) {
        _lock_release(&display->lvgl_api_lock);
        return;
    }

    draw_spectrum(display, bins, bin_count);
    draw_ruler(display);

    if (wh > 1) {
        memmove(display->waterfall.pixels + (y0 + 1) * w,
                display->waterfall.pixels + y0 * w,
                (size_t)(wh - 1) * w * sizeof(uint16_t));
    }

    uint16_t *row = display->waterfall.pixels + y0 * w;
    for (int x = 0; x < w; x++) {
        row[x] = palette(scaled_bin_at(bins, bin_count, x, w));
    }

    display->have_samples = true;
    mark_waterfall_dirty(display);
    _lock_release(&display->lvgl_api_lock);
}

void app_display_render_idle(app_display_t *display)
{
    if (!display) {
        return;
    }
    _lock_acquire(&display->lvgl_api_lock);
    display->status_dirty = true;
    mark_waterfall_dirty(display);
    _lock_release(&display->lvgl_api_lock);
}
