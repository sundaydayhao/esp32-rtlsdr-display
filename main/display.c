#include "display.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "hal/lcd_types.h"
#include "sdkconfig.h"

struct waterfall_display {
    waterfall_display_config_t cfg;
    esp_lcd_panel_handle_t panel;
    uint16_t *framebuffer;
    int header_height;
    int spectrum_top;
    int spectrum_height;
    int ruler_top;
    int ruler_height;
    int waterfall_top;
    int waterfall_height;
    char status[96];
    uint32_t center_hz;
    uint32_t sample_rate_hz;
};

static const char *TAG = "display";

static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint16_t)(r & 0xf8) << 8) | ((uint16_t)(g & 0xfc) << 3) | (uint16_t)(b >> 3);
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

static void compute_layout(waterfall_display_t *display)
{
    int h = display->cfg.v_res;
    display->header_height = (h >= 420) ? 56 : 42;
    display->spectrum_height = (h >= 420) ? 88 : 58;
    display->ruler_height = (h >= 300) ? 20 : 14;

    int used = display->header_height + display->spectrum_height + display->ruler_height;
    if (used > h - 48) {
        display->header_height = h / 7;
        if (display->header_height < 28) {
            display->header_height = 28;
        }
        display->spectrum_height = h / 5;
        if (display->spectrum_height < 28) {
            display->spectrum_height = 28;
        }
        display->ruler_height = 14;
    }

    display->spectrum_top = display->header_height;
    display->ruler_top = display->spectrum_top + display->spectrum_height;
    display->waterfall_top = display->ruler_top + display->ruler_height;
    display->waterfall_height = h - display->waterfall_top;
    if (display->waterfall_height < 1) {
        display->waterfall_height = 1;
        display->waterfall_top = h - 1;
    }
}

static void fill_rect(waterfall_display_t *display, int x, int y, int w, int h, uint16_t color)
{
    int x2 = x + w;
    int y2 = y + h;
    if (x < 0) {
        x = 0;
    }
    if (y < 0) {
        y = 0;
    }
    if (x2 > display->cfg.h_res) {
        x2 = display->cfg.h_res;
    }
    if (y2 > display->cfg.v_res) {
        y2 = display->cfg.v_res;
    }
    for (int row = y; row < y2; row++) {
        uint16_t *dst = display->framebuffer + row * display->cfg.h_res + x;
        for (int col = x; col < x2; col++) {
            *dst++ = color;
        }
    }
}

static void draw_hline(waterfall_display_t *display, int x, int y, int w, uint16_t color)
{
    fill_rect(display, x, y, w, 1, color);
}

static void draw_vline(waterfall_display_t *display, int x, int y, int h, uint16_t color)
{
    fill_rect(display, x, y, 1, h, color);
}

static int text_width_px(const char *text, int scale)
{
    return (int)strlen(text) * 6 * scale;
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

static const uint8_t *glyph5x7(char c)
{
    static const uint8_t blank[5] = {0, 0, 0, 0, 0};
    static const uint8_t digits[10][5] = {
        {0x3e, 0x51, 0x49, 0x45, 0x3e},
        {0x00, 0x42, 0x7f, 0x40, 0x00},
        {0x42, 0x61, 0x51, 0x49, 0x46},
        {0x21, 0x41, 0x45, 0x4b, 0x31},
        {0x18, 0x14, 0x12, 0x7f, 0x10},
        {0x27, 0x45, 0x45, 0x45, 0x39},
        {0x3c, 0x4a, 0x49, 0x49, 0x30},
        {0x01, 0x71, 0x09, 0x05, 0x03},
        {0x36, 0x49, 0x49, 0x49, 0x36},
        {0x06, 0x49, 0x49, 0x29, 0x1e},
    };
    static const uint8_t letters[26][5] = {
        {0x7e, 0x11, 0x11, 0x11, 0x7e},
        {0x7f, 0x49, 0x49, 0x49, 0x36},
        {0x3e, 0x41, 0x41, 0x41, 0x22},
        {0x7f, 0x41, 0x41, 0x22, 0x1c},
        {0x7f, 0x49, 0x49, 0x49, 0x41},
        {0x7f, 0x09, 0x09, 0x09, 0x01},
        {0x3e, 0x41, 0x49, 0x49, 0x7a},
        {0x7f, 0x08, 0x08, 0x08, 0x7f},
        {0x00, 0x41, 0x7f, 0x41, 0x00},
        {0x20, 0x40, 0x41, 0x3f, 0x01},
        {0x7f, 0x08, 0x14, 0x22, 0x41},
        {0x7f, 0x40, 0x40, 0x40, 0x40},
        {0x7f, 0x02, 0x0c, 0x02, 0x7f},
        {0x7f, 0x04, 0x08, 0x10, 0x7f},
        {0x3e, 0x41, 0x41, 0x41, 0x3e},
        {0x7f, 0x09, 0x09, 0x09, 0x06},
        {0x3e, 0x41, 0x51, 0x21, 0x5e},
        {0x7f, 0x09, 0x19, 0x29, 0x46},
        {0x46, 0x49, 0x49, 0x49, 0x31},
        {0x01, 0x01, 0x7f, 0x01, 0x01},
        {0x3f, 0x40, 0x40, 0x40, 0x3f},
        {0x1f, 0x20, 0x40, 0x20, 0x1f},
        {0x7f, 0x20, 0x18, 0x20, 0x7f},
        {0x63, 0x14, 0x08, 0x14, 0x63},
        {0x03, 0x04, 0x78, 0x04, 0x03},
        {0x61, 0x51, 0x49, 0x45, 0x43},
    };
    static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
    static const uint8_t dot[5] = {0x00, 0x40, 0x60, 0x00, 0x00};
    static const uint8_t dash[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t slash[5] = {0x20, 0x10, 0x08, 0x04, 0x02};

    if (c >= '0' && c <= '9') {
        return digits[c - '0'];
    }
    if (c >= 'a' && c <= 'z') {
        c = (char)(c - 'a' + 'A');
    }
    if (c >= 'A' && c <= 'Z') {
        return letters[c - 'A'];
    }
    if (c == ':') {
        return colon;
    }
    if (c == '.') {
        return dot;
    }
    if (c == '-') {
        return dash;
    }
    if (c == '/') {
        return slash;
    }
    return blank;
}

static void draw_char(waterfall_display_t *display, int x, int y, char c, uint16_t fg, uint16_t bg, int scale)
{
    const uint8_t *glyph = glyph5x7(c);
    for (int col = 0; col < 6; col++) {
        uint8_t bits = (col < 5) ? glyph[col] : 0;
        for (int row = 0; row < 8; row++) {
            uint16_t color = (bits & (1U << row)) ? fg : bg;
            fill_rect(display, x + col * scale, y + row * scale, scale, scale, color);
        }
    }
}

static void draw_text(waterfall_display_t *display, int x, int y, const char *text, uint16_t fg, uint16_t bg, int scale)
{
    while (*text && x < display->cfg.h_res - 6 * scale) {
        draw_char(display, x, y, *text++, fg, bg, scale);
        x += 6 * scale;
    }
}

static void draw_header(waterfall_display_t *display)
{
    uint16_t bg = rgb565(3, 7, 10);
    uint16_t fg = rgb565(225, 238, 232);
    uint16_t rule = rgb565(28, 50, 54);
    int title_scale = (display->cfg.h_res >= 420 && display->header_height >= 50) ? 2 : 1;
    int title_y = (title_scale == 2) ? 8 : 6;
    int status_y = (title_scale == 2) ? 34 : 24;

    fill_rect(display, 0, 0, display->cfg.h_res, display->header_height, bg);

    char line[96];
    snprintf(line, sizeof(line), "RTLSDR WATERFALL  %.3f MHZ  %.3f MSPS",
             (double)display->center_hz / 1000000.0,
             (double)display->sample_rate_hz / 1000000.0);
    draw_text(display, 10, title_y, line, fg, bg, title_scale);
    draw_text(display, 10, status_y, display->status, rgb565(112, 220, 170), bg, 1);
    draw_hline(display, 0, display->header_height - 1, display->cfg.h_res, rule);
}

static void draw_spectrum_background(waterfall_display_t *display)
{
    uint16_t bg = rgb565(1, 4, 8);
    uint16_t grid = rgb565(12, 26, 30);
    uint16_t center = rgb565(34, 70, 66);
    int w = display->cfg.h_res;
    int top = display->spectrum_top;
    int h = display->spectrum_height;

    fill_rect(display, 0, top, w, h, bg);

    for (int i = 1; i < 8; i++) {
        int x = (w * i) / 8;
        draw_vline(display, x, top, h, (i == 4) ? center : grid);
    }
    for (int i = 1; i < 4; i++) {
        int y = top + (h * i) / 4;
        draw_hline(display, 0, y, w, grid);
    }
    draw_hline(display, 0, top + h - 1, w, rgb565(28, 50, 54));
}

static void draw_freq_label(waterfall_display_t *display, int x, int y, const char *text, int align)
{
    uint16_t bg = rgb565(3, 7, 10);
    uint16_t fg = rgb565(150, 170, 164);
    int width = text_width_px(text, 1);
    if (align == 1) {
        x -= width / 2;
    } else if (align == 2) {
        x -= width;
    }
    if (x < 2) {
        x = 2;
    }
    if (x + width >= display->cfg.h_res) {
        x = display->cfg.h_res - width - 2;
    }
    draw_text(display, x, y, text, fg, bg, 1);
}

static void format_freq(char *buf, size_t len, int64_t hz)
{
    snprintf(buf, len, "%.3f MHZ", (double)hz / 1000000.0);
}

static void draw_ruler(waterfall_display_t *display)
{
    uint16_t bg = rgb565(3, 7, 10);
    uint16_t tick = rgb565(52, 82, 78);
    int w = display->cfg.h_res;
    int top = display->ruler_top;
    int h = display->ruler_height;
    int y_text = top + ((h > 16) ? 4 : 2);
    int64_t center = (int64_t)display->center_hz;
    int64_t half_span = (int64_t)display->sample_rate_hz / 2;
    char left[24];
    char mid[24];
    char right[24];

    fill_rect(display, 0, top, w, h, bg);
    for (int i = 0; i <= 8; i++) {
        int x = (i == 8) ? (w - 1) : (w * i) / 8;
        int tick_h = (i == 0 || i == 4 || i == 8) ? h : h / 2;
        draw_vline(display, x, top + h - tick_h, tick_h, tick);
    }

    format_freq(left, sizeof(left), center - half_span);
    format_freq(mid, sizeof(mid), center);
    format_freq(right, sizeof(right), center + half_span);
    draw_freq_label(display, 4, y_text, left, 0);
    draw_freq_label(display, w / 2, y_text, mid, 1);
    draw_freq_label(display, w - 4, y_text, right, 2);
    draw_hline(display, 0, top + h - 1, w, rgb565(22, 42, 46));
}

static void draw_spectrum(waterfall_display_t *display, const uint8_t *bins, size_t bin_count)
{
    int w = display->cfg.h_res;
    int top = display->spectrum_top;
    int h = display->spectrum_height;
    int bottom = top + h - 2;
    uint16_t trace = rgb565(230, 242, 214);

    draw_spectrum_background(display);

    int prev_y = -1;
    for (int x = 0; x < w; x++) {
        uint8_t value = scaled_bin_at(bins, bin_count, x, w);
        int bar = 1 + ((int)value * (h - 5)) / 255;
        int y_trace = bottom - bar;
        if (y_trace < top + 1) {
            y_trace = top + 1;
        }

        uint16_t color = palette(value);
        for (int y = y_trace + 1; y <= bottom; y++) {
            display->framebuffer[y * w + x] = color;
        }

        if (prev_y >= 0) {
            int y0 = prev_y < y_trace ? prev_y : y_trace;
            int y1 = prev_y > y_trace ? prev_y : y_trace;
            for (int y = y0; y <= y1; y++) {
                display->framebuffer[y * w + x] = trace;
            }
        } else {
            display->framebuffer[y_trace * w + x] = trace;
        }
        prev_y = y_trace;
    }

    draw_vline(display, w / 2, top, h, rgb565(46, 78, 72));
}

static void flush_frame(waterfall_display_t *display)
{
    esp_lcd_panel_draw_bitmap(display->panel,
                              0,
                              0,
                              display->cfg.h_res,
                              display->cfg.v_res,
                              display->framebuffer);
}

static void flush_rect(waterfall_display_t *display, int x, int y, int w, int h)
{
    int x2 = x + w;
    int y2 = y + h;
    if (x < 0) {
        x = 0;
    }
    if (y < 0) {
        y = 0;
    }
    if (x2 > display->cfg.h_res) {
        x2 = display->cfg.h_res;
    }
    if (y2 > display->cfg.v_res) {
        y2 = display->cfg.v_res;
    }
    if (x >= x2 || y >= y2) {
        return;
    }

    esp_lcd_panel_draw_bitmap(display->panel,
                              x,
                              y,
                              x2,
                              y2,
                              display->framebuffer + y * display->cfg.h_res + x);
}

esp_err_t waterfall_display_create(const waterfall_display_config_t *config, waterfall_display_t **out)
{
    ESP_RETURN_ON_FALSE(config && out, ESP_ERR_INVALID_ARG, TAG, "bad args");
    esp_err_t ret = ESP_OK;
    waterfall_display_t *display = calloc(1, sizeof(*display));
    ESP_RETURN_ON_FALSE(display, ESP_ERR_NO_MEM, TAG, "no memory");
    display->cfg = *config;
    compute_layout(display);
    strcpy(display->status, "WAITING FOR RTL2832U");

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

    ESP_GOTO_ON_ERROR(esp_lcd_new_rgb_panel(&panel_config, &display->panel), fail, TAG, "new RGB panel");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(display->panel), fail, TAG, "panel reset");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(display->panel), fail, TAG, "panel init");
    ESP_GOTO_ON_ERROR(esp_lcd_rgb_panel_get_frame_buffer(display->panel, 1, (void **)&display->framebuffer),
                      fail, TAG, "get frame buffer");

    fill_rect(display, 0, 0, config->h_res, config->v_res, rgb565(0, 0, 0));
    draw_header(display);
    draw_spectrum_background(display);
    draw_ruler(display);
    flush_frame(display);
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
    if (display->panel) {
        esp_lcd_panel_del(display->panel);
    }
    free(display);
}

void waterfall_display_set_status(waterfall_display_t *display, const char *status)
{
    if (!display || !status) {
        return;
    }
    snprintf(display->status, sizeof(display->status), "%s", status);
    draw_header(display);
}

void waterfall_display_set_radio_info(waterfall_display_t *display, uint32_t center_hz, uint32_t sample_rate_hz)
{
    if (!display) {
        return;
    }
    display->center_hz = center_hz;
    display->sample_rate_hz = sample_rate_hz;
    draw_header(display);
    draw_ruler(display);
}

void waterfall_display_push_line(waterfall_display_t *display, const uint8_t *bins, size_t bin_count)
{
    if (!display || !bins || bin_count == 0) {
        return;
    }

    int h_res = display->cfg.h_res;
    int y0 = display->waterfall_top;
    int wh = display->waterfall_height;

    draw_spectrum(display, bins, bin_count);

    if (wh > 1) {
        memmove(display->framebuffer + (y0 + 1) * h_res,
                display->framebuffer + y0 * h_res,
                (size_t)(wh - 1) * h_res * sizeof(uint16_t));
    }

    uint16_t *row = display->framebuffer + y0 * h_res;
    for (int x = 0; x < h_res; x++) {
        row[x] = palette(scaled_bin_at(bins, bin_count, x, h_res));
    }
    flush_rect(display, 0, display->spectrum_top, display->cfg.h_res,
               display->cfg.v_res - display->spectrum_top);
}

void waterfall_display_render_idle(waterfall_display_t *display)
{
    if (!display) {
        return;
    }
    flush_frame(display);
}
