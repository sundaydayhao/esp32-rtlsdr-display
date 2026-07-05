#include "korvo_audio.h"

#include <string.h>
#include "driver/i2c_master.h"
#include "driver/i2s_common.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "es8389_codec.h"

#define KORVO_AUDIO_CODEC_ADDR          ES8389_CODEC_DEFAULT_ADDR
#define KORVO_AUDIO_I2C_FREQ_HZ         400000
#define KORVO_AUDIO_STEREO_FRAME_COUNT  128
#define KORVO_AUDIO_CHANNELS            2

struct korvo_audio_output {
    korvo_audio_output_config_t cfg;
    i2c_master_bus_handle_t i2c_bus;
    bool owns_i2c_bus;
    i2s_chan_handle_t i2s_tx;
    const audio_codec_data_if_t *data_if;
    const audio_codec_ctrl_if_t *ctrl_if;
    const audio_codec_gpio_if_t *gpio_if;
    const audio_codec_if_t *codec_if;
    esp_codec_dev_handle_t codec_dev;
    int16_t stereo_frame[KORVO_AUDIO_STEREO_FRAME_COUNT * KORVO_AUDIO_CHANNELS];
};

static const char *TAG = "korvo_audio";

static esp_err_t init_i2c(korvo_audio_output_t *audio)
{
    esp_err_t ret = i2c_master_get_bus_handle(audio->cfg.i2c_port, &audio->i2c_bus);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Using existing I2C bus %d for ES8389", audio->cfg.i2c_port);
        return ESP_OK;
    }
    if (ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = audio->cfg.i2c_port,
        .sda_io_num = audio->cfg.i2c_sda_gpio,
        .scl_io_num = audio->cfg.i2c_scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &audio->i2c_bus), TAG, "new i2c bus");
    audio->owns_i2c_bus = true;
    ESP_LOGI(TAG, "Created I2C bus %d for ES8389", audio->cfg.i2c_port);
    return ESP_OK;
}

static esp_err_t init_i2s(korvo_audio_output_t *audio)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(audio->cfg.i2s_port, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    chan_cfg.dma_desc_num = 6;
    chan_cfg.dma_frame_num = 240;

    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &audio->i2s_tx, NULL), TAG, "new i2s tx");

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(audio->cfg.sample_rate_hz),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = (gpio_num_t)audio->cfg.i2s_mclk_gpio,
            .bclk = (gpio_num_t)audio->cfg.i2s_bclk_gpio,
            .ws = (gpio_num_t)audio->cfg.i2s_ws_gpio,
            .dout = (gpio_num_t)audio->cfg.i2s_dout_gpio,
            .din = (gpio_num_t)audio->cfg.i2s_din_gpio,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(audio->i2s_tx, &std_cfg), TAG, "init i2s std");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(audio->i2s_tx), TAG, "enable i2s tx");
    return ESP_OK;
}

static esp_err_t init_codec(korvo_audio_output_t *audio)
{
    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = audio->cfg.i2s_port,
        .rx_handle = NULL,
        .tx_handle = audio->i2s_tx,
    };
    audio->data_if = audio_codec_new_i2s_data(&i2s_cfg);
    ESP_RETURN_ON_FALSE(audio->data_if, ESP_ERR_NO_MEM, TAG, "new i2s codec data");

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = audio->cfg.i2c_port,
        .addr = KORVO_AUDIO_CODEC_ADDR,
        .bus_handle = audio->i2c_bus,
    };
    audio->ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_RETURN_ON_FALSE(audio->ctrl_if, ESP_ERR_NO_MEM, TAG, "new i2c codec ctrl");

    audio->gpio_if = audio_codec_new_gpio();
    ESP_RETURN_ON_FALSE(audio->gpio_if, ESP_ERR_NO_MEM, TAG, "new codec gpio");

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0,
        .codec_dac_voltage = 3.3,
    };
    es8389_codec_cfg_t codec_cfg = {
        .ctrl_if = audio->ctrl_if,
        .gpio_if = audio->gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin = audio->cfg.pa_gpio,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk = audio->cfg.i2s_mclk_gpio >= 0,
        .hw_gain = gain,
    };
    audio->codec_if = es8389_codec_new(&codec_cfg);
    ESP_RETURN_ON_FALSE(audio->codec_if, ESP_ERR_NO_MEM, TAG, "new es8389 codec");

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = audio->codec_if,
        .data_if = audio->data_if,
    };
    audio->codec_dev = esp_codec_dev_new(&dev_cfg);
    ESP_RETURN_ON_FALSE(audio->codec_dev, ESP_ERR_NO_MEM, TAG, "new codec dev");

    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = I2S_DATA_BIT_WIDTH_16BIT,
        .channel = KORVO_AUDIO_CHANNELS,
        .channel_mask = 0x03,
        .sample_rate = audio->cfg.sample_rate_hz,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    };
    ESP_RETURN_ON_FALSE(esp_codec_dev_open(audio->codec_dev, &fs) == ESP_CODEC_DEV_OK,
                        ESP_FAIL,
                        TAG,
                        "open codec");

    int volume = audio->cfg.volume_percent;
    if (volume < 0) {
        volume = 0;
    } else if (volume > 100) {
        volume = 100;
    }
    ESP_RETURN_ON_FALSE(esp_codec_dev_set_out_vol(audio->codec_dev, volume) == ESP_CODEC_DEV_OK,
                        ESP_FAIL,
                        TAG,
                        "set codec volume");
    ESP_RETURN_ON_FALSE(esp_codec_dev_set_out_mute(audio->codec_dev, false) == ESP_CODEC_DEV_OK,
                        ESP_FAIL,
                        TAG,
                        "unmute codec");
    return ESP_OK;
}

esp_err_t korvo_audio_output_create(const korvo_audio_output_config_t *config,
                                    korvo_audio_output_t **out)
{
    ESP_RETURN_ON_FALSE(config && out, ESP_ERR_INVALID_ARG, TAG, "bad args");
    ESP_RETURN_ON_FALSE(config->sample_rate_hz >= 8000, ESP_ERR_INVALID_ARG, TAG, "bad sample rate");

    korvo_audio_output_t *audio = heap_caps_calloc(1, sizeof(*audio), MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(audio, ESP_ERR_NO_MEM, TAG, "no memory");
    audio->cfg = *config;

    esp_err_t ret = init_i2c(audio);
    if (ret == ESP_OK) {
        ret = init_i2s(audio);
    }
    if (ret == ESP_OK) {
        ret = init_codec(audio);
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Korvo speaker init failed: %s", esp_err_to_name(ret));
        korvo_audio_output_destroy(audio);
        return ret;
    }

    ESP_LOGI(TAG,
             "Korvo speaker ready: ES8389 0x%02x, %lu Hz, I2S%d BCLK=%d WS=%d DOUT=%d PA=%d",
             KORVO_AUDIO_CODEC_ADDR,
             (unsigned long)audio->cfg.sample_rate_hz,
             audio->cfg.i2s_port,
             audio->cfg.i2s_bclk_gpio,
             audio->cfg.i2s_ws_gpio,
             audio->cfg.i2s_dout_gpio,
             audio->cfg.pa_gpio);
    *out = audio;
    return ESP_OK;
}

void korvo_audio_output_destroy(korvo_audio_output_t *audio)
{
    if (!audio) {
        return;
    }
    if (audio->codec_dev) {
        (void)esp_codec_dev_set_out_mute(audio->codec_dev, true);
        (void)esp_codec_dev_close(audio->codec_dev);
        esp_codec_dev_delete(audio->codec_dev);
        audio->codec_dev = NULL;
    }
    if (audio->i2s_tx) {
        (void)i2s_channel_disable(audio->i2s_tx);
        (void)i2s_del_channel(audio->i2s_tx);
        audio->i2s_tx = NULL;
    }
    if (audio->owns_i2c_bus && audio->i2c_bus) {
        (void)i2c_del_master_bus(audio->i2c_bus);
        audio->i2c_bus = NULL;
    }
    heap_caps_free(audio);
}

esp_err_t korvo_audio_output_write_mono(korvo_audio_output_t *audio,
                                        const int16_t *samples,
                                        size_t sample_count,
                                        uint32_t timeout_ms,
                                        size_t *samples_written)
{
    ESP_RETURN_ON_FALSE(audio && audio->codec_dev && samples, ESP_ERR_INVALID_ARG, TAG, "bad write args");

    size_t done = 0;
    while (done < sample_count) {
        size_t n = sample_count - done;
        if (n > KORVO_AUDIO_STEREO_FRAME_COUNT) {
            n = KORVO_AUDIO_STEREO_FRAME_COUNT;
        }
        for (size_t i = 0; i < n; i++) {
            int16_t sample = samples[done + i];
            audio->stereo_frame[(i * 2)] = sample;
            audio->stereo_frame[(i * 2) + 1] = sample;
        }

        int bytes = (int)(n * KORVO_AUDIO_CHANNELS * sizeof(audio->stereo_frame[0]));
        int ret = esp_codec_dev_write(audio->codec_dev, audio->stereo_frame, bytes);
        if (ret != ESP_CODEC_DEV_OK) {
            if (samples_written) {
                *samples_written = done;
            }
            return ESP_FAIL;
        }
        done += n;

        if (timeout_ms == 0) {
            break;
        }
    }

    if (samples_written) {
        *samples_written = done;
    }
    return done == sample_count ? ESP_OK : ESP_ERR_TIMEOUT;
}
