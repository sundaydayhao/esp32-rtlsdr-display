#include "rtlsdr.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "usb_host_service.h"

#define RTLSDR_VID_REALTEK        0x0bda
#define RTLSDR_PID_RTL2832U       0x2832
#define RTLSDR_PID_RTL2838UHIDIR  0x2838

#define CTRL_TIMEOUT_MS           1000
#define CTRL_EVENT_POLL_MS        10
#define CTRL_DRAIN_POLLS          4
#define CLIENT_EVENT_MSGS         8
#define DEV_MAX_COUNT             128

#define CTRL_IN  (USB_BM_REQUEST_TYPE_DIR_IN | USB_BM_REQUEST_TYPE_TYPE_VENDOR | USB_BM_REQUEST_TYPE_RECIP_DEVICE)
#define CTRL_OUT (USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_VENDOR | USB_BM_REQUEST_TYPE_RECIP_DEVICE)

#define USB_SYSCTL                0x2000
#define USB_EPA_CTL               0x2148
#define USB_EPA_MAXPKT            0x2158

#define SYS_DEMOD_CTL             0x3000
#define SYS_GPO                   0x3001
#define SYS_GPOE                  0x3003
#define SYS_DEMOD_CTL_1           0x300b
#define SYS_IR_SUSPEND            0x300c

#define RTL_BLOCK_DEMOD           0
#define RTL_BLOCK_USB             1
#define RTL_BLOCK_SYS             2
#define RTL_BLOCK_IIC             6

#define FIR_LEN                   16

#define RTL_XTAL_HZ               28800000U
#define R82XX_I2C_ADDR_R820T      0x34
#define R82XX_I2C_ADDR_R828D      0x74
#define R82XX_CHECK_ADDR          0x00
#define R82XX_CHECK_VAL           0x69
#define R82XX_IF_FREQ_HZ          3570000U
#define R82XX_TUNER_XTAL_HZ       28800000U
#define R82XX_REG_SHADOW_START    5
#define R82XX_NUM_REGS            30
#define R82XX_VER_NUM             49
#define R82XX_MAX_I2C_MSG_LEN     8

typedef enum {
    R82XX_CHIP_NONE = 0,
    R82XX_CHIP_R820T,
    R82XX_CHIP_R828D,
} r82xx_chip_t;

typedef enum {
    R82XX_XTAL_LOW_CAP_30P = 0,
    R82XX_XTAL_LOW_CAP_20P,
    R82XX_XTAL_LOW_CAP_10P,
    R82XX_XTAL_LOW_CAP_0P,
    R82XX_XTAL_HIGH_CAP_0P,
} r82xx_xtal_cap_t;

typedef struct {
    uint32_t freq_mhz;
    uint8_t open_d;
    uint8_t rf_mux_ploy;
    uint8_t tf_c;
    uint8_t xtal_cap20p;
    uint8_t xtal_cap10p;
    uint8_t xtal_cap0p;
} r82xx_freq_range_t;

static const int fir_default[FIR_LEN] = {
    -54, -36, -41, -40, -32, -14, 14, 53,
    101, 156, 215, 273, 327, 372, 404, 421,
};

static const uint8_t r82xx_init_array[R82XX_NUM_REGS] = {
    0x83, 0x32, 0x75,
    0xc0, 0x40, 0xd6, 0x6c,
    0xf5, 0x63, 0x75, 0x68,
    0x6c, 0x83, 0x80, 0x00,
    0x0f, 0x00, 0xc0, 0x30,
    0x48, 0xcc, 0x60, 0x00,
    0x54, 0xae, 0x4a, 0xc0,
};

static const r82xx_freq_range_t r82xx_freq_ranges[] = {
    {0, 0x08, 0x02, 0xdf, 0x02, 0x01, 0x00},
    {50, 0x08, 0x02, 0xbe, 0x02, 0x01, 0x00},
    {55, 0x08, 0x02, 0x8b, 0x02, 0x01, 0x00},
    {60, 0x08, 0x02, 0x7b, 0x02, 0x01, 0x00},
    {65, 0x08, 0x02, 0x69, 0x02, 0x01, 0x00},
    {70, 0x08, 0x02, 0x58, 0x02, 0x01, 0x00},
    {75, 0x00, 0x02, 0x44, 0x02, 0x01, 0x00},
    {80, 0x00, 0x02, 0x44, 0x02, 0x01, 0x00},
    {90, 0x00, 0x02, 0x34, 0x01, 0x01, 0x00},
    {100, 0x00, 0x02, 0x34, 0x01, 0x01, 0x00},
    {110, 0x00, 0x02, 0x24, 0x01, 0x01, 0x00},
    {120, 0x00, 0x02, 0x24, 0x01, 0x01, 0x00},
    {140, 0x00, 0x02, 0x14, 0x01, 0x01, 0x00},
    {180, 0x00, 0x02, 0x13, 0x00, 0x00, 0x00},
    {220, 0x00, 0x02, 0x13, 0x00, 0x00, 0x00},
    {250, 0x00, 0x02, 0x11, 0x00, 0x00, 0x00},
    {280, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00},
    {310, 0x00, 0x41, 0x00, 0x00, 0x00, 0x00},
    {450, 0x00, 0x41, 0x00, 0x00, 0x00, 0x00},
    {588, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00},
    {650, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00},
};

struct rtlsdr {
    rtlsdr_config_t cfg;
    rtlsdr_samples_cb_t samples_cb;
    rtlsdr_status_cb_t status_cb;
    void *cb_ctx;
    usb_host_client_handle_t client;
    usb_device_handle_t dev_hdl;
    TaskHandle_t task;
    SemaphoreHandle_t done_sem;
    volatile bool shutting_down;
    volatile bool streaming;
    portMUX_TYPE tune_lock;
    volatile bool tune_pending;
    uint32_t pending_center_freq_hz;
    uint8_t new_dev_addr;
    bool new_dev_pending;
    bool gone;
    uint8_t interface_num;
    uint8_t alt_setting;
    uint8_t bulk_in_ep;
    uint16_t bulk_mps;
    r82xx_chip_t tuner_chip;
    uint8_t tuner_i2c_addr;
    uint8_t r82xx_regs[R82XX_NUM_REGS];
    r82xx_xtal_cap_t r82xx_xtal_cap_sel;
    uint32_t r82xx_int_freq_hz;
    uint8_t r82xx_fil_cal_code;
    uint8_t r82xx_input;
    bool r82xx_has_lock;
    bool r82xx_init_done;
    usb_transfer_t *transfers[CONFIG_RTLSDR_NUM_TRANSFERS];
};

static const char *TAG = "rtlsdr";

static bool is_rtlsdr_vid_pid(uint16_t vid, uint16_t pid)
{
    return vid == RTLSDR_VID_REALTEK && (pid == RTLSDR_PID_RTL2832U || pid == RTLSDR_PID_RTL2838UHIDIR);
}

static void set_status(rtlsdr_t *dev, const char *status)
{
    ESP_LOGI(TAG, "%s", status);
    if (dev->status_cb) {
        dev->status_cb(status, dev->cb_ctx);
    }
}

static void ctrl_done_cb(usb_transfer_t *transfer)
{
    xSemaphoreGive((SemaphoreHandle_t)transfer->context);
}

static esp_err_t wait_for_transfer_done(rtlsdr_t *dev, SemaphoreHandle_t done, uint32_t timeout_ms)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (xTaskGetTickCount() < deadline) {
        if (xSemaphoreTake(done, 0) == pdTRUE) {
            return ESP_OK;
        }

        esp_err_t ret = usb_host_client_handle_events(dev->client, pdMS_TO_TICKS(CTRL_EVENT_POLL_MS));
        if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
            return ret;
        }
    }

    return ESP_ERR_TIMEOUT;
}

static esp_err_t ctrl_transfer(rtlsdr_t *dev,
                               uint8_t bm_request_type,
                               uint8_t request,
                               uint16_t value,
                               uint16_t index,
                               uint8_t *data,
                               uint16_t len)
{
    usb_transfer_t *transfer = NULL;
    ESP_RETURN_ON_ERROR(usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + len, 0, &transfer),
                        TAG, "alloc ctrl transfer");

    SemaphoreHandle_t done = xSemaphoreCreateBinary();
    if (!done) {
        usb_host_transfer_free(transfer);
        return ESP_ERR_NO_MEM;
    }

    usb_setup_packet_t *setup = (usb_setup_packet_t *)transfer->data_buffer;
    setup->bmRequestType = bm_request_type;
    setup->bRequest = request;
    setup->wValue = value;
    setup->wIndex = index;
    setup->wLength = len;
    if (!(bm_request_type & USB_BM_REQUEST_TYPE_DIR_IN) && len && data) {
        memcpy(transfer->data_buffer + sizeof(*setup), data, len);
    }

    transfer->num_bytes = sizeof(*setup) + len;
    transfer->device_handle = dev->dev_hdl;
    transfer->callback = ctrl_done_cb;
    transfer->context = done;

    esp_err_t ret = usb_host_transfer_submit_control(dev->client, transfer);
    if (ret == ESP_OK) {
        ret = wait_for_transfer_done(dev, done, CTRL_TIMEOUT_MS);
    }
    if (ret == ESP_OK && transfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        ESP_LOGW(TAG, "control transfer status %d", transfer->status);
        ret = ESP_FAIL;
    }
    if (ret == ESP_OK && (bm_request_type & USB_BM_REQUEST_TYPE_DIR_IN) && len && data) {
        memcpy(data, transfer->data_buffer + sizeof(*setup), len);
    }

    vSemaphoreDelete(done);
    usb_host_transfer_free(transfer);
    if (ret != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(CTRL_EVENT_POLL_MS));
        for (int i = 0; i < CTRL_DRAIN_POLLS; i++) {
            esp_err_t drain_ret = usb_host_client_handle_events(dev->client, pdMS_TO_TICKS(CTRL_EVENT_POLL_MS));
            if (drain_ret == ESP_ERR_TIMEOUT) {
                break;
            }
            if (drain_ret != ESP_OK) {
                ESP_LOGW(TAG, "drain ctrl events: %s", esp_err_to_name(drain_ret));
                break;
            }
        }
    }
    return ret;
}

static esp_err_t rtlsdr_write_array(rtlsdr_t *dev, uint8_t block, uint16_t addr, const uint8_t *data, uint16_t len)
{
    uint8_t tmp[32];
    ESP_RETURN_ON_FALSE(len <= sizeof(tmp), ESP_ERR_INVALID_ARG, TAG, "write too large");
    memcpy(tmp, data, len);
    uint16_t index = ((uint16_t)block << 8) | 0x10;
    return ctrl_transfer(dev, CTRL_OUT, 0, addr, index, tmp, len);
}

static esp_err_t rtlsdr_read_array(rtlsdr_t *dev, uint8_t block, uint16_t addr, uint8_t *data, uint16_t len)
{
    uint16_t index = ((uint16_t)block << 8);
    return ctrl_transfer(dev, CTRL_IN, 0, addr, index, data, len);
}

static esp_err_t rtlsdr_write_reg(rtlsdr_t *dev, uint8_t block, uint16_t addr, uint16_t value, uint8_t len)
{
    uint8_t data[2] = {
        (uint8_t)((len == 1) ? (value & 0xff) : (value >> 8)),
        (uint8_t)(value & 0xff),
    };
    return rtlsdr_write_array(dev, block, addr, data, len);
}

static esp_err_t rtlsdr_demod_read_reg(rtlsdr_t *dev, uint8_t page, uint16_t addr, uint8_t len, uint16_t *out)
{
    uint8_t data[2] = {0};
    uint16_t value = (uint16_t)((addr << 8) | 0x20);
    ESP_RETURN_ON_FALSE(out && len > 0 && len <= sizeof(data), ESP_ERR_INVALID_ARG, TAG, "bad demod read");
    ESP_RETURN_ON_ERROR(ctrl_transfer(dev, CTRL_IN, 0, value, page, data, len), TAG, "demod read");
    *out = (uint16_t)((data[1] << 8) | data[0]);
    return ESP_OK;
}

static esp_err_t rtlsdr_demod_write_reg(rtlsdr_t *dev, uint8_t page, uint16_t addr, uint16_t value, uint8_t len)
{
    uint16_t index = 0x10 | page;
    uint16_t val = (uint16_t)((addr << 8) | 0x20);
    uint8_t data[2] = {
        (uint8_t)((len == 1) ? (value & 0xff) : (value >> 8)),
        (uint8_t)(value & 0xff),
    };
    ESP_RETURN_ON_ERROR(ctrl_transfer(dev, CTRL_OUT, 0, val, index, data, len), TAG, "demod write");

    uint16_t dummy = 0;
    (void)rtlsdr_demod_read_reg(dev, 0x0a, 0x01, 1, &dummy);
    return ESP_OK;
}

static esp_err_t rtlsdr_i2c_write(rtlsdr_t *dev, uint8_t i2c_addr, const uint8_t *data, uint8_t len)
{
    return rtlsdr_write_array(dev, RTL_BLOCK_IIC, i2c_addr, data, len);
}

static esp_err_t rtlsdr_i2c_read(rtlsdr_t *dev, uint8_t i2c_addr, uint8_t *data, uint8_t len)
{
    return rtlsdr_read_array(dev, RTL_BLOCK_IIC, i2c_addr, data, len);
}

static esp_err_t rtlsdr_i2c_read_reg(rtlsdr_t *dev, uint8_t i2c_addr, uint8_t reg, uint8_t *out)
{
    ESP_RETURN_ON_ERROR(rtlsdr_i2c_write(dev, i2c_addr, &reg, 1), TAG, "i2c reg addr");
    return rtlsdr_i2c_read(dev, i2c_addr, out, 1);
}

static esp_err_t rtlsdr_set_i2c_repeater(rtlsdr_t *dev, bool on)
{
    return rtlsdr_demod_write_reg(dev, 1, 0x01, on ? 0x18 : 0x10, 1);
}

static esp_err_t rtlsdr_set_if_freq(rtlsdr_t *dev, uint32_t freq_hz)
{
    int32_t if_freq = (int32_t)(((uint64_t)freq_hz << 22) / RTL_XTAL_HZ) * -1;

    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, 0x19, (uint8_t)((if_freq >> 16) & 0x3f), 1),
                        TAG, "if freq high");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, 0x1a, (uint8_t)((if_freq >> 8) & 0xff), 1),
                        TAG, "if freq mid");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, 0x1b, (uint8_t)(if_freq & 0xff), 1),
                        TAG, "if freq low");
    return ESP_OK;
}

static uint8_t r82xx_bitrev(uint8_t byte)
{
    static const uint8_t lut[16] = {
        0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
        0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf,
    };
    return (uint8_t)((lut[byte & 0x0f] << 4) | lut[byte >> 4]);
}

static void r82xx_shadow_store(rtlsdr_t *dev, uint8_t reg, const uint8_t *val, int len)
{
    int offset = reg - R82XX_REG_SHADOW_START;
    if (offset < 0) {
        len += offset;
        val -= offset;
        offset = 0;
    }
    if (len <= 0) {
        return;
    }
    if (len > R82XX_NUM_REGS - offset) {
        len = R82XX_NUM_REGS - offset;
    }
    memcpy(&dev->r82xx_regs[offset], val, len);
}

static bool r82xx_shadow_equal(rtlsdr_t *dev, uint8_t reg, const uint8_t *val, int len)
{
    int offset = reg - R82XX_REG_SHADOW_START;
    if (offset < 0 || len < 0 || len > R82XX_NUM_REGS - offset) {
        return false;
    }
    return memcmp(&dev->r82xx_regs[offset], val, len) == 0;
}

static esp_err_t r82xx_write(rtlsdr_t *dev, uint8_t reg, const uint8_t *val, unsigned int len)
{
    uint8_t buf[R82XX_MAX_I2C_MSG_LEN];
    unsigned int pos = 0;

    if (r82xx_shadow_equal(dev, reg, val, (int)len)) {
        return ESP_OK;
    }
    r82xx_shadow_store(dev, reg, val, (int)len);

    while (len > 0) {
        unsigned int size = len;
        if (size > R82XX_MAX_I2C_MSG_LEN - 1) {
            size = R82XX_MAX_I2C_MSG_LEN - 1;
        }
        buf[0] = reg;
        memcpy(&buf[1], &val[pos], size);
        ESP_RETURN_ON_ERROR(rtlsdr_i2c_write(dev, dev->tuner_i2c_addr, buf, (uint8_t)(size + 1)),
                            TAG, "r82xx write");
        reg += size;
        pos += size;
        len -= size;
    }

    return ESP_OK;
}

static esp_err_t r82xx_write_reg(rtlsdr_t *dev, uint8_t reg, uint8_t val)
{
    return r82xx_write(dev, reg, &val, 1);
}

static int r82xx_read_cache_reg(rtlsdr_t *dev, uint8_t reg)
{
    int offset = reg - R82XX_REG_SHADOW_START;
    if (offset >= 0 && offset < R82XX_NUM_REGS) {
        return dev->r82xx_regs[offset];
    }
    return -1;
}

static esp_err_t r82xx_write_reg_mask(rtlsdr_t *dev, uint8_t reg, uint8_t val, uint8_t bit_mask)
{
    int current = r82xx_read_cache_reg(dev, reg);
    ESP_RETURN_ON_FALSE(current >= 0, ESP_ERR_INVALID_ARG, TAG, "bad r82xx reg 0x%02x", reg);
    uint8_t next = (uint8_t)((current & ~bit_mask) | (val & bit_mask));
    return r82xx_write(dev, reg, &next, 1);
}

static esp_err_t r82xx_read(rtlsdr_t *dev, uint8_t reg, uint8_t *val, uint8_t len)
{
    uint8_t data[R82XX_MAX_I2C_MSG_LEN] = {0};
    ESP_RETURN_ON_FALSE(len <= sizeof(data), ESP_ERR_INVALID_ARG, TAG, "r82xx read too large");
    ESP_RETURN_ON_ERROR(rtlsdr_i2c_write(dev, dev->tuner_i2c_addr, &reg, 1), TAG, "r82xx read addr");
    ESP_RETURN_ON_ERROR(rtlsdr_i2c_read(dev, dev->tuner_i2c_addr, data, len), TAG, "r82xx read");
    for (uint8_t i = 0; i < len; i++) {
        val[i] = r82xx_bitrev(data[i]);
    }
    return ESP_OK;
}

static uint8_t mask_reg8(uint8_t reg, uint8_t val, uint8_t mask)
{
    return (uint8_t)((reg & ~mask) | (val & mask));
}

static esp_err_t r82xx_set_mux(rtlsdr_t *dev, uint32_t freq_hz)
{
    size_t range_idx = 0;
    uint32_t freq_mhz = freq_hz / 1000000U;
    for (size_t i = 0; i + 1 < sizeof(r82xx_freq_ranges) / sizeof(r82xx_freq_ranges[0]); i++) {
        if (freq_mhz < r82xx_freq_ranges[i + 1].freq_mhz) {
            range_idx = i;
            break;
        }
        range_idx = i + 1;
    }

    const r82xx_freq_range_t *range = &r82xx_freq_ranges[range_idx];
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x17, range->open_d, 0x08), TAG, "r82xx open drain");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x1a, range->rf_mux_ploy, 0xc3), TAG, "r82xx mux");
    ESP_RETURN_ON_ERROR(r82xx_write_reg(dev, 0x1b, range->tf_c), TAG, "r82xx tf band");

    uint8_t cap;
    switch (dev->r82xx_xtal_cap_sel) {
    case R82XX_XTAL_LOW_CAP_30P:
    case R82XX_XTAL_LOW_CAP_20P:
        cap = (uint8_t)(range->xtal_cap20p | 0x08);
        break;
    case R82XX_XTAL_LOW_CAP_10P:
        cap = (uint8_t)(range->xtal_cap10p | 0x08);
        break;
    case R82XX_XTAL_HIGH_CAP_0P:
        cap = (uint8_t)(range->xtal_cap0p | 0x00);
        break;
    case R82XX_XTAL_LOW_CAP_0P:
    default:
        cap = (uint8_t)(range->xtal_cap0p | 0x08);
        break;
    }
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x10, cap, 0x0b), TAG, "r82xx xtal cap");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x08, 0x00, 0x3f), TAG, "r82xx tracking 1");
    return r82xx_write_reg_mask(dev, 0x09, 0x00, 0x3f);
}

static esp_err_t r82xx_set_pll(rtlsdr_t *dev, uint32_t freq_hz)
{
    uint32_t freq_khz = (freq_hz + 500U) / 1000U;
    uint8_t mix_div = 2;
    uint8_t div_num = 0;
    bool found_divider = false;
    uint8_t data[5] = {0};
    uint8_t regs[7] = {0};

    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x1a, 0x00, 0x0c), TAG, "r82xx pll autotune");
    memcpy(regs, &dev->r82xx_regs[0x10 - R82XX_REG_SHADOW_START], sizeof(regs));
    regs[0] = mask_reg8(regs[0], 0x00, 0x10);
    regs[2] = mask_reg8(regs[2], 0x80, 0xe0);

    while (mix_div <= 64) {
        uint32_t mixed_khz = freq_khz * mix_div;
        if (mixed_khz >= 1770000U && mixed_khz < 3540000U) {
            uint8_t div_buf = mix_div;
            while (div_buf > 2) {
                div_buf >>= 1;
                div_num++;
            }
            found_divider = true;
            break;
        }
        mix_div <<= 1;
    }
    ESP_RETURN_ON_FALSE(found_divider, ESP_ERR_INVALID_ARG, TAG, "no r82xx PLL divider for %lu Hz",
                        (unsigned long)freq_hz);

    ESP_RETURN_ON_ERROR(r82xx_read(dev, 0x00, data, sizeof(data)), TAG, "r82xx pll status");

    uint8_t vco_power_ref = (dev->tuner_chip == R82XX_CHIP_R828D) ? 1 : 2;
    uint8_t vco_fine_tune = (uint8_t)((data[4] & 0x30) >> 4);
    if (vco_fine_tune > vco_power_ref && div_num > 0) {
        div_num--;
    } else if (vco_fine_tune < vco_power_ref) {
        div_num++;
    }
    regs[0] = mask_reg8(regs[0], (uint8_t)(div_num << 5), 0xe0);

    uint64_t vco_freq = (uint64_t)freq_hz * mix_div;
    uint64_t vco_div = (R82XX_TUNER_XTAL_HZ + 65536ULL * vco_freq) / (2ULL * R82XX_TUNER_XTAL_HZ);
    uint32_t nint = (uint32_t)(vco_div / 65536ULL);
    uint32_t sdm = (uint32_t)(vco_div % 65536ULL);
    ESP_RETURN_ON_FALSE(nint >= 13 && nint <= (uint32_t)((128 / vco_power_ref) - 1),
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "bad r82xx PLL nint %lu for %lu Hz",
                        (unsigned long)nint,
                        (unsigned long)freq_hz);

    uint8_t ni = (uint8_t)((nint - 13U) / 4U);
    uint8_t si = (uint8_t)(nint - 4U * ni - 13U);
    regs[4] = (uint8_t)(ni + (si << 6));
    regs[2] = mask_reg8(regs[2], sdm == 0 ? 0x08 : 0x00, 0x08);
    regs[5] = (uint8_t)(sdm & 0xff);
    regs[6] = (uint8_t)(sdm >> 8);

    ESP_RETURN_ON_ERROR(r82xx_write(dev, 0x10, regs, sizeof(regs)), TAG, "r82xx pll regs");

    for (int i = 0; i < 2; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_RETURN_ON_ERROR(r82xx_read(dev, 0x00, data, 3), TAG, "r82xx pll lock");
        if (data[2] & 0x40) {
            dev->r82xx_has_lock = true;
            return r82xx_write_reg_mask(dev, 0x1a, 0x08, 0x08);
        }
        if (i == 0) {
            ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x12, 0x60, 0xe0), TAG, "r82xx vco current");
        }
    }

    dev->r82xx_has_lock = false;
    ESP_LOGW(TAG, "R82xx PLL did not lock at %lu Hz", (unsigned long)freq_hz);
    return ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t r82xx_set_tv_standard(rtlsdr_t *dev)
{
    uint8_t data[5] = {0};
    dev->r82xx_int_freq_hz = R82XX_IF_FREQ_HZ;
    memcpy(dev->r82xx_regs, r82xx_init_array, sizeof(r82xx_init_array));

    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x0c, 0x00, 0x0f), TAG, "r82xx init flag");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x13, R82XX_VER_NUM, 0x3f), TAG, "r82xx version");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x1d, 0x00, 0x38), TAG, "r82xx lna cal");
    vTaskDelay(pdMS_TO_TICKS(2));

    bool calibrated = false;
    for (int i = 0; i < 2; i++) {
        ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x0b, 0x6b, 0x60), TAG, "r82xx filter cap");
        ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x0f, 0x04, 0x04), TAG, "r82xx cal clock on");
        ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x10, 0x00, 0x03), TAG, "r82xx cal xtal cap");
        ESP_RETURN_ON_ERROR(r82xx_set_pll(dev, 56000000U), TAG, "r82xx filter pll");
        ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x0b, 0x10, 0x10), TAG, "r82xx cal trigger");
        vTaskDelay(pdMS_TO_TICKS(2));
        ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x0b, 0x00, 0x10), TAG, "r82xx cal stop");
        ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x0f, 0x00, 0x04), TAG, "r82xx cal clock off");
        ESP_RETURN_ON_ERROR(r82xx_read(dev, 0x00, data, sizeof(data)), TAG, "r82xx cal read");

        dev->r82xx_fil_cal_code = data[4] & 0x0f;
        if (dev->r82xx_fil_cal_code && dev->r82xx_fil_cal_code != 0x0f) {
            calibrated = true;
            break;
        }
    }
    if (!calibrated && dev->r82xx_fil_cal_code == 0x0f) {
        dev->r82xx_fil_cal_code = 0;
    }

    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x0a, (uint8_t)(0x10 | dev->r82xx_fil_cal_code), 0x1f),
                        TAG, "r82xx filter cal code");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x0b, 0x6b, 0xef), TAG, "r82xx filter hp");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x07, 0x00, 0x80), TAG, "r82xx image");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x06, 0x10, 0x30), TAG, "r82xx filter gain");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x1e, 0x60, 0x60), TAG, "r82xx filter extension");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x05, 0x01, 0x80), TAG, "r82xx loop through");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x1f, 0x00, 0x80), TAG, "r82xx loop attenuation");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x0f, 0x00, 0x80), TAG, "r82xx widest off");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x19, 0x60, 0x60), TAG, "r82xx poly current");
    return ESP_OK;
}

static esp_err_t r82xx_sysfreq_sel(rtlsdr_t *dev)
{
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x1d, 0xe5, 0xc7), TAG, "r82xx lna top");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x1c, 0x24, 0xf8), TAG, "r82xx mixer top");
    ESP_RETURN_ON_ERROR(r82xx_write_reg(dev, 0x0d, 0x53), TAG, "r82xx lna thresh");
    ESP_RETURN_ON_ERROR(r82xx_write_reg(dev, 0x0e, 0x75), TAG, "r82xx mixer thresh");
    dev->r82xx_input = 0x00;
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x05, 0x00, 0x60), TAG, "r82xx input");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x06, 0x00, 0x08), TAG, "r82xx cable2");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x11, 0x38, 0x38), TAG, "r82xx charge pump");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x17, 0x30, 0x30), TAG, "r82xx div buffer");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x0a, 0x40, 0x60), TAG, "r82xx filter current");

    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x1d, 0x00, 0x38), TAG, "r82xx lna bottom");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x1c, 0x00, 0x04), TAG, "r82xx mixer normal");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x06, 0x00, 0x40), TAG, "r82xx predetect off");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x1a, 0x30, 0x30), TAG, "r82xx agc clock fast");
    vTaskDelay(pdMS_TO_TICKS(250));
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x1d, 0x18, 0x38), TAG, "r82xx lna top final");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x1c, 0x24, 0x04), TAG, "r82xx mixer discharge");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x1e, 14, 0x1f), TAG, "r82xx lna discharge");
    return r82xx_write_reg_mask(dev, 0x1a, 0x20, 0x30);
}

static esp_err_t r82xx_set_gain(rtlsdr_t *dev, bool manual, int gain_tenths_db)
{
    static const int lna_steps[] = {
        0, 9, 13, 40, 38, 13, 31, 22, 26, 31, 26, 14, 19, 5, 35, 13,
    };
    static const int mixer_steps[] = {
        0, 5, 10, 10, 19, 9, 10, 25, 17, 10, 8, 16, 13, 6, 3, -8,
    };

    if (!manual) {
        ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x05, 0x00, 0x10), TAG, "r82xx lna auto");
        ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x07, 0x10, 0x10), TAG, "r82xx mixer auto");
        return r82xx_write_reg_mask(dev, 0x0c, 0x0b, 0x9f);
    }

    uint8_t lna_index = 0;
    uint8_t mixer_index = 0;
    int total_gain = 0;
    for (int i = 0; i < 15; i++) {
        if (total_gain >= gain_tenths_db) {
            break;
        }
        total_gain += lna_steps[++lna_index];
        if (total_gain >= gain_tenths_db) {
            break;
        }
        total_gain += mixer_steps[++mixer_index];
    }

    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x05, 0x10, 0x10), TAG, "r82xx lna manual");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x07, 0x00, 0x10), TAG, "r82xx mixer manual");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x0c, 0x08, 0x9f), TAG, "r82xx vga fixed");
    ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x05, lna_index, 0x0f), TAG, "r82xx lna gain");
    return r82xx_write_reg_mask(dev, 0x07, mixer_index, 0x0f);
}

static esp_err_t r82xx_probe_addr(rtlsdr_t *dev,
                                  uint8_t i2c_addr,
                                  r82xx_chip_t chip,
                                  const char *name)
{
    uint8_t reg = 0;

    ESP_RETURN_ON_ERROR(rtlsdr_set_i2c_repeater(dev, true), TAG, "i2c repeater on");
    esp_err_t ret = rtlsdr_i2c_read_reg(dev, i2c_addr, R82XX_CHECK_ADDR, &reg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "%s probe addr 0x%02x failed: %s", name, i2c_addr, esp_err_to_name(ret));
        return ret;
    }

    esp_err_t off_ret = rtlsdr_set_i2c_repeater(dev, false);
    if (off_ret != ESP_OK) {
        ESP_LOGW(TAG, "%s probe repeater-off failed: %s", name, esp_err_to_name(off_ret));
        return off_ret;
    }

    if (reg != R82XX_CHECK_VAL) {
        ESP_LOGI(TAG, "%s probe addr 0x%02x read 0x%02x", name, i2c_addr, reg);
        return ESP_ERR_NOT_FOUND;
    }

    dev->tuner_chip = chip;
    dev->tuner_i2c_addr = i2c_addr;
    ESP_LOGI(TAG, "Found Rafael %s tuner at 0x%02x", name, i2c_addr);
    return ESP_OK;
}

static esp_err_t r82xx_probe_tuner(rtlsdr_t *dev)
{
    esp_err_t ret = r82xx_probe_addr(dev, R82XX_I2C_ADDR_R820T, R82XX_CHIP_R820T, "R820T/R820T2");
    if (ret == ESP_OK) {
        return ESP_OK;
    }
    if (ret != ESP_OK && ret != ESP_ERR_NOT_FOUND) {
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    ret = r82xx_probe_addr(dev, R82XX_I2C_ADDR_R828D, R82XX_CHIP_R828D, "R828D/R860");
    if (ret == ESP_OK) {
        ESP_LOGW(TAG, "Found Rafael R828D/R860 tuner; using 28.8 MHz xtal path");
        return ESP_OK;
    }

    dev->tuner_chip = R82XX_CHIP_NONE;
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t r82xx_init_tuner(rtlsdr_t *dev)
{
    if (dev->tuner_chip == R82XX_CHIP_NONE) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    dev->r82xx_xtal_cap_sel = R82XX_XTAL_HIGH_CAP_0P;
    memset(dev->r82xx_regs, 0, sizeof(dev->r82xx_regs));

    ESP_RETURN_ON_ERROR(rtlsdr_set_i2c_repeater(dev, true), TAG, "i2c repeater on");
    esp_err_t ret = r82xx_write(dev, 0x05, r82xx_init_array, sizeof(r82xx_init_array));
    if (ret == ESP_OK) {
        ret = r82xx_set_tv_standard(dev);
    }
    if (ret == ESP_OK) {
        ret = r82xx_sysfreq_sel(dev);
    }
    if (ret == ESP_OK) {
        ret = r82xx_set_gain(dev, dev->cfg.tuner_gain_tenths_db > 0, dev->cfg.tuner_gain_tenths_db);
    }
    (void)rtlsdr_set_i2c_repeater(dev, false);

    if (ret == ESP_OK) {
        dev->r82xx_init_done = true;
    }
    return ret;
}

static esp_err_t r82xx_set_freq(rtlsdr_t *dev, uint32_t center_freq_hz)
{
    uint32_t lo_freq = center_freq_hz + dev->r82xx_int_freq_hz;
    ESP_RETURN_ON_ERROR(r82xx_set_mux(dev, lo_freq), TAG, "r82xx set mux");
    ESP_RETURN_ON_ERROR(r82xx_set_pll(dev, lo_freq), TAG, "r82xx set pll");

    if (dev->tuner_chip == R82XX_CHIP_R828D) {
        uint8_t air_cable1 = (center_freq_hz > 345000000U) ? 0x00 : 0x60;
        if (air_cable1 != dev->r82xx_input) {
            dev->r82xx_input = air_cable1;
            ESP_RETURN_ON_ERROR(r82xx_write_reg_mask(dev, 0x05, air_cable1, 0x60),
                                TAG, "r828d input switch");
        }
    }

    return ESP_OK;
}

static esp_err_t rtlsdr_apply_center_freq(rtlsdr_t *dev, uint32_t center_freq_hz)
{
    if (!dev->dev_hdl || dev->tuner_chip == R82XX_CHIP_NONE || !dev->r82xx_init_done) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(rtlsdr_set_i2c_repeater(dev, true), TAG, "i2c repeater on");
    esp_err_t ret = r82xx_set_freq(dev, center_freq_hz);
    (void)rtlsdr_set_i2c_repeater(dev, false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "R82xx tune %lu Hz failed: %s", (unsigned long)center_freq_hz, esp_err_to_name(ret));
        return ret;
    }

    dev->cfg.center_freq_hz = center_freq_hz;
    ESP_LOGI(TAG, "RTL-SDR tuned to %lu Hz", (unsigned long)center_freq_hz);
    return ESP_OK;
}

static bool rtlsdr_take_pending_tune(rtlsdr_t *dev, uint32_t *freq_hz)
{
    bool pending;
    portENTER_CRITICAL(&dev->tune_lock);
    pending = dev->tune_pending;
    if (pending) {
        *freq_hz = dev->pending_center_freq_hz;
        dev->tune_pending = false;
    }
    portEXIT_CRITICAL(&dev->tune_lock);
    return pending;
}

static esp_err_t rtlsdr_set_fir(rtlsdr_t *dev)
{
    uint8_t fir[20];

    for (int i = 0; i < 8; i++) {
        int val = fir_default[i];
        ESP_RETURN_ON_FALSE(val >= -128 && val <= 127, ESP_ERR_INVALID_ARG, TAG, "bad FIR coefficient");
        fir[i] = (uint8_t)val;
    }

    for (int i = 0; i < 8; i += 2) {
        int val0 = fir_default[8 + i];
        int val1 = fir_default[8 + i + 1];
        ESP_RETURN_ON_FALSE(val0 >= -2048 && val0 <= 2047 && val1 >= -2048 && val1 <= 2047,
                            ESP_ERR_INVALID_ARG, TAG, "bad FIR coefficient");
        fir[8 + i * 3 / 2] = (uint8_t)(val0 >> 4);
        fir[8 + i * 3 / 2 + 1] = (uint8_t)((val0 << 4) | ((val1 >> 8) & 0x0f));
        fir[8 + i * 3 / 2 + 2] = (uint8_t)val1;
    }

    for (int i = 0; i < (int)sizeof(fir); i++) {
        ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, (uint16_t)(0x1c + i), fir[i], 1),
                            TAG, "FIR coeff");
    }
    return ESP_OK;
}

static esp_err_t rtlsdr_set_sample_rate(rtlsdr_t *dev, uint32_t sample_rate)
{
    if (sample_rate == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t rtl_xtal = 28800000;
    uint32_t ratio = (uint32_t)(((uint64_t)rtl_xtal * 0x400000ULL) / sample_rate);
    ratio &= 0x0ffffffc;
    ESP_LOGI(TAG, "sample rate %lu Hz, ratio 0x%08lx", (unsigned long)sample_rate, (unsigned long)ratio);

    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, 0x9f, (uint16_t)(ratio >> 16), 2), TAG, "sample ratio high");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, 0xa1, (uint16_t)(ratio & 0xffff), 2), TAG, "sample ratio low");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, 0x01, 0x14, 1), TAG, "demod reset");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, 0x01, 0x10, 1), TAG, "demod run");
    return ESP_OK;
}

static esp_err_t rtlsdr_reset_buffer(rtlsdr_t *dev)
{
    ESP_RETURN_ON_ERROR(rtlsdr_write_reg(dev, RTL_BLOCK_USB, USB_EPA_CTL, 0x1002, 2), TAG, "reset buffer on");
    ESP_RETURN_ON_ERROR(rtlsdr_write_reg(dev, RTL_BLOCK_USB, USB_EPA_CTL, 0x0000, 2), TAG, "reset buffer off");
    return ESP_OK;
}

static esp_err_t rtlsdr_minimal_init(rtlsdr_t *dev)
{
    set_status(dev, "RTL2832U CONTROL INIT");

    ESP_RETURN_ON_ERROR(rtlsdr_write_reg(dev, RTL_BLOCK_USB, USB_SYSCTL, 0x09, 1), TAG, "usb sysctl");
    ESP_RETURN_ON_ERROR(rtlsdr_write_reg(dev, RTL_BLOCK_USB, USB_EPA_MAXPKT, 0x0002, 2), TAG, "usb epa mps");
    ESP_RETURN_ON_ERROR(rtlsdr_write_reg(dev, RTL_BLOCK_USB, USB_EPA_CTL, 0x1002, 2), TAG, "usb epa ctl");

    ESP_RETURN_ON_ERROR(rtlsdr_write_reg(dev, RTL_BLOCK_SYS, SYS_DEMOD_CTL_1, 0x22, 1), TAG, "demod power");
    ESP_RETURN_ON_ERROR(rtlsdr_write_reg(dev, RTL_BLOCK_SYS, SYS_DEMOD_CTL, 0xe8, 1), TAG, "demod ctl");
    ESP_RETURN_ON_ERROR(rtlsdr_write_reg(dev, RTL_BLOCK_SYS, SYS_IR_SUSPEND, 0x00, 1), TAG, "ir suspend");
    ESP_RETURN_ON_ERROR(rtlsdr_write_reg(dev, RTL_BLOCK_SYS, SYS_GPOE, 0x00, 1), TAG, "gpio en");
    ESP_RETURN_ON_ERROR(rtlsdr_write_reg(dev, RTL_BLOCK_SYS, SYS_GPO, 0x00, 1), TAG, "gpio val");

    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, 0x01, 0x14, 1), TAG, "demod adc");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, 0x01, 0x10, 1), TAG, "demod adc run");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, 0x15, 0x00, 1), TAG, "disable spectrum inversion");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, 0x16, 0x0000, 2), TAG, "disable adjacent rejection");
    for (uint8_t i = 0; i < 6; i++) {
        ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, (uint16_t)(0x16 + i), 0x00, 1), TAG, "clear DDC");
    }
    ESP_RETURN_ON_ERROR(rtlsdr_set_fir(dev), TAG, "set FIR");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 0, 0x19, 0x05, 1), TAG, "enable SDR mode");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, 0x93, 0xf0, 1), TAG, "FSM state 0");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, 0x94, 0x0f, 1), TAG, "FSM state 1");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, 0x11, 0x00, 1), TAG, "disable DAGC");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, 0x04, 0x00, 1), TAG, "disable RF/IF AGC");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 0, 0x61, 0x60, 1), TAG, "disable PID filter");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 0, 0x06, 0x80, 1), TAG, "ADC datapath");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, 0xb1, 0x1b, 1), TAG, "zero IF mode");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 0, 0x0d, 0x83, 1), TAG, "disable clock output");
    ESP_RETURN_ON_ERROR(rtlsdr_set_sample_rate(dev, dev->cfg.sample_rate_hz), TAG, "sample rate");

    ESP_RETURN_ON_ERROR(rtlsdr_reset_buffer(dev), TAG, "reset USB buffer");
    return ESP_OK;
}

static esp_err_t rtlsdr_configure_r82xx_if(rtlsdr_t *dev)
{
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, 0xb1, 0x1a, 1), TAG, "disable zero IF");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 0, 0x08, 0x4d, 1), TAG, "use I ADC");
    ESP_RETURN_ON_ERROR(rtlsdr_set_if_freq(dev, R82XX_IF_FREQ_HZ), TAG, "set R82xx IF");
    ESP_RETURN_ON_ERROR(rtlsdr_demod_write_reg(dev, 1, 0x15, 0x01, 1), TAG, "enable spectrum inversion");
    return rtlsdr_demod_write_reg(dev, 0, 0x06, 0x80, 1);
}

static bool find_bulk_endpoint(rtlsdr_t *dev, const usb_config_desc_t *config)
{
    int offset = 0;
    const usb_intf_desc_t *intf = NULL;

    while ((intf = usb_parse_interface_descriptor(config, dev->interface_num, dev->alt_setting, &offset)) != NULL) {
        break;
    }

    if (!intf) {
        offset = 0;
        const usb_standard_desc_t *desc = (const usb_standard_desc_t *)config;
        while ((desc = usb_parse_next_descriptor_of_type(desc,
                                                         config->wTotalLength,
                                                         USB_B_DESCRIPTOR_TYPE_INTERFACE,
                                                         &offset)) != NULL) {
            const usb_intf_desc_t *candidate = (const usb_intf_desc_t *)desc;
            for (int ep_idx = 0; ep_idx < candidate->bNumEndpoints; ep_idx++) {
                int ep_offset = offset;
                const usb_ep_desc_t *ep = usb_parse_endpoint_descriptor_by_index(candidate,
                                                                                 ep_idx,
                                                                                 config->wTotalLength,
                                                                                 &ep_offset);
                if (ep && USB_EP_DESC_GET_XFERTYPE(ep) == USB_TRANSFER_TYPE_BULK &&
                    USB_EP_DESC_GET_EP_DIR(ep)) {
                    dev->interface_num = candidate->bInterfaceNumber;
                    dev->alt_setting = candidate->bAlternateSetting;
                    dev->bulk_in_ep = ep->bEndpointAddress;
                    dev->bulk_mps = USB_EP_DESC_GET_MPS(ep);
                    return true;
                }
            }
        }
        return false;
    }

    for (int ep_idx = 0; ep_idx < intf->bNumEndpoints; ep_idx++) {
        int ep_offset = offset;
        const usb_ep_desc_t *ep = usb_parse_endpoint_descriptor_by_index(intf,
                                                                         ep_idx,
                                                                         config->wTotalLength,
                                                                         &ep_offset);
        if (ep && USB_EP_DESC_GET_XFERTYPE(ep) == USB_TRANSFER_TYPE_BULK && USB_EP_DESC_GET_EP_DIR(ep)) {
            dev->bulk_in_ep = ep->bEndpointAddress;
            dev->bulk_mps = USB_EP_DESC_GET_MPS(ep);
            return true;
        }
    }
    return false;
}

static void bulk_done_cb(usb_transfer_t *transfer)
{
    rtlsdr_t *dev = (rtlsdr_t *)transfer->context;
    if (!dev || dev->shutting_down || !dev->streaming) {
        return;
    }

    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED && transfer->actual_num_bytes > 0) {
        if (dev->samples_cb) {
            dev->samples_cb(transfer->data_buffer, (size_t)transfer->actual_num_bytes, dev->cb_ctx);
        }
        transfer->num_bytes = usb_round_up_to_mps(CONFIG_RTLSDR_BULK_TRANSFER_SIZE, dev->bulk_mps);
        esp_err_t ret = usb_host_transfer_submit(transfer);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "bulk resubmit failed: %s", esp_err_to_name(ret));
            dev->streaming = false;
        }
    } else {
        ESP_LOGW(TAG, "bulk transfer status %d len %d", transfer->status, transfer->actual_num_bytes);
        dev->streaming = false;
    }
}

static esp_err_t start_bulk_stream(rtlsdr_t *dev)
{
    int transfer_size = usb_round_up_to_mps(CONFIG_RTLSDR_BULK_TRANSFER_SIZE, dev->bulk_mps);
    ESP_RETURN_ON_FALSE(transfer_size > 0, ESP_ERR_INVALID_STATE, TAG, "bad bulk MPS");

    for (int i = 0; i < CONFIG_RTLSDR_NUM_TRANSFERS; i++) {
        ESP_RETURN_ON_ERROR(usb_host_transfer_alloc(transfer_size, 0, &dev->transfers[i]), TAG, "alloc bulk");
        dev->transfers[i]->device_handle = dev->dev_hdl;
        dev->transfers[i]->bEndpointAddress = dev->bulk_in_ep;
        dev->transfers[i]->num_bytes = transfer_size;
        dev->transfers[i]->callback = bulk_done_cb;
        dev->transfers[i]->context = dev;
    }

    dev->streaming = true;
    for (int i = 0; i < CONFIG_RTLSDR_NUM_TRANSFERS; i++) {
        esp_err_t ret = usb_host_transfer_submit(dev->transfers[i]);
        if (ret != ESP_OK) {
            dev->streaming = false;
            return ret;
        }
    }
    set_status(dev, "STREAMING RTLSDR IQ");
    return ESP_OK;
}

static void free_bulk_transfers(rtlsdr_t *dev)
{
    for (int i = 0; i < CONFIG_RTLSDR_NUM_TRANSFERS; i++) {
        if (dev->transfers[i]) {
            usb_host_transfer_free(dev->transfers[i]);
            dev->transfers[i] = NULL;
        }
    }
}

static void close_device(rtlsdr_t *dev)
{
    dev->streaming = false;
    dev->r82xx_init_done = false;
    dev->r82xx_has_lock = false;
    dev->tuner_chip = R82XX_CHIP_NONE;
    if (dev->dev_hdl) {
        if (dev->bulk_in_ep) {
            usb_host_endpoint_halt(dev->dev_hdl, dev->bulk_in_ep);
            usb_host_endpoint_flush(dev->dev_hdl, dev->bulk_in_ep);
        }
        free_bulk_transfers(dev);
        usb_host_interface_release(dev->client, dev->dev_hdl, dev->interface_num);
        usb_host_device_close(dev->client, dev->dev_hdl);
        dev->dev_hdl = NULL;
    }
}

static esp_err_t open_if_rtlsdr(rtlsdr_t *dev, uint8_t addr)
{
    ESP_RETURN_ON_ERROR(usb_host_device_open(dev->client, addr, &dev->dev_hdl), TAG, "device open");

    const usb_device_desc_t *desc = NULL;
    ESP_RETURN_ON_ERROR(usb_host_get_device_descriptor(dev->dev_hdl, &desc), TAG, "device desc");
    ESP_LOGI(TAG, "USB device %04x:%04x", desc->idVendor, desc->idProduct);
    if (!is_rtlsdr_vid_pid(desc->idVendor, desc->idProduct)) {
        usb_host_device_close(dev->client, dev->dev_hdl);
        dev->dev_hdl = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    const usb_config_desc_t *config = NULL;
    ESP_RETURN_ON_ERROR(usb_host_get_active_config_descriptor(dev->dev_hdl, &config), TAG, "config desc");
    dev->interface_num = 0;
    dev->alt_setting = 0;
    ESP_RETURN_ON_FALSE(find_bulk_endpoint(dev, config), ESP_ERR_NOT_FOUND, TAG, "bulk IN endpoint not found");
    ESP_LOGI(TAG, "RTL-SDR bulk IN endpoint 0x%02x MPS %u interface %u alt %u",
             dev->bulk_in_ep, dev->bulk_mps, dev->interface_num, dev->alt_setting);

    ESP_RETURN_ON_ERROR(usb_host_interface_claim(dev->client, dev->dev_hdl, dev->interface_num, dev->alt_setting),
                        TAG, "claim interface");

    esp_err_t ret = ESP_OK;
    if (dev->cfg.baseband_init) {
        ret = rtlsdr_minimal_init(dev);
    } else {
        rtlsdr_reset_buffer(dev);
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "RTL2832U baseband init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = r82xx_probe_tuner(dev);
    if (ret != ESP_OK) {
        set_status(dev, "R820T/R828D TUNER NOT FOUND");
        ESP_LOGW(TAG, "Only R820T/R828D tuner control is implemented in this ESP32 driver");
        return ret;
    }
    ESP_RETURN_ON_ERROR(rtlsdr_configure_r82xx_if(dev), TAG, "configure R82xx IF");
    ESP_RETURN_ON_ERROR(r82xx_init_tuner(dev), TAG, "init R82xx tuner");
    ESP_RETURN_ON_ERROR(rtlsdr_apply_center_freq(dev, dev->cfg.center_freq_hz), TAG, "initial tune");
    ESP_RETURN_ON_ERROR(start_bulk_stream(dev), TAG, "start stream");
    return ESP_OK;
}

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    rtlsdr_t *dev = (rtlsdr_t *)arg;
    switch (event_msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        dev->new_dev_addr = event_msg->new_dev.address;
        dev->new_dev_pending = true;
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        if (event_msg->dev_gone.dev_hdl == dev->dev_hdl) {
            dev->gone = true;
            dev->streaming = false;
        }
        break;
    default:
        break;
    }
}

static void rtlsdr_task(void *arg)
{
    rtlsdr_t *dev = (rtlsdr_t *)arg;
    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = CLIENT_EVENT_MSGS,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = dev,
        },
    };

    esp_err_t ret = usb_host_client_register(&client_config, &dev->client);
    if (ret != ESP_OK) {
        set_status(dev, "USB CLIENT REGISTER FAILED");
        xSemaphoreGive(dev->done_sem);
        vTaskDelete(NULL);
        return;
    }

    ret = usb_host_service_power_on();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB root port power failed: %s", esp_err_to_name(ret));
        set_status(dev, "USB ROOT POWER FAILED");
        usb_host_client_deregister(dev->client);
        dev->client = NULL;
        xSemaphoreGive(dev->done_sem);
        vTaskDelete(NULL);
        return;
    }

    set_status(dev, "WAITING FOR RTL2832U");
    while (!dev->shutting_down) {
        ret = usb_host_client_handle_events(dev->client, pdMS_TO_TICKS(100));
        if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "client events: %s", esp_err_to_name(ret));
        }

        if (dev->new_dev_pending) {
            uint8_t addr = dev->new_dev_addr;
            dev->new_dev_pending = false;
            if (!dev->dev_hdl) {
                ret = open_if_rtlsdr(dev, addr);
                if (ret == ESP_ERR_NOT_FOUND) {
                    set_status(dev, "USB DEVICE IS NOT RTLSDR");
                } else if (ret == ESP_ERR_NOT_SUPPORTED) {
                    set_status(dev, "R820T/R828D TUNER NOT FOUND");
                } else if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "open RTL-SDR failed: %s", esp_err_to_name(ret));
                    set_status(dev, "RTLSDR OPEN FAILED");
                    close_device(dev);
                }
            }
        }

        if (dev->gone) {
            dev->gone = false;
            close_device(dev);
            set_status(dev, "RTLSDR DISCONNECTED");
        }

        uint32_t tune_hz = 0;
        if (rtlsdr_take_pending_tune(dev, &tune_hz)) {
            if (dev->dev_hdl) {
                ret = rtlsdr_apply_center_freq(dev, tune_hz);
                if (ret == ESP_OK) {
                    char status[64];
                    snprintf(status,
                             sizeof(status),
                             "TUNED %lu.%03lu MHz",
                             (unsigned long)(tune_hz / 1000000U),
                             (unsigned long)((tune_hz / 1000U) % 1000U));
                    set_status(dev, status);
                } else {
                    set_status(dev, "RTLSDR TUNE FAILED");
                }
            } else {
                dev->cfg.center_freq_hz = tune_hz;
            }
        }
    }

    close_device(dev);
    if (dev->client) {
        usb_host_client_deregister(dev->client);
        dev->client = NULL;
    }
    xSemaphoreGive(dev->done_sem);
    vTaskDelete(NULL);
}

esp_err_t rtlsdr_start(rtlsdr_t **out_dev,
                       const rtlsdr_config_t *config,
                       rtlsdr_samples_cb_t samples_cb,
                       rtlsdr_status_cb_t status_cb,
                       void *cb_ctx)
{
    ESP_RETURN_ON_FALSE(out_dev && config && samples_cb, ESP_ERR_INVALID_ARG, TAG, "bad args");
    rtlsdr_t *dev = calloc(1, sizeof(*dev));
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "no memory");
    dev->cfg = *config;
    dev->samples_cb = samples_cb;
    dev->status_cb = status_cb;
    dev->cb_ctx = cb_ctx;
    dev->tune_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    dev->pending_center_freq_hz = config->center_freq_hz;
    dev->done_sem = xSemaphoreCreateBinary();
    if (!dev->done_sem) {
        free(dev);
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(rtlsdr_task, "rtlsdr", 8192, dev, 6, &dev->task, 0);
    if (ok != pdTRUE) {
        vSemaphoreDelete(dev->done_sem);
        free(dev);
        return ESP_ERR_NO_MEM;
    }

    *out_dev = dev;
    return ESP_OK;
}

void rtlsdr_stop(rtlsdr_t *dev)
{
    if (!dev) {
        return;
    }
    dev->shutting_down = true;
    if (dev->client) {
        usb_host_client_unblock(dev->client);
    }
    xSemaphoreTake(dev->done_sem, pdMS_TO_TICKS(3000));
    vSemaphoreDelete(dev->done_sem);
    free(dev);
}

esp_err_t rtlsdr_set_center_freq_hz(rtlsdr_t *dev, uint32_t center_freq_hz)
{
    if (!dev || center_freq_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&dev->tune_lock);
    dev->pending_center_freq_hz = center_freq_hz;
    dev->tune_pending = true;
    portEXIT_CRITICAL(&dev->tune_lock);

    if (dev->client) {
        usb_host_client_unblock(dev->client);
    }
    return ESP_OK;
}

uint32_t rtlsdr_get_center_freq_hz(const rtlsdr_t *dev)
{
    return dev ? dev->cfg.center_freq_hz : 0;
}

uint32_t rtlsdr_get_sample_rate_hz(const rtlsdr_t *dev)
{
    return dev ? dev->cfg.sample_rate_hz : 0;
}
