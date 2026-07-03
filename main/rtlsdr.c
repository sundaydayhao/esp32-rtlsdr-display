#include "rtlsdr.h"

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#define RTLSDR_VID_REALTEK        0x0bda
#define RTLSDR_PID_RTL2832U       0x2832
#define RTLSDR_PID_RTL2838UHIDIR  0x2838

#define CTRL_TIMEOUT_MS           1000
#define CTRL_EVENT_POLL_MS        10
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

#define FIR_LEN                   16

static const int fir_default[FIR_LEN] = {
    -54, -36, -41, -40, -32, -14, 14, 53,
    101, 156, 215, 273, 327, 372, 404, 421,
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
    uint8_t new_dev_addr;
    bool new_dev_pending;
    bool gone;
    uint8_t interface_num;
    uint8_t alt_setting;
    uint8_t bulk_in_ep;
    uint16_t bulk_mps;
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
    uint32_t ratio = (rtl_xtal * 0x400000U) / sample_rate;
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
    ESP_LOGW(TAG, "External tuner PLL/gain programming is not implemented yet; add R820T/R828D support in rtlsdr.c for real RF tuning");
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

uint32_t rtlsdr_get_center_freq_hz(const rtlsdr_t *dev)
{
    return dev ? dev->cfg.center_freq_hz : 0;
}

uint32_t rtlsdr_get_sample_rate_hz(const rtlsdr_t *dev)
{
    return dev ? dev->cfg.sample_rate_hz : 0;
}
