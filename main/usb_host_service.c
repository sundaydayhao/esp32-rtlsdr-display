#include "usb_host_service.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "usb/usb_host.h"

static const char *TAG = "usb_host";
static TaskHandle_t s_host_task;
static bool s_started;
static bool s_root_powered;

static bool enum_filter_cb(const usb_device_desc_t *dev_desc, uint8_t *bConfigurationValue)
{
    (void)dev_desc;
    *bConfigurationValue = 1;
    return true;
}

static void usb_host_task(void *arg)
{
    TaskHandle_t waiter = (TaskHandle_t)arg;
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .root_port_unpowered = true,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
#if CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK
        .enum_filter_cb = enum_filter_cb,
#endif
        .peripheral_map = 0,
    };

    esp_err_t ret = usb_host_install(&host_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_install failed: %s", esp_err_to_name(ret));
        xTaskNotify(waiter, (uint32_t)ret, eSetValueWithOverwrite);
        vTaskDelete(NULL);
        return;
    }

    xTaskNotify(waiter, ESP_OK, eSetValueWithOverwrite);
    ESP_LOGI(TAG, "USB host installed");

    bool has_clients = true;
    bool has_devices = false;
    while (has_clients) {
        uint32_t event_flags = 0;
        ret = usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "usb_host_lib_handle_events: %s", esp_err_to_name(ret));
            continue;
        }

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            if (usb_host_device_free_all() == ESP_OK) {
                has_clients = false;
            } else {
                has_devices = true;
            }
        }
        if (has_devices && (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)) {
            has_clients = false;
        }
    }

    ESP_LOGI(TAG, "USB host uninstalling");
    ESP_ERROR_CHECK_WITHOUT_ABORT(usb_host_uninstall());
    s_started = false;
    s_root_powered = false;
    s_host_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t usb_host_service_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(usb_host_task,
                                            "usb_host",
                                            4096,
                                            xTaskGetCurrentTaskHandle(),
                                            5,
                                            &s_host_task,
                                            0);
    ESP_RETURN_ON_FALSE(ok == pdTRUE, ESP_ERR_NO_MEM, TAG, "failed to create USB host task");

    uint32_t notify = ESP_FAIL;
    ESP_RETURN_ON_FALSE(xTaskNotifyWait(0, UINT32_MAX, &notify, pdMS_TO_TICKS(2000)) == pdTRUE,
                        ESP_ERR_TIMEOUT, TAG, "USB host start timeout");
    ESP_RETURN_ON_ERROR((esp_err_t)notify, TAG, "USB host task failed");
    s_started = true;
    return ESP_OK;
}

esp_err_t usb_host_service_power_on(void)
{
    if (!s_started) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_root_powered) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Powering USB root port");
    esp_err_t ret = usb_host_lib_set_root_port_power(true);
    if (ret == ESP_OK) {
        s_root_powered = true;
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    return ret;
}
