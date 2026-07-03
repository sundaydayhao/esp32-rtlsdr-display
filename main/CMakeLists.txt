idf_component_register(
    SRCS
        "app_main.c"
        "display.c"
        "dsp.c"
        "rtlsdr.c"
        "usb_host_service.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_lcd
        esp_driver_gpio
        esp_ringbuf
        esp_timer
        usb
)
