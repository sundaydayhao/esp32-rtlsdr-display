# ESP32-S31-Korvo-1 RTL-SDR Waterfall

ESP-IDF project for using the ESP32-S31-Korvo-1 USB OTG Type-A host port with an RTL2832U RTL-SDR dongle and drawing an FFT waterfall on the RGB LCD connector.

The default GPIO mapping follows the ESP32-S31-Korvo-1 documentation: RGB LCD data lines on GPIO8-19 and GPIO33-36, PCLK/DE/HSYNC/VSYNC on GPIO40/43/44/45, and the USB host port through the board Type-A connector.

## Build

```powershell
& C:\esp\v6.0.2\esp-idf\export.ps1
idf.py --preview set-target esp32s31
idf.py --preview menuconfig
idf.py --preview build
idf.py --preview flash monitor
```

ESP32-S31 support is exposed by ESP-IDF preview targets, so run all `idf.py` commands for this project with `--preview`. This project was validated with ESP-IDF v6.2.0 from `C:\esp\v6.0.2\esp-idf` and the `riscv32-esp-elf` toolchain `esp-16.1.0_20260609`.

Important menuconfig entries are under `RTL-SDR Waterfall`:

- Displayed center frequency and RTL2832U sample rate.
- FFT size and waterfall dimensions.
- LCD RGB timing and GPIO mapping.
- LVGL draw buffer/task sizing.
- I2C touch input. The default ESP32-S31-Korvo-1 LCD subboard setting uses SDA GPIO0, SCL GPIO1, and GT911 address 0x14. The firmware also probes FT5x06-compatible controllers; set SDA/SCL/INT/RST and axis transform options to match other LCD subboards.
- Optional RTL2832U baseband initialization.

The first hardware bring-up should use the default low sample rate. Raise it only after the device enumerates, bulk reads are continuous, and the LCD refresh is stable.

## Current Scope

This project currently validates the ESP-IDF path end to end: USB Host enumeration, RTL2832U bulk IN transfers, IQ buffering, FFT processing, and RGB LCD waterfall drawing. If no RTL-SDR stream is available, it draws a synthetic waterfall so the LCD path can be tested independently.

The display is now LVGL-based and mirrors the touch navigation from the Linux/Qt `rtlsdr-display` project. The home screen exposes ADS-B, FM, Waterfall, Sensors, IQ Record, and Scanner tiles. The Waterfall page is wired to the existing live FFT pipeline and supports touch selection of a marker frequency. The other feature pages are present as LVGL navigation targets and status/configuration placeholders until their ESP32-side decoder, audio, tuner-control, storage, or scanner logic is implemented.

The waterfall layout follows the ESP32-DIV spectrum screens: a live FFT overview is drawn above the scrolling waterfall, the horizontal ruler marks the left edge, center, and right edge of the sampled span, and signal strength uses a blue/purple to yellow/white RGB565 palette. Narrow peaks are preserved when the FFT bins are scaled to the LCD width by taking the peak value in each output pixel column.

The external tuner PLL/gain programming for common R820T/R828D RTL-SDR dongles is intentionally isolated in `main/rtlsdr.c` and is the next hardware-specific step. Until that is filled in, the center-frequency setting is shown on screen but does not tune the RF front end.

Some RGB LCD subboards also require controller initialization over the ESP32-S31-Korvo-1 3-wire LCD SPI pins before the RGB stream is visible. The RGB data/timing path is implemented; add the subboard's panel-init table if the backlight turns on but no image appears.
