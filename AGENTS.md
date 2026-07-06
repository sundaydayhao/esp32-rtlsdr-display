# Project Context for Codex

This repository is an ESP-IDF firmware project for the ESP32-S31-Korvo-1 board.
It connects to an RTL2832U RTL-SDR dongle over the board USB OTG Type-A host
port, renders a live FFT waterfall on the RGB LCD, and includes an FM receiver
page with audio waveform/status UI.

## Non-Negotiable Build Rule

- Target is `esp32s31`, which is an ESP-IDF preview target.
- Always use `idf.py --preview ...` for project commands.
- Correct build command:

```powershell
idf.py --preview build
```

- Do not use plain `idf.py build` for this project.
- Typical setup commands:

```powershell
& C:\esp\v6.0.2\esp-idf\export.ps1
idf.py --preview set-target esp32s31
idf.py --preview menuconfig
idf.py --preview build
idf.py --preview flash monitor
```

- The project has been used with ESP-IDF under `C:\esp\v6.0.2\esp-idf`.
- The shell is usually PowerShell on Windows. If Unicode warnings appear during
  builds, they are usually console encoding warnings, not necessarily firmware
  build failures.

## Important Files

- `main/app_main.c`: top-level wiring. Creates app display, DSP waterfall, FM
  receiver, USB host service, and RTL-SDR device. It registers the display
  center-frequency callback that eventually calls `rtlsdr_set_center_freq_hz`.
- `main/display.c` / `main/display.h`: LVGL UI, page navigation, waterfall
  image buffer, FM waveform image buffer, touch handling, frequency dialog, and
  user-facing metrics.
- `main/rtlsdr.c` / `main/rtlsdr.h`: RTL2832U USB device control, tuner setup,
  pending center-frequency tune requests, R82xx and Fitipower tuner paths, and
  IQ streaming.
- `main/dsp.c` / `main/dsp.h`: FFT waterfall processing. Consumes IQ samples
  and emits scaled FFT bins for display.
- `main/fm_receiver.c` / `main/fm_receiver.h`: WFM/NFM demodulation chain,
  audio filtering/decimation, waveform callbacks, and optional audio output.
- `main/korvo_audio.c` / `main/korvo_audio.h`: Korvo speaker codec/I2S output
  support for FM audio.
- `main/usb_host_service.c` / `main/usb_host_service.h`: ESP-IDF USB host
  service task and device enumeration support.
- `main/Kconfig.projbuild`: project configuration, including RTL-SDR, LCD,
  LVGL, touch, and FM receiver settings.

## Runtime Architecture

- `app_main.c` creates a single `app_context_t` holding:
  - `app_display_t *display`
  - `dsp_waterfall_t *dsp`
  - `fm_receiver_t *fm`
  - `rtlsdr_t *radio`
  - `requested_center_hz`
- The RTL-SDR IQ stream is shared:
  - IQ goes into the waterfall DSP path.
  - IQ also goes into the FM demodulator path.
- There is only one physical RTL-SDR tuner, so only one RF center frequency is
  active in hardware at a time. UI pages may keep separate desired center
  frequencies and retune the hardware when entering that page.
- FM demodulation and speaker audio should run only while the FM page is active.
  Other pages should not output FM audio.
- If no usable RTL-SDR stream is available, the display can render synthetic
  waterfall lines for LCD/UI testing.

## Display/UI Details

- UI uses LVGL and a root page with tiles:
  - ADS-B
  - FM Receiver
  - Waterfall
  - Sensors
  - IQ Record
  - Scanner
- Only Waterfall and FM are currently live receiver pages. The other pages are
  placeholders/navigation targets.
- `display.c` owns a generic `app_display_t`. Page-specific state is grouped
  into per-page structs, for example Waterfall image/marker/swipe state and FM
  waveform/audio status state.
- Waterfall page:
  - Shows a spectrum overview plus scrolling waterfall.
  - Touching the spectrum area sets a marker frequency.
  - Horizontal swipe in the upper half steps the waterfall center frequency.
- FM page:
  - Shows decoded audio waveform, mode, IQ rate, audio rate, audio output
    readiness, and audio level.
  - The center frequency row opens the same frequency edit dialog.
- Touch support probes configured GT911/FT5x06-compatible controllers through
  I2C. Touch axis swap/mirror options are configured through Kconfig.

## Frequency UI Behavior

- The frequency dialog accepts MHz text, for example `100.000`.
- Valid center-frequency range is:
  - minimum: `24 MHz`
  - maximum: `1766 MHz`
- Invalid or empty input should not be silently clamped. It should keep the
  dialog open and show a user-facing error.
- Empty text should remain visually empty; do not use `100.000` as a placeholder
  because it reappears after deleting all text.
- Waterfall and FM center frequencies should be independent UI states:
  - Changing Waterfall center frequency must not overwrite FM page's saved
    center frequency.
  - Changing FM center frequency must not overwrite Waterfall page's saved
    center frequency.
  - When entering Waterfall or FM, apply that page's saved center frequency to
    the single RTL-SDR hardware tuner through the registered callback.

## Current Local Change State

At the time this context file was created, `main/display.c` had local edits for:

- Splitting the former single UI center frequency into:
  - `waterfall_center_hz`
  - `fm_center_hz`
- Using the active page's saved center frequency for labels, dialog initial
  value, waterfall swipes, and marker calculations.
- Applying the selected page's saved center frequency when switching into
  Waterfall or FM.
- Rejecting out-of-range MHz input instead of clamping it.
- Removing the `100.000` placeholder from the frequency text area.
- Adding an error label and red textarea border for invalid frequency input.
- Renaming the public display type/API from `waterfall_display_t` /
  `waterfall_display_*` to `app_display_t` / `app_display_*`.
- Grouping Waterfall and FM UI state into independent page state structs under
  the generic app display.
- Adding an app display page-change callback so `app_main.c` can enable FM
  audio only on the FM page.
- Adding runtime FM volume control APIs and configurable VOL-/VOL+ GPIOs.

These edits should be verified with:

```powershell
idf.py --preview build
```

The previous plain `idf.py build` attempt was intentionally aborted/ignored
because this project requires `--preview`.

## Coding Notes

- Prefer existing style in `main/display.c`: static helper functions, explicit
  LVGL object references in `waterfall_display_t`, and `_lock_acquire` around
  public display API updates.
- Keep UI changes small and compatible with low-resolution LCD layouts.
- Use `apply_patch` for manual edits.
- Avoid unrelated README rewrites unless explicitly requested; PowerShell may
  display its Chinese text with mojibake if the console code page is not UTF-8.
