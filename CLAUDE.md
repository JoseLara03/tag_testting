# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Zephyr RTOS firmware for a custom IoT tracking tag based on the Nordic nRF52833 SoC (Arm Cortex-M4, 128 KB RAM, 512 KB Flash). The tag integrates Ultra-Wideband (DW3000 via SPI1), BLE NUS, NFC Tag Type 4 (T4T), a WS2812 RGB LED, a button, a LIS2HH12 accelerometer (I2C0), and a BQ274xx fuel gauge (I2C0).

Board definition: `boards/Innovaforce/nRF52833_tag/`  
Application entry point: `src/main.c`

## Build & Flash

The user builds and flashes the firmware themselves and will report any errors or issues.

## Source Layout

```
src/
  main.c            — minimal bring-up: LED status, BLE NUS, DW3000 init (see Application Behavior)
  ble_log.c/h       — BLE NUS wrapper (TX-only; blocks until notifications enabled; retries on -ENOMEM)
  nfc_tag.c/h       — NFC T4T emulation; default URI https://google.mx; forwards phone writes via ble_log_send()
  uwb.c/h           — DW3000 init with retry logic; exposes uwb_get_dev_id()
  batt.c/h          — BQ274xx periodic read (k_timer 10 s → k_work); sends "BATT: xxmV xx%\n" over BLE NUS
  lis2hh12_if.c/h   — LIS2HH12 I2C callback registration
platform/
  port.c/h          — DW3000 GPIO/reset/wakeup HAL; DW3000_IRQ_Pin=30, RST=37, WUP=15
  deca_spi.c/h      — SPI1 driver (4 MHz init → 32 MHz fast); static EasyDMA RX buffer
  deca_probe_interface.c/h — dwt_probe_s function pointers for Decawave library
  deca_mutex.c      — decamutexon/off critical sections around UWB ISR
  deca_sleep.c      — deca_sleep (ms) / deca_usleep (µs) wrappers
drivers/
  lis2hh12-pid/     — ST register-map driver (lis2hh12_reg.c/h)
Shared/
  dwt_uwb_driver/   — Decawave precompiled library (libdwt_uwb_driver-m4-sfp-6.0.7.a) + headers
```

New source files must be added to `CMakeLists.txt` via `target_sources(app PRIVATE ...)`.  
The Decawave library exposes a custom linker section; `dw_drivers.ld` places it in Flash, and `zephyr_ld_options(-Wl,--undefined=dw3000_driver)` in `CMakeLists.txt` pulls the driver registration object into the link (see Key Patterns).

Note: `nfc_tag.c`, `batt.c`, `lis2hh12_if.c`, and `drivers/lis2hh12-pid/lis2hh12_reg.c` are still listed in `CMakeLists.txt` and compile into the image, but `main.c` does not invoke them — they are dormant code awaiting reintegration.

## Hardware Peripherals

| Peripheral | Pins | DTS Alias / Node | Purpose |
|---|---|---|---|
| SPI1 | SCK P0.4, MISO P0.5, MOSI P1.9, CS P0.11 | — | DW3000 UWB transceiver |
| SPI3 | MOSI P0.20 | `led-strip` | WS2812 RGB LED (driven by `led_strip` Zephyr driver) |
| I2C0 | SDA P0.2, SCL P0.3 | — | LIS2HH12 accel (addr 0x1E) + BQ274xx fuel gauge (addr 0x55) |
| GPIO P0.17 | — | `button0` | User button (interrupt on both edges; long-press ≥3 s) |
| GPIO P0.30 | — | `led0` | Status LED |
| NFCT | P0.09 (NFC1), P0.10 (NFC2) | `nfct` | NFC T4T antenna (dedicated analog pins, auto-reserved by `CONFIG_NFC_T4T_NRFXLIB=y`) |

Console/UART/logging are **disabled** (`CONFIG_SERIAL=n`, `CONFIG_LOG=n`). BLE NUS is the only debug channel.

## Active Zephyr Configuration (prj.conf)

Key configs currently enabled:
- `CONFIG_BT=y`, `CONFIG_BT_PERIPHERAL=y`, `CONFIG_BT_NUS=y` — BLE peripheral + Nordic UART Service
- `CONFIG_NFC_T4T_NRFXLIB=y`, `CONFIG_NFC_NDEF=y`, `CONFIG_NFC_NDEF_MSG=y`, `CONFIG_NFC_NDEF_RECORD=y`, `CONFIG_NFC_NDEF_URI_REC=y`, `CONFIG_NFC_NDEF_URI_MSG=y`, `CONFIG_NFC_NDEF_PARSER=y` — NFC T4T + NDEF encode/parse
- `CONFIG_LED_STRIP=y`, `CONFIG_WS2812_STRIP_SPI=y` — RGB LED
- `CONFIG_SENSOR=y`, `CONFIG_I2C=y`, `CONFIG_FUEL_GAUGE=y`, `CONFIG_BQ274XX=y` — fuel gauge (BQ274xx is a sensor driver in nCS 3.2.4; requires `CONFIG_SENSOR=y`)
- `CONFIG_ARM_MPU=y`, `CONFIG_HW_STACK_PROTECTION=y`

Board-level defaults live in `boards/Innovaforce/nRF52833_tag/nRF52833_tag_defconfig`.  
To enable a new subsystem, add the `CONFIG_*` line to `prj.conf`.

## Application Behavior (main.c)

Stripped down to the minimum needed to verify DW3000 SPI bring-up over BLE NUS:

1. Get WS2812 RGB LED device; hang on failure.
2. `ble_log_init()` — starts BLE advertising.
3. `ble_log_wait_ready()` — blocks until BLE central enables NUS TX notifications.
4. LED **cyan** — initializing DW3000.
5. `uwb_init(3)` — three retries; sends `probe fail N/3` or `init fail N/3` on each failure, `OK ID=0x........` on success, `all 3 fail` on total failure.
6. On success: LED **green**, send `ID=0x........\n` over NUS.
7. On failure: LED **red**, send `DW3000: init failed\n` over NUS.
8. `k_sleep(K_FOREVER)`.

NFC, battery, accelerometer, and button handling are intentionally not initialised; the goal of this build is end-to-end DW3000 ID read.

## Key Patterns

- **BLE NUS transmit:** `ble_log_init()` → `ble_log_wait_ready()` → `ble_log_send(msg)`. The wait blocks on a semaphore until the central enables notifications; do not call `ble_log_send` before that. `ble_log_send` retries internally on `-ENOMEM` (TX buffer momentarily exhausted by back-to-back sends).
- **NUS 20-byte payload limit:** Default ATT MTU is 23, so a single notification carries at most 20 bytes. `bt_nus_send` does not auto-fragment — payloads >20 bytes fail with `-EMSGSIZE` and are silently dropped. Keep diagnostic strings short (e.g. `"probe fail 1/3\n"`, `"ID=0xDEADBEEF\n"`), or initiate an MTU exchange before transmitting long messages.
- **Linking the precompiled Decawave driver:** `libdwt_uwb_driver-*.a` registers per-chip drivers (DW3000/DW3700/DW3720) into the `.dw_drivers` linker section via static objects. Nothing in app code references those symbols, so the linker would skip the .o files. `zephyr_ld_options(-Wl,--undefined=dw3000_driver)` in `CMakeLists.txt` forces `dw3000_device.c.obj` into the link, populating the section so `dwt_probe()` can find a matching driver. Do **not** use `-Wl,--whole-archive` — CMake 3.21 (nCS 3.2.4) strips the matching `--no-whole-archive` as a duplicate and the flag leaks into picolibc/libgcc, causing multiple-definition errors on `calloc`/`free`/`malloc`/`__retarget_lock_*`/etc.
- **GPIO interrupts:** Use `gpio_pin_interrupt_configure_dt()` with `GPIO_INT_EDGE_BOTH`; never use the button subsystem — GPIO is managed manually to avoid conflicts with the NUS setup.
- **SPI rate switching:** Call `openspi()` / `closespi()` in `deca_spi.c` when changing between init (4 MHz) and fast (32 MHz) modes; the static RX buffer is required for EasyDMA.
- **DW3000 critical sections:** Always bracket Decawave library calls that touch the IRQ with `decamutexon()` / `decamutexoff()`.
- **Adding a peripheral to I2C0:** Add the node under `&i2c0` in `nRF52833_tag.dts`; the bus is already enabled with the correct pins.
- **BQ274xx fuel gauge:** In nCS 3.2.4 the driver lives under `drivers/sensor/ti/bq274xx/` and implements the **sensor API**, not the fuel_gauge API. Use `sensor_sample_fetch(dev)` then `sensor_channel_get(dev, SENSOR_CHAN_GAUGE_VOLTAGE, &val)` and `sensor_channel_get(dev, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE, &val)`. Voltage: `val.val1` = V, `val.val2` = µV fractional → mV = `val1 * 1000 + val2 / 1000`. SoC: `val.val1` = %. The driver requires `CONFIG_SENSOR=y`; `CONFIG_FUEL_GAUGE=y` and `CONFIG_BQ274XX=y` are in the board defconfig. The gauge is not ready without a battery connected — `device_is_ready()` returns false on USB-only power. Battery parameters (DTS node `fuel_gauge: bq274xx@55` under `&i2c0`): design-voltage=4200 mV, design-capacity=400 mAh, taper-current=40 mA, terminate-voltage=3000 mV.
- **NFC T4T:** `nfc_tag_init()` encodes a default URI, registers an RW payload buffer, and starts emulation. The `NFC_T4T_EVENT_NDEF_UPDATED` callback runs in the nrfxlib NFC thread (not ISR) — `ble_log_send()` is safe to call from it. Writes before BLE connects are silently dropped. The NFCT node must be enabled in the board DTS (`&nfct { status = "okay"; }`) for `HAS_HW_NRF_NFCT` to be set, which is required for `NFC_PLATFORM` and `NRFX_NFCT` to build. URI prefix codes follow NFC Forum RTD: 0x03 = `"http://"`, 0x04 = `"https://"` (already includes `://`). Do not pass `"//host"` as the URI string when using these prefix codes.
- **UWB antenna delay (calibration):** Per-unit, solved on-device and persisted in NVS — see `src/cal.c`/`src/cal_math.c`. Trigger over BLE NUS with `cal <mm>` (reference distance in millimetres); the initiator (`src/uwb_ss_initiator.c`) collects ~100 ranges, rejects outliers (median/MAD), and corrects the combined TX+RX delay toward the reference by a linear step (~2.34 mm per combined unit) over up to 4 iterations until the residual is ≤15 mm, then stores a CRC-checked, PHY-tagged `cal_record` in NVS (DTS `storage_partition`, resolved via devicetree because this NCS Partition Manager build gives all flash to a single `app` region). Other commands: `cal status`, `cal clear`, `cal selftest` (runs `cal_math_selftest()` → `SELFTEST 0` on success). **NVS is mandatory:** without a valid record for the current PHY the ranging thread reports `CAL REQUIRED` and does not range until a `cal` run succeeds. `TX_ANT_DLY = RX_ANT_DLY = 16371` in `src/phy_config.h` is now only the factory-reference seed/fallback for an uncalibrated unit. Values are PHY-dependent (calibrated for `CONFIG_OPTION_07`: Ch5, PLEN-1024, 850k); `phy_option` in the record invalidates a stored value if the PHY changes. Note: the `CAL loaded`/`CAL REQUIRED` message sent from `main.c` at boot fires before a central connects and is dropped — use `cal status` to read state after connecting.

## Flash Memory Layout

| Region | Start | Size |
|---|---|---|
| MCUBoot | 0x00000000 | 48 KB |
| Image-0 (active) | 0x0000c000 | 220 KB |
| Image-1 (update slot) | 0x00043000 | 220 KB |
| Storage | 0x0007a000 | 24 KB |
