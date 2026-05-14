# Design: BLE NUS Debug Transport + LIS2HH12 WHO_AM_I Verification

**Date:** 2026-05-14  
**Status:** Approved  

## Goal

Establish BLE as the sole debug channel (UART/console disabled on the board) and verify the LIS2HH12 accelerometer is reachable on I2C0 by reading its WHO_AM_I register and streaming the result to a connected BLE central (e.g. nRF Connect mobile app).

## Hardware Context

- MCU: nRF52833 (custom Innovaforce tag board)
- I2C0: SDA → P0.2, SCL → P0.3
- LIS2HH12 SA0: driven to GND → 7-bit address `0x1E` (8-bit write `0x3D`, alias `LIS2HH12_I2C_ADD_L`)
- WHO_AM_I register: `0x0F`, expected value: `0x41`
- No UART / no console / no LOG subsystem

## Architecture

```
main.c
  │
  ├── ble_log.c/.h          thin wrapper: ble_log_send(const char *msg)
  │     └── bt_nus_send()
  │
  └── lis2hh12_if.c/.h      stmdev_ctx_t I2C callbacks + init helper
        └── i2c_reg_read_byte() / i2c_write_read()
              └── DEVICE_DT_GET(DT_NODELABEL(i2c0))

drivers/lis2hh12-pid/       submodule — untouched
  ├── lis2hh12_reg.h
  └── lis2hh12_reg.c
```

## Components

### `src/ble_log.h/.c`
- Holds the active `struct bt_conn *` updated via `BT_CONN_CB_DEFINE` callbacks
- `ble_log_init()`: calls `bt_enable()`, `bt_nus_init()`, `bt_le_adv_start()`
- `ble_log_send(const char *msg)`: calls `bt_nus_send(current_conn, msg, strlen(msg))`; silently drops if no active connection

### `src/lis2hh12_if.h/.c`
- Implements `platform_write(handle, reg, buf, len)` and `platform_read(handle, reg, buf, len)` using the Zephyr I2C API
- `lis2hh12_if_init(stmdev_ctx_t *ctx)`: wires up the callbacks and sets `handle` to the `struct device *` for `i2c0`
- Address `0x1E` is a compile-time constant `LIS2HH12_ADDR` defined in this file

### `src/main.c`
1. Call `ble_log_init()`
2. Call `lis2hh12_if_init(&dev_ctx)`
3. Poll until BLE connection is established (checking `k_msleep` loop)
4. Call `lis2hh12_dev_id_get(&dev_ctx, &who_am_i)`
5. Format and send: `"WHO_AM_I = 0xXX (OK)"` if `0x41`, else `"WHO_AM_I = 0xXX (FAIL expected 0x41)"`
6. `k_sleep(K_FOREVER)`

## prj.conf Additions

```
CONFIG_BT=y
CONFIG_BT_HCI=y
CONFIG_BT_PERIPHERAL=y
CONFIG_BT_DEVICE_NAME="nRF52_Tag"
CONFIG_BT_NUS=y
```

`CONFIG_I2C=y` is already set in `nRF52833_tag_defconfig`.

## CMakeLists.txt Changes

- Add `src/ble_log.c` and `src/lis2hh12_if.c` via `target_sources`
- Add `drivers/lis2hh12-pid` to include path via `target_include_directories`
- Add `drivers/lis2hh12-pid/lis2hh12_reg.c` to sources

## Out of Scope

- Full Zephyr sensor driver (`SENSOR_DRIVER_API`) — not needed for this step
- FIFO, interrupts, ODR/FS configuration — deferred
- DTS child node for LIS2HH12 — not required when using raw I2C API directly
