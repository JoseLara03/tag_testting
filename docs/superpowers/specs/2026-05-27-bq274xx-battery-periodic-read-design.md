# BQ274xx Battery Periodic Read — Design Spec

**Date:** 2026-05-27  
**Status:** Approved

## Goal

Read battery voltage (mV) and state of charge (%) from the BQ274xx fuel gauge every 10 seconds and forward the values over BLE NUS.

## Context

The BQ274xx is already fully configured:
- DTS node `fuel_gauge: bq274xx@55` under `&i2c0` with corrected parameters: design-voltage=4200 mV, design-capacity=400 mAh, taper-current=40 mA, terminate-voltage=3000 mV.
- `CONFIG_FUEL_GAUGE=y` and `CONFIG_BQ274XX=y` in `boards/Innovaforce/nRF52833_tag/nRF52833_tag_defconfig`.
- Zephyr's built-in BQ274xx driver handles all I2C communication via the standard `fuel_gauge` API.

No custom I2C driver is needed.

## Approach

**k_timer + system work queue.** A 10 s repeating `k_timer` submits a `k_work` item to the system work queue on each expiry. The work handler reads two fuel gauge properties and sends the result over BLE NUS. No dedicated thread; no extra stack allocation.

## New Files

### `src/batt.h`
Exposes a single function:
```c
int batt_init(void);
```
Returns 0 on success, negative errno if the fuel gauge device is not ready.

### `src/batt.c`

**Statics:**
- `const struct device *fg` — obtained via `DEVICE_DT_GET(DT_NODELABEL(fuel_gauge))`
- `struct k_work batt_work`
- `struct k_timer batt_timer`

**Timer callback** (ISR context — no I2C allowed):
```c
static void batt_timer_cb(struct k_timer *t) {
    k_work_submit(&batt_work);
}
```

**Work handler** (system work queue context — I2C safe):
```c
static void batt_work_handler(struct k_work *w) {
    struct fuel_gauge_get_property props[2] = {
        { .property_type = FUEL_GAUGE_VOLTAGE },
        { .property_type = FUEL_GAUGE_STATE_OF_CHARGE },
    };
    int err = fuel_gauge_get_prop(fg, props, ARRAY_SIZE(props));
    if (err) {
        ble_log_send("BATT: ERR\n");
        return;
    }
    char msg[32];
    snprintf(msg, sizeof(msg), "BATT: %dmV %d%%\n",
             props[0].value.voltage / 1000,
             props[1].value.state_of_charge);
    ble_log_send(msg);
}
```

**`batt_init()`:**
```c
int batt_init(void) {
    if (!device_is_ready(fg)) return -ENODEV;
    k_work_init(&batt_work, batt_work_handler);
    k_timer_init(&batt_timer, batt_timer_cb, NULL);
    k_timer_start(&batt_timer, K_SECONDS(10), K_SECONDS(10));
    return 0;
}
```

## Changes to Existing Files

### `src/main.c`
Add `#include "batt.h"` and insert `batt_init()` call between `ble_log_init()` and `nfc_tag_init()`:

```c
ble_log_init()
batt_init()          // timer starts; readings silently dropped until BLE connects
nfc_tag_init()
ble_log_wait_ready()
uwb_init()
```

If `batt_init()` returns non-zero, halt with `k_sleep(K_FOREVER)` (consistent with existing error handling pattern).

### `CMakeLists.txt`
Add `src/batt.c` to `target_sources(app PRIVATE ...)`.

## BLE Message Format

```
BATT: 3720mV 85%
```

- Voltage in millivolts (Zephyr fuel_gauge returns µV; divide by 1000).
- SoC as integer percent.
- Sent via `ble_log_send()`, which already guards against no BLE connection — readings are silently dropped when no central is connected.

## Error Handling

| Condition | Behavior |
|---|---|
| Device not ready at init | `batt_init()` returns `-ENODEV`; main halts |
| `fuel_gauge_get_prop` fails at runtime | Sends `"BATT: ERR\n"` over BLE |
| No BLE connection at send time | `ble_log_send()` returns silently |

## Out of Scope

- Alerting on low battery threshold.
- Persistent logging or NFC forwarding of battery data.
- Configurable read interval at runtime.
