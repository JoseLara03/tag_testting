# BQ274xx Battery Periodic Read — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Read battery voltage and state of charge from the BQ274xx every 10 s and forward them over BLE NUS as `"BATT: 3720mV 85%\n"`.

**Architecture:** A `k_timer` fires every 10 s and submits a `k_work` item to the system work queue. The work handler calls Zephyr's `fuel_gauge_get_prop()` for `FUEL_GAUGE_VOLTAGE` (µV) and `FUEL_GAUGE_STATE_OF_CHARGE` (%), formats a string, and sends it via `ble_log_send()`. The DTS node, Kconfig, and I2C bus are already configured — no board changes needed.

**Tech Stack:** Zephyr RTOS, `<zephyr/drivers/fuel_gauge.h>`, `k_timer`, `k_work`, BLE NUS via existing `ble_log` module.

> **Note on testing:** This project has no on-host test framework. Verification steps mean: ask the user to build with `west build`, report any compiler errors, then flash and connect a BLE terminal to observe output.

---

## File Map

| File | Action | Responsibility |
|---|---|---|
| `src/batt.h` | Create | Public API: `batt_init()` declaration |
| `src/batt.c` | Create | Timer, work handler, fuel gauge reads |
| `CMakeLists.txt` | Modify (line 13) | Add `src/batt.c` to sources |
| `src/main.c` | Modify | Include `batt.h`, call `batt_init()` |

---

### Task 1: Create `src/batt.h`

**Files:**
- Create: `src/batt.h`

- [ ] **Step 1: Write the header**

```c
#ifndef BATT_H_
#define BATT_H_

int batt_init(void);

#endif /* BATT_H_ */
```

- [ ] **Step 2: Commit**

```bash
git add src/batt.h
git commit -m "feat: add batt module header"
```

---

### Task 2: Create `src/batt.c`

**Files:**
- Create: `src/batt.c`

- [ ] **Step 1: Write the implementation**

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <stdio.h>
#include "batt.h"
#include "ble_log.h"

static const struct device *fg = DEVICE_DT_GET(DT_NODELABEL(fuel_gauge));
static struct k_work  batt_work;
static struct k_timer batt_timer;

static void batt_work_handler(struct k_work *w)
{
    ARG_UNUSED(w);

    struct fuel_gauge_get_property props[] = {
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
             (int)(props[0].value.voltage / 1000U),
             (int)props[1].value.state_of_charge);
    ble_log_send(msg);
}

static void batt_timer_cb(struct k_timer *t)
{
    ARG_UNUSED(t);
    k_work_submit(&batt_work);
}

int batt_init(void)
{
    if (!device_is_ready(fg)) {
        return -ENODEV;
    }

    k_work_init(&batt_work, batt_work_handler);
    k_timer_init(&batt_timer, batt_timer_cb, NULL);
    k_timer_start(&batt_timer, K_SECONDS(10), K_SECONDS(10));

    return 0;
}
```

- [ ] **Step 2: Commit**

```bash
git add src/batt.c
git commit -m "feat: add batt module (k_timer + fuel_gauge periodic read)"
```

---

### Task 3: Register `src/batt.c` in the build

**Files:**
- Modify: `CMakeLists.txt`

Current `target_sources` block (lines 6–18):
```cmake
target_sources(app PRIVATE
    src/main.c
    src/ble_log.c
    src/lis2hh12_if.c
    src/uwb.c
    src/nfc_tag.c
    drivers/lis2hh12-pid/lis2hh12_reg.c
    platform/port.c
    platform/deca_spi.c
    platform/deca_probe_interface.c
    platform/deca_mutex.c
    platform/deca_sleep.c
)
```

- [ ] **Step 1: Add `src/batt.c`**

Replace the block above with:
```cmake
target_sources(app PRIVATE
    src/main.c
    src/ble_log.c
    src/lis2hh12_if.c
    src/uwb.c
    src/nfc_tag.c
    src/batt.c
    drivers/lis2hh12-pid/lis2hh12_reg.c
    platform/port.c
    platform/deca_spi.c
    platform/deca_probe_interface.c
    platform/deca_mutex.c
    platform/deca_sleep.c
)
```

- [ ] **Step 2: Commit**

```bash
git add CMakeLists.txt
git commit -m "build: add src/batt.c to target sources"
```

---

### Task 4: Wire `batt_init()` into `main.c`

**Files:**
- Modify: `src/main.c`

- [ ] **Step 1: Add the include**

After the existing includes at the top of `src/main.c` (after `#include "nfc_tag.h"`), add:
```c
#include "batt.h"
```

- [ ] **Step 2: Call `batt_init()` in `main()`**

In `main()`, after the `ble_log_init()` block and before `nfc_tag_init()`. The current code is:

```c
    if (ble_log_init() != 0) {
        k_sleep(K_FOREVER);
        return 0;
    }

    if (nfc_tag_init() != 0) {
```

Insert between them:

```c
    if (ble_log_init() != 0) {
        k_sleep(K_FOREVER);
        return 0;
    }

    if (batt_init() != 0) {
        k_sleep(K_FOREVER);
        return 0;
    }

    if (nfc_tag_init() != 0) {
```

- [ ] **Step 3: Commit**

```bash
git add src/main.c
git commit -m "feat: start battery periodic read on boot"
```

---

### Task 5: Build verification

- [ ] **Step 1: Ask the user to build**

Ask the user to run:
```bash
west build -b nRF52833_tag
```
Expected: build completes with no errors and no new warnings.

Common issues and fixes:
- `error: 'FUEL_GAUGE_VOLTAGE' undeclared` → check that `#include <zephyr/drivers/fuel_gauge.h>` is present in `src/batt.c`.
- `error: implicit declaration of 'fuel_gauge_get_prop'` → same include missing.
- `error: 'struct fuel_gauge_property_val' has no member named 'voltage'` → the property union member name may differ in your nCS version; check `<zephyr/drivers/fuel_gauge.h>` for the correct field name (may be `val` instead of `value`, or `voltage_uV`).
- `error: 'DT_NODELABEL(fuel_gauge)' not found` → verify `&i2c0 { fuel_gauge: bq274xx@55 { ... }; };` is present and `status = "okay"` on `&i2c0` in the DTS.

- [ ] **Step 2: Report build result**

If the build fails, share the error output and fix before proceeding.

---

### Task 6: Flash and observe BLE output

- [ ] **Step 1: Ask the user to flash and connect a BLE terminal**

Flash the firmware and connect a BLE NUS terminal (e.g., nRF Toolbox → UART, or LightBlue). After the central enables notifications, wait up to 10 seconds.

- [ ] **Step 2: Verify expected output**

Expected messages, one every 10 s:
```
BATT: 3720mV 85%
```
(Exact values depend on battery state.)

If `BATT: ERR` appears instead, the BQ274xx I2C communication failed — check wiring and that the IC is powered.

- [ ] **Step 3: Confirm DW3000 and NFC still work**

- Short button press → still sends `DW3000_ID=0x...` (existing behavior unchanged).
- NFC tap from phone → still forwards URI over BLE (existing behavior unchanged).
