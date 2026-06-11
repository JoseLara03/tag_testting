# Tag Workflow Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add motion-adaptive SS-TWR cadence, a button/LED user interface, on-demand battery display, and BLE-independent ranging to the nRF52833 + DW3000 tag firmware.

**Architecture:** Approach A from the design spec — reuse the existing ranging (prio 2) and BLE sender (prio 6) threads, add one UI thread (prio 7) and a motion module. Motion and button are GPIO-interrupt driven. UWB stays the highest-priority subsystem.

**Tech Stack:** Zephyr RTOS (nCS 3.2.4), DW3000 Decawave driver, LIS2HH12 ST pid register driver, BQ274xx sensor driver, WS2812 led_strip, BLE NUS.

> **Testing note:** This is embedded firmware with no host test harness. The developer (user) builds and flashes; this plan's "verify" steps are (a) a clean `west build` and (b) a specific observable behavior over BLE NUS / the WS2812 LED. There is no pytest.

> **Spec deviation flagged for approval:** The design spec (`docs/superpowers/specs/2026-06-11-tag-workflow-design.md`, Section 2) described a *software* 5 s no-motion timer fed by a pulsed accel interrupt. During planning we found the LIS2HH12 activity/inactivity feature (`ACT_THS` + `ACT_DUR`) can do the 5 s timing on-chip and expose moving/still as an INT1 *level*. This plan uses that approach (`uwb_set_moving(bool)` instead of `uwb_motion_notify()`), which is simpler, lower-power, and behaviorally identical. Confirm before executing Task 3/Task 5.

---

## File Structure

| File | Action | Responsibility |
|---|---|---|
| `examples/uwb_ds_initiator.{c,h}` | move from `src/` | Reference DS-TWR initiator, not compiled |
| `examples/uwb_simple_tx.{c,h}` | move from `src/` | Reference TX example, not compiled |
| `examples/uwb_simple_rx.{c,h}` | move from `src/` | Reference RX example, not compiled |
| `CMakeLists.txt` | modify | Drop the 3 example `.c` from `target_sources`; add `motion.c`, `tag_ui.c` |
| `src/uwb.{c,h}` | modify | Remove orphaned `uwb_simple_tx/rx` wrappers |
| `src/uwb_ss_initiator.{c,h}` | modify | Adaptive cadence; `uwb_set_moving(bool)` |
| `src/motion.{c,h}` | create | LIS2HH12 activity interrupt → `uwb_set_moving()` |
| `src/batt.{c,h}` | modify | On-demand `batt_read_soc(int *)`; drop periodic timer |
| `src/tag_ui.{c,h}` | create | Button gesture detection + LED state machine |
| `src/main.c` | modify | Non-blocking boot; start motion + UI |
| `boards/Innovaforce/nRF52833_tag/nRF52833_tag.dts` | modify | Add `accel_int` GPIO node (P0.28) |

---

## Task 1: Move example files out of the build

**Files:**
- Move: `src/uwb_ds_initiator.c`, `src/uwb_ds_initiator.h`, `src/uwb_simple_tx.c`, `src/uwb_simple_tx.h`, `src/uwb_simple_rx.c`, `src/uwb_simple_rx.h` → `examples/`
- Modify: `CMakeLists.txt:6-24`
- Modify: `src/uwb.c:10-11,88-96`, `src/uwb.h:8-9`

- [ ] **Step 1: Move the six files into `examples/`**

```bash
git mv src/uwb_ds_initiator.c src/uwb_ds_initiator.h examples/
git mv src/uwb_simple_tx.c   src/uwb_simple_tx.h   examples/
git mv src/uwb_simple_rx.c   src/uwb_simple_rx.h   examples/
```

(If `examples/` does not exist, `git mv` creates it. On Windows PowerShell, run these via the Bash tool or `git` directly — `git mv` works the same.)

- [ ] **Step 2: Remove the three example sources from `CMakeLists.txt`**

In `target_sources(app PRIVATE ...)`, delete these three lines:

```
    src/uwb_simple_tx.c
    src/uwb_simple_rx.c
    src/uwb_ds_initiator.c
```

The block should now end:

```cmake
target_sources(app PRIVATE
    src/main.c
    src/ble_log.c
    src/batt.c
    src/lis2hh12_if.c
    src/uwb.c
    src/nfc_tag.c
    src/phy_config.c
    src/uwb_ss_initiator.c
    drivers/lis2hh12-pid/lis2hh12_reg.c
    platform/port.c
    platform/deca_spi.c
    platform/deca_probe_interface.c
    platform/deca_mutex.c
    platform/deca_sleep.c
)
```

- [ ] **Step 3: Remove orphaned wrappers from `src/uwb.c`**

Delete these two `#include` lines (currently lines 10-11):

```c
#include "uwb_simple_tx.h"
#include "uwb_simple_rx.h"
```

Delete the two wrapper functions at the end of the file (currently lines 88-96):

```c
void uwb_simple_tx(void)
{
    simple_tx();
}

void uwb_simple_rx(void)
{
    simple_rx();
}
```

- [ ] **Step 4: Remove the declarations from `src/uwb.h`**

Delete these two lines:

```c
void     uwb_simple_tx(void);
void     uwb_simple_rx(void);
```

`uwb.h` should now declare only `uwb_init` and `uwb_get_dev_id`.

- [ ] **Step 5: Build**

Run: `west build` (developer)
Expected: clean build. No undefined-reference to `simple_tx`/`simple_rx`, no "file not found" for the moved sources.

- [ ] **Step 6: Commit**

```bash
git add CMakeLists.txt src/uwb.c src/uwb.h examples/
git commit -m "refactor(tag): move DS-TWR/simple TX-RX examples out of build"
```

---

## Task 2: Decouple ranging from the BLE connection at boot

**Files:**
- Modify: `src/main.c:25`

The ranging result path already drops when disconnected (`ble_log_send` returns early at `src/ble_log.c:86`). The only change needed is to stop blocking boot on a BLE central.

- [ ] **Step 1: Remove the boot-time BLE wait**

In `src/main.c`, delete this line (currently line 25):

```c
    ble_log_wait_ready();
```

Leave everything else in `main()` unchanged. `ble_log_init()` still starts advertising; UWB init now proceeds immediately.

- [ ] **Step 2: Build**

Run: `west build`
Expected: clean build. (`ble_log_wait_ready` is still defined in `ble_log.c`; leaving it unused is fine.)

- [ ] **Step 3: Verify on hardware — ranging without BLE**

Flash. With **no** BLE central connected, place the tag near a running SS-TWR responder/anchor.
Expected: LED turns **green** (UWB init OK) shortly after boot, independent of any BLE connection.

- [ ] **Step 4: Verify on hardware — logging after connect**

Connect a BLE central (e.g. nRF Connect) and enable NUS notifications.
Expected: `D:x.xxm` lines begin streaming within ~1 s. Disconnect; ranging continues (LED stays green), no crash.

- [ ] **Step 5: Commit**

```bash
git add src/main.c
git commit -m "feat(tag): start UWB ranging at boot independent of BLE connection"
```

---

## Task 3: Adaptive ranging cadence scaffolding

**Files:**
- Modify: `src/uwb_ss_initiator.h`
- Modify: `src/uwb_ss_initiator.c:32-33,157-172,226-232`

Add a `moving` flag and a wake semaphore so the post-cycle wait is 1 s while moving and 5 s while still, with motion able to cut a long wait short. No motion source yet (added in Task 5); default `moving = true` so behavior is unchanged (1 s) until then.

- [ ] **Step 1: Declare the public setter in `src/uwb_ss_initiator.h`**

Replace the file contents with:

```c
#ifndef UWB_SS_INITIATOR_H_
#define UWB_SS_INITIATOR_H_

#include <stdbool.h>

void uwb_ss_initiator_start(void);

/* Set ranging cadence: true -> fast (1 s), false -> slow (5 s).
 * Safe to call from ISR context. Switching to fast also wakes the
 * ranging thread immediately if it is in a long idle wait. */
void uwb_set_moving(bool moving);

#endif /* UWB_SS_INITIATOR_H_ */
```

(If the existing header has a different include guard or extra declarations, preserve `uwb_ss_initiator_start` and add `uwb_set_moving` + the `<stdbool.h>` include.)

- [ ] **Step 2: Replace the fixed delay with cadence state in `src/uwb_ss_initiator.c`**

Change the timing defines. Replace (currently line 33):

```c
#define RNG_DELAY_MS                1000U
```

with:

```c
#define RNG_FAST_MS                 1000U   /* cadence while moving */
#define RNG_SLOW_MS                 5000U   /* cadence after ~5 s of no motion */
```

Add module state near the other file-scope statics (e.g. just below the `irq_sem` definition around line 103):

```c
static volatile bool   ss_moving = true;     /* start fast until motion module reports otherwise */
static K_SEM_DEFINE(range_tick, 0, 1);
```

- [ ] **Step 3: Implement `uwb_set_moving`**

Add this function (e.g. just above `uwb_ss_initiator_start`):

```c
void uwb_set_moving(bool moving)
{
    ss_moving = moving;
    if (moving) {
        k_sem_give(&range_tick);   /* cut a slow wait short */
    }
}
```

- [ ] **Step 4: Replace the end-of-loop sleep**

In `ss_twr_fn`, replace (currently line 231):

```c
        k_msleep(RNG_DELAY_MS);
```

with:

```c
        uint32_t wait_ms = ss_moving ? RNG_FAST_MS : RNG_SLOW_MS;
        k_sem_take(&range_tick, K_MSEC(wait_ms));
```

- [ ] **Step 5: Build**

Run: `west build`
Expected: clean build.

- [ ] **Step 6: Verify on hardware**

Flash, connect BLE, watch `D:x.xxm`.
Expected: still ranges every ~1 s (because `ss_moving` defaults to true). Behavior unchanged from Task 2 — this task only adds the mechanism.

- [ ] **Step 7: Commit**

```bash
git add src/uwb_ss_initiator.c src/uwb_ss_initiator.h
git commit -m "feat(tag): add motion-gated ranging cadence (1s/5s) to SS-TWR thread"
```

---

## Task 4: Add the accelerometer INT1 GPIO to the DTS

**Files:**
- Modify: `boards/Innovaforce/nRF52833_tag/nRF52833_tag.dts:30-36`

The LIS2HH12 is not a DTS device (it is driven via the ST pid driver over raw I2C), so the INT line is declared as a `gpio-buttons` child node and read with `GPIO_DT_SPEC_GET`, matching the manual-GPIO pattern the project uses.

- [ ] **Step 1: Add an `accel_int` node**

In the `buttons { ... }` block, add a second child so it reads:

```dts
	buttons {
		compatible = "gpio-buttons";
		user_button: button_0 {
			gpios = <&gpio0 17 GPIO_ACTIVE_HIGH>;
			label = "Button 0";
		};
		accel_int: accel_int_0 {
			gpios = <&gpio0 28 GPIO_ACTIVE_HIGH>;
			label = "Accel INT1";
		};
	};
```

- [ ] **Step 2: Build**

Run: `west build`
Expected: clean build (the node is unused for now). `DT_NODELABEL(accel_int)` is available to C code after this.

- [ ] **Step 3: Commit**

```bash
git add boards/Innovaforce/nRF52833_tag/nRF52833_tag.dts
git commit -m "feat(tag): declare LIS2HH12 INT1 GPIO (P0.28) in board DTS"
```

---

## Task 5: Motion module — LIS2HH12 activity interrupt drives cadence

**Files:**
- Create: `src/motion.h`
- Create: `src/motion.c`
- Modify: `CMakeLists.txt` (add `src/motion.c`)
- Modify: `src/main.c` (call `motion_init()`)

Configure the LIS2HH12 activity/inactivity engine: above `ACT_THS` it reports "active", and after `ACT_DUR` below threshold it reports "inactive". Route this to INT1 (P0.28). A GPIO both-edge interrupt reads the pin level and calls `uwb_set_moving()`.

- [ ] **Step 1: Create `src/motion.h`**

```c
#ifndef MOTION_H_
#define MOTION_H_

/* Configure the LIS2HH12 activity interrupt and the INT1 GPIO.
 * On motion the ranging cadence is set fast; after ~5 s of stillness, slow.
 * Returns 0 on success, negative errno on failure. */
int motion_init(void);

#endif /* MOTION_H_ */
```

- [ ] **Step 2: Create `src/motion.c`**

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "motion.h"
#include "lis2hh12_if.h"
#include "uwb_ss_initiator.h"

/* INT1 pin: P0.28, declared as accel_int in the board DTS. */
static const struct gpio_dt_spec accel_int =
    GPIO_DT_SPEC_GET(DT_NODELABEL(accel_int), gpios);

/* Logical pin level that means "moving". The LIS2HH12 INT1_INACT polarity
 * is verified on hardware in Step 6; flip this (0/1) if cadence is inverted. */
#define ACCEL_INT_MOVING_LEVEL  1

/* Activity tuning (LIS2HH12, FS=2g, ODR=50 Hz):
 *  ACT_THS LSB = FS/128 = 2000 mg / 128 ≈ 15.6 mg.  8 -> ~125 mg.
 *  ACT_DUR LSB = 8 / ODR = 8 / 50 = 0.16 s.  31 -> ~5.0 s no-motion window. */
#define ACT_THRESHOLD   8U
#define ACT_DURATION    31U

static stmdev_ctx_t dev_ctx;
static struct gpio_callback int_cb;

static void int_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    int level = gpio_pin_get_dt(&accel_int);   /* logical level (debounced by hardware engine) */

    uwb_set_moving(level == ACCEL_INT_MOVING_LEVEL);
}

int motion_init(void)
{
    int err;

    err = lis2hh12_if_init(&dev_ctx);
    if (err) {
        return err;
    }

    /* Sensor: 2g full scale, 50 Hz ODR. */
    if (lis2hh12_xl_full_scale_set(&dev_ctx, LIS2HH12_2g) != 0) {
        return -EIO;
    }
    if (lis2hh12_xl_data_rate_set(&dev_ctx, LIS2HH12_XL_ODR_50Hz) != 0) {
        return -EIO;
    }

    /* Activity/inactivity engine. */
    if (lis2hh12_act_threshold_set(&dev_ctx, ACT_THRESHOLD) != 0) {
        return -EIO;
    }
    if (lis2hh12_act_duration_set(&dev_ctx, ACT_DURATION) != 0) {
        return -EIO;
    }

    /* INT1 electrical config: active high, not latched (level follows state). */
    if (lis2hh12_pin_polarity_set(&dev_ctx, LIS2HH12_ACTIVE_HIGH) != 0) {
        return -EIO;
    }
    if (lis2hh12_pin_notification_set(&dev_ctx, LIS2HH12_INT_PULSED) != 0) {
        return -EIO;
    }

    /* Route inactivity/activity signal to INT1. */
    lis2hh12_pin_int1_route_t route = {0};
    route.int1_inact = 1;
    if (lis2hh12_pin_int1_route_set(&dev_ctx, route) != 0) {
        return -EIO;
    }

    /* GPIO interrupt on INT1 (P0.28), both edges. */
    if (!gpio_is_ready_dt(&accel_int)) {
        return -ENODEV;
    }
    err = gpio_pin_configure_dt(&accel_int, GPIO_INPUT);
    if (err) {
        return err;
    }
    err = gpio_pin_interrupt_configure_dt(&accel_int, GPIO_INT_EDGE_BOTH);
    if (err) {
        return err;
    }
    gpio_init_callback(&int_cb, int_handler, BIT(accel_int.pin));
    gpio_add_callback(accel_int.port, &int_cb);

    /* Seed current state from the pin so we do not wait for the first edge. */
    uwb_set_moving(gpio_pin_get_dt(&accel_int) == ACCEL_INT_MOVING_LEVEL);

    return 0;
}
```

- [ ] **Step 3: Add `src/motion.c` to `CMakeLists.txt`**

In `target_sources(app PRIVATE ...)` add:

```
    src/motion.c
```

- [ ] **Step 4: Call `motion_init()` from `main.c`**

Add the include near the other app includes at the top of `src/main.c`:

```c
#include "motion.h"
```

Inside the `if (uwb_init(3) == 0) { ... }` success branch, after `uwb_ss_initiator_start();`, add:

```c
        if (motion_init() != 0) {
            ble_log_send("motion init fail\n");
        }
```

(On failure the tag still ranges — it just stays in fast cadence because `ss_moving` defaults to true.)

- [ ] **Step 5: Build**

Run: `west build`
Expected: clean build.

- [ ] **Step 6: Verify on hardware — cadence follows motion**

Flash, connect BLE, watch `D:x.xxm` timing.
Expected: while you move/handle the tag, lines arrive ~1 s apart; leave it still ~5 s and lines slow to ~5 s apart; move again and it returns to ~1 s.
If the behavior is **inverted** (slow while moving, fast while still), flip `ACCEL_INT_MOVING_LEVEL` to `0` and rebuild.

- [ ] **Step 7: Commit**

```bash
git add src/motion.c src/motion.h CMakeLists.txt src/main.c
git commit -m "feat(tag): motion-adaptive ranging cadence via LIS2HH12 activity interrupt"
```

---

## Task 6: On-demand battery state-of-charge read

**Files:**
- Modify: `src/batt.h`
- Modify: `src/batt.c`
- Modify: `src/main.c` (drop `batt_init()` call if present — see note)

Replace the periodic 10 s timer with a synchronous read used by the UI on a single button press.

> Note: `main.c` does not currently call `batt_init()` (battery code is dormant), so no call site needs removing. If a `batt_init()` call is later added, it is unnecessary with this change.

- [ ] **Step 1: Replace `src/batt.h`**

```c
#ifndef BATT_H_
#define BATT_H_

/* Read battery state of charge as a percentage (0-100).
 * Returns 0 on success and writes *soc; negative errno on failure
 * (e.g. -ENODEV when the gauge is not ready / no battery attached). */
int batt_read_soc(int *soc);

#endif /* BATT_H_ */
```

- [ ] **Step 2: Replace `src/batt.c`**

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include "batt.h"

static const struct device *fg = DEVICE_DT_GET(DT_NODELABEL(fuel_gauge));

int batt_read_soc(int *soc)
{
    struct sensor_value val;

    if (soc == NULL) {
        return -EINVAL;
    }
    if (!device_is_ready(fg)) {
        return -ENODEV;
    }
    if (sensor_sample_fetch(fg) < 0) {
        return -EIO;
    }
    if (sensor_channel_get(fg, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE, &val) < 0) {
        return -EIO;
    }

    *soc = val.val1;   /* val1 = percent */
    return 0;
}
```

- [ ] **Step 3: Build**

Run: `west build`
Expected: clean build. (`batt.c` is already in `CMakeLists.txt`; no change there.)

- [ ] **Step 4: Commit**

```bash
git add src/batt.c src/batt.h
git commit -m "refactor(tag): replace periodic battery poll with on-demand batt_read_soc"
```

---

## Task 7: UI module — button gesture detection (logging only)

**Files:**
- Create: `src/tag_ui.h`
- Create: `src/tag_ui.c`
- Modify: `CMakeLists.txt` (add `src/tag_ui.c`)
- Modify: `src/main.c` (call `tag_ui_init()`)

Build the button half first: a GPIO ISR posts debounced press timestamps; the UI thread classifies single vs double press and a "stop blink" press, and logs the gesture over BLE. LED effects are added in Task 8. This lets gesture detection be verified independently.

- [ ] **Step 1: Create `src/tag_ui.h`**

```c
#ifndef TAG_UI_H_
#define TAG_UI_H_

/* Start the UI thread: button gesture handling + LED feedback. */
void tag_ui_init(void);

#endif /* TAG_UI_H_ */
```

- [ ] **Step 2: Create `src/tag_ui.c` (gesture detection)**

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "tag_ui.h"
#include "ble_log.h"

/* Button: P0.17, button0 alias in the board DTS. */
static const struct gpio_dt_spec button =
    GPIO_DT_SPEC_GET(DT_ALIAS(button0), gpios);

#define DEBOUNCE_MS   30U
#define DOUBLE_MS     350U    /* window to detect a second press */

/* Press timestamps (uptime ms) flow ISR -> UI thread. */
K_MSGQ_DEFINE(press_q, sizeof(uint32_t), 8, 4);

static struct gpio_callback btn_cb;

static void button_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    static uint32_t last_ms;
    uint32_t now = k_uptime_get_32();

    /* Only count the active (pressed) edge, debounced. */
    if (gpio_pin_get_dt(&button) != 1) {
        return;
    }
    if ((now - last_ms) < DEBOUNCE_MS) {
        return;
    }
    last_ms = now;

    k_msgq_put(&press_q, &now, K_NO_WAIT);   /* drop if full */
}

/* UI state. BLINK handling/LED come in Task 8; here we track the flag only. */
static bool blinking;

#define UI_PRIO    7
#define UI_STACK   1024

K_THREAD_STACK_DEFINE(ui_stack, UI_STACK);
static struct k_thread ui_tid;

static void ui_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    uint32_t t;

    while (1) {
        /* Wait for the first press of a gesture. */
        if (k_msgq_get(&press_q, &t, K_FOREVER) != 0) {
            continue;
        }

        if (blinking) {
            /* Any press stops blinking; consume it (no battery display). */
            blinking = false;
            ble_log_send("UI: blink stop\n");
            continue;
        }

        /* Wait up to DOUBLE_MS for a second press. */
        if (k_msgq_get(&press_q, &t, K_MSEC(DOUBLE_MS)) == 0) {
            blinking = true;
            ble_log_send("UI: double\n");
        } else {
            ble_log_send("UI: single\n");
        }
    }
}

void tag_ui_init(void)
{
    if (!gpio_is_ready_dt(&button)) {
        return;
    }
    gpio_pin_configure_dt(&button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&btn_cb, button_isr, BIT(button.pin));
    gpio_add_callback(button.port, &btn_cb);

    k_thread_create(&ui_tid, ui_stack, K_THREAD_STACK_SIZEOF(ui_stack),
                    ui_fn, NULL, NULL, NULL, UI_PRIO, 0, K_NO_WAIT);
}
```

- [ ] **Step 3: Add `src/tag_ui.c` to `CMakeLists.txt`**

In `target_sources(app PRIVATE ...)` add:

```
    src/tag_ui.c
```

- [ ] **Step 4: Call `tag_ui_init()` from `main.c`**

Add the include at the top of `src/main.c`:

```c
#include "tag_ui.h"
```

In the `uwb_init` success branch, after the `motion_init()` block from Task 5, add:

```c
        tag_ui_init();
```

- [ ] **Step 5: Build**

Run: `west build`
Expected: clean build.

- [ ] **Step 6: Verify on hardware — gestures over BLE**

Flash, connect BLE.
Expected:
- One press → `UI: single` (after ~350 ms).
- Two fast presses → `UI: double`.
- After a `UI: double`, one press → `UI: blink stop` (and that press does **not** produce `UI: single`).

- [ ] **Step 7: Commit**

```bash
git add src/tag_ui.c src/tag_ui.h CMakeLists.txt src/main.c
git commit -m "feat(tag): button single/double/stop gesture detection in UI thread"
```

---

## Task 8: UI module — LED feedback (battery color + orange blink)

**Files:**
- Modify: `src/tag_ui.c`

Add the WS2812 LED behaviors to the gestures from Task 7: single press → battery color for 5 s; double press → orange blink until a press stops it.

- [ ] **Step 1: Add LED + battery includes and the strip device to `src/tag_ui.c`**

Add to the includes at the top:

```c
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include "batt.h"
```

Add the device handle and color helpers near the top of the file (after the includes):

```c
static const struct device *strip = DEVICE_DT_GET(DT_ALIAS(led_strip));

#define BLINK_MS       500U
#define BATTERY_MS    5000U

static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    struct led_rgb px = { .r = r, .g = g, .b = b };
    led_strip_update_rgb(strip, &px, 1);
}

#define LED_OFF()     led_set(0, 0, 0)
#define LED_ORANGE()  led_set(10, 4, 0)

/* Discrete SoC -> color (matches the design spec). */
static void led_show_battery(void)
{
    int soc;

    if (batt_read_soc(&soc) != 0) {
        led_set(0, 0, 10);          /* dim blue: no gauge data */
        return;
    }
    if (soc >= 75) {
        led_set(0, 10, 0);          /* green */
    } else if (soc >= 50) {
        led_set(10, 10, 0);         /* yellow */
    } else if (soc >= 25) {
        led_set(10, 4, 0);          /* orange */
    } else {
        led_set(10, 0, 0);          /* red */
    }
}
```

- [ ] **Step 2: Replace the single-press branch to show battery for 5 s**

In `ui_fn`, replace the `else` branch that logs `"UI: single\n"`:

```c
        } else {
            ble_log_send("UI: single\n");
        }
```

with a battery display that can be interrupted by a new press:

```c
        } else {
            led_show_battery();
            /* Hold for 5 s, but let a new press start a fresh gesture. */
            if (k_msgq_get(&press_q, &t, K_MSEC(BATTERY_MS)) == 0) {
                LED_OFF();
                k_msgq_put(&press_q, &t, K_NO_WAIT);   /* re-queue: handle as new gesture */
                continue;
            }
            LED_OFF();
        }
```

- [ ] **Step 3: Replace the double-press branch to blink until a press**

Replace the `if (... == 0)` double branch that sets `blinking = true` and logs:

```c
        if (k_msgq_get(&press_q, &t, K_MSEC(DOUBLE_MS)) == 0) {
            blinking = true;
            ble_log_send("UI: double\n");
        } else {
```

with a self-contained blink loop (no separate `blinking` flag needed):

```c
        if (k_msgq_get(&press_q, &t, K_MSEC(DOUBLE_MS)) == 0) {
            /* Blink orange until any press; toggle every BLINK_MS. */
            bool on = true;
            while (1) {
                if (on) { LED_ORANGE(); } else { LED_OFF(); }
                on = !on;
                if (k_msgq_get(&press_q, &t, K_MSEC(BLINK_MS)) == 0) {
                    break;   /* press -> stop blinking */
                }
            }
            LED_OFF();
        } else {
```

- [ ] **Step 4: Remove the now-unused `blinking` flag and its handler**

Delete the file-scope declaration:

```c
static bool blinking;
```

Delete the `if (blinking) { ... }` block at the top of the `while (1)` loop (the blink is now handled inline in Step 3, and the press that breaks the loop is consumed there).

- [ ] **Step 5: Build**

Run: `west build`
Expected: clean build, no unused-variable warning for `blinking`.

- [ ] **Step 6: Verify on hardware**

Flash. (BLE optional.)
Expected:
- Single press → LED shows battery color (green/yellow/orange/red, or dim blue if no battery) for 5 s, then off.
- Double press → LED blinks orange ~2 Hz; a single press stops it and the LED goes off; that press does not show battery.
- Ranging continues normally throughout (cadence still adapts to motion).

- [ ] **Step 7: Commit**

```bash
git add src/tag_ui.c
git commit -m "feat(tag): LED battery indicator and orange blink mode"
```

---

## Self-Review

**Spec coverage:**
- Motion-adaptive 1 s/5 s cadence → Tasks 3 + 5. ✓ (on-chip ACT_DUR timing — deviation flagged for approval)
- Single press → battery color 5 s then off → Task 8 Step 2. ✓
- Double press → orange blink until next press → Task 8 Step 3. ✓
- "Stop blink" press does not show battery → Task 8 Step 3 (press consumed by the blink loop). ✓
- Distance over BLE only if connected, else dropped → existing `ble_log_send` + Task 2. ✓
- Ranging independent of BLE → Task 2. ✓
- UWB highest priority → ranging prio 2 unchanged; UI prio 7, BLE prio 6. ✓
- Discrete battery colors incl. no-data → Task 8 Step 1. ✓
- Examples moved out of build → Task 1. ✓
- DTS INT line P0.28 → Task 4. ✓

**Placeholder scan:** No TBD/TODO; all code blocks are concrete. Two hardware-tunable constants (`ACCEL_INT_MOVING_LEVEL`, `ACT_THRESHOLD`/`ACT_DURATION`) are explicitly documented with verification steps, not placeholders.

**Type consistency:** `uwb_set_moving(bool)` declared in Task 3 header, called in Task 5. `batt_read_soc(int *)` declared/defined in Task 6, called in Task 8. `tag_ui_init()`/`motion_init()` declared in their headers, called in `main.c`. `press_q` shared across Task 7/8. LIS2HH12 functions and enums verified against `lis2hh12_reg.h`.
