# Power Saving — Layer 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Cut the tag's average current (~35 mA baseline) by deep-sleeping the DW3000 between each superframe's activity, letting the SoC enter PM idle, keeping the LED dark, and adding on-device current monitoring — without changing the TDMA on-air contract or the gateway.

**Architecture:** The DW3000-specific glue lives in `src/uwb_net_runner.c` (the `uwb_radio_*` ops impl); the chip SLEEP/wake is inserted there, in the existing `UWB_ACT_SLEEP` slack, gated by a runtime flag toggled over BLE NUS (`pwr sleep on|off`). SoC sleep is enabled by `CONFIG_PM`. Monitoring reads `SENSOR_CHAN_GAUGE_AVG_CURRENT` from the BQ27421 and keeps a min/mean/max accumulator over the BLE-disconnected window (the real field current).

**Tech Stack:** Zephyr RTOS (nCS 3.2.4), nRF52833, Decawave DW3000 precompiled driver, BQ274xx sensor driver, BLE NUS.

## Global Constraints

- **NUS 20-byte payload limit:** Default ATT MTU 23 → a single `ble_log_send()` carries ≤20 bytes; `bt_nus_send` does not fragment (>20 bytes silently dropped). Keep every diagnostic string ≤20 bytes including `\n`.
- **DW3000 critical sections:** Bracket every `dwt_*` call that touches the IRQ/chip state with `decamutexon()` / `decamutexoff()`.
- **Sleep mode = SLEEP, not DEEPSLEEP** (faster wake, config retained via AON).
- **Antenna-delay re-application after wake is mandatory** — skipping it silently degrades ranging accuracy.
- **No TDMA-logic change:** host tests (`tests/uwb_net`, `tests/uwb_frame`, `tests/cal_math`) must still pass; sleep/wake is target-only HAL.
- **New `.c` files must be added** to `CMakeLists.txt` via `target_sources(app PRIVATE ...)`.
- **Host test build** (Windows, WinLibs gcc — not on PATH, not clang):
  ```
  GCC="/c/Users/JoseAntonioLaraPerez/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe/mingw64/bin/gcc.exe"
  ```
- The user builds/flashes firmware and reports results; **target tasks are verified on-device** with the exact expected BLE NUS output stated in each task.

---

### Task 1: LED dark in normal operation

Remove the persistent boot/BLE status LED. LED stays off except the button-driven battery display (already in `tag_ui.c`) and the fatal DW3000-init-failure **red** latch.

**Files:**
- Modify: `src/main.c` (remove `on_ble_state` + its registration + boot LED writes; keep red on init failure)

**Interfaces:**
- Consumes: nothing new.
- Produces: nothing for later tasks (Task 4 re-adds a BLE state callback for monitoring).

- [ ] **Step 1: Edit `src/main.c`** — delete the `on_ble_state` function (lines ~18-28) and the `ble_log_set_state_cb(on_ble_state);` call (~line 43). Remove the cyan init LED (~49-51) and the green success LED (~54-56). Keep the red latch in the failure branch. The body becomes:

```c
int main(void)
{
    if (!device_is_ready(strip)) {
        k_sleep(K_FOREVER);
        return 0;
    }

    if (ble_log_init() != 0) {
        k_sleep(K_FOREVER);
        return 0;
    }

    /* Arm the boot-guard watchdog: if DW3000 bring-up hangs or fails, the
     * watchdog is never fed and the SoC resets in ~10 s, retrying the boot. */
    tag_wdt_start_boot_guard();

    if (uwb_init(3) == 0) {
        ble_log_send("Config OK\n");

        uint8_t eui[8];
        sys_put_le32(NRF_FICR->DEVICEID[0], &eui[0]);
        sys_put_le32(NRF_FICR->DEVICEID[1], &eui[4]);

        if (cal_init()) {
            ble_log_send("CAL loaded\n");
        } else {
            ble_log_send("CAL REQUIRED\n");
        }
        uwb_ss_initiator_start();
        uwb_net_runner_start(eui);
        if (motion_init() != 0) {
            ble_log_send("motion init fail\n");
        }
        tag_ui_init();
        tag_wdt_run_feeder();
    } else {
        /* Red: fatal DW3000 init failure — the only signal with no BLE. */
        struct led_rgb pixel = (struct led_rgb){.r = 10, .g = 0, .b = 0};
        led_strip_update_rgb(strip, &pixel, 1);
        ble_log_send("DW3000: init failed\n");
    }

    k_sleep(K_FOREVER);
    return 0;
}
```

- [ ] **Step 2: Build.** Expected: compiles clean (no unused `on_ble_state`, no unused `pixel` in the success path).

- [ ] **Step 3: Verify on device.** Power the tag with no phone connected. Expected: LED **off** throughout normal operation (no green/blue/cyan). Press the button → battery color shows (unchanged). Force a DW3000 init failure (e.g., disconnect the module) → LED latches **red**.

- [ ] **Step 4: Commit.**

```bash
git add src/main.c
git commit -m "feat(power): LED dark in normal operation, keep red fatal-fail latch"
```

---

### Task 2: Read battery current + report on button press

Add `batt_read_current()` and append an `I:<ma>mA` line to the button battery display so deltas can be measured live over BLE.

**Files:**
- Modify: `src/batt.h`, `src/batt.c`, `src/tag_ui.c`

**Interfaces:**
- Produces: `int batt_read_current(int *ma);` — returns 0 and writes `*ma` (discharge magnitude in mA); negative errno on failure.

- [ ] **Step 1: Declare in `src/batt.h`** (above the `#endif`):

```c
/* Read average battery current in milliamps (discharge as a positive
 * magnitude). Returns 0 on success; negative errno on failure. */
int batt_read_current(int *ma);
```

- [ ] **Step 2: Implement in `src/batt.c`** (append after `batt_read_soc`):

```c
int batt_read_current(int *ma)
{
    struct sensor_value val;

    if (ma == NULL) {
        return -EINVAL;
    }
    if (!device_is_ready(fg)) {
        return -ENODEV;
    }
    if (sensor_sample_fetch(fg) < 0) {
        return -EIO;
    }
    if (sensor_channel_get(fg, SENSOR_CHAN_GAUGE_AVG_CURRENT, &val) < 0) {
        return -EIO;
    }

    /* Zephyr current units: val1 = whole amps, val2 = microamp fraction. */
    long ua  = (long)val.val1 * 1000000L + val.val2;
    long mma = ua / 1000L;
    *ma = (int)(mma < 0 ? -mma : mma);
    return 0;
}
```

- [ ] **Step 3: Append current to the report in `src/tag_ui.c`** — inside `led_show_battery()`, after the existing `ble_log_send(msg);` that sends `BATT:`, add:

```c
    int ma;
    if (batt_read_current(&ma) == 0) {
        char imsg[16];
        snprintf(imsg, sizeof(imsg), "I:%dmA\n", ma);
        ble_log_send(imsg);
    }
```

- [ ] **Step 4: Build.** Expected: compiles clean.

- [ ] **Step 5: Verify on device.** Connect a phone (NUS), press the button. Expected two lines: `BATT: <n>%` then `I:<n>mA` (each ≤20 bytes). With a battery attached, the mA value should be plausible (tens of mA today).

- [ ] **Step 6: Commit.**

```bash
git add src/batt.h src/batt.c src/tag_ui.c
git commit -m "feat(power): read BQ27421 average current, report I:<mA> on button"
```

---

### Task 3: `batt_window` accumulator (pure module + host test)

A dependency-free min/mean/max accumulator for current samples. Pure C so it is host-testable.

**Files:**
- Create: `src/batt_window.h`, `src/batt_window.c`, `tests/batt_window/test_batt_window.c`

**Interfaces:**
- Produces:
  - `void batt_window_reset(struct batt_window *w);`
  - `void batt_window_add(struct batt_window *w, int ma);`
  - `int  batt_window_get(const struct batt_window *w, int *min_ma, int *mean_ma, int *max_ma, uint32_t *count);` — returns 1 if ≥1 sample (out params written), else 0.

- [ ] **Step 1: Write the failing test** `tests/batt_window/test_batt_window.c`:

```c
#include "../../src/batt_window.h"
#include <stdio.h>

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

int main(void)
{
    struct batt_window w;

    /* Empty window: get returns 0, nothing written. */
    batt_window_reset(&w);
    CHECK(batt_window_get(&w, NULL, NULL, NULL, NULL) == 0);

    /* Three samples: min/mean/max/count. */
    batt_window_add(&w, 30);
    batt_window_add(&w, 50);
    batt_window_add(&w, 40);
    int mn = -1, me = -1, mx = -1; uint32_t n = 0;
    CHECK(batt_window_get(&w, &mn, &me, &mx, &n) == 1);
    CHECK(mn == 30);
    CHECK(mx == 50);
    CHECK(me == 40);
    CHECK(n == 3);

    /* Single sample: min == mean == max. */
    batt_window_reset(&w);
    batt_window_add(&w, 7);
    CHECK(batt_window_get(&w, &mn, &me, &mx, &n) == 1);
    CHECK(mn == 7 && me == 7 && mx == 7 && n == 1);

    /* Reset clears. */
    batt_window_reset(&w);
    CHECK(batt_window_get(&w, NULL, NULL, NULL, NULL) == 0);

    printf("batt_window: %d failure(s)\n", fails);
    return fails == 0 ? 0 : 1;
}
```

- [ ] **Step 2: Write the header** `src/batt_window.h`:

```c
#ifndef BATT_WINDOW_H_
#define BATT_WINDOW_H_

#include <stdint.h>

struct batt_window {
    int      min_ma;
    int      max_ma;
    long     sum_ma;
    uint32_t count;
};

void batt_window_reset(struct batt_window *w);
void batt_window_add(struct batt_window *w, int ma);
int  batt_window_get(const struct batt_window *w,
                     int *min_ma, int *mean_ma, int *max_ma, uint32_t *count);

#endif /* BATT_WINDOW_H_ */
```

- [ ] **Step 3: Run the test, verify it FAILS** (no implementation yet):

```bash
GCC="/c/Users/JoseAntonioLaraPerez/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe/mingw64/bin/gcc.exe"
"$GCC" -std=c99 -Wall -Wextra -Werror -I src tests/batt_window/test_batt_window.c src/batt_window.c -o /tmp/tbw.exe && /tmp/tbw.exe
```
Expected: **link error** (`batt_window.c` does not exist yet / undefined references).

- [ ] **Step 4: Implement** `src/batt_window.c`:

```c
#include "batt_window.h"

void batt_window_reset(struct batt_window *w)
{
    w->min_ma = 0;
    w->max_ma = 0;
    w->sum_ma = 0;
    w->count  = 0;
}

void batt_window_add(struct batt_window *w, int ma)
{
    if (w->count == 0) {
        w->min_ma = ma;
        w->max_ma = ma;
    } else {
        if (ma < w->min_ma) { w->min_ma = ma; }
        if (ma > w->max_ma) { w->max_ma = ma; }
    }
    w->sum_ma += ma;
    w->count++;
}

int batt_window_get(const struct batt_window *w,
                    int *min_ma, int *mean_ma, int *max_ma, uint32_t *count)
{
    if (w->count == 0) {
        return 0;
    }
    if (min_ma)  { *min_ma  = w->min_ma; }
    if (max_ma)  { *max_ma  = w->max_ma; }
    if (mean_ma) { *mean_ma = (int)(w->sum_ma / (long)w->count); }
    if (count)   { *count   = w->count; }
    return 1;
}
```

- [ ] **Step 5: Run the test, verify it PASSES:**

```bash
"$GCC" -std=c99 -Wall -Wextra -Werror -I src tests/batt_window/test_batt_window.c src/batt_window.c -o /tmp/tbw.exe && /tmp/tbw.exe
```
Expected: `batt_window: 0 failure(s)` and exit 0.

- [ ] **Step 6: Add to build** — in `CMakeLists.txt`, add `src/batt_window.c` to the `target_sources(app PRIVATE ...)` list.

- [ ] **Step 7: Commit.**

```bash
git add src/batt_window.h src/batt_window.c tests/batt_window/test_batt_window.c CMakeLists.txt
git commit -m "feat(power): add host-tested batt_window min/mean/max accumulator"
```

---

### Task 4: Disconnected-window current sampling + report on reconnect

Sample current at a low rate while BLE is disconnected, accumulate into a `batt_window`, and dump `Idle:<mean> <min>/<max> n<count>` on the next connect (the BLE-off field current).

**Files:**
- Modify: `src/batt.h`, `src/batt.c`, `src/main.c`

**Interfaces:**
- Consumes: `batt_read_current()` (Task 2), `batt_window_*` (Task 3), `ble_state_t` (`ble_log.h`).
- Produces:
  - `void batt_monitor_start(void);` — starts the periodic sampling timer.
  - `void batt_on_ble_state(ble_state_t state);` — call on every BLE state change.

- [ ] **Step 1: Declare in `src/batt.h`** (add the include and prototypes):

```c
#include "ble_log.h"

/* Start the periodic current-sampling timer (call once after BLE is up). */
void batt_monitor_start(void);

/* Feed BLE connection state: resets the window on disconnect, arms the
 * idle-window report on connect. Call from the BLE state callback. */
void batt_on_ble_state(ble_state_t state);
```

- [ ] **Step 2: Implement the sampler in `src/batt.c`** — add includes and the timer/work below `batt_read_current`:

```c
#include "batt_window.h"

#define BATT_SAMPLE_MS 5000

static struct batt_window idle_win;
static volatile bool      batt_connected;
static volatile bool      report_pending;

static void batt_work_fn(struct k_work *work)
{
    ARG_UNUSED(work);

    int ma;
    if (batt_read_current(&ma) != 0) {
        return;   /* no gauge / no battery: skip this tick */
    }

    if (batt_connected) {
        if (report_pending) {
            int mn = 0, me = 0, mx = 0; uint32_t n = 0;
            if (batt_window_get(&idle_win, &mn, &me, &mx, &n)) {
                char msg[20];
                snprintf(msg, sizeof(msg), "Idle:%d %d/%d n%u\n", me, mn, mx, n);
                ble_log_send(msg);
            }
            report_pending = false;
        }
    } else {
        batt_window_add(&idle_win, ma);
    }
}

K_WORK_DEFINE(batt_work, batt_work_fn);

static void batt_timer_fn(struct k_timer *t)
{
    ARG_UNUSED(t);
    k_work_submit(&batt_work);   /* gauge I2C reads must not run in timer ISR */
}

K_TIMER_DEFINE(batt_timer, batt_timer_fn, NULL);

void batt_monitor_start(void)
{
    batt_window_reset(&idle_win);
    k_timer_start(&batt_timer, K_MSEC(BATT_SAMPLE_MS), K_MSEC(BATT_SAMPLE_MS));
}

void batt_on_ble_state(ble_state_t state)
{
    if (state == BLE_STATE_CONNECTED) {
        batt_connected = true;
        report_pending = true;   /* dump the idle window on the next tick */
    } else {
        batt_connected = false;
        batt_window_reset(&idle_win);   /* start a fresh disconnected window */
    }
}
```

- [ ] **Step 3: Wire into `src/main.c`** — add a thin BLE state callback that forwards to batt, register it, and start the monitor. After `ble_log_init()` succeeds and before `tag_wdt_start_boot_guard()` (or right after), add:

```c
    ble_log_set_state_cb(batt_on_ble_state);
```

and inside the `uwb_init(3) == 0` success branch, after `tag_ui_init();`, add:

```c
    batt_monitor_start();
```

Add `#include "batt.h"` near the other includes in `main.c`.

- [ ] **Step 4: Build.** Expected: compiles clean.

- [ ] **Step 5: Verify on device.** Boot with no phone. Wait ~30 s (≥6 samples). Connect the phone. Expected: within ~5 s a line `Idle:<mean> <min>/<max> n<count>` appears — the average current while BLE was off. Disconnect, wait, reconnect → a fresh `Idle:` for the new window.

- [ ] **Step 6: Commit.**

```bash
git add src/batt.h src/batt.c src/main.c
git commit -m "feat(power): sample current while BLE-disconnected, report Idle: on connect"
```

---

### Task 5: Enable SoC Power Management

Let the SoC reach deep idle during the inter-superframe slack.

**Files:**
- Modify: `prj.conf`

**Interfaces:** none.

- [ ] **Step 1: Add to `prj.conf`** (after `CONFIG_TASK_WDT=y`):

```
CONFIG_PM=y
```

- [ ] **Step 2: Build.** Expected: compiles clean.

- [ ] **Step 3: Verify on device — BLE survives sleep.** Connect a phone and keep the NUS session open for ≥2 min. Expected: link does **not** drop; the button battery display still works; `Idle:` reporting still works.

- [ ] **Step 4: Verify on device — current drops.** Compare the `Idle:` mean (Task 4) before vs after this change. Expected: a measurable drop vs the pre-PM build (SoC no longer busy between superframes).

- [ ] **Step 5: Commit.**

```bash
git add prj.conf
git commit -m "feat(power): enable CONFIG_PM for SoC deep idle between superframes"
```

> Note: `CONFIG_PM_DEVICE=y` (suspend SPI1 while the DW3000 sleeps) is deferred — evaluate after Task 7 if more saving is needed; it interacts with the WS2812 SPI3 and I2C0 gauge and is out of Layer-1 scope unless required.

---

### Task 6: `pwr` command dispatcher + DW3000-sleep enable flag

Introduce a single NUS command dispatcher that owns the RX handler, parses `pwr sleep on|off` (and `pwr` status), and forwards everything else to the existing `cal` parser. Add the runtime flag the next task reads.

**Files:**
- Modify: `src/cal.h`, `src/cal.c`, `src/uwb_net_runner.h`, `src/uwb_net_runner.c`, `src/main.c`, `CMakeLists.txt`
- Create: `src/tag_cmd.h`, `src/tag_cmd.c`

**Interfaces:**
- Produces:
  - `void uwb_radio_set_sleep_enabled(bool en);` / `bool uwb_radio_sleep_enabled(void);`
  - `void cal_on_rx(const uint8_t *data, uint16_t len);` (un-static'd)
  - `void tag_cmd_init(void);`

- [ ] **Step 1: Expose the flag in `src/uwb_net_runner.h`** (add prototypes):

```c
#include <stdbool.h>

/* Layer-1 power saving: enable/disable DW3000 SLEEP between superframes.
 * Default enabled. Toggled at runtime via the `pwr sleep on|off` NUS command. */
void uwb_radio_set_sleep_enabled(bool en);
bool uwb_radio_sleep_enabled(void);
```

- [ ] **Step 2: Define the flag in `src/uwb_net_runner.c`** — near the other file-scope state (e.g. after the `tier_pending` declaration ~line 80):

```c
static volatile bool dw_sleep_enabled = true;

void uwb_radio_set_sleep_enabled(bool en) { dw_sleep_enabled = en; }
bool uwb_radio_sleep_enabled(void)        { return dw_sleep_enabled; }
```

- [ ] **Step 3: Un-static the cal parser.** In `src/cal.h`, declare it (above `#endif`):

```c
/* NUS command parser for `cal ...` commands. Invoked by the tag_cmd dispatcher. */
void cal_on_rx(const uint8_t *data, uint16_t len);
```

In `src/cal.c`, change `static void cal_on_rx(...)` to `void cal_on_rx(...)`, and in `cal_init()` **remove** the line `ble_log_set_rx_handler(cal_on_rx);` (the dispatcher now owns registration).

- [ ] **Step 4: Create the dispatcher header** `src/tag_cmd.h`:

```c
#ifndef TAG_CMD_H_
#define TAG_CMD_H_

/* Registers the single NUS RX handler. Dispatches `pwr ...` locally and
 * forwards all other commands to the cal parser. Call once after cal_init(). */
void tag_cmd_init(void);

#endif /* TAG_CMD_H_ */
```

- [ ] **Step 5: Create the dispatcher** `src/tag_cmd.c`:

```c
#include <string.h>
#include "tag_cmd.h"
#include "ble_log.h"
#include "cal.h"
#include "uwb_net_runner.h"

static void tag_cmd_on_rx(const uint8_t *data, uint16_t len)
{
    char buf[24];
    uint16_t n = (len < sizeof(buf) - 1) ? len : (uint16_t)(sizeof(buf) - 1);

    memcpy(buf, data, n);
    buf[n] = '\0';
    while (n > 0 && (buf[n - 1] == '\r' || buf[n - 1] == '\n')) {
        buf[--n] = '\0';
    }

    if (strncmp(buf, "pwr", 3) == 0) {
        if (strcmp(buf, "pwr sleep on") == 0) {
            uwb_radio_set_sleep_enabled(true);
            ble_log_send("PWR sleep on\n");
        } else if (strcmp(buf, "pwr sleep off") == 0) {
            uwb_radio_set_sleep_enabled(false);
            ble_log_send("PWR sleep off\n");
        } else {
            ble_log_send(uwb_radio_sleep_enabled() ? "PWR sleep on\n"
                                                   : "PWR sleep off\n");
        }
        return;
    }

    /* Not a power command: forward the raw write to the cal parser. */
    cal_on_rx(data, len);
}

void tag_cmd_init(void)
{
    ble_log_set_rx_handler(tag_cmd_on_rx);
}
```

- [ ] **Step 6: Register in `src/main.c`** — add `#include "tag_cmd.h"`, and in the success branch after `cal_init()` returns, add `tag_cmd_init();` (after the `CAL loaded`/`CAL REQUIRED` send, before `uwb_ss_initiator_start()`).

- [ ] **Step 7: Add to build** — add `src/tag_cmd.c` to `target_sources(app PRIVATE ...)` in `CMakeLists.txt`.

- [ ] **Step 8: Build.** Expected: compiles clean (cal still parses; dispatcher owns the handler).

- [ ] **Step 9: Verify on device.** Connect a phone. Send `pwr` → reply `PWR sleep on`. Send `pwr sleep off` → `PWR sleep off`. Send `cal status` → unchanged cal reply (proves forwarding works).

- [ ] **Step 10: Commit.**

```bash
git add src/cal.h src/cal.c src/uwb_net_runner.h src/uwb_net_runner.c src/tag_cmd.h src/tag_cmd.c src/main.c CMakeLists.txt
git commit -m "feat(power): NUS command dispatcher + pwr sleep toggle flag"
```

---

### Task 7: DW3000 SLEEP/wake between superframes

The core saving. In the `UWB_ACT_SLEEP` slack, put the DW3000 into SLEEP, sleep the SoC until a guard before the next superframe edge, then wake and re-apply the config the runner owns. Gated by the Task-6 flag.

**Files:**
- Modify: `src/uwb_net_runner.c`

**Interfaces:**
- Consumes: `dw_sleep_enabled` (Task 6), `wakeup_device_with_io()` (`platform/port.c`), `cal_get_ant_dly()` (`cal.h`), DW3000 API (`deca_device_api.h`, already included).

- [ ] **Step 1: Add the include and guard constant.** Near the top of `src/uwb_net_runner.c` add `#include "cal.h"` (port.h and deca_device_api.h are already included). With the other timing `#define`s (near `T_SUPERFRAME_MS`) add:

```c
#define DW_WAKE_GUARD_MS 3u   /* wake the DW3000 this long before the next beacon */
```

- [ ] **Step 2: Add the sleep/wake helpers** above `runner_fn`:

```c
static void dw_enter_sleep(void)
{
    decamutexon();
    /* DWT_CONFIG: restore configuration from AON on wake.
     * Wake on the WAKEUP pin; enable sleep (SLEEP, not deep sleep). */
    dwt_configuresleep(DWT_CONFIG, DWT_WAKE_WUP | DWT_SLP_EN);
    dwt_entersleep(DWT_DW_IDLE);   /* auto INIT2IDLE → IDLE_PLL on wake */
    decamutexoff();
}

static void dw_wake(void)
{
    decamutexon();
    wakeup_device_with_io();
    /* Wait for the device to reach IDLE_RC after wake (~ms worst case). */
    for (int i = 0; i < 50 && !dwt_checkidlerc(); i++) {
        k_busy_wait(100);
    }
    /* Re-apply the runner-owned config (cheap; belt-and-suspenders over AON). */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    uint16_t tx, rx;
    cal_get_ant_dly(&tx, &rx);
    dwt_settxantennadelay(tx);
    dwt_setrxantennadelay(rx);
    decamutexoff();
}
```

- [ ] **Step 3: Replace the `UWB_ACT_SLEEP` block** (currently lines ~553-555):

```c
        if (act & UWB_ACT_SLEEP) {
            uint32_t wake_ms = t0_ms + T_SUPERFRAME_MS;
            if (dw_sleep_enabled) {
                dw_enter_sleep();
                if ((int32_t)(wake_ms - DW_WAKE_GUARD_MS - uwb_radio_now_ms()) > 0) {
                    uwb_radio_sleep_until(wake_ms - DW_WAKE_GUARD_MS);
                }
                dw_wake();
                uwb_radio_sleep_until(wake_ms);   /* re-align to the superframe edge */
            } else {
                uwb_radio_sleep_until(wake_ms);
            }
        }
```

- [ ] **Step 4: Build.** Expected: compiles clean.

- [ ] **Step 5: Verify on device — ranging still works with sleep ON.** Connect a phone, ensure `pwr sleep on` (default). With ≥3 anchors, confirm `P:x.xx,y.yy` still streams at the moving cadence and the value is accurate (antenna delay restored). Watch for `RESCAN` lines — they must **not** increase vs `pwr sleep off`.

- [ ] **Step 6: Verify on device — the saving.** Disconnect, operate ~30 s, reconnect: compare the `Idle:` mean with `pwr sleep on` vs `pwr sleep off` (toggle, disconnect, re-measure). Expected: a large drop with sleep on (the DW3000 idle current removed for ~85% of each superframe).

- [ ] **Step 7: Validate the µA floor (PPK2, optional).** Capture one superframe with a PPK2: confirm the current drops to µA between the beacon/sweep activity. (Documented because the BQ averages ~1 s and cannot resolve the deep-sleep floor.)

- [ ] **Step 8: Commit.**

```bash
git add src/uwb_net_runner.c
git commit -m "feat(power): DW3000 SLEEP between superframes, gated by pwr toggle"
```

---

## Tuning note (post-implementation)

If `RESCAN` rises with sleep on, the wake latency is eating into the beacon window. Mitigations in order: (1) increase `DW_WAKE_GUARD_MS`; (2) if still bad, reduce the sleep fraction (wake earlier) — still a net win over no sleep. Do **not** abandon the sleep; shorten it.

## Self-review checklist (done)

- **Spec coverage:** §3.1 DW3000 sleep → Task 7; §3.2 SoC PM → Task 5; §3.3 LED → Task 1; §3.4 (A) live + I: → Task 2, (B) disconnected window → Tasks 3+4, toggle command → Task 6, (C) PPK2 → Task 7 step 7. Verification §5 folded into each task's on-device steps.
- **Placeholders:** none — every code step has full code.
- **Type consistency:** `batt_window_*`, `batt_read_current(int*)`, `batt_on_ble_state(ble_state_t)`, `uwb_radio_set_sleep_enabled(bool)`/`uwb_radio_sleep_enabled(void)`, `cal_on_rx(const uint8_t*, uint16_t)`, `tag_cmd_init(void)` consistent across tasks.
