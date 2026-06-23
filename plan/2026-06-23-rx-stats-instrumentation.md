# RX Duty-Cycle Instrumentation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add software-only instrumentation that measures the beacon-listen RX-on duration and the beacon arrival offset, reported over BLE NUS via a `pwr rx` command, to size the future narrow-window mechanism (Camino A — Spec 2).

**Architecture:** A pure, Zephyr-free accumulator core (`rx_stats_core`, reusing the existing `batt_window` min/mean/max accumulator) does all the arithmetic and is host-tested. A thin Zephyr glue layer (`rx_stats`) captures cycle stamps via `k_cycle_get_32()` and owns a singleton. The runner feeds it three timestamp calls in the beacon-listen loop; `tag_cmd` reads it on demand. This mirrors the codebase's pure-core + glue split (`cal_math.c`/`cal.c`, `uwb_net.c`/`uwb_net_runner.c`).

**Tech Stack:** C99, Zephyr RTOS (nCS 3.2.4), nRF52833. Host tests built with WinLibs GCC.

## Global Constraints

- **Spec:** `spec/2026-06-23-rx-stats-instrumentation-design.md`. Scope is instrumentation ONLY; the narrow-window mechanism is Spec 2.
- **No Zephyr in the pure core** — `rx_stats_core.c` may include only `rx_stats_core.h`, `batt_window.h`, and C standard headers, so it compiles on the host.
- **NUS 20-byte payload limit** — every `ble_log_send()` string must stay ≤20 bytes including `\n`. Default ATT MTU is 23; `bt_nus_send` does not fragment and drops oversize payloads with `-EMSGSIZE`.
- **Nominal superframe = 200 ms** — `T_SUPERFRAME_MS` is defined privately in `src/uwb_net_runner.c:31`. The glue layer redefines the value with a comment to keep the module self-contained (the spec accepts this small documented duplication).
- **New source files must be added to `CMakeLists.txt`** via `target_sources(app PRIVATE ...)`.
- **The user builds and flashes the firmware** — firmware tasks are verified by code review here plus the user's build; only the host-test task runs a compiler in this environment.
- **Host test compiler** (not on PATH, full path required; use Git Bash / Bash tool):
  ```
  GCC="/c/Users/JoseAntonioLaraPerez/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe/mingw64/bin/gcc.exe"
  ```

---

### Task 1: Pure accumulator core `rx_stats_core` (host-tested)

**Files:**
- Create: `src/rx_stats_core.h`
- Create: `src/rx_stats_core.c`
- Test: `tests/rx_stats/test_rx_stats.c`

**Interfaces:**
- Consumes: `struct batt_window`, `batt_window_reset/add/get` from `src/batt_window.h` (existing).
- Produces:
  - `struct rx_stats_core` (fields below).
  - `void rx_stats_core_reset(struct rx_stats_core *c, uint32_t nominal_sf_cyc, uint32_t cyc_per_sec);`
  - `void rx_stats_core_beacon(struct rx_stats_core *c, uint32_t arm_cyc, uint32_t now_cyc);`
  - `void rx_stats_core_miss(struct rx_stats_core *c);`
  - `int rx_stats_core_get(const struct rx_stats_core *c, int *on_mean_ms, int *on_max_ms, int *off_min_us, int *off_max_us, uint32_t *count, uint32_t *misses);`

- [ ] **Step 1: Write the header**

Create `src/rx_stats_core.h`:

```c
#ifndef RX_STATS_CORE_H_
#define RX_STATS_CORE_H_

#include <stdint.h>
#include <stdbool.h>
#include "batt_window.h"

/* Pure (Zephyr-free) accumulator core for beacon RX duty-cycle instrumentation.
 * Operates on raw cycle-counter values supplied by the caller, so it is fully
 * host-testable. The Zephyr glue (rx_stats.c) captures the cycle stamps. */
struct rx_stats_core {
    struct batt_window on_win;     /* RX-on duration, microseconds */
    struct batt_window off_win;    /* beacon arrival offset, microseconds (signed) */
    uint32_t prev_beacon_cyc;      /* cycle stamp of the previous beacon */
    bool     prev_valid;           /* false until first beacon / after a miss */
    uint32_t miss_count;
    uint32_t nominal_sf_cyc;       /* nominal superframe period, in cycles */
    uint32_t cyc_per_sec;          /* cycle-counter frequency (Hz), for cyc->us */
};

/* Reset all state. nominal_sf_cyc = nominal superframe in cycles;
 * cyc_per_sec = cycle-counter frequency (Hz). */
void rx_stats_core_reset(struct rx_stats_core *c,
                         uint32_t nominal_sf_cyc, uint32_t cyc_per_sec);

/* Record a received beacon. arm_cyc = cycle stamp when the listen window was
 * armed; now_cyc = cycle stamp at beacon arrival. Records on-duration
 * (now-arm) and, if a previous beacon is known, arrival offset
 * (now - (prev + nominal)). Advances the phase reference to now_cyc. */
void rx_stats_core_beacon(struct rx_stats_core *c,
                          uint32_t arm_cyc, uint32_t now_cyc);

/* Record a missed beacon: invalidate the phase reference and count it. */
void rx_stats_core_miss(struct rx_stats_core *c);

/* Read stats. Returns 1 if >=1 beacon recorded (out params written), else 0.
 * on_* in milliseconds, off_* in microseconds. Any out param may be NULL. */
int rx_stats_core_get(const struct rx_stats_core *c,
                      int *on_mean_ms, int *on_max_ms,
                      int *off_min_us, int *off_max_us,
                      uint32_t *count, uint32_t *misses);

#endif /* RX_STATS_CORE_H_ */
```

- [ ] **Step 2: Write the failing test**

Create `tests/rx_stats/test_rx_stats.c`. Uses `cyc_per_sec = 1_000_000` so one cycle equals one microsecond, making every assertion exact:

```c
#include "../../src/rx_stats_core.h"
#include <stdio.h>

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

#define HZ      1000000u   /* 1 cycle == 1 microsecond */
#define SF_CYC   200000u   /* 200 ms superframe in cycles */

int main(void)
{
    struct rx_stats_core c;
    int on_mean = -1, on_max = -1, off_min = -1, off_max = -1;
    uint32_t n = 0, miss = 0;

    /* Empty: get returns 0. */
    rx_stats_core_reset(&c, SF_CYC, HZ);
    CHECK(rx_stats_core_get(&c, &on_mean, &on_max, &off_min, &off_max, &n, &miss) == 0);

    /* First beacon: on-duration recorded, NO offset yet.
     * arm=1000, beacon=151000 -> on = 150000 us -> 150 ms. */
    rx_stats_core_beacon(&c, 1000u, 151000u);
    CHECK(rx_stats_core_get(&c, &on_mean, &on_max, &off_min, &off_max, &n, &miss) == 1);
    CHECK(on_mean == 150);
    CHECK(n == 1);
    CHECK(off_min == 0 && off_max == 0);   /* no offset sample yet */

    /* Second beacon LATE: prev=151000, predicted=351000, actual=351500 -> +500 us.
     * on = 351500-200000 = 151500 us -> 151 ms. */
    rx_stats_core_beacon(&c, 200000u, 351500u);
    rx_stats_core_get(&c, &on_mean, &on_max, &off_min, &off_max, &n, &miss);
    CHECK(off_min == 500 && off_max == 500);
    CHECK(on_max == 151);
    CHECK(n == 2);

    /* Third beacon EARLY: prev=351500, predicted=551500, actual=551200 -> -300 us. */
    rx_stats_core_beacon(&c, 400000u, 551200u);
    rx_stats_core_get(&c, &on_mean, &on_max, &off_min, &off_max, &n, &miss);
    CHECK(off_min == -300 && off_max == 500);
    CHECK(n == 3);

    /* Miss: invalidates phase + counts. The next beacon must NOT record an
     * offset across the gap. */
    rx_stats_core_miss(&c);
    rx_stats_core_beacon(&c, 800000u, 951000u);   /* fresh reference, no offset */
    rx_stats_core_get(&c, &on_mean, &on_max, &off_min, &off_max, &n, &miss);
    CHECK(miss == 1);
    CHECK(n == 4);
    CHECK(off_min == -300 && off_max == 500);     /* unchanged: no new offset */

    /* A following beacon DOES record an offset from the post-miss reference.
     * prev=951000, predicted=1151000, actual=1151100 -> +100 us (within spread). */
    rx_stats_core_beacon(&c, 1000000u, 1151100u);
    rx_stats_core_get(&c, &on_mean, &on_max, &off_min, &off_max, &n, &miss);
    CHECK(off_min == -300 && off_max == 500);
    CHECK(n == 5);

    /* Reset clears everything. */
    rx_stats_core_reset(&c, SF_CYC, HZ);
    CHECK(rx_stats_core_get(&c, NULL, NULL, NULL, NULL, NULL, NULL) == 0);

    printf("rx_stats_core: %d failure(s)\n", fails);
    return fails == 0 ? 0 : 1;
}
```

- [ ] **Step 3: Run the test to verify it fails (no implementation yet)**

```bash
GCC="/c/Users/JoseAntonioLaraPerez/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe/mingw64/bin/gcc.exe"
"$GCC" -std=c99 -Wall -Wextra -Werror -I src tests/rx_stats/test_rx_stats.c src/rx_stats_core.c src/batt_window.c -o /tmp/rx.exe && /tmp/rx.exe
```
Expected: FAIL — link error, `undefined reference to 'rx_stats_core_reset'` (the `.c` does not exist yet).

- [ ] **Step 4: Write the implementation**

Create `src/rx_stats_core.c`:

```c
#include "rx_stats_core.h"

/* Convert a signed cycle delta to microseconds. The int64 intermediate avoids
 * overflow (a delta of up to ~1 superframe of cycles times 1e6). */
static int cyc_to_us(const struct rx_stats_core *c, int32_t d)
{
    return (int)(((int64_t)d * 1000000) / (int64_t)c->cyc_per_sec);
}

void rx_stats_core_reset(struct rx_stats_core *c,
                         uint32_t nominal_sf_cyc, uint32_t cyc_per_sec)
{
    batt_window_reset(&c->on_win);
    batt_window_reset(&c->off_win);
    c->prev_beacon_cyc = 0;
    c->prev_valid      = false;
    c->miss_count      = 0;
    c->nominal_sf_cyc  = nominal_sf_cyc;
    c->cyc_per_sec     = cyc_per_sec;
}

void rx_stats_core_beacon(struct rx_stats_core *c,
                          uint32_t arm_cyc, uint32_t now_cyc)
{
    /* on-duration: how long RX was on before the beacon arrived. int32 delta
     * is wrap-safe within one superframe. */
    batt_window_add(&c->on_win, cyc_to_us(c, (int32_t)(now_cyc - arm_cyc)));

    if (c->prev_valid) {
        int32_t off = (int32_t)(now_cyc - (c->prev_beacon_cyc + c->nominal_sf_cyc));
        batt_window_add(&c->off_win, cyc_to_us(c, off));
    }

    c->prev_beacon_cyc = now_cyc;
    c->prev_valid      = true;
}

void rx_stats_core_miss(struct rx_stats_core *c)
{
    c->prev_valid = false;   /* do not measure offset across the gap */
    c->miss_count++;
}

int rx_stats_core_get(const struct rx_stats_core *c,
                      int *on_mean_ms, int *on_max_ms,
                      int *off_min_us, int *off_max_us,
                      uint32_t *count, uint32_t *misses)
{
    int on_mean_us = 0, on_max_us = 0;
    uint32_t n = 0;

    if (!batt_window_get(&c->on_win, NULL, &on_mean_us, &on_max_us, &n)) {
        return 0;
    }
    if (on_mean_ms) { *on_mean_ms = on_mean_us / 1000; }
    if (on_max_ms)  { *on_max_ms  = on_max_us / 1000; }

    /* off_win is empty until a second beacon arrives; leave 0/0 in that case. */
    int omin = 0, omax = 0;
    batt_window_get(&c->off_win, &omin, NULL, &omax, NULL);
    if (off_min_us) { *off_min_us = omin; }
    if (off_max_us) { *off_max_us = omax; }

    if (count)  { *count  = n; }
    if (misses) { *misses = c->miss_count; }
    return 1;
}
```

- [ ] **Step 5: Run the test to verify it passes**

```bash
GCC="/c/Users/JoseAntonioLaraPerez/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe/mingw64/bin/gcc.exe"
"$GCC" -std=c99 -Wall -Wextra -Werror -I src tests/rx_stats/test_rx_stats.c src/rx_stats_core.c src/batt_window.c -o /tmp/rx.exe && /tmp/rx.exe
```
Expected: PASS — prints `rx_stats_core: 0 failure(s)` and exits 0.

- [ ] **Step 6: Commit**

```bash
git add src/rx_stats_core.h src/rx_stats_core.c tests/rx_stats/test_rx_stats.c
git commit -m "feat(power): pure rx_stats_core accumulator + host test"
```

---

### Task 2: Zephyr glue `rx_stats` + build wiring + startup reset

**Files:**
- Create: `src/rx_stats.h`
- Create: `src/rx_stats.c`
- Modify: `CMakeLists.txt` (add the two new sources to the app target)
- Modify: `src/main.c` (include + `rx_stats_reset()` after `batt_monitor_start();`)

**Interfaces:**
- Consumes: `rx_stats_core_*` from Task 1; Zephyr `k_cycle_get_32()`, `k_ms_to_cyc_near32()`, `sys_clock_hw_cycles_per_sec()`.
- Produces:
  - `void rx_stats_reset(void);`
  - `void rx_stats_arm(void);`
  - `void rx_stats_beacon(void);`
  - `void rx_stats_miss(void);`
  - `int rx_stats_get(int *on_mean_ms, int *on_max_ms, int *off_min_us, int *off_max_us, uint32_t *count, uint32_t *misses);`

- [ ] **Step 1: Write the glue header**

Create `src/rx_stats.h`:

```c
#ifndef RX_STATS_H_
#define RX_STATS_H_

#include <stdint.h>

/* Beacon RX duty-cycle instrumentation (singleton). The runner calls
 * arm/beacon/miss around the beacon-listen window; `pwr rx` reads it. */

/* Reset all accumulators and phase state. Call once at startup. */
void rx_stats_reset(void);

/* Mark the start of a beacon-listen window (RX about to be armed). */
void rx_stats_arm(void);

/* A real beacon was received: record on-duration and arrival offset. */
void rx_stats_beacon(void);

/* The beacon was missed this superframe. */
void rx_stats_miss(void);

/* Read stats: on_* in ms, off_* in us. Returns 1 if >=1 beacon recorded. */
int rx_stats_get(int *on_mean_ms, int *on_max_ms,
                 int *off_min_us, int *off_max_us,
                 uint32_t *count, uint32_t *misses);

#endif /* RX_STATS_H_ */
```

- [ ] **Step 2: Write the glue implementation**

Create `src/rx_stats.c`:

```c
#include <zephyr/kernel.h>
#include "rx_stats.h"
#include "rx_stats_core.h"

/* Must match T_SUPERFRAME_MS in src/uwb_net_runner.c (private there). */
#define RX_STATS_SUPERFRAME_MS 200u

static struct rx_stats_core core;
static uint32_t             arm_cyc;

void rx_stats_reset(void)
{
    rx_stats_core_reset(&core,
                        k_ms_to_cyc_near32(RX_STATS_SUPERFRAME_MS),
                        sys_clock_hw_cycles_per_sec());
}

void rx_stats_arm(void)
{
    arm_cyc = k_cycle_get_32();
}

void rx_stats_beacon(void)
{
    rx_stats_core_beacon(&core, arm_cyc, k_cycle_get_32());
}

void rx_stats_miss(void)
{
    rx_stats_core_miss(&core);
}

int rx_stats_get(int *on_mean_ms, int *on_max_ms,
                 int *off_min_us, int *off_max_us,
                 uint32_t *count, uint32_t *misses)
{
    return rx_stats_core_get(&core, on_mean_ms, on_max_ms,
                             off_min_us, off_max_us, count, misses);
}
```

- [ ] **Step 3: Add the sources to the build**

In `CMakeLists.txt`, find the existing `target_sources(app PRIVATE ...)` block that lists `src/batt.c` / `src/tag_cmd.c` and add these two lines inside it:

```cmake
  src/rx_stats_core.c
  src/rx_stats.c
```

- [ ] **Step 4: Wire startup reset in main.c**

In `src/main.c`, add the include alongside the other `src` includes near the top:

```c
#include "rx_stats.h"
```

Then add a reset call immediately after the existing `batt_monitor_start();` line (currently `src/main.c:57`):

```c
        batt_monitor_start();
        rx_stats_reset();
```

- [ ] **Step 5: Verify it compiles (user build)**

This task adds no host test (it is Zephyr glue + wiring). Verification: the user builds the firmware. Expected: clean compile and link; no behavior change yet (nothing calls `arm`/`beacon`/`miss`).

- [ ] **Step 6: Commit**

```bash
git add src/rx_stats.h src/rx_stats.c CMakeLists.txt src/main.c
git commit -m "feat(power): rx_stats Zephyr glue, build wiring, startup reset"
```

---

### Task 3: Beacon-listen hooks in the runner

**Files:**
- Modify: `src/uwb_net_runner.c` (include + three timestamp calls in the beacon-listen loop, `src/uwb_net_runner.c:459-505`)

**Interfaces:**
- Consumes: `rx_stats_arm()`, `rx_stats_beacon()`, `rx_stats_miss()` from Task 2.
- Produces: nothing new (internal instrumentation only).

- [ ] **Step 1: Add the include**

In `src/uwb_net_runner.c`, add alongside the other `src` includes near the top:

```c
#include "rx_stats.h"
```

- [ ] **Step 2: Hook the window arm**

In the beacon-listen section, immediately before the inner re-arm loop (the `for (;;)` at `src/uwb_net_runner.c:462`), add `rx_stats_arm();`. The result reads:

```c
        uint8_t beacon_buf[UWB_FRAME_MAX_LEN];
        int beacon_len = -ETIMEDOUT;
        uint32_t bcn_deadline = uwb_radio_now_ms() + T_SUPERFRAME_MS + T_BEACON_MS;
        rx_stats_arm();
        for (;;) {
            int32_t rem = (int32_t)(bcn_deadline - uwb_radio_now_ms());
            if (rem <= 0) {
                break;
            }
```

- [ ] **Step 3: Hook the real-beacon and miss branches**

In the event-build block (`src/uwb_net_runner.c:481-505`), add `rx_stats_beacon();` in the successful-parse branch and `rx_stats_miss();` in BOTH miss branches. The result reads:

```c
        if (beacon_len == UWB_FRAME_LEN_BEACON &&
            uwb_frame_is_beacon(beacon_buf, (size_t)beacon_len)) {

            uint8_t  proto_ver   = 0;
            uint32_t frame_ctr   = 0;
            uint16_t slot_map[UWB_FRAME_N_CFP];
            uint8_t  n_slots     = 0;

            if (uwb_frame_parse_beacon(beacon_buf, (size_t)beacon_len,
                                       &proto_ver, &frame_ctr,
                                       slot_map, &n_slots) == 0) {
                int slot_idx = uwb_frame_beacon_find_addr(slot_map, n_slots,
                                                          ctx.short_addr);

                ev.kind          = UWB_EV_BEACON;
                ev.proto_ver     = proto_ver;
                ev.frame_counter = frame_ctr;
                ev.in_map        = (slot_idx >= 0);
                ev.map_slot      = (slot_idx >= 0) ? (uint8_t)slot_idx : 0;
                rx_stats_beacon();
            } else {
                ev.kind = UWB_EV_BEACON_MISS;
                rx_stats_miss();
            }
        } else {
            ev.kind = UWB_EV_BEACON_MISS;
            rx_stats_miss();
        }
```

- [ ] **Step 4: Verify it compiles (user build)**

Verification: the user builds and flashes. Expected: clean compile; ranging unchanged (`P:` output still appears); the instrumentation now accumulates but is not yet readable (Task 4 adds the command).

- [ ] **Step 5: Commit**

```bash
git add src/uwb_net_runner.c
git commit -m "feat(power): feed rx_stats from the beacon-listen loop"
```

---

### Task 4: `pwr rx` NUS command

**Files:**
- Modify: `src/tag_cmd.c` (include + `pwr rx` branch alongside `pwr idle`, `src/tag_cmd.c:27-37`)

**Interfaces:**
- Consumes: `rx_stats_get(...)` from Task 2; existing `ble_log_send()`, `snprintf`.
- Produces: the `pwr rx` command. Report uses four short lines (refined from the spec's three-line sketch to keep every payload strictly ≤20 bytes even when an offset is several digits):
  - `RXon <mean>/<max>ms\n`
  - `RXoffmin <min>us\n`
  - `RXoffmax <max>us\n`
  - `RXn <count> m<misses>\n`

- [ ] **Step 1: Add the include**

In `src/tag_cmd.c`, add alongside the existing includes (after `#include "batt.h"`):

```c
#include "rx_stats.h"
```

- [ ] **Step 2: Add the `pwr rx` branch**

In `tag_cmd_on_rx`, insert a new branch between the `pwr idle` branch and the final `else` (after `src/tag_cmd.c:37`):

```c
		} else if (strcmp(buf, "pwr rx") == 0) {
			int on_mean = 0, on_max = 0, off_min = 0, off_max = 0;
			uint32_t cnt = 0, miss = 0;
			if (rx_stats_get(&on_mean, &on_max, &off_min, &off_max,
					 &cnt, &miss)) {
				char msg[24];
				snprintf(msg, sizeof(msg), "RXon %d/%dms\n",
					 on_mean, on_max);
				ble_log_send(msg);
				snprintf(msg, sizeof(msg), "RXoffmin %dus\n", off_min);
				ble_log_send(msg);
				snprintf(msg, sizeof(msg), "RXoffmax %dus\n", off_max);
				ble_log_send(msg);
				snprintf(msg, sizeof(msg), "RXn %u m%u\n", cnt, miss);
				ble_log_send(msg);
			} else {
				ble_log_send("RX none\n");
			}
```

- [ ] **Step 3: Verify it compiles and runs (user build + on-device)**

Verification: the user builds, flashes, lets the tag range, connects over BLE, subscribes, and sends `pwr rx`. Expected: four lines, e.g.
```
RXon 150/198ms
RXoffmin -280us
RXoffmax 2100us
RXn 64 m2
```
`RXon` should confirm the receiver is on for most of the superframe today; `RXoffmin`/`RXoffmax` give the arrival-offset spread that sizes the Spec 2 guard band.

- [ ] **Step 4: Commit**

```bash
git add src/tag_cmd.c
git commit -m "feat(power): add 'pwr rx' command to report RX duty-cycle stats"
```

---

## Self-Review

**1. Spec coverage:**
- `rx_stats_core` (pure, reuses `batt_window`, signed offset) → Task 1. ✓
- Cycle clock, `cyc_to_us`, nominal-superframe constant → Task 1 (core math) + Task 2 (Zephyr values). ✓
- `rx_stats_beacon` logic (on-duration + offset + phase advance), miss invalidation, first-beacon handling → Task 1, tested. ✓
- Runner hooks (arm before loop, beacon on parse, miss in both branches) → Task 3. ✓
- `pwr rx` command, ≤20-byte lines, `RX none` empty case → Task 4. ✓
- Startup reset not gated on BLE → Task 2 (main.c). ✓
- Build wiring → Task 2 (CMakeLists). ✓
- Host test with the six spec cases (offset ±, on-duration, miss-invalidation, first-beacon, reset) → Task 1 Step 2. ✓
- Non-goals (no narrow window, no DW3000 timestamps, no SS-TWR RX accounting, no GPIO) → respected; nothing in the plan touches them. ✓

**2. Placeholder scan:** No TBD/TODO/"handle edge cases"/"similar to". Every code step shows complete code; every command shows expected output. ✓

**3. Type consistency:** `rx_stats_get` / `rx_stats_core_get` signatures identical across Tasks 1, 2, 4 (`int*, int*, int*, int*, uint32_t*, uint32_t*`). `rx_stats_core_beacon(c, arm_cyc, now_cyc)` matches the glue call and the test. Field names (`on_win`, `off_win`, `prev_beacon_cyc`, `prev_valid`, `miss_count`, `nominal_sf_cyc`, `cyc_per_sec`) consistent between header and `.c`. ✓

**Note on spec divergence:** the report is four lines, not the spec's three-line sketch, to guarantee the 20-byte limit for multi-digit offsets. Same data, strictly safer framing.
