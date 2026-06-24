# Narrow Beacon Window Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Cut DW3000 receiver duty-cycle by listening for the gateway beacon only in a narrow window around an EMA-predicted arrival time, dropping RX-on from ~182 ms to ~10 ms per superframe.

**Architecture:** A pure, host-testable core (`beacon_track_core`) maintains an EMA estimate of the gateway beacon period and a two-state machine (ACQUIRING = full-window listen, TRACKING = narrow window). The `uwb_net_runner` thread owns one instance, asks it each superframe when to arm RX and how wide a window to use, sleeps the radio until just before the predicted beacon, then arms a short-timeout listen. A miss inside the narrow window forces one full-window frame to re-acquire.

**Tech Stack:** C99, Zephyr RTOS (nCS 3.2.4), nRF52833, Decawave DW3000. Host unit tests compiled with WinLibs GCC.

## Global Constraints

- **Time base is milliseconds** via `uwb_radio_now_ms()` (`k_uptime_get_32()`). The spec described cycles (`k_cycle_get_32`); this plan deliberately uses ms because the entire runner already times in ms (`uwb_radio_sleep_until`, deadlines), and mixing k_cycle with k_uptime in the arm path would be fragile. 1 ms quantization is negligible vs the ±5 ms guard.
- **No separate Zephyr glue file** (spec mentioned `beacon_track.c`): the core takes timestamps as parameters and is therefore directly usable from the runner, which holds the singleton instance and resets it in its own init. This is DRY/YAGNI; no `main.c` change.
- **Do not touch anchor/gateway firmware.**
- **Do not change the existing `UWB_NET_MISS_MAX → RESCAN` lost-sync logic.** Verified `UWB_NET_MISS_MAX = 3` in `src/uwb_net.h:10` (≥ 2, so an isolated false miss cannot trip a rescan — the safety property the spec requires).
- **Host test build command** (WinLibs GCC, full path; gcc is NOT on PATH):
  ```
  GCC="/c/Users/JoseAntonioLaraPerez/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe/mingw64/bin/gcc.exe"
  "$GCC" -std=c99 -Wall -Wextra -Werror -I src tests/beacon_track/test_beacon_track.c src/beacon_track_core.c -o /tmp/bt.exe && /tmp/bt.exe
  ```
- The user builds/flashes the Zephyr firmware and reports results (per CLAUDE.md). Task 2's on-device checks are performed by the user.
- Tuning constants (all adjustable): `GUARD = 5 ms`, `WARMUP_N = 8`, `EMA_SHIFT = 3`, wake margin reuses existing `DW_WAKE_GUARD_MS = 5 ms`.

---

### Task 1: Pure beacon-tracker core + host tests

**Files:**
- Create: `src/beacon_track_core.h`
- Create: `src/beacon_track_core.c`
- Test: `tests/beacon_track/test_beacon_track.c`

**Interfaces:**
- Consumes: nothing (pure C, `<stdint.h>`/`<stdbool.h>` only).
- Produces (used by Task 2):
  - `struct beacon_track` — the tracker state.
  - `void beacon_track_reset(struct beacon_track *c, uint32_t period_seed_ms, uint32_t guard_ms, uint16_t warmup_n, uint8_t ema_shift);`
  - `void beacon_track_plan(const struct beacon_track *c, bool *narrow, uint32_t *arm_at_ms, uint32_t *window_ms);`
  - `void beacon_track_beacon(struct beacon_track *c, uint32_t now_ms);`
  - `void beacon_track_miss(struct beacon_track *c);`
  - `uint32_t beacon_track_period_ms(const struct beacon_track *c);`

- [ ] **Step 1: Write the failing test**

Create `tests/beacon_track/test_beacon_track.c`:

```c
#include "../../src/beacon_track_core.h"
#include <stdio.h>

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

int main(void)
{
    struct beacon_track c;
    bool narrow;
    uint32_t arm = 0, win = 0;

    /* --- Test A: lock transition + plan, period unchanged (gaps == seed) --- */
    /* seed 200ms, guard 5ms, warmup_n 3, ema_shift 3 */
    beacon_track_reset(&c, 200u, 5u, 3u, 3u);

    /* No beacon yet -> ACQUIRING (full window). */
    beacon_track_plan(&c, &narrow, &arm, &win);
    CHECK(narrow == false);

    beacon_track_beacon(&c, 1000u);                 /* warmup 1, no gap yet */
    beacon_track_plan(&c, &narrow, &arm, &win);
    CHECK(narrow == false);                         /* 1 < 3 */

    beacon_track_beacon(&c, 1200u);                 /* gap 200 -> err 0; warmup 2 */
    beacon_track_plan(&c, &narrow, &arm, &win);
    CHECK(narrow == false);                         /* 2 < 3 */

    beacon_track_beacon(&c, 1400u);                 /* gap 200; warmup 3 -> TRACKING */
    beacon_track_plan(&c, &narrow, &arm, &win);
    CHECK(narrow == true);
    CHECK(arm == 1400u + 200u - 5u);                /* last + period_est - guard = 1595 */
    CHECK(win == 10u);                              /* 2 * guard */
    CHECK(beacon_track_period_ms(&c) == 200u);      /* unchanged: all gaps == seed */

    /* --- Test B: EMA convergence on a positive error --- */
    beacon_track_reset(&c, 200u, 5u, 3u, 3u);
    beacon_track_beacon(&c, 0u);                    /* first, no gap */
    CHECK(beacon_track_period_ms(&c) == 200u);
    beacon_track_beacon(&c, 208u);                  /* gap 208, err +8, +8>>3 = +1 */
    CHECK(beacon_track_period_ms(&c) == 201u);
    beacon_track_beacon(&c, 416u);                  /* gap 208, err +7, +7>>3 = 0 */
    CHECK(beacon_track_period_ms(&c) == 201u);

    /* --- Test C: miss -> ACQUIRING, warmup reset, period_est retained, re-lock --- */
    beacon_track_reset(&c, 200u, 5u, 3u, 3u);
    beacon_track_beacon(&c, 1000u);
    beacon_track_beacon(&c, 1200u);
    beacon_track_beacon(&c, 1400u);                 /* TRACKING, period 200 */
    beacon_track_plan(&c, &narrow, &arm, &win);
    CHECK(narrow == true);

    beacon_track_miss(&c);                          /* drop to ACQUIRING */
    beacon_track_plan(&c, &narrow, &arm, &win);
    CHECK(narrow == false);
    CHECK(beacon_track_period_ms(&c) == 200u);      /* period retained */

    /* Next beacon re-establishes the reference WITHOUT an EMA update (have_last
     * was cleared by the miss, so the post-gap sample cannot poison the EMA). */
    beacon_track_beacon(&c, 1700u);                 /* warmup 1; no gap applied */
    CHECK(beacon_track_period_ms(&c) == 200u);
    beacon_track_beacon(&c, 1900u);                 /* warmup 2 */
    beacon_track_beacon(&c, 2100u);                 /* warmup 3 -> TRACKING */
    beacon_track_plan(&c, &narrow, &arm, &win);
    CHECK(narrow == true);
    CHECK(arm == 2100u + 200u - 5u);                /* 2295 */

    printf("beacon_track_core: %d failure(s)\n", fails);
    return fails == 0 ? 0 : 1;
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run:
```
GCC="/c/Users/JoseAntonioLaraPerez/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe/mingw64/bin/gcc.exe"
"$GCC" -std=c99 -Wall -Wextra -Werror -I src tests/beacon_track/test_beacon_track.c src/beacon_track_core.c -o /tmp/bt.exe && /tmp/bt.exe
```
Expected: FAIL — compile error, `beacon_track_core.h`/`.c` do not exist yet (`fatal error: beacon_track_core.h: No such file or directory`).

- [ ] **Step 3: Write the header**

Create `src/beacon_track_core.h`:

```c
#ifndef BEACON_TRACK_CORE_H_
#define BEACON_TRACK_CORE_H_

#include <stdint.h>
#include <stdbool.h>

/* Pure (Zephyr-free) beacon-arrival predictor for the narrow-window RX scheme.
 * The caller supplies beacon arrival times in milliseconds (any monotonic ms
 * clock); the core maintains an EMA of the gateway beacon period and a two-state
 * machine: ACQUIRING (listen full superframe) until warmup_n clean beacons in a
 * row, then TRACKING (listen only in a narrow window around the prediction).
 * A miss drops back to ACQUIRING but retains the period estimate. */
struct beacon_track {
    uint32_t period_est_ms;   /* EMA of the beacon period, in ms */
    uint32_t last_beacon_ms;  /* arrival time of the last caught beacon */
    bool     have_last;       /* false until first beacon / after a miss */
    uint16_t warmup_count;    /* consecutive clean beacons */
    uint32_t guard_ms;        /* half-width of the narrow window, in ms */
    uint16_t warmup_n;        /* clean beacons required before TRACKING */
    uint8_t  ema_shift;       /* EMA smoothing: period += err >> ema_shift */
    bool     tracking;        /* false = ACQUIRING, true = TRACKING */
};

/* period_seed_ms = nominal beacon period (e.g. 200); guard_ms, warmup_n,
 * ema_shift = tuning constants. */
void beacon_track_reset(struct beacon_track *c, uint32_t period_seed_ms,
                        uint32_t guard_ms, uint16_t warmup_n, uint8_t ema_shift);

/* Plan the next listen window.
 *   *narrow     = true only in TRACKING with a known reference.
 *   *arm_at_ms  = absolute ms to arm RX (narrow only) = last + period_est - guard.
 *   *window_ms  = RX-on span to allow (narrow only) = 2 * guard.
 * When *narrow is false the caller uses the full-superframe listen. Any out
 * pointer may be NULL. Pure read; does not mutate state. */
void beacon_track_plan(const struct beacon_track *c,
                       bool *narrow, uint32_t *arm_at_ms, uint32_t *window_ms);

/* Record a caught beacon at now_ms: update the EMA (skipped on the first beacon
 * or the first beacon after a miss, to avoid poisoning it across a gap), advance
 * the reference, advance warmup, and enter TRACKING once warmup_n is reached. */
void beacon_track_beacon(struct beacon_track *c, uint32_t now_ms);

/* Record a missed beacon: drop to ACQUIRING, reset warmup, clear the reference,
 * keep period_est. */
void beacon_track_miss(struct beacon_track *c);

/* Current period estimate (ms) — diagnostics / tests. */
uint32_t beacon_track_period_ms(const struct beacon_track *c);

#endif /* BEACON_TRACK_CORE_H_ */
```

- [ ] **Step 4: Write the implementation**

Create `src/beacon_track_core.c`:

```c
#include "beacon_track_core.h"

void beacon_track_reset(struct beacon_track *c, uint32_t period_seed_ms,
                        uint32_t guard_ms, uint16_t warmup_n, uint8_t ema_shift)
{
    c->period_est_ms  = period_seed_ms;
    c->last_beacon_ms = 0;
    c->have_last      = false;
    c->warmup_count   = 0;
    c->guard_ms       = guard_ms;
    c->warmup_n       = warmup_n;
    c->ema_shift      = ema_shift;
    c->tracking       = false;
}

void beacon_track_plan(const struct beacon_track *c,
                       bool *narrow, uint32_t *arm_at_ms, uint32_t *window_ms)
{
    if (c->tracking && c->have_last) {
        if (narrow)    { *narrow    = true; }
        if (arm_at_ms) { *arm_at_ms = c->last_beacon_ms + c->period_est_ms - c->guard_ms; }
        if (window_ms) { *window_ms = 2u * c->guard_ms; }
    } else {
        if (narrow)    { *narrow    = false; }
        if (arm_at_ms) { *arm_at_ms = 0; }
        if (window_ms) { *window_ms = 0; }
    }
}

void beacon_track_beacon(struct beacon_track *c, uint32_t now_ms)
{
    if (c->have_last) {
        /* int32 deltas are wrap-safe for ~ms intervals. */
        int32_t gap = (int32_t)(now_ms - c->last_beacon_ms);
        int32_t err = gap - (int32_t)c->period_est_ms;
        c->period_est_ms = (uint32_t)((int32_t)c->period_est_ms + (err >> c->ema_shift));
    }
    c->last_beacon_ms = now_ms;
    c->have_last      = true;

    if (c->warmup_count < c->warmup_n) {
        c->warmup_count++;
    }
    if (c->warmup_count >= c->warmup_n) {
        c->tracking = true;
    }
}

void beacon_track_miss(struct beacon_track *c)
{
    c->tracking     = false;
    c->warmup_count = 0;
    c->have_last    = false;   /* keep period_est; just drop the phase reference */
}

uint32_t beacon_track_period_ms(const struct beacon_track *c)
{
    return c->period_est_ms;
}
```

- [ ] **Step 5: Run the test to verify it passes**

Run the same command as Step 2.
Expected: `beacon_track_core: 0 failure(s)` and exit code 0.

- [ ] **Step 6: Commit**

```bash
git add src/beacon_track_core.h src/beacon_track_core.c tests/beacon_track/test_beacon_track.c
git commit -m "feat(power): beacon-track core — EMA period predictor + lock FSM

Pure host-testable core for the narrow beacon window: EMA of the gateway
beacon period (ms) and an ACQUIRING/TRACKING state machine. plan() returns
when to arm RX and how wide a window; a miss drops to ACQUIRING and clears
the phase reference but retains the period estimate.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 2: Wire the tracker into the runner (CMake + narrow-window listen)

**Files:**
- Modify: `CMakeLists.txt:25` (add `src/beacon_track_core.c` to `target_sources`)
- Modify: `src/uwb_net_runner.c` (constants ~line 33; include ~line 24; runner init ~line 432; step-2 listen ~lines 459–510; end-of-loop sleep ~lines 622–634)

**Interfaces:**
- Consumes (from Task 1): `struct beacon_track`, `beacon_track_reset`, `beacon_track_plan`, `beacon_track_beacon`, `beacon_track_miss`.
- Produces: behavioral change only (no new symbols). Verified on hardware via the existing `pwr rx` report.

- [ ] **Step 1: Add the core to the firmware build**

In `CMakeLists.txt`, after line 26 (`src/rx_stats.c`), add the tracker core:

```cmake
    src/rx_stats_core.c
    src/rx_stats.c
    src/beacon_track_core.c
    src/wdt.c
```

(Insert `src/beacon_track_core.c` as the new line between `src/rx_stats.c` and `src/wdt.c`.)

- [ ] **Step 2: Add the include and tuning constants in the runner**

In `src/uwb_net_runner.c`, add the include alongside the other local includes near the top of the file (next to `#include "rx_stats.h"`):

```c
#include "beacon_track_core.h"
```

Then, just after the existing `#define DW_WAKE_GUARD_MS 5u` block (line 33), add:

```c
/* Narrow beacon window (Spec 2) — all adjustable. GUARD ≈ 2× the measured
 * ~±2.6 ms arrival jitter; the wake margin reuses DW_WAKE_GUARD_MS. */
#define BT_GUARD_MS    5u
#define BT_WARMUP_N    8u
#define BT_EMA_SHIFT   3u
```

- [ ] **Step 3: Declare and reset the tracker in the runner thread**

In `src/uwb_net_runner.c`, in `runner_fn()` just after `uwb_net_init(&ctx, runner_eui);` (line 432), add the tracker instance, its reset, and the radio-sleep flag:

```c
    struct beacon_track bt;
    beacon_track_reset(&bt, T_SUPERFRAME_MS, BT_GUARD_MS, BT_WARMUP_N, BT_EMA_SHIFT);
    bool radio_asleep = false;   /* tracks whether the DW3000 is in deep sleep */
```

- [ ] **Step 4: Replace the step-2 listen with the planned narrow window**

In `src/uwb_net_runner.c`, replace the current step-2 block (the comment "2. Listen for THE beacon…" through the `for (;;)` re-arm loop, lines ~453–477) with the version below. The re-arm loop body is unchanged; only the deadline source and the preceding sleep/wake are new:

```c
        /* 2. Listen for THE beacon, discarding cross-traffic.  In TRACKING the
         * beacon tracker predicts arrival and we sleep the radio until just
         * before it, then arm a short window; in ACQUIRING we listen the whole
         * superframe (as before) to (re)lock.  uwb_radio_rx_beacon() returns on
         * the FIRST frame of any type, so the re-arm loop keeps waiting through
         * cross-traffic until the real beacon or the deadline. */
        uint8_t beacon_buf[UWB_FRAME_MAX_LEN];
        int beacon_len = -ETIMEDOUT;

        bool     bt_narrow;
        uint32_t bt_arm_ms, bt_window_ms;
        beacon_track_plan(&bt, &bt_narrow, &bt_arm_ms, &bt_window_ms);

        uint32_t bcn_deadline;
        if (bt_narrow) {
            /* Sleep (radio stays in deep sleep) until just before the predicted
             * beacon, leaving DW_WAKE_GUARD_MS for the wake to settle. */
            if ((int32_t)(bt_arm_ms - DW_WAKE_GUARD_MS - uwb_radio_now_ms()) > 0) {
                uwb_radio_sleep_until(bt_arm_ms - DW_WAKE_GUARD_MS);
            }
            if (radio_asleep) { dw_wake(); radio_asleep = false; }
            bcn_deadline = bt_arm_ms + bt_window_ms;
        } else {
            if (radio_asleep) { dw_wake(); radio_asleep = false; }
            bcn_deadline = uwb_radio_now_ms() + T_SUPERFRAME_MS + T_BEACON_MS;
        }

        rx_stats_arm();
        for (;;) {
            int32_t rem = (int32_t)(bcn_deadline - uwb_radio_now_ms());
            if (rem <= 0) {
                break;
            }
            int len = uwb_radio_rx_beacon(beacon_buf, sizeof(beacon_buf),
                                          (uint32_t)rem);
            if (len == UWB_FRAME_LEN_BEACON &&
                uwb_frame_is_beacon(beacon_buf, (size_t)len)) {
                beacon_len = len;
                break;
            }
            /* non-beacon frame or rx timeout/error: keep waiting for the beacon */
        }
        uint32_t t0_ms = uwb_radio_now_ms();
```

- [ ] **Step 5: Feed the tracker from the event-build branches**

In `src/uwb_net_runner.c`, in the step-3 event build, add `beacon_track_*` calls next to the existing `rx_stats_*` calls. In the beacon parse-success branch (where `rx_stats_beacon();` is), append:

```c
                rx_stats_beacon();
                beacon_track_beacon(&bt, t0_ms);
```

In **both** `UWB_EV_BEACON_MISS` branches (the parse-failure branch and the no-beacon `else` branch, where `rx_stats_miss();` is), append after each:

```c
                rx_stats_miss();
                beacon_track_miss(&bt);
```

- [ ] **Step 6: Hand wake timing to step 2 in the end-of-loop sleep**

In `src/uwb_net_runner.c`, replace the `if (act & UWB_ACT_SLEEP)` block (lines ~622–634) with:

```c
        if (act & UWB_ACT_SLEEP) {
            if (dw_sleep_enabled) {
                /* Deep-sleep the radio; the wake/arm timing is owned by step 2's
                 * beacon-window planner (which sleeps the MCU until just before
                 * the predicted beacon, or wakes immediately for a full listen in
                 * ACQUIRING).  No timed wake here. */
                dw_enter_sleep();
                radio_asleep = true;
            } else {
                uwb_radio_sleep_until(t0_ms + T_SUPERFRAME_MS);
            }
        }
```

- [ ] **Step 7: Build, flash, and verify on hardware (user)**

Ask the user to build, flash, connect BLE, let the tag lock (ranging, no `RESCAN`), then run:
1. `pwr rxrst` → expect `RX reset`.
2. Leave the tag still ~1 min.
3. `pwr rx`.

Expected:
- `RXon` mean drops from ~182 ms to **~10–15 ms** (ACQUIRING frames lift the mean slightly).
- `misses` stays ≤ ~3 % baseline. If markedly higher, the window is too tight → raise `BT_GUARD_MS` and re-flash.
- `P:x,y` position output continues; BLE stays connected; no `RESCAN miss` storm.

- [ ] **Step 8: Commit**

```bash
git add CMakeLists.txt src/uwb_net_runner.c
git commit -m "feat(power): narrow beacon RX window in the runner

Drive the beacon listen from beacon_track_core: in TRACKING, sleep the
DW3000 until just before the EMA-predicted beacon and arm a short ±5 ms
window instead of an open-ended ~182 ms listen; in ACQUIRING, full-window
listen to (re)lock. A miss forces one full-window frame. The end-of-loop
sleep now just deep-sleeps the radio and hands wake timing to step 2.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Self-Review

**1. Spec coverage:**
- Software-timed window (sleep-compatible) → Task 2 Step 4 (ms-based sleep_until + dw_wake). ✓
- Reject hardware delayed-RX → not implemented; documented in spec. ✓
- EMA period prediction → Task 1 `beacon_track_beacon`. ✓
- ACQUIRING/TRACKING state machine, full-window fallback → Task 1 (FSM) + Task 2 Steps 4–6. ✓
- Pure core + runner glue → Task 1 core; Task 2 holds instance in runner (deliberate simplification of spec's glue file, noted in Global Constraints). ✓
- Cold start seed, wake margin, loop-late, false-miss-bounded-to-1, gateway-lost, wrap-safe → seed (Task 2 Step 3), wake margin (Step 4, DW_WAKE_GUARD_MS), loop-late (Step 4 `if > 0` guard + `rem <= 0` break), false-miss bound (Step 6 forces full window after miss + `UWB_NET_MISS_MAX = 3`), gateway-lost (unchanged FSM), wrap-safe (int32 deltas in core + runner). ✓
- `MISS_MAX ≥ 2` constraint → verified = 3, recorded in Global Constraints. ✓
- Host tests (EMA, lock, plan, miss/recovery, edges) → Task 1 Tests A/B/C. ✓
- On-device validation via `pwr rx` → Task 2 Step 7. ✓
- Tuning constants table → Global Constraints + Task 2 Step 2. ✓

**2. Placeholder scan:** No TBD/TODO/"handle edge cases"/"similar to". All code blocks are complete. ✓

**3. Type consistency:** `struct beacon_track`, `beacon_track_reset(c, period_seed_ms, guard_ms, warmup_n, ema_shift)`, `beacon_track_plan(c, *narrow, *arm_at_ms, *window_ms)`, `beacon_track_beacon(c, now_ms)`, `beacon_track_miss(c)`, `beacon_track_period_ms(c)` — identical across Task 1 header, Task 1 test, and Task 2 call sites. Constants `BT_GUARD_MS`/`BT_WARMUP_N`/`BT_EMA_SHIFT` defined once (Task 2 Step 2), used once (Step 3). `radio_asleep` declared (Step 3), set (Steps 4, 6). ✓
