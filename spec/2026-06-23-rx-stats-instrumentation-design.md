# RX Duty-Cycle Instrumentation (Camino A — Spec 1) — Design

**Date:** 2026-06-23
**Status:** Approved (design)
**Scope:** Instrumentation only. The narrow beacon-window mechanism (Camino A — Spec 2)
is a separate spec, gated on the numbers this one produces.

## Problem

Measured field current is dominated by the DW3000 receiver, not by the LED or by the
intra-superframe SoC sleep. Layer-1 sleep saved ~10 mA (56 mA on vs 66 mA off, static),
hitting its ceiling because the receiver is on most of every superframe.

Root cause (confirmed in code):

- `uwb_radio_rx_beacon()` (`src/uwb_net_runner.c:201`) arms RX with **no hardware
  timeout** — `dwt_setrxtimeout(0)` + `dwt_setpreambledetecttimeout(0)`. The receiver
  stays physically on until a frame arrives or the kernel `wait_event()` timeout fires.
- The beacon-listen loop (`src/uwb_net_runner.c:459-475`) opens RX with a deadline of a
  **full superframe** (`T_SUPERFRAME_MS + T_BEACON_MS` ≈ 200 ms) and re-arms on every
  cross-traffic frame. So the receiver is on for nearly the whole superframe, every
  superframe, just to catch the beacon.

The eventual fix (Spec 2) is to open RX only in a **narrow window** around the predicted
beacon arrival, using the tag's superframe clock. To size that window we need two
measured numbers we do not currently have:

1. **Beacon arrival offset** vs. prediction — how far the actual beacon arrival deviates
   from `previous_beacon + nominal_superframe`. Its spread (min/max) is the guard band the
   narrow window must include (gateway TX jitter + crystal drift over one superframe).
2. **Beacon-listen RX-on duration** — how long the receiver is on before the beacon
   arrives. This quantifies today's waste and the savings opportunity.

## Goal

Add software-only instrumentation that measures (1) and (2) and reports them over BLE NUS,
without touching the ranging path. Build it, measure on hardware, and feed the numbers into
Spec 2.

## Non-Goals (YAGNI)

- No narrow-window / delayed-RX mechanism — that is Spec 2.
- No DW3000 hardware RX timestamps (device-time wrap handling not worth it at ms-scale
  window sizing).
- No total-RX duty accounting across the SS-TWR sweep path in `uwb_ss_initiator.c` — the
  beacon listen is the camino-A target and the only RX that runs every superframe.
- No GPIO/PPK2 hardware probe — the software accumulator is sufficient to size the window.
  (PPK2 stays available for final validation in Spec 2 if wanted.)

## Architecture

One new isolated module accumulates statistics; the runner feeds it timestamps at three
points in the beacon-listen loop; a NUS command reads the accumulated stats. The runner
hooks are timestamp calls only — no logic moves into the runner.

```
beacon-listen loop (uwb_net_runner.c)
    rx_stats_arm()      ── before the listen for(;;)
    rx_stats_beacon()   ── on real beacon
    rx_stats_miss()     ── on beacon miss
                │
                ▼
        rx_stats.c (singleton)
        ├─ on-duration accumulator  (struct batt_window)
        ├─ offset accumulator        (struct batt_window)
        └─ prev_beacon_cyc + prev_valid + miss_count
                │
                ▼
        tag_cmd.c  "pwr rx"  ──▶  3 short NUS notifications
```

## Components

### 1. `src/rx_stats.c` / `src/rx_stats.h` (new)

A singleton (file-static state, mirroring `src/batt.c`). Reuses the existing
`struct batt_window` min/mean/max accumulator (`src/batt_window.h`) — its fields are
`int` and `long`, so signed values (offset can be negative) work without change.

Internal state:

```c
static struct batt_window on_win;    /* RX-on duration, microseconds */
static struct batt_window off_win;   /* beacon arrival offset, microseconds (signed) */
static uint32_t arm_cyc;             /* k_cycle_get_32() at window arm */
static uint32_t prev_beacon_cyc;     /* k_cycle_get_32() at previous beacon arrival */
static bool     prev_valid;          /* false until first beacon / after a miss */
static uint32_t miss_count;
```

Public interface (`rx_stats.h`):

```c
#ifndef RX_STATS_H_
#define RX_STATS_H_

#include <stdint.h>

/* Reset all accumulators and phase state. Call once at startup. */
void rx_stats_reset(void);

/* Mark the start of a beacon-listen window (RX about to be armed).
 * Captures the arm cycle stamp used for the on-duration metric. */
void rx_stats_arm(void);

/* A real beacon was received. Records the on-duration (now - arm) and, if a
 * previous beacon is known, the arrival offset (now - (prev + nominal SF)).
 * Updates the phase reference to now. */
void rx_stats_beacon(void);

/* The beacon was missed this superframe. Invalidates the phase reference (so
 * the next offset is not measured across a gap) and increments miss_count. */
void rx_stats_miss(void);

/* Read accumulated stats. Returns 1 if at least one beacon was recorded
 * (out params written), else 0. Durations in ms, offsets in microseconds. */
int rx_stats_get(int *on_mean_ms, int *on_max_ms,
                 int *off_min_us, int *off_max_us,
                 uint32_t *count, uint32_t *misses);

#endif /* RX_STATS_H_ */
```

Timing helpers (implementation detail, in `rx_stats.c`):

- `nominal_sf_cyc` = `k_ms_to_cyc_near32(T_SUPERFRAME_MS)` (computed once; `T_SUPERFRAME_MS`
  comes from the existing UWB net timing header).
- `cyc_to_us(int32_t d)` = `(int)(((int64_t)d * 1000000) / sys_clock_hw_cycles_per_sec())`.
  Signed; the int64 intermediate avoids overflow.
- All deltas computed as `(int32_t)(a - b)` so the 32-bit cycle wrap (~67 s) is irrelevant
  within a 200 ms superframe.

`rx_stats_beacon()` logic:

```c
uint32_t now = k_cycle_get_32();
batt_window_add(&on_win, cyc_to_us((int32_t)(now - arm_cyc)));
if (prev_valid) {
    int32_t off = (int32_t)(now - (prev_beacon_cyc + nominal_sf_cyc));
    batt_window_add(&off_win, cyc_to_us(off));
}
prev_beacon_cyc = now;
prev_valid = true;
```

`rx_stats_get()` reads `on_win` (mean+max in ms — divide the µs accumulator by 1000) and
`off_win` (min+max in µs), plus `count` (= `on_win.count`) and `misses`.

### 2. Hooks in `src/uwb_net_runner.c`

Three timestamp calls in the beacon-listen loop (`src/uwb_net_runner.c:459-505`), no logic:

- `rx_stats_arm();` immediately before the `for (;;)` re-arm loop (line ~462).
- `rx_stats_beacon();` in the branch where a real beacon is parsed and
  `ev.kind = UWB_EV_BEACON` is set (line ~495).
- `rx_stats_miss();` in each branch that sets `ev.kind = UWB_EV_BEACON_MISS`
  (lines ~501 and ~504).

Add `#include "rx_stats.h"`.

### 3. `pwr rx` command in `src/tag_cmd.c`

Add a branch alongside `pwr idle`. NUS payloads must stay ≤20 bytes, so emit three short
notifications via `ble_log_send()` (which already retries on -ENOMEM):

```
RX on:<mean>/<max>ms      e.g. "RX on:150/198ms\n"
RX off:<min>/<max>us      e.g. "RX off:-280/2100us\n"
RX n<count> miss<m>       e.g. "RX n64 miss2\n"
```

If `rx_stats_get()` returns 0: send `RX none\n`.

### 4. Startup

Call `rx_stats_reset()` from `batt_monitor_start()` in `src/batt.c` (already invoked once
from `main.c` after BLE is up), or add a direct call in `main.c`. The accumulator runs
continuously — unlike the `idle` window it is **not** gated on BLE connection state, since
the offset and waste metrics are valid whether or not a central is connected.

### 5. Build

Add `src/rx_stats.c` to `CMakeLists.txt` via `target_sources(app PRIVATE ...)`.

## Data Flow

1. Each superframe the runner calls `rx_stats_arm()`, then listens for the beacon.
2. On a real beacon → `rx_stats_beacon()`: records on-duration and (after the first
   beacon) arrival offset; advances the phase reference.
3. On a miss → `rx_stats_miss()`: invalidates the phase reference and counts the miss.
4. On demand, `pwr rx` reads and reports the accumulated stats.

## Error / Edge Handling

- **First beacon:** no previous reference → offset skipped, on-duration recorded,
  `prev_valid` set.
- **Miss:** phase reference invalidated so the next offset is not measured across the gap;
  on-duration not recorded for a missed superframe (no arrival to anchor it).
- **Cycle wrap:** all deltas via `int32_t` subtraction; safe within a superframe.
- **No samples yet:** `rx_stats_get()` returns 0 → `pwr rx` sends `RX none`.
- **NUS 20-byte limit:** three terse lines, each well under 20 bytes.

## Testing

`rx_stats` is pure logic → host C test `tests/rx_stats/test_rx_stats.c`, built and run with
WinLibs gcc (full path; see project test setup). To make the logic host-testable, the
cycle source and `sys_clock_hw_cycles_per_sec()` / `k_ms_to_cyc_near32()` must be injectable
or stubbed — the test feeds a synthetic cycle stream and a fixed cycle frequency, so the
module's arithmetic is exercised without Zephyr. (Implementation choice for the seam is left
to the plan: a thin `rx_stats_now_cyc()` weak hook, or compiling against a small test shim
that defines the Zephyr symbols.)

Test cases:

1. **Offset positive:** beacon arrives later than predicted → positive offset of the
   expected magnitude; min/max track it across several samples.
2. **Offset negative:** beacon arrives earlier than predicted → negative offset recorded;
   `off_min_us` goes negative.
3. **On-duration:** arm→beacon gap recorded; mean and max correct over several superframes
   with differing gaps.
4. **Miss invalidates phase:** `arm → beacon → miss → arm → beacon`: the offset across the
   miss is NOT recorded (the post-miss beacon starts a fresh reference); `misses == 1`.
5. **First beacon:** no offset recorded on the very first beacon; on-duration is.
6. **Reset:** after samples, `rx_stats_reset()` → `rx_stats_get()` returns 0.

The runner hooks and the `pwr rx` command are validated on hardware by the user (integration,
not unit-tested).

## Success Criteria

- `pwr rx` returns three well-formed lines after the tag has been ranging.
- `RX on` confirms the receiver is on for most of the superframe today (expected ~150–200 ms),
  quantifying the waste.
- `RX off` gives a real min/max arrival-offset spread in µs — the input that sizes the
  Spec 2 guard band.
- Host tests pass.
- No regression in ranging (`P:` output unchanged) and no new BLE-msgq pressure (the report
  is on-demand, not per-superframe).

## Handoff to Spec 2

The guard band for the narrow beacon window will be:
`guard = max(|off_min_us|, |off_max_us|) + safety_margin`, with `safety_margin` chosen from
the observed spread stability. Spec 2's brainstorming starts once these numbers are in hand.
