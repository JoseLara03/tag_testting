# Narrow Beacon Window — Design (Spec 2)

**Date:** 2026-06-24
**Branch:** feat/power-saving
**Status:** Approved design, ready for implementation plan
**Predecessor:** `spec/2026-06-23-rx-stats-instrumentation-design.md` (Spec 1, instrumentation — landed; produced the measured numbers this design relies on)

## Problem

The tag draws ~56–66 mA. Instrumentation (`pwr rx`, Spec 1) confirmed the dominant
drain quantitatively: the DW3000 receiver is on **~91 % of every superframe**
(`RXon 182/190 ms` out of 200 ms). The cause is the open-ended beacon-listen: the
runner arms RX with no hardware timeout and waits for the beacon, which arrives
~180 ms into the window — so the receiver is physically on nearly the whole
superframe, every superframe.

The gateway beacon is periodic. If the tag predicts *when* the beacon will arrive
and only turns the receiver on for a short window around that instant, RX-on
drops from ~182 ms to ~10 ms per superframe.

## Measured inputs (from Spec 1, steady-state, tag locked + `pwr rxrst`)

```
RXon 182/190ms        receiver on ~91% of the superframe  → the drain
RXoffmin -10192us     folded beacon-arrival offset, min
RXoffmax -4913us      folded beacon-arrival offset, max
RXn 258 m8            258 beacons, 8 misses (~3%) over ~1 min
```

Interpretation:
- The folded offset is consistently **negative**: the gateway beacon period is
  **~195 ms in the tag's clock**, not 200 ms. A naive "predict last + 200 ms"
  accumulates ~5 ms of error per superframe — this is why the prediction must
  track the *measured* period, not assume 200 ms.
- The frame-to-frame spread is **5.3 ms** (gap range 189.8–195.1 ms), i.e. the
  beacon arrival jitters by about **±2.6 ms** as seen through the software timing
  path (DW3000 ISR → kernel scheduling → parse → `k_cycle_get_32()` capture).
  This is the noise floor a software-timed window must clear.

## Goal

Reduce DW3000 receiver duty-cycle by listening for the gateway beacon only in a
narrow window centred on the predicted arrival, **without touching the
anchor/gateway firmware** and **without losing the BLE debug channel**.

Target: `RXon` mean ≤ ~15 ms after lock (from ~182 ms), miss rate no worse than
the ~3 % baseline, no BLE regression, measurable current reduction.

## Non-goals

- No changes to anchor/gateway firmware.
- No hardware delayed-RX (`dwt_setdelayedtrxtime`) — see "Rejected: hardware-timed
  window" below; it conflicts with the existing DW3000 sleep.
- No change to the existing lost-sync / `MISS_MAX → RESCAN` behaviour in
  `uwb_net.c`. This design sits *on top of* a successful lock.
- No new BLE command — on-device validation reuses the Spec 1 `pwr rx` report.

## Architecture

### Timing basis: software-timed (sleep-compatible)

The window is scheduled against the **MCU cycle counter** (`k_cycle_get_32()`),
the same clock Spec 1's `rx_stats` already uses. The MCU clock runs through DW3000
sleep, so this composes cleanly with the Layer-1 sleep
(`dwt_entersleep(DWT_DW_IDLE_RC)`).

**Rejected: hardware-timed window.** A DW3000 delayed-RX referenced to the
previous beacon's hardware RX timestamp would give sub-µs placement (a ±1 ms
window). But DW3000 SLEEP mode stops the device's ~64 GHz system-time counter and
resets it on wake, so the hardware timestamp reference is lost across each
inter-beacon sleep. Keeping the radio awake to preserve it would burn the ~180 ms
of idle current that Layer-1 sleep currently saves — trading ~180 ms of deep
sleep for ~4 ms less RX, a net loss. Software timing is the correct choice given
the sleep.

Consequence: the achievable window floor is the software jitter (~±2.6 ms), so the
guard band is **±5 ms** (≈2× the observed jitter), exposed as a tunable constant.

### Core mechanism (runner step 2)

Today:
```
arm RX now → wait up to 202 ms → beacon arrives ~182 ms later (radio ON throughout)
```

New, in TRACKING state:
```
predict next_beacon = last_beacon_cyc + period_est        (MCU k_cycle clock)
sleep until (next_beacon − GUARD − WAKE_MARGIN)           (radio stays in Layer-1 sleep)
dw_wake()                                                  (~2–3 ms settle)
arm RX at (next_beacon − GUARD), HW rxtimeout = 2·GUARD    (radio ON ~10 ms only)
```

The radio is simply *not turned on* during the ~170 ms gap; it stays in the sleep
state Layer 1 already left it in. The existing re-arm loop (the multi-tag fix that
discards cross-traffic frames) is preserved unchanged except its deadline becomes
the short window end `next_beacon + GUARD`.

`period_est` is maintained by an exponential moving average:
```
gap = now_cyc − last_beacon_cyc
period_est += (gap − period_est) >> EMA_SHIFT        // EMA_SHIFT = 3, ≈8-frame average
```

### State machine

```
ACQUIRING  ── full window every frame; feed EMA on each caught beacon
   │             when warmup_count reaches WARMUP_N clean beacons in a row →
   ▼
TRACKING   ── narrow window (±GUARD); on each catch update EMA and stay
   │             on ANY miss inside the window →
   └─────────►  back to ACQUIRING (warmup_count = 0)
```

- Start in **ACQUIRING** — never narrow until `period_est` is stable (WARMUP_N
  clean frames).
- `period_est` is **never reset** between states — after a miss the EMA stays
  warm, so re-lock costs only WARMUP_N full-window frames (~1.6 s), not a cold
  restart.
- A single threshold `WARMUP_N` governs both initial lock and re-lock — uniform.

### Module split (pure core + glue, per cal_math/cal and rx_stats convention)

All prediction, EMA, and state-transition logic lives in a **pure, Zephyr-free,
host-testable core** `src/beacon_track_core.{c,h}`. The runner is glue: it supplies
timestamps and asks the core what to do next.

Core interface (cycle values supplied by the caller, like `rx_stats_core`):
```c
struct beacon_track_core {
    uint32_t period_est_cyc;   /* EMA of the gateway beacon period, in cycles */
    uint32_t last_beacon_cyc;  /* cycle stamp of the last caught beacon */
    bool     have_last;        /* false until first beacon */
    uint16_t warmup_count;     /* consecutive clean beacons */
    uint32_t guard_cyc;        /* ±guard in cycles (tunable) */
    uint16_t warmup_n;         /* WARMUP_N */
    uint8_t  ema_shift;        /* EMA_SHIFT */
    bool     tracking;         /* false = ACQUIRING, true = TRACKING */
};

/* period_seed_cyc = nominal superframe in cycles (200 ms); guard_cyc, warmup_n,
 * ema_shift = tuning constants. */
void beacon_track_core_reset(struct beacon_track_core *c,
                             uint32_t period_seed_cyc, uint32_t guard_cyc,
                             uint16_t warmup_n, uint8_t ema_shift);

/* Plan the next listen window. now_cyc = current cycle stamp.
 *   *narrow      = true if a narrow window applies (TRACKING + have_last)
 *   *arm_at_cyc  = cycle stamp to arm RX (narrow only; = next_beacon − guard)
 *   *window_cyc  = HW rxtimeout span to use (narrow only; = 2·guard)
 * When *narrow is false the caller uses the full-window listen (as today). */
void beacon_track_core_plan(const struct beacon_track_core *c, uint32_t now_cyc,
                            bool *narrow, uint32_t *arm_at_cyc, uint32_t *window_cyc);

/* A beacon was caught at now_cyc: update EMA, advance reference, advance warmup,
 * and transition to TRACKING once warmup_count reaches warmup_n. */
void beacon_track_core_beacon(struct beacon_track_core *c, uint32_t now_cyc);

/* The beacon was missed: drop to ACQUIRING, reset warmup_count, keep period_est. */
void beacon_track_core_miss(struct beacon_track_core *c);
```

The Zephyr glue (`src/beacon_track.{c,h}`, singleton) captures `k_cycle_get_32()`,
holds `WAKE_MARGIN`, and is called from the runner's step 2. `rx_stats` continues
to run alongside, unchanged, providing the `pwr rx` measurement.

## Data flow (runner step 2, per superframe)

1. `beacon_track_plan(now, &narrow, &arm_at, &window)`.
2. If `narrow`: sleep until `arm_at − WAKE_MARGIN`; `dw_wake()`; arm RX at `arm_at`
   with HW rxtimeout `window`; run the re-arm loop with deadline `arm_at + window`.
   Else: full-window listen exactly as today (arm now, deadline `now + 202 ms`).
3. On a real beacon parse-success: `beacon_track_beacon(now)`; build `UWB_EV_BEACON`.
4. On no beacon by the deadline: `beacon_track_miss()`; build `UWB_EV_BEACON_MISS`.
5. Hand the event to `uwb_net_handle()` exactly as today (FSM unchanged).

## Error handling / edge cases

1. **Cold start** — no `period_est`; seeded with the 200 ms nominal. EMA converges
   to ~195 ms over the WARMUP_N ACQUIRING frames before any narrowing.
2. **Radio wake before arming** — the radio is asleep during the gap; `dw_wake()`
   needs ~2–3 ms to settle. The runner sleeps until `arm_at − WAKE_MARGIN`, wakes
   the radio, then arms RX at `arm_at`. The core returns `arm_at_cyc`; the runner
   subtracts its own `WAKE_MARGIN`.
3. **Loop late** — if `now ≥ arm_at`, arm immediately with the remaining window; if
   `now ≥ arm_at + window` (whole window already past), that frame falls back to a
   full-window listen and is treated as a miss.
4. **False miss from over-tight window** — a real beacon could land just outside
   ±GUARD. Safety rule: **a miss in the narrow window forces the next frame to a
   full window** (drop to ACQUIRING). This guarantees **at most one false miss in a
   row**. Because the existing `MISS_MAX → RESCAN` fires on *consecutive* misses, an
   isolated false miss cannot bounce a well-synced tag to SCAN.
   **Constraint to verify in the plan:** `MISS_MAX ≥ 2` in `uwb_net.c`.
5. **Gateway genuinely lost** — real misses → ACQUIRING (full window) → the existing
   `MISS_MAX → RESCAN` logic acts exactly as today. This design does not change
   lost-sync handling.
6. **`k_cycle_get_32()` wrap** — all deltas are intra-frame (~195 ms ≪ int32 range);
   int32-cast subtraction is wrap-safe, as in `rx_stats_core`.

## Testing

### Host C tests (TDD) — `tests/beacon_track/test_beacon_track.c`

Standalone, WinLibs GCC, same harness as `tests/rx_stats`:
- **EMA convergence:** seed 200 ms, feed gaps of 195 ms → `period_est` moves toward
  195 ms.
- **Lock transition:** after WARMUP_N clean beacons, `plan()` flips from
  `narrow=false` to `narrow=true`.
- **Plan in TRACKING:** `plan()` returns `arm_at = last_beacon + period_est − guard`
  and `window = 2·guard`.
- **Miss → recovery:** `miss()` returns to ACQUIRING, resets `warmup_count`, keeps
  `period_est`; after WARMUP_N catches it returns to TRACKING (fast re-lock).
- **Edges:** first beacon (no prediction), wrap-safe deltas, late-arm plan.

### On-device validation (reuses Spec 1 `pwr rx`)

- `RXon` mean drops from ~182 ms to ~10–15 ms after lock.
- `misses` stay ≤ ~3 % baseline; if higher, the window is too tight → widen `GUARD`
  (tunable). The `pwr rx` miss count is the empirical tightening signal.
- BLE stays connected; measure current draw with the fuel gauge.

## Tuning constants (initial values, all adjustable)

| Constant | Initial | Meaning |
|---|---|---|
| `GUARD` | ±5 ms | half-width of the narrow window (≈2× measured ±2.6 ms jitter) |
| `WARMUP_N` | 8 | consecutive clean beacons before (re-)narrowing |
| `EMA_SHIFT` | 3 | EMA smoothing (≈8-frame average) |
| `WAKE_MARGIN` | 3 ms | radio wake-up lead time before arming RX |

## Success criteria

- Host tests pass (TDD).
- On device after lock: `RXon` ≤ ~15 ms mean; miss rate ≤ ~3 % baseline.
- No BLE regression.
- Measurable average-current reduction vs the Layer-1 baseline.

## Handoff

After this design is approved and committed, the next step is the implementation
plan (`superpowers:writing-plans`) producing `plan/2026-06-24-narrow-beacon-window.md`.
