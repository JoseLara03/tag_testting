# Tag Workflow Design — Motion-Adaptive SS-TWR + Button/LED UI

**Date:** 2026-06-11
**Status:** Approved design, pending implementation plan
**Target:** Zephyr RTOS firmware, nRF52833 + DW3000 (DW3220) custom tracking tag

## Goal

Define the normal operating workflow of the tag, layered on top of the existing
interrupt-driven SS-TWR initiator:

1. Range continuously via SS-TWR, with a **motion-adaptive cadence**: every 1 s
   while moving; every 5 s after 5 s of no motion.
2. **Button** with two gestures:
   - Single press → show battery charge as a discrete color for 5 s, then off.
   - Double press (fast) → blink the NeoPixel orange until the next button press.
3. Stream each distance over BLE NUS **only if a central is connected**; otherwise
   drop it. Ranging must **not** depend on a BLE connection.
4. **UWB is the highest-priority subsystem**; everything else is lower.

A full MAC layer is explicitly out of scope for now but must remain easy to add
later (ranging stays modular behind its current module boundary).

## Current State (baseline)

- `src/uwb_ss_initiator.c` already implements interrupt-driven SS-TWR: a ranging
  thread (prio 2) issues a poll, waits on a DW3000 IRQ semaphore, computes
  clock-offset-corrected distance, and enqueues `"D:x.xxm"` into a `k_msgq`. A
  separate BLE sender thread (prio 6) drains the queue. It currently ranges every
  fixed 1000 ms.
- `src/main.c` **blocks on `ble_log_wait_ready()`** before `uwb_init()`, so today
  ranging does not start until a central connects.
- `ble_log_send()` already returns early when no central is connected, so
  best-effort drop behavior is mostly in place.
- Accelerometer (`lis2hh12_if`), fuel gauge (`batt`), and button are dormant /
  not initialized. The button alias `button0` (P0.17) exists in the board DTS.
- LIS2HH12 INT1 is physically wired to **P0.28** but not yet declared in the DTS.

## Chosen Approach — Minimal event-driven threads (Approach A)

Reuse the existing ranging and BLE threads untouched, add exactly **one** new
application thread, and drive motion and button via GPIO interrupts.

### Thread / ISR model

| Thread / ISR        | Priority | Owns                                                        |
|---------------------|----------|-------------------------------------------------------------|
| Ranging (existing)  | 2 (top)  | DW3000, SS-TWR exchange, adaptive 1 s / 5 s cadence         |
| BLE sender (existing)| 6       | Drains result `k_msgq`, best-effort `ble_log_send()`        |
| UI (new, `tag_ui`)  | 7        | Button gesture logic, LED state machine, on-demand battery  |
| DW3000 IRQ          | ISR      | Gives ranging event semaphore (unchanged)                   |
| Accel INT (P0.28)   | ISR      | Stamps `last_motion`, wakes ranging thread                  |
| Button (P0.17)      | ISR      | Posts debounced press events to UI thread                   |

UWB priority (2) is strictly above every other application thread.

## Components

### `uwb_ss_initiator` (modify)

- Replace the fixed end-of-loop `k_msleep(1000)` with an adaptive wait gated by a
  semaphore so a motion event can shorten a long idle wait:
  ```
  moving = (k_uptime_get() - last_motion) < MOTION_STILL_MS   // 5000
  wait   = moving ? RNG_FAST_MS : RNG_SLOW_MS                 // 1000 : 5000
  k_sem_take(&range_tick, K_MSEC(wait))
  ```
- `last_motion` (a shared timestamp) is seeded to boot time so the tag starts in
  fast (1 s) mode for the first 5 s, then settles to 5 s if idle.
- Expose `void uwb_motion_notify(void)`:
  ```
  last_motion = k_uptime_get();
  k_sem_give(&range_tick);
  ```
  Called from the accel INT ISR. The ranging thread's structure (poll → wait IRQ
  → compute → log) is otherwise unchanged.
- Remove the unused `uwb_simple_tx()` / `uwb_simple_rx()` wrappers (see Examples
  move below); they are not part of this workflow.

### `motion` (new)

- Configure LIS2HH12 over I2C0 for a **pulsed wake-up (activity) interrupt** on
  INT1 (threshold + short duration), routed to GPIO P0.28. Pulsed (not latched)
  so no I2C read is needed in the ISR.
- GPIO ISR body is minimal: call `uwb_motion_notify()`.
- Threshold and duration are tunable constants; start with sensitivity that
  reliably triggers on handling/walking but not on bench vibration. Final values
  are calibrated empirically during implementation.

### `tag_ui` (new) — button + LED + battery

The WS2812 LED is owned **solely** by this thread. The button GPIO ISR (both
edges, ~30 ms debounce) posts press events; the thread runs a state machine:

- **IDLE** — LED off.
- First press while in **BLINK** → exit to IDLE/off. (Any press stops blinking;
  this press is *consumed* and does **not** trigger a battery display.)
- First press while not in BLINK → open a **~350 ms double-click window**:
  - Second press within the window → **DOUBLE** → enter **BLINK** (orange,
    ~500 ms on/off) until the next press.
  - Window expires with one press → **SINGLE** → **BATTERY_SHOW**: read SoC,
    display the discrete color for 5 s, then off → IDLE.
- A new gesture overrides whatever the LED is currently doing.

Consequence: a single-press battery display is delayed by ~350 ms (the window
needed to rule out a double press). Accepted trade-off.

### `batt` (trim)

- Reduce to on-demand: `int batt_read_soc(int *soc)` returning 0 on success.
- Uses the BQ274xx sensor API: `sensor_sample_fetch()` then
  `sensor_channel_get(SENSOR_CHAN_GAUGE_STATE_OF_CHARGE)`; `val.val1` = %.
- Returns an error if the gauge is not ready (USB-only, no battery attached).
- Remove the periodic `k_timer`/`k_work` read path — the workflow only reads on a
  single button press.

### `ble_log` (minimal change)

- Send path unchanged (already drops when disconnected).
- The boot-time blocking wait is removed from `main.c` (see below). No API change
  strictly required; `ble_log_wait_ready()` simply stops being called on the
  critical path.

### `main` (modify)

Boot sequence, **without blocking on BLE**:

1. Get WS2812 device; hang on failure (unchanged).
2. `ble_log_init()` — start advertising.
3. LED cyan — initializing DW3000.
4. `uwb_init(3)`; on success LED green and `uwb_ss_initiator_start()`, on failure
   LED red.
5. `motion_init()` and `tag_ui_init()`.
6. `k_sleep(K_FOREVER)`.

`ble_log_wait_ready()` is **not** called — UWB ranging starts immediately and runs
regardless of BLE connection state. Boot diagnostic strings sent before a central
connects are simply dropped; LED colors convey boot status.

## Battery color mapping (discrete)

| SoC        | Color     |
|------------|-----------|
| ≥ 75 %     | Green     |
| 50–74 %    | Yellow    |
| 25–49 %    | Orange    |
| < 25 %     | Red       |
| gauge not ready | Dim blue (= "no data") |

LED brightness kept low (single-digit RGB values) consistent with existing
status-LED usage.

## BLE logging behavior

- Ranging result path is unchanged: `twr_log()` → `ss_twr_msgq` → BLE sender
  thread → `ble_log_send()`.
- `ble_log_send()` drops the message when no central is connected. No distance is
  buffered for later delivery.

## DTS changes

- Add the LIS2HH12 INT1 line at **P0.28** to the accelerometer node, e.g.
  `irq-gpios = <&gpio0 28 GPIO_ACTIVE_HIGH>` (exact flags confirmed against the
  LIS2HH12 INT polarity during implementation).
- `button0` (P0.17) alias already present; no change.

## Examples folder move

Move the following reference files out of the application build into a new
`examples/` folder and drop them from `target_sources` in `CMakeLists.txt`:

- `src/uwb_ds_initiator.c` / `src/uwb_ds_initiator.h`
- `src/uwb_simple_tx.c` / `src/uwb_simple_tx.h`
- `src/uwb_simple_rx.c` / `src/uwb_simple_rx.h`

Because these leave the build, remove the now-orphaned `uwb_simple_tx()` /
`uwb_simple_rx()` wrapper declarations from `uwb.h` and their implementations and
`#include`s from `uwb.c`. `uwb_ss_initiator.c` is the only ranging module that
remains compiled.

## Priorities summary

UWB ranging (2) > BLE sender (6) > UI (7). ISRs handle DW3000, accel motion, and
button edges. This guarantees the timing-critical ranging exchange is never
preempted by UI, battery, or BLE work.

## Out of scope

- MAC layer (future; ranging stays behind its current module boundary so it can
  be driven by a scheduler later).
- NFC.
- NVS persistence of the calibrated antenna delay.
- The moved `examples/` files remain as reference only and are not compiled.

## Success criteria

- Tag boots and begins SS-TWR ranging with no BLE central connected.
- Distance appears over BLE NUS (`D:x.xxm`) within one cycle of a central
  connecting, and stops/drops cleanly on disconnect — ranging continues.
- Ranging cadence is ~1 s while the tag is moved and ~5 s after ~5 s at rest,
  switching back to ~1 s promptly when motion resumes.
- Single button press shows the correct discrete battery color for 5 s, then off.
- Double button press blinks orange until any subsequent press stops it; that
  stopping press does not also show battery.
- Firmware builds with the example files moved out of `target_sources`.
