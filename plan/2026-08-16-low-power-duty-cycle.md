# Low-power duty cycle — implementation plan

> **For agentic workers:** steps use checkbox (`- [ ]`) syntax for tracking. Implement
> task-by-task, in order; each task ends in a state that builds and tests clean.

**Design doc:** `spec/2026-08-16-low-power-duty-cycle-design.md` — read it first. This
plan implements it and does not restate the rationale.

**Goal:** 30 days on 250 mAh (347 µA), ~2 months on 400 mAh (278 µA). Design target
**≤250 µA**; the tag measures ~22 mA today.

**Architecture:** Decouple the beacon re-sync cadence (`listen_skip`) from the position
fix cadence (`range_every`), per motion tier. The runner sleeps through whole superframes
and re-acquires the beacon using a long-baseline period estimate, so the window stays a
few milliseconds wide even after a 60 s skip. Out of coverage, a motion-gated back-off
ladder replaces the current permanent open receiver. Two new pure host-tested cores:
`beacon_sched_core` (period estimate + wake planning + miss ladder) and
`scan_backoff_core` (coverage probe ladder).

**Tech Stack:** C99, Zephyr RTOS (nCS 3.2.4), nRF52833, DW3000. Host unit tests with
WinLibs GCC.

## Global Constraints

- **Host test build command** (gcc is NOT on PATH):
  ```
  GCC="/c/Users/JoseAntonioLaraPerez/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe/mingw64/bin/gcc.exe"
  "$GCC" -std=c99 -Wall -Wextra -Werror -I src tests/<dir>/test_<x>.c src/<x>.c -o /tmp/t.exe && /tmp/t.exe
  ```
- **The user builds and flashes the Zephyr firmware** and reports results. All on-device
  verification steps are performed by the user.
- **Every new `.c` goes into `CMakeLists.txt`** via `target_sources(app PRIVATE ...)`.
- **NUS lines stay ≤ 19 chars + NUL.** `bt_nus_send` drops anything over 20 bytes
  silently, and `twr_log()` truncates at 20 before that.
- **Do not add a third `wait_event()` caller** and do not take a `uwb_radio_owner` claim
  in the runner's own loop — it already owns the radio there.
- **Keep 4 anchors.** `ANCHOR_SELECT_MAX` stays at 4 for ranging accuracy and future 3D
  positioning. Do not "optimise" it down; the budget already assumes 4.
- **`listen_skip` is capped at 25 until the gateway implements the §7 lease contract.**
  FAST works against today's gateway; SLOW and IDLE do not. The cap is a runtime constant
  (`UWB_LISTEN_SKIP_CAP`), lifted in one edit when the gateway lands.
- **The two `dwt_setlnapamode()` call sites must stay identical** (`src/uwb.c` init and
  `dw_wake()`), per the existing Key Patterns invariant.
- Tuning constants live in one place per module: tier parameters in `uwb_net.h`, scheduler
  constants in `beacon_sched_core.h`, ladder constants in `scan_backoff_core.h`.

## Implementation notes (added during Tasks 2–9, 13)

Things that were not in the plan and had to be decided, or that the plan got wrong.
Tasks 1, 10, 11 and 12 were deliberately **not** implemented — they are blocked on
hardware measurements or on a separate firmware repo.

1. **The lease must age by elapsed superframes, not by one — and the plan does not
   mention it.** `UWB_NET_LEASE_SF` is 50 and the gateway ages every lease once per
   superframe whether or not the tag listened. Decrementing the tag's local
   `lease_remaining` by one per *received* beacon was correct only while the tag
   participated in every superframe; at `listen_skip = 25` it renews 25× too late and the
   seat is reclaimed on the very first skip. Without this fix, Task 9 Step 10 ("no
   `RESCAN` in 30 minutes at FAST = 25 1") could not pass. Added `lease_age()` in
   `uwb_net.c`, pinned by `test_lease_ages_by_elapsed()`.
2. **`beacon_track` must not be fed across a skip.** Its EMA assumes consecutive
   superframes; one skipped window would push its period estimate to thousands of ms.
   The runner now only calls `beacon_track_beacon()` when the effective skip is 1. See
   the comment at that call site and the Layer-3 entry in `CLAUDE.md`.
3. **Task 6 Step 8's ≤ 10 ms gate is only met for realistic baselines** — see the note
   under that step.
4. **Task 9 Step 7's `beacon_sched` reset was deliberately not implemented** — see the
   note under that step.
5. **`uwb_radio_sleep_until()` kept the interruptible name** and a new
   `uwb_radio_sleep_until_strict()` is used for the CFP slot wait, per Task 5 Step 2.
   `uwb_radio_ops.h` changed signature (`void` → `bool`); the runner is the only caller.
6. **`uwb_net_set_moving(bool)` was added** so the runner sees the raw INT1 state rather
   than a tier already derived from it — `uwb_net_tier_filter()` needs the edge.
   `uwb_net_set_tier()` is kept as a direct override but the filter re-evaluates every
   superframe, so it is a one-superframe override rather than a latch.
7. **`beacon_sched_stats_reset()` and `beacon_sched_have_ref()` were added** beyond the
   plan's interface listing: `beacon_sched_reset()` is called on radio handover and must
   not wipe the `pwr sched` counters, and the caller needs to know when `arm_ms` is
   meaningless (no observation yet, or the last re-sync missed).
8. **The SCAN probe sleep is gated on `radio_asleep`**, not just on `dw_sleep_enabled`,
   so the first probe after boot is immediate rather than 10 s late.
9. **The DTS was edited** (`boards/Innovaforce/nRF52833_tag/nRF52833_tag.dts`): Task 4
   Step 3 needs `zephyr,pm-device-runtime-auto` on `&spi3` and `&i2c0`, and
   `CONFIG_PM_DEVICE_RUNTIME=y` alongside `CONFIG_PM_DEVICE=y`. `&spi1` deliberately
   untouched.
10. **The host-test compiler is not at the path in the Global Constraints.** On this
    machine WinLibs gcc 13.1.0 (same vendor, `built by Brecht Sanders`) lives at
    `/c/strawberry/c/bin/gcc`; the `JoseAntonioLaraPerez` WinGet path does not exist.
    `tests/rx_stats/` also needs `src/batt_window.c`, which the CLAUDE.md table omitted.

## Rollout order rationale

Tasks 1–4 are independent, low-risk, and together recover ~150 µA — over half the design
target — before any architectural change is touched. Task 5 is a prerequisite for the
core work (Tasks 6–8) and must land before it. Task 9 depends on a measurement from
Task 1. Task 12 is coupled to firmware this repo does not contain.

---

### Task 1: Measurement baseline

**No code.** This is the user's task and it gates Tasks 9 and 10.

**Steps:**

- [ ] Step 1: With a PPK2 or source meter in series with the cell, record **total current,
      tag idle, DW3000 in SLEEP** (`pwr sleep on`, tag out of coverage). Record the
      average over 60 s and the waveform.
- [ ] Step 2: Record **LDO2 (1.8 V, LNA rail) current with the DW3000 in SLEEP**. If it is
      not ≈0, the LNA is being left enabled across sleep — report this, and Task 9 moves to
      the front of the plan.
- [ ] Step 3: Record **total current with the LIS2HH12 held in power-down**
      (`LIS2HH12_XL_ODR_OFF`, temporary build). Confirms the ~50 µA figure on this board.
- [ ] Step 4: Confirm from the schematic whether the board has a **32.768 kHz crystal** or
      uses the internal RC. This sets the maximum usable `listen_skip` (design §4.3) and
      therefore the Task 7 constants.
- [ ] Step 5: Record the anchor's **poll-to-response turnaround** (scope or anchor-side
      log) for Task 10.

**Verification:** five numbers recorded. Compare Step 1 against the design §2 model; if it
disagrees by more than ~2×, stop and revise the model before continuing.

---

### Task 2: Accelerometer ODR 50 Hz → 10 Hz

**Files:** Modify `src/motion.c`

Expected saving: ~130 µA. One-line change plus one constant that must move with it.

**Steps:**

- [x] Step 1: Change `lis2hh12_xl_data_rate_set()` to `LIS2HH12_XL_ODR_10Hz`.
- [x] Step 2: Change `ACT_DURATION` from 31 to **6**. `ACT_DUR` LSB is `8/ODR` seconds, so
      at 10 Hz the LSB is 0.80 s and 6 gives ~4.8 s — the same inactivity window the 50 Hz
      setting intended. Leaving it at 31 would give ~24.8 s.
- [x] Step 3: Update the comment block above `ACT_THRESHOLD` to state ODR 10 Hz and the new
      LSB arithmetic. `ACT_THS` is unchanged (its LSB is `FS/128`, ODR-independent).

**Verification:**
- [ ] Step 4 (user, on device): walk the tag, then set it down. Confirm the ranging tier
      still switches FAST↔SLOW, and that the still→FAST transition is prompt and the
      FAST→SLOW transition takes ~5 s, not ~25 s. `P:` line cadence is the readout.
- [ ] Step 5 (user): measure total current, stationary. Expect ~130 µA lower than Task 1
      Step 1.

---

### Task 3: BLE advertising policy

**Files:** Modify `src/ble_log.c`, `src/ble_log.h`, `src/tag_ui.c`, `src/nfc_tag.c`,
`src/tag_cmd.c`

Expected saving: ~130 µA. Nothing operational depends on BLE — positions reach the
gateway over the `0xEA` POS frame — so this is diagnostics only.

**Interfaces produced:**

```c
/* Open a bounded advertising window. Called from the button gesture and from the
 * NFC field callback. Restarts the 60 s timer if already advertising. */
void ble_log_adv_window(void);
```

**Steps:**

- [x] Step 1: Replace `BT_LE_ADV_CONN_FAST_2` with a slow connectable parameter set
      (`BT_GAP_ADV_SLOW_INT_MIN`/`MAX`, 1–1.2 s) in both `ble_log_init()` and
      `adv_work_fn()`.
- [x] Step 2: Add `ble_log_adv_window()`: starts advertising and arms a 60 s
      `k_work_delayable` that calls `bt_le_adv_stop()`. Idempotent — a second call restarts
      the timer. Must not stop advertising while a connection is up; on disconnect,
      `adv_work_fn` opens a fresh 60 s window rather than advertising forever.
- [x] Step 3: Do **not** advertise at boot by default. Call `ble_log_adv_window()` from the
      `tag_ui.c` single-press handler (the same gesture that shows battery colour) and from
      the `NFC_T4T_EVENT_NDEF_UPDATED` / field-detect path in `nfc_tag.c`.
- [x] Step 4: Keep advertising continuously while `uwb_radio_sleep_enabled() == false`
      (`pwr sleep off`), so bench debugging is unaffected. Check this in `adv_work_fn` and
      in the window-expiry handler.
- [x] Step 5: Add `pwr adv on|off` to `tag_cmd.c` to force continuous advertising for a
      debug session, persisted nowhere (RAM only, resets on reboot).

**Verification:**
- [ ] Step 6 (user): with the tag idle and unconnected, confirm it is not discoverable
      after 60 s from boot; press the button and confirm it becomes discoverable within a
      few seconds and stays so for 60 s.
- [ ] Step 7 (user): confirm `pwr adv on` keeps it discoverable indefinitely and survives a
      disconnect.
- [ ] Step 8 (user): measure total current, stationary, no connection. Expect ~130 µA lower
      than after Task 2.

---

### Task 4: Housekeeping wakes

**Files:** Modify `src/batt.c`, `prj.conf`

**Steps:**

- [x] Step 1: Raise the battery sampler period from 5 s to 60 s. The idle-current window
      accumulates per sample (`batt_window.c`), so its statistics stay valid — only the
      sample count changes. Update the comment in `batt.c` and the CLAUDE.md battery
      pattern entry (Task 13).
- [x] Step 2: Add `CONFIG_PM_DEVICE=y` to `prj.conf`. This is what makes the
      `pinctrl-1 = <&..._sleep>` states already present in the board DTS take effect.
- [x] Step 3: Add runtime PM to SPI3 (WS2812) so the bus is suspended except during an LED
      write, and to I2C0 between battery/accelerometer transactions. **Do not** add runtime
      PM to SPI1 — the DW3000 path is driven from ISR context and the suspend/resume calls
      are not safe there.

**Verification:**
- [ ] Step 4 (user): confirm the LED readout and the battery reading still work
      (`pwr idle`, single press).
- [ ] Step 5 (user): confirm ranging is unaffected — `P:` lines at the expected cadence,
      no new `RESCAN` lines over 10 minutes. If SPI1 behaviour changed at all, revert
      Step 2 and investigate before continuing.

---

### Task 5: Interruptible runner sleep

**Files:** Modify `src/uwb_net_runner.c`, `src/uwb_net_runner.h`, `src/tag_alert.c`,
`src/motion.c`

**Prerequisite for Tasks 6–8.** Without it a motion event or a HELP press waits up to a
full skip period — 60 s at the IDLE tier. Landing it first also means it can be verified
in isolation, while the loop still turns over every 200 ms and the change is a no-op.

**Interfaces produced:**

```c
/* Cut short whatever sleep the runner is in and force a full-window beacon
 * re-sync on the next pass. Safe from ISR and from any thread. */
void uwb_net_runner_wake(void);
```

**Steps:**

- [x] Step 1: Add a static `K_SEM_DEFINE(runner_wake, 0, 1)` and implement
      `uwb_net_runner_wake()` as `k_sem_give()` plus a `volatile bool force_full_window`.
- [x] Step 2: Replace the body of `uwb_radio_sleep_until()` with
      `k_sem_take(&runner_wake, K_MSEC(rem))`, returning a `bool` that reports whether the
      sleep was cut short. Update both call sites (the beacon planner at
      `uwb_net_runner.c:528-530` and the CFP slot wait at `:723`). **The CFP slot wait must
      NOT be interruptible** — cutting it short would transmit outside the tag's slot.
      Give that call site a non-interruptible variant.
- [x] Step 3: Drain the semaphore with `k_sem_reset()` at the top of `runner_fn`'s loop
      after handling a wake, so a give that arrives during an exchange does not cause a
      spurious skip-abort two iterations later.
- [x] Step 4: Call `uwb_net_runner_wake()` from `uwb_set_moving()` (motion tier change) and
      from `tag_alert_raise()` / `tag_alert_cancel()`.
- [x] Step 5: On a cut-short sleep, set `listen_skip` to 1 and force a full-superframe
      window for the next re-sync. A wasted full-window listen is the correct price for a
      HELP press.

**Verification:**
- [x] Step 6: Existing host tests still pass (`tests/uwb_net/`, `tests/beacon_track/`).
- [ ] Step 7 (user): behaviour is unchanged — `P:` lines at the same cadence, no `RESCAN`
      over 10 minutes. This task is a no-op at `listen_skip = 1` and any visible change is
      a bug.
- [ ] Step 8 (user): confirm a double press still raises HELP and the frame is transmitted
      (anchor/gateway RX log).

---

### Task 6: `beacon_sched_core` — long-baseline scheduler

**Files:**
- Create: `src/beacon_sched_core.c`, `src/beacon_sched_core.h`
- Create: `tests/beacon_sched/test_beacon_sched.c`
- Modify: `CMakeLists.txt`

Pure module, no Zephyr. This is the piece that makes deep skipping viable: estimating the
superframe period over a long baseline gives an error of `jitter/n` instead of the EMA's
`~jitter`, and that error is what gets multiplied by the skip factor (design §4.2).

**Interfaces produced:**

```c
#define BSCHED_PPM_TOTAL      40u   /* tag + gateway, crystal; 500 for internal RC */
#define BSCHED_WINDOW_MIN_MS   4u
#define BSCHED_WINDOW_MARGIN_MS 2u
#define BSCHED_BASELINE_MAX  128u   /* superframes retained for the period estimate */

struct beacon_sched {
    uint32_t nominal_ms;
    uint32_t t0_ms;        /* first arrival of the current baseline */
    uint32_t fc0;          /* frame counter at t0 */
    uint32_t t_last_ms;
    uint32_t fc_last;
    uint32_t period_q16;   /* estimated period, ms in Q16.16 */
    uint8_t  n_obs;        /* observations in the current baseline */
    uint8_t  misses;       /* consecutive missed re-syncs */
    uint32_t ok_count, miss_count;   /* diagnostics for `pwr sched` */
};

void beacon_sched_reset(struct beacon_sched *s, uint32_t nominal_period_ms);
void beacon_sched_observe(struct beacon_sched *s, uint32_t t_ms, uint32_t frame_ctr);
void beacon_sched_miss(struct beacon_sched *s);
void beacon_sched_plan(const struct beacon_sched *s, uint32_t skip,
                       uint32_t *arm_ms, uint32_t *window_ms, uint32_t *effective_skip);
uint32_t beacon_sched_period_q16(const struct beacon_sched *s);
```

**Steps:**

- [x] Step 1: Implement `beacon_sched_observe()`. Period estimate is the two-point
      long baseline `(t_last - t0) / (fc_last - fc0)` in Q16.16 fixed point — **not
      floating point**, and **not an EMA**. Use the gateway's frame counter for the
      denominator so a skip contributes its full span to the baseline. Roll the baseline
      forward when `n_obs` reaches `BSCHED_BASELINE_MAX` (drop `t0` to a midpoint
      observation) so a slow clock drift is tracked rather than averaged away.
- [x] Step 2: Implement `beacon_sched_plan()`:
      `arm_ms = t_last + skip × P - W/2 - guard`, and
      `W = 2 × skip × P × ppm/1e6 + 2 × skip × ε_P + BSCHED_WINDOW_MARGIN_MS`,
      floored at `BSCHED_WINDOW_MIN_MS`. Derive `ε_P` from the baseline length
      (`jitter / n_obs`), so a short baseline widens the window automatically.
- [x] Step 3: Implement the miss ladder in `beacon_sched_miss()`: 1 miss → effective skip
      halved and window doubled; 2 misses → `effective_skip = 1` and
      `window_ms = nominal + 2`; 3 → the caller escalates to `UWB_ST_SCAN` (the module
      only reports `misses`, it does not own the state machine). Retain the baseline
      across a single miss.
- [x] Step 4: Handle `uint32_t` wrap on both the millisecond clock and the frame counter
      using signed difference arithmetic throughout.
- [x] Step 5: Write `tests/beacon_sched/test_beacon_sched.c` covering: period convergence
      under injected jitter; estimate error scaling as `1/n_obs`; window width matching
      the hand-computed drift bound for skip = 1, 25, 75, 300; frame-counter and
      millisecond wrap; the miss ladder up and its reset on success; baseline survival
      across one miss.
- [x] Step 6: Add to `CMakeLists.txt`.

**Verification:**
- [x] Step 7: `"$GCC" -std=c99 -Wall -Wextra -Werror -I src tests/beacon_sched/test_beacon_sched.c src/beacon_sched_core.c -o /tmp/t.exe && /tmp/t.exe`
- [ ] Step 8: A 300-superframe skip with ±0.12 ms jitter and a 100-observation baseline
      must plan a window ≤ 10 ms. If it does not, the estimator is wrong and the rest of
      the plan does not work — stop here.

      **PARTIALLY MET — left unticked deliberately. Read this before lifting
      `UWB_LISTEN_SKIP_CAP`.** The estimator's 1/n scaling is correct and pinned by
      `test_error_scales_as_one_over_n()`. The ≤ 10 ms number depends on how the
      "100-observation baseline" was itself taken, because the baseline is measured in
      *superframes elapsed*, not in observations:

      | baseline | span (sf) | window planned for skip 300 |
      |---|---|---|
      | 100 obs at skip 1  |    99 | **14 ms** |
      | 100 obs at skip 4  |   396 | 9 ms |
      | 100 obs at skip 25 | 2 475 | 7 ms |
      | 100 obs at skip 300| 29 700| 7 ms |

      The gap is `BSCHED_QUANT_US`: `beacon_sched_observe()` takes a **millisecond**
      timestamp (per this plan's own interface and Task 9 Step 3), and
      `k_uptime_get_32()` truncates, so the baseline *difference* carries up to ±1 ms of
      error — 8× the ±0.12 ms arrival jitter design §4.2 accounts for. Design §4.2 omits
      this term.

      Why this is not a blocker: a 300-superframe skip only ever arises while the tag is
      *running* at a deep skip, at which point the baseline spans hundreds of superframes
      and the window is 7 ms. And at the shipped `UWB_LISTEN_SKIP_CAP = 25` the planned
      window is the 4 ms floor from any baseline. The pathological row is pinned by
      `test_deep_skip_window()` so a future change to the constants is caught.

      If a deep skip ever has to work from a short baseline, the fix is to feed this
      module microseconds (`k_cycle_get_32()`-derived, as `rx_stats.c` already does) and
      convert at the runner boundary — not to shrink the constant.

---

### Task 7: Tier parameters and hysteresis

**Files:** Modify `src/uwb_net.c`, `src/uwb_net.h`, `tests/uwb_net/test_uwb_net.c`

Pure. Replaces `tier_cadence()`'s single number with the two-parameter set from design §6.

**Interfaces produced:**

```c
#define UWB_LISTEN_SKIP_CAP  25u   /* until the gateway implements the §7 lease contract */

struct uwb_tier_params {
    uint16_t listen_skip;   /* superframes slept between beacon re-syncs */
    uint16_t range_every;   /* participations between position fixes */
};

/* Defaults: FAST {25,1}, SLOW {75,1}, IDLE {300,1}. All clamped to
 * UWB_LISTEN_SKIP_CAP on read until the gateway contract lands. */
void uwb_net_set_tier_params(uwb_tier_t t, const struct uwb_tier_params *p);
void uwb_net_get_tier_params(uwb_tier_t t, struct uwb_tier_params *out);

/* Tier hysteresis (design §6.2). Fed the raw motion state and a timestamp;
 * returns the tier the runner should actually use. */
#define UWB_TIER_HOLD_FAST_MS  30000u
#define UWB_TIER_HOLD_SLOW_MS  60000u
uwb_tier_t uwb_net_tier_filter(struct uwb_net_ctx *c, bool moving, uint32_t now_ms);
```

**Steps:**

- [x] Step 1: Add the parameter table with the design §6.1 defaults, and clamp
      `listen_skip` to `UWB_LISTEN_SKIP_CAP` inside `uwb_net_get_tier_params()` — clamping
      on read, not on write, so lifting the cap does not require rewriting stored values.
- [x] Step 2: Replace `uwb_tier_due()` with a `range_every` counter held in
      `struct uwb_net_ctx`, incremented per participation rather than derived from
      `frame_counter % cadence`. The frame counter is no longer a usable cadence reference
      once superframes are skipped.
- [x] Step 3: Implement `uwb_net_tier_filter()`: FAST is held for `TIER_HOLD_FAST_MS` after
      the last activity edge; FAST → SLOW on the inactivity edge; SLOW → IDLE after
      `TIER_HOLD_SLOW_MS` of continued stillness. Any activity edge goes straight to FAST.
- [x] Step 4: Emit `UWB_ACT_SLEEP` from `UWB_ST_SCAN` (Open Work item 3). In `SCAN` the
      action means "sleep the radio until the next scheduled probe" rather than "until the
      next superframe" — the runner reads the interval from `scan_backoff_core` (Task 8).
      Keep `UWB_ACT_SEND_ALERT` firing on both `UWB_EV_BEACON` and `UWB_EV_BEACON_MISS` in
      `SCAN`, unchanged.
- [x] Step 5: Extend `tests/uwb_net/test_uwb_net.c`: tier parameter get/set and the cap
      clamp; `range_every` counting across skips; the full hysteresis state machine
      including the hold expiring exactly at the boundary; `UWB_ST_SCAN` now returning
      `UWB_ACT_SLEEP`; and confirm `test_gate_actions()` still passes — `UWB_ACT_SLEEP`
      must remain outside `UWB_ACT_RANGING_MASK` so an uncalibrated tag still deep-sleeps.

**Verification:**
- [x] Step 6: `"$GCC" -std=c99 -Wall -Wextra -Werror -I src tests/uwb_net/test_uwb_net.c src/uwb_net.c -o /tmp/t.exe && /tmp/t.exe`

---

### Task 8: `scan_backoff_core` — coverage probe ladder

**Files:**
- Create: `src/scan_backoff_core.c`, `src/scan_backoff_core.h`
- Create: `tests/scan_backoff/test_scan_backoff.c`
- Modify: `CMakeLists.txt`

Pure. Implements the maintainer's "stop listening when no beacon, re-probe periodically"
with the sizing from design §5.

**Interfaces produced:**

```c
/* 10 s, 30 s, 2 min, 5 min, 15 min, 30 min (terminal). */
#define SCAN_BACKOFF_RUNGS  6

struct scan_backoff { uint8_t rung; bool alert_pinned; };

void     scan_backoff_reset(struct scan_backoff *b);
void     scan_backoff_fail(struct scan_backoff *b);
void     scan_backoff_motion(struct scan_backoff *b);
void     scan_backoff_alert(struct scan_backoff *b, bool active);
uint32_t scan_backoff_next_ms(const struct scan_backoff *b);
uint8_t  scan_backoff_rung(const struct scan_backoff *b);
```

**Steps:**

- [x] Step 1: Implement the ladder as a static `const uint32_t` interval table.
- [x] Step 2: `scan_backoff_fail()` climbs one rung and saturates at the terminal rung;
      it is a no-op while `alert_pinned`.
- [x] Step 3: `scan_backoff_motion()` resets to rung 0 from any rung.
      `scan_backoff_alert(b, true)` pins rung 0; `false` releases the pin at the current
      rung without jumping.
- [x] Step 4: Write `tests/scan_backoff/test_scan_backoff.c` covering: climb and
      saturation; motion reset from every rung; the alert pin ignoring `fail()`; release
      leaving the rung where it was; interval table monotonic.
- [x] Step 5: Add to `CMakeLists.txt`.

**Verification:**
- [x] Step 6: `"$GCC" -std=c99 -Wall -Wextra -Werror -I src tests/scan_backoff/test_scan_backoff.c src/scan_backoff_core.c -o /tmp/t.exe && /tmp/t.exe`

---

### Task 9: Runner integration — superframe skipping

**Files:** Modify `src/uwb_net_runner.c`

The task that actually spends the budget. Everything before it is either preparation or
independent savings.

**Steps:**

- [x] Step 1: Add `struct beacon_sched sched` and `struct scan_backoff backoff` alongside
      the existing `struct beacon_track bt` in `runner_fn`. `beacon_track` keeps its
      ACQUIRING/TRACKING role for the *initial* lock; `beacon_sched` takes over once
      TRACKING is reached. Do not merge them.
- [x] Step 2: In step 2 of the loop (the beacon window), when `beacon_track` reports
      TRACKING, plan the window with `beacon_sched_plan()` using the current tier's
      `listen_skip` instead of the fixed single-superframe prediction. When ACQUIRING,
      keep the existing full-window behaviour untouched.
- [x] Step 3: Feed `beacon_sched_observe(&sched, bcn_rx_ms, frame_ctr)` on a parsed beacon
      and `beacon_sched_miss(&sched)` on a miss, next to the existing `rx_stats_*` and
      `beacon_track_*` calls.
- [x] Step 4: In `UWB_ST_SCAN`, use `scan_backoff_next_ms()` as the sleep interval for
      `UWB_ACT_SLEEP` and probe with a full-superframe window (`T_SUPERFRAME_MS +
      T_BEACON_MS`). Call `scan_backoff_fail()` on a failed probe,
      `scan_backoff_reset()` on a beacon. Wire `scan_backoff_motion()` into the tier-change
      injection at the top of the loop and `scan_backoff_alert()` to `tag_alert_active()`.
- [x] Step 5: At rung ≥2, suppress discovery entirely (design §5.3) — skip the
      `UWB_ACT_RUN_DISCOVER` block while out of coverage at a high rung.
- [x] Step 6: Make re-discovery time-based rather than participation-based:
      `REDISCOVER_INTERVAL_SF` counted in participations becomes ~10 minutes at the IDLE
      tier. Replace with a `REDISCOVER_INTERVAL_MS` (60 s) compared against
      `uwb_radio_now_ms()`.
- [x] Step 7: Apply `uwb_net_tier_filter()` to the motion input so the hysteresis from
      Task 7 is actually in the path, and reset `beacon_sched` on a tier change only if the
      new `listen_skip` is *larger* — shrinking the skip is always safe with an existing
      estimate.

      **Implemented except for the reset, which was deliberately not done — this half of
      the step is wrong as written.** `beacon_sched`'s period estimate and phase reference
      are properties of the *gateway's* clock, not of the tag's tier. A longer skip is
      precisely when the long baseline is worth most, so resetting there would force the
      widest possible window at the moment the narrowest is needed. A short baseline
      already widens its own window automatically (`est_us = 2·skip·ε/span`), so growing
      the skip is safe with whatever estimate exists. The runner carries a comment saying
      so at the tier-change site.
- [x] Step 8: On `uwb_radio_yield()` returning true (a real handover), reset **both**
      `beacon_track` and `beacon_sched` — the existing comment at
      `src/uwb_net_runner.c:490-497` explains why, and it applies at least as strongly to
      the long-baseline estimate.

**Verification:**
- [ ] Step 9 (user, `listen_skip` forced to 1 via `pwr tier f 1 1`): behaviour identical to
      Task 5. No `RESCAN` in 10 minutes, `P:` at 200 ms.
- [ ] Step 10 (user, FAST = `25 1`): `P:` lines every ~5 s, `n=4`, no `RESCAN` in 30
      minutes. `pwr rx` shows RXon mean far below the current ~30 ms.
- [ ] Step 11 (user): measure current — this is the step that must show the large drop.
      Expect ≤400 µA moving.
- [ ] Step 12 (user): unplug/shield the gateway. Confirm the tag climbs the ladder
      (`pwr scan` reports the rung), current falls to <100 µA, and it rejoins within one
      probe interval when the gateway returns — and immediately on motion.
- [ ] Step 13 (user): HELP press from the IDLE tier is transmitted within 2 s.

---

### Task 10: TWR timing tightening

**Files:** Modify `src/uwb_net_runner.c`

Depends on Task 1 Step 5. Expected ~27% off the per-fix cost at no link-budget cost.

**Steps:**

- [ ] Step 1: Set `POLL_TX_TO_RESP_RX_DLY_UUS` and `RESP_RX_TIMEOUT_UUS` from the measured
      anchor turnaround plus margin. **One step, not iteratively** — over-tightening turns
      a marginal-SNR anchor into a missing one, and a missing anchor costs a whole fix.
- [ ] Step 2: Leave `PRE_TIMEOUT = 128` alone. It is what makes an *absent* anchor cheap
      and it is already correct.
- [ ] Step 3: Note in the comment block that these values are paired with the anchor
      firmware's response delay and must be revisited together.

**Verification:**
- [ ] Step 4 (user): over ≥300 sweeps, `n=4` holds at the same rate as before the change.
      If `n` drops at all, revert and re-measure the turnaround.
- [ ] Step 5 (user): `pwr rx` and a current measurement confirm the sweep got shorter.

---

### Task 11: LNA rail gating

**Files:** Modify `src/uwb_net_runner.c`, `prj.conf`

**Conditional on Task 1 Step 2.** If LDO2 already reads ≈0 during DW3000 SLEEP, the LNA
enable is being handled correctly by EXTRXE and this task is a small refinement rather
than a fix — implement it anyway (the LNA is only needed while RX is on) but expect a
small saving. If LDO2 reads milliamps, this is a large win and should have been promoted
ahead of Task 9.

**Steps:**

- [ ] Step 1: Get the LDO2 regulator device from the DTS (`DEVICE_DT_GET` on the nPM1304
      LDO2 node) and confirm `CONFIG_REGULATOR=y` gives `regulator_enable/disable`.
- [ ] Step 2: `regulator_disable(ldo2)` at the end of `dw_enter_sleep()`.
- [ ] Step 3: `regulator_enable(ldo2)` at the **start** of `dw_wake()`, before the
      existing `wakeup_device_with_io()`, so the rail settles during the 2 ms
      INIT_RC→IDLE_RC wait that is already there.
- [ ] Step 4: Confirm the existing `dwt_setlnapamode(DWT_LNA_ENABLE)` call in `dw_wake()`
      still runs *after* the rail is up, and that it remains byte-identical to the
      `src/uwb.c` init call.
- [ ] Step 5: Add the LDO2 settle time to `DW_WAKE_GUARD_MS` if the measured settle exceeds
      the existing 5 ms budget.

**Verification:**
- [ ] Step 6 (user): LDO2 reads ≈0 with the DW3000 in SLEEP.
- [ ] Step 7 (user): **RX sensitivity is unchanged** — this is the risk. Compare `n=4`
      rate and the CIR quality scores at the same physical distance before and after. A
      rail that is not settled before the first RX shows up as lost anchors at range, not
      as a hard failure.

---

### Task 12: Gateway lease contract

**No code in this repo.** The gateway/anchor firmware is a separate codebase.

**Steps:**

- [ ] Step 1: Hand design §7 to whoever owns the gateway firmware, before it is written.
      The three requirements: the tag declares its `listen_skip`; the gateway sizes the
      lease as `max(UWB_NET_LEASE_SF, 3 × declared_listen_skip)`; the slot map keeps a
      sleeping tag's seat reserved.
- [ ] Step 2: Once implemented, raise `UWB_LISTEN_SKIP_CAP` from 25 to 300 and re-run the
      Task 9 verification at the SLOW and IDLE tiers.
- [ ] Step 3: Re-measure against acceptance criteria 1, 3 and 4 (design §10.2) — the IDLE
      tier is where the 250 mAh × 30 day figure actually comes from, and it is unreachable
      until this lands.

---

### Task 13: Documentation

**Files:** Modify `CLAUDE.md`

- [x] Step 1: Update the "Power saving — RX duty-cycle" Key Patterns entry: Layer 3 is
      superframe skipping; `listen_skip` vs `range_every`; the `UWB_LISTEN_SKIP_CAP`
      constraint and why it exists.
- [x] Step 2: Update the "Motion → ranging cadence" entry for ODR 10 Hz and `ACT_DUR = 6`,
      including the `8/ODR` LSB arithmetic that ties them together.
- [x] Step 3: Add `beacon_sched_core.c/h` and `scan_backoff_core.c/h` to the Source Layout
      and to the host-test table.
- [x] Step 4: Update the "BLE NUS transmit" entry for the event-driven advertising window —
      anything that reads state over NUS now needs a button press first.
- [x] Step 5: Update the battery pattern entry for the 60 s sampler period.
- [x] Step 6: Close Open Work item 3 (`UWB_ST_SCAN` never sleeps) and update item 4 with
      the measured result.
- [x] Step 7: Add the new `pwr` commands to the "BLE NUS receive / commands" entry.
