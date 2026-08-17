# Low-power duty cycle — design

Status: **design only, nothing implemented.** Covers Open Work items 3 ("`UWB_ST_SCAN`
never emits `UWB_ACT_SLEEP`") and 4 ("current consumption still too high").

Goal: **30 days on a 250 mAh cell**, and ~2 months on 400 mAh. That is an average of
**347 µA** and **278 µA** respectively; this design targets **≤250 µA** to leave margin
for cell aging, self-discharge and the cutoff voltage.

The tag measures ~22 mA today. That is ~64× over budget — 250 mAh lasts about 11 hours.
This is not reachable by tuning the existing constants, and §3 explains why.

> **Working on the gateway/anchor firmware? Start at §7.** Superframe skipping changes
> the lease contract: a tag that sleeps through 300 superframes cannot renew a 50-
> superframe lease. §7 is the contract the gateway must implement, and it needs to be
> settled *before* the gateway firmware is written, not retrofitted.

---

## 1. The budget

| Goal | Average current allowed |
|---|---|
| 250 mAh × 30 days (720 h) | 347 µA |
| 400 mAh × 60 days (1440 h) | 278 µA |
| **Design target** | **250 µA** |

Note that 400 mAh is 1.6× of 250 mAh, so at an unchanged average it buys ~48 days, not
60. The second goal implicitly requires a *lower* average than the first, not just a
bigger cell. The 250 µA target satisfies both.

### 1.1 Target allocation

| Item | µA | Fixed by | Notes |
|---|---|---|---|
| LIS2HH12 @ 10 Hz | 50 | hardware | datasheet; no low-power mode on this part |
| nRF52833 idle + activity bursts | 40 | §8 | System ON idle, RTC only |
| BLE advertising | 20 | §8.1 | 1–2 s interval or event-driven |
| nPM1304 + 4 rails quiescent | 20 | hardware | |
| DW3000 SLEEP leak + WS2812 static | 2 | hardware | WS2812B-2020-V6 is ≤1 µA static |
| UWB activity | 70 | §3–§6 | the whole subject of this document |
| **Total** | **202** | | 250 mAh → 51 days; 400 mAh → 82 days |

### 1.2 Deliberately not in this design

- **Replacing the LIS2HH12** with a LIS2DW12/LIS2DH12 (~5 µA). Recorded as a future
  board revision; 50 µA at ODR 10 Hz is accepted here.
- **Power-gating the WS2812.** The -2020-V6 part draws ≤1 µA static (datasheet §电气参数).
  No load switch is needed.
- **Reducing the anchor count to 3.** Four anchors are retained for ranging accuracy and
  for future 3D positioning. This is a product decision, not a power decision, and it
  costs ~25% of the sweep. The budget above already assumes 4.
- **TDoA uplink instead of SS-TWR.** A single uplink blink is ~10× cheaper per fix than a
  4-anchor TWR sweep and needs no beacon sync at all. It is the right architecture for a
  month-long wearable, but it requires synchronized anchors and moves the solver to the
  gateway. Out of scope; revisit if the fix-rate requirement tightens.
- **Dropping the NUS log output** (Open Work item 7). Independent of this work.

---

## 2. Where the 22 mA goes

Estimated from the code and the tag's own `pwr rx` instrumentation. Reconcile against a
PPK2 measurement before acting (see §10.1) — these are the working hypotheses, not
measurements.

| Source | Estimate | Evidence |
|---|---|---|
| Beacon RX every superframe | ~10 mA | `pwr rx` reports RXon mean ~30 ms / 200 ms × 66 mA |
| 4-anchor sweep at FAST tier (every superframe) | ~7.7 mA | 22 ms measured worst case / 200 ms × ~70 mA |
| `dw_wake()` overhead every superframe | ~1.6 mA | ~4 ms of IDLE_RC/PLL + SPI restore per superframe |
| LIS2HH12 @ 50 Hz | ~0.18 mA | `motion.c` sets `LIS2HH12_XL_ODR_50Hz` |
| BLE advertising @ 100–150 ms | ~0.15 mA | `BT_LE_ADV_CONN_FAST_2`, `ble_log.c:110` |
| nRF52833 runner activity, PMIC, misc | ~1–2 mA | |

The `UWB_ST_SCAN` case is worse than all of the above: an unjoined tag holds RX open
~100% of every superframe and never deep-sleeps, because `UWB_ST_SCAN` returns
`UWB_ACT_NONE` on every event (`src/uwb_net.c:72-80`) and `UWB_ACT_SLEEP` is emitted only
from `UWB_ST_RANGING`. Out-of-coverage is currently the tag's *worst* power state.

---

## 3. Why the existing power work cannot reach the target

The two RX duty-cycle layers already in the firmware (DW3000 deep sleep, narrow beacon
window) took the tag from ~66 mA to ~22 mA. They cannot go much further, for a reason
that is structural rather than a matter of tuning.

Take the best case the current architecture can ever reach: a perfect 3 ms narrow beacon
window, radio in SLEEP for the rest of every superframe, no ranging at all.

```
3 ms / 200 ms × 66 mA  =  990 µA
```

That is **4× the design target, before a single range, before the MCU, before BLE,
before the accelerometer.** Narrowing the window further runs into the arrival jitter
floor (~±0.12 ms measured, plus the DW3000 preamble acquisition time) and into
`dw_wake()`'s ~4 ms settle, which is already larger than the window it is protecting.

**The conclusion: the tag must skip whole superframes.** Maintaining 200 ms sync
continuously is not affordable at any window size.

### 3.1 The specific gap in the current code

The tier mechanism looks like it already does this, and it does not.

`uwb_tier_due()` (`src/uwb_net.c:13-16`) gates **`UWB_ACT_RUN_SWEEP` only**
(`src/uwb_net.c:176-180`). The runner's beacon listen at `src/uwb_net_runner.c:541-555`
runs on *every* loop iteration regardless of tier. So `UWB_TIER_IDLE` (cadence 25) saves
the sweep and leaves the dominant cost — the beacon RX and the wake overhead — completely
untouched. Roughly 12 mA of the present 22 mA is invisible to the tier setting.

**The core change in this design is to decouple two cadences that are currently one:**

| Parameter | Meaning | Today |
|---|---|---|
| `listen_skip` | superframes slept between beacon re-syncs | always 1 (hardcoded) |
| `range_every` | participations between position fixes | `tier_cadence()`, 1/5/25 |

`listen_skip` is where the power is. `range_every` is where the position update rate is.
They must be settable independently, per tier.

### 3.2 What one participation costs

One superframe of participation = beacon window (~3 ms) + 4-anchor sweep (~22 ms today,
~16 ms after §6) at ~68 mA average:

```
19 ms × 68 mA = 1.29 mC = 0.36 µAh per fix
```

| Fix interval | Average current |
|---|---|
| 0.2 s (today, FAST tier) | 6 460 µA |
| 2 s | 646 µA |
| 5 s | 258 µA |
| 10 s | 129 µA |
| 60 s | 22 µA |
| 300 s | 4 µA |

With a wearable duty cycle of ~20% in motion, `0.2 × 258 + 0.8 × 22 = 69 µA` for a
5-second moving update rate. That is the 70 µA line in §1.1, and it is where the target
comes from. **The ranging rate was never the problem; the continuous listening was.**

---

## 4. Superframe skipping

### 4.1 Mechanism

After a participation in which the beacon arrived at `t_bcn` with the gateway's frame
counter `fc`:

```
next_participation_fc = fc + listen_skip
t_predict             = t_bcn + listen_skip × P
arm window            = [t_predict - W/2, t_predict + W/2]
MCU sleeps until        t_predict - W/2 - DW_WAKE_GUARD_MS
```

`P` is the estimated superframe period **in the tag's clock** — measured at ~195 ms, not
the nominal 200 ms; predicting from the nominal value accumulates ~5 ms of error per
superframe and is the mistake the existing `beacon_track_core.c` was written to avoid.

The beacon carries `frame_counter`, so on every re-sync the tag learns exactly how many
superframes actually elapsed. That is what makes long skips self-correcting: the
predictor is validated against ground truth at every wake, not left to drift.

### 4.2 Sizing `W` — the period estimate dominates, not the crystal

Two error sources accumulate over a skip of `K` superframes:

1. **Clock drift.** `W_drift = 2 × K × P × (ppm_tag + ppm_gw)`.
2. **Period estimate error.** `W_est = 2 × K × ε_P`, where `ε_P` is the error in `P`.

For `K = 300` (60 s) at 40 ppm total, `W_drift = 2 × 60 s × 40e-6 = 4.8 ms`. Cheap.

The second term is the one that bites. `beacon_track_core.c` estimates `P` with an EMA
over consecutive arrivals. With ±0.12 ms of arrival jitter, an EMA converges to roughly
±0.05 ms of residual error — and **that error is multiplied by K**: over 300 superframes
it becomes ±15 ms, three times the drift term and expensive to cover
(30 ms × 66 mA / 60 s = 33 µA, half the UWB budget spent on window width alone).

**Design decision: estimate `P` over the longest available baseline, not from an EMA of
consecutive arrivals.** Given two beacon arrivals separated by `n` superframes,

```
P = (t_n - t_0) / n
```

has an error of `jitter / n`. With ±0.12 ms jitter and a 100-superframe baseline,
`ε_P ≈ 1.2 µs`, so `W_est` over `K = 300` is **±0.36 ms** — an order of magnitude below
the drift term and effectively free. This is what makes deep skipping viable, and it is
a new pure module (`beacon_sched_core.c`, §9.1), not a tweak to the EMA.

Recommended window: `W = W_drift + W_est + 2 ms` fixed margin, floored at 4 ms.

### 4.3 LFCLK source is a hard prerequisite

No `CONFIG_CLOCK_CONTROL_NRF_K32SRC_*` appears in
`boards/Innovaforce/nRF52833_tag/nRF52833_tag_defconfig`, so the LFCLK source is the SoC
default and must be confirmed against the schematic.

| LFCLK | Tolerance | Max usable `listen_skip` |
|---|---|---|
| 32.768 kHz crystal | ±20 ppm | 300+ (60 s), `W ≈ 7 ms` |
| Internal RC (calibrated) | ±250 ppm | ~50 (10 s), `W ≈ 7 ms` |

With the RC oscillator, a 60 s skip needs `W ≈ 30 ms` and the IDLE tier costs ~33 µA
instead of ~7 µA. Survivable, but it caps the stationary tier at ~10 s and gives up
about a third of the UWB budget. **Confirm this before choosing the tier constants in
§5.** The tier parameters are runtime-configurable precisely so this can be measured
rather than guessed.

### 4.4 Miss handling — rungs, not a cliff

`UWB_NET_MISS_MAX = 3`. With a 60 s skip, three consecutive misses is 3 minutes before
`RESCAN`, which is acceptable — but waiting a further 60 s after each miss is not, because
a miss is evidence the prediction is wrong and the next attempt at the same width will
miss too.

On a miss, step *down* the skip ladder and widen:

| Consecutive misses | Action |
|---|---|
| 1 | halve `listen_skip`, double `W`, retry immediately |
| 2 | full-superframe window (`W = P + T_BEACON_MS`), `listen_skip = 1` |
| 3 | `UWB_ACT_TO_SCAN` → `UWB_ST_SCAN`, enter the §5 coverage ladder |

A successful beacon resets to the tier's configured `listen_skip` and recomputes `P` from
the long baseline. Retain the baseline across a single miss — the arrival that was missed
does not invalidate the arrivals that were seen.

### 4.5 The runner's sleep must be interruptible

This falls straight out of skipping, and it is easy to miss.

`uwb_radio_sleep_until()` (`src/uwb_net_runner.c:198-205`) is a plain `k_sleep()`, and
`tier_pending` is only read at the top of the loop
(`src/uwb_net_runner.c:502-509`). Today the loop turns over every 200 ms so nothing
notices. With a 60 s skip:

- A **motion event** would take up to 60 s to raise the tier — the tag would be well into
  motion before it started ranging at the moving cadence.
- A **HELP alert** (`tag_alert_raise()`, double press) would wait up to 60 s to transmit.
  For an emergency button that is a defect, not a latency figure.

**Design decision: replace the runner's `k_sleep()` with a wait on a wake signal with
timeout** (a `k_sem` given by the motion handler and by `tag_alert_raise()`). On an
early wake the runner abandons the skip, drops to `listen_skip = 1` with a full window,
re-syncs on the next beacon, and proceeds — a wasted full-window listen is the correct
price for a HELP press.

Note the interaction with the alert emission rule
(`spec/2026-08-16-uwb-help-alert-design.md` §5): in synced states the alert only fires on
an actual `UWB_EV_BEACON`, so an alert raised mid-skip costs one re-sync before it can
be sent. In `UWB_ST_SCAN` it fires on a miss too, so an out-of-coverage HELP still goes
out blind, as designed.

---

## 5. Out-of-coverage: `UWB_ST_SCAN` and the probe ladder

This is Open Work item 3 and the maintainer's own proposal ("stop listening when no
beacon for ~2 min, re-arm every 5 min"). Both are right; the sizing needs care.

### 5.1 A probe is not free

To be certain of catching a beacon the tag must listen at least one full superframe:

```
202 ms × 66 mA = 13.3 mC = 3.7 µAh per probe
```

| Probe interval | Average current |
|---|---|
| 10 s | 1 330 µA |
| 30 s | 444 µA |
| 60 s | 222 µA |
| 5 min | 44 µA (18% of the whole budget) |
| 15 min | 15 µA |
| 30 min | 7 µA |

A fixed 5-minute probe spends 18% of the total budget answering "am I home yet?". That is
why the ladder below backs off further than the original proposal, and why it is
motion-gated.

### 5.2 The ladder

Consecutive failed probes climb; **any accelerometer activity edge resets to rung 0**.
A tag leaving or entering a building is always in motion, so motion is a far better
predictor of a coverage change than elapsed time is.

| Rung | Interval | Rationale |
|---|---|---|
| 0 | 10 s | just lost the beacon; likely transient (a doorway, a body block) |
| 1 | 30 s | |
| 2 | 2 min | matches the maintainer's "no beacon for ~2 min" instinct |
| 3 | 5 min | |
| 4 | 15 min | stationary and out of coverage — a tag in a drawer |
| 5 (terminal) | 30 min | |

Average current out of coverage: ~7–15 µA at the top rungs. **Rung 0 costs ~1.3 mA**, not
the ~200 µA an earlier draft of this section claimed — 202 ms every 10 s at 66 mA. The
ladder climbs out of it in 40 seconds, so it is a transient, but it means a tag repeatedly
flapping in and out of coverage (a doorway, a loading bay) sits near rung 0 and is
expensive. If that turns out to be a real field pattern rather than a hypothetical, the
fix is to make rung 0 cheaper (a partial-superframe probe that accepts a ~50% catch
probability and retries) rather than to lengthen it — losing coverage for 30 s because the
tag was not listening is the worse failure. A tag left off-site overnight settles at rung 5
and costs less than the accelerometer does.

### 5.3 While out of coverage, do nothing else

At rung ≥2 the tag must also suppress everything that is not the probe:

- no discovery (`run_discovery()` broadcasts and holds a 15 ms window for anchors that
  are, by definition, not there),
- no BLE advertising beyond the §8.1 policy,
- no LED.

Beacon and alert behaviour are unchanged: `UWB_ACT_SEND_ALERT` still fires in
`UWB_ST_SCAN` on both `UWB_EV_BEACON` and `UWB_EV_BEACON_MISS`, so a HELP raised out of
coverage is transmitted blind on each probe. A raised alert must also pin the ladder at
rung 0 — an emergency is exactly when the tag should be trying hardest to find a network.

---

## 6. Tier parameters

### 6.1 New per-tier parameter set

Replacing `tier_cadence()`'s single number. Values below are the starting point; all are
runtime-configurable and NVS-persisted (§6.3) so they can be tuned on hardware rather
than by reflashing.

| Tier | `listen_skip` | Re-sync interval | `range_every` | Fix interval | µA |
|---|---|---|---|---|---|
| FAST (moving) | 25 | 5 s | 1 | 5 s | 258 |
| SLOW (settling) | 75 | 15 s | 1 | 15 s | 86 |
| IDLE (stationary) | 300 | 60 s | 1 | 60 s | 22 |

`range_every` is retained but defaults to 1 in every tier: with `listen_skip` doing the
work, there is no longer a reason to wake, re-sync, and then *not* range — the beacon
re-sync is the expensive part and the sweep is already paid for once you are awake. It
stays in the design for the case where the position rate must be decoupled from the seat-
maintenance rate (e.g. a very long lease that still needs periodic keepalives).

**If the LFCLK is the internal RC (§4.3), cap `listen_skip` at 50 in every tier** and
accept a 10 s stationary update; the IDLE row becomes ~110 µA and the total lands at
~290 µA — 250 mAh × 36 days, still meeting the primary goal but not the 400 mAh one.

### 6.2 Motion hysteresis

Today `uwb_set_moving()` maps the LIS2HH12 INT1 level straight onto FAST/SLOW with no
hold-off. The accelerometer's own `ACT_DUR` provides a ~5 s inactivity window, but the
*activity* edge is immediate, so a person shifting in a chair flaps the tag into FAST.

Add a **tier hold-down in `uwb_net.c`**: on entering FAST, stay there for at least
`TIER_HOLD_FAST_MS` (30 s default) regardless of further INT1 edges; on the inactivity
edge, go FAST → SLOW immediately and SLOW → IDLE after `TIER_HOLD_SLOW_MS` (60 s
default) of continued stillness. Pure logic, host-testable in `tests/uwb_net/`.

### 6.3 Persistence and commands

New NVS record, **storage id 4** (ids 1 = cal, 2 = NFC name, 3 = alert epoch). CRC-checked
and versioned like `cal_record`, so a firmware change that alters the layout falls back
to compiled defaults rather than reading garbage.

New NUS commands in `tag_cmd.c` (≤19 chars + NUL per reply, per the NUS limit):

```
pwr tier                  -> "F 25 1" / "S 75 1" / "I 300 1"   (three lines)
pwr tier f <skip> <every> -> set FAST, persist
pwr tier s <skip> <every> -> set SLOW
pwr tier i <skip> <every> -> set IDLE
pwr tier def              -> restore compiled defaults
pwr scan                  -> current ladder rung + next probe in s
```

---

## 7. Lease and keepalive — gateway contract

**This section is a contract for the gateway firmware, which is not written yet. Settle
it before that firmware is built.**

`UWB_NET_LEASE_SF = 50` superframes = 10 s (`src/uwb_net.h:9`), renewed when
`lease_remaining <= LEASE_SF/2`, i.e. every ~25 superframes / 5 s. A tag with
`listen_skip = 300` wakes once per 60 s and cannot renew a 10 s lease. It would lose its
seat on every skip and pay a full JOIN + GRANT + re-discovery cycle each time — which
costs more than the skip saves.

### 7.1 The tag declares its skip factor

The JOIN and KEEPALIVE frames already carry a `tier` byte
(`uwb_frame_join_build`, `uwb_frame_keepalive_build`). Redefine it, or add a field, so the
tag declares the `listen_skip` it intends to use.

### 7.2 The gateway sizes the lease from the declaration

```
lease_sf = max(UWB_NET_LEASE_SF, 3 × declared_listen_skip)
```

Three skips of slack tolerates two consecutive missed re-syncs (§4.4) before the seat is
reclaimed — which is exactly the point at which the tag gives up and goes to `SCAN`
anyway. The two timeouts should expire together, not fight each other.

### 7.3 Slot map implications

The gateway must keep a tag's slot reserved across its skip, so the beacon slot map is
sparse in time: most superframes will have no tag transmitting in most CFP slots. That is
already true (the CFP is sized for 11 seats and the deployment runs 3), but it becomes the
normal case rather than the exception. Capacity is unchanged — the seat count still bounds
the tag count, not the airtime.

### 7.4 Interim behaviour before the gateway implements this

Until the gateway is updated, cap `listen_skip` at `UWB_NET_LEASE_SF / 2 = 25`
(5 s), which the current lease already tolerates. That is the FAST tier value in §6.1, so
**FAST works against today's gateway and SLOW/IDLE do not.** Ship the tag-side work behind
that cap and lift it when the gateway lands. This is a real constraint on the rollout
order, not a footnote.

---

## 8. Peripheral and platform items

### 8.1 BLE advertising

`BT_LE_ADV_CONN_FAST_2` (`src/ble_log.c:37` and `:110`) is 100–150 ms, connectable, and
runs forever. That is ~150 µA — 60% of the entire target — for a channel that is used
only for diagnostics and configuration. Positions already reach the gateway over the
`0xEA` POS frame, so nothing operational depends on BLE.

Policy:

- Advertise at `BT_GAP_ADV_SLOW_INT_MIN`/`MAX` (1–1.2 s) by default: ~20 µA.
- **Event-driven window:** advertise for 60 s after a button press or an NFC field event,
  then stop entirely (~0 µA). Restart on the next press.
- Keep advertising continuously while `dw_sleep_enabled == false` (i.e. `pwr sleep off`),
  so bench debugging is unaffected.

The 60 s window must be discoverable without a manual, since the readout gestures are
already documented on the button: the single press that shows battery colour also opens
the BLE window.

### 8.2 Accelerometer ODR

`motion.c` sets `LIS2HH12_XL_ODR_50Hz` (~180 µA). Drop to `LIS2HH12_XL_ODR_10Hz`
(~50 µA), saving ~130 µA — over half the design target, for a one-line change.

**`ACT_DUR` must be recomputed at the same time.** Its LSB is `8 / ODR` seconds:

| ODR | `ACT_DUR` LSB | `ACT_DUR = 31` gives |
|---|---|---|
| 50 Hz | 0.16 s | ~5.0 s (intended) |
| 10 Hz | 0.80 s | **~24.8 s** (wrong) |

For the same ~5 s inactivity window at 10 Hz, `ACT_DUR = 6` (4.8 s). Changing the ODR
without this makes the tag take 25 s to notice it has stopped moving, which silently
costs ranging cycles at the FAST cadence — the opposite of the intent.

`ACT_THS` is unaffected: its LSB is `FS/128`, independent of ODR.

### 8.3 External LNA rail (LDO2, 1.8 V)

LDO2 supplies the RX-only LNA. `dwt_setlnapamode(DWT_LNA_ENABLE)` puts its enable under a
DW3000 GPIO (EXTRXE), so it *should* be duty-cycled with RX — but the DW3000's GPIO output
state across `dwt_entersleep()`/wake is not documented in a way worth trusting, and the
existing `dw_wake()` comment (`src/uwb_net_runner.c:416-424`) already records that GPIO
mode is not restored by AON or `dwt_restoreconfig()`. A UWB LNA left enabled draws roughly
5–15 mA; if that is happening during SLEEP it is a large share of the present 22 mA.

**Measure the LDO2 rail current with the DW3000 in SLEEP before implementing anything
here** (§10.1). Regardless of the result, the LNA is only needed while the receiver is on,
so gate the rail explicitly:

```
dw_enter_sleep()  →  regulator_disable(LDO2)
dw_wake()         →  regulator_enable(LDO2), settle, then dwt_setlnapamode(DWT_LNA_ENABLE)
```

LDO1 (2.5 V) stays up — it is the DW3000's own supply and cutting it loses the SLEEP
config retention that `dwt_configuresleep(DWT_CONFIG | DWT_PGFCAL, ...)` depends on.

The rail enable must sit immediately before the existing `dwt_setlnapamode()` call, and
its settle time is added to `DW_WAKE_GUARD_MS`. **The two `dwt_setlnapamode()` call sites
(`src/uwb.c` at init and `dw_wake()`) must stay identical** — that invariant is already
documented and this change must not break it.

### 8.4 TWR timing

A 4-anchor sweep takes ~22 ms but the air time is roughly 10 ms. The rest is slack in
`POLL_TX_TO_RESP_RX_DLY_UUS = 1000` and `RESP_RX_TIMEOUT_UUS = 2000`
(`src/uwb_net_runner.c:53-55`) — the receiver sits on at 66 mA waiting through it.

Measure the actual poll-to-response turnaround at the anchor and set both to that plus
margin. A plausible outcome is 22 ms → ~16 ms, a ~27% cut in the dominant per-fix cost at
zero link-budget cost. `PRE_TIMEOUT = 128` symbols is already doing its job (it is what
makes a *missing* anchor cheap) and should be left alone.

This is the one item that must not be over-tightened: too short a `RESP_RX_TIMEOUT_UUS`
turns a marginal-SNR anchor into a missing one, and losing an anchor costs a position fix.
Change it in one step, verify `n=4` still holds over a few hundred sweeps, and stop.

### 8.5 Housekeeping

- **Battery sampler at 5 s** (`batt.c`) wakes the MCU 12×/min for an I2C transaction to
  read a quantity that moves over hours. Raise to 60 s. The BLE-idle current window keeps
  its resolution because it accumulates per sample, not per second.
- **`CONFIG_PM_DEVICE=y`.** `CONFIG_PM=y` is set but not `PM_DEVICE`, so SPIM1, SPIM3 and
  TWIM0 are never suspended and the `pinctrl-1 = <&..._sleep>` states already defined in
  the board DTS are never applied. Enable it and add runtime PM to SPI3 (WS2812) and I2C0.
- **`task_wdt` hardware fallback.** `TASK_WDT_MIN_TIMEOUT = 100` + `HW_FALLBACK_DELAY = 20`
  arms the hardware WDT at ~120 ms, fed from a kernel timer — a wake roughly every 120 ms,
  forever. Small in absolute terms but it is the highest-frequency periodic wake in the
  system once the runner starts skipping. Review whether the fallback period can be
  lengthened without losing the halt-detection property that made it valuable during the
  calibration investigation (see Open Work item 2 — `rst` = `dog` with a sub-2 s return is
  the signature of a halt, and that diagnostic must survive).

---

## 9. New modules

Following the repo's split: logic in a pure `*_core.c` with a host test, Zephyr glue
separate.

### 9.1 `src/beacon_sched_core.c/h` — long-baseline scheduler

Replaces the prediction role of `beacon_track_core.c`. Pure; host-tested in
`tests/beacon_sched/`.

```c
struct beacon_sched;

void beacon_sched_reset(struct beacon_sched *s, uint32_t nominal_period_ms);

/* Feed an observed beacon: arrival time in the tag's clock and the gateway's
 * frame counter (which gives the true elapsed superframe count, even across a
 * skip). Updates the long-baseline period estimate. */
void beacon_sched_observe(struct beacon_sched *s, uint32_t t_ms, uint32_t frame_ctr);

/* Feed a missed re-sync: steps down the skip ladder and widens the window. */
void beacon_sched_miss(struct beacon_sched *s);

/* Plan the next wake. `skip` is the tier's configured listen_skip; the module may
 * return a smaller effective skip after a miss. */
void beacon_sched_plan(const struct beacon_sched *s, uint32_t skip,
                       uint32_t *arm_ms, uint32_t *window_ms,
                       uint32_t *effective_skip);
```

Test cases worth pinning:

- Period estimate converges to the true period with injected jitter, and its error scales
  as `1/n` with baseline length (this is the whole point of the module).
- A skip of `K` superframes produces a window that covers the worst-case drift for the
  configured ppm, and no more.
- Frame-counter arithmetic is correct across a `uint32_t` wrap.
- The miss ladder halves the skip and doubles the window, and a success restores the tier
  value.
- The baseline survives a single miss (arrivals seen before the miss stay usable).

`beacon_track_core.c` keeps its ACQUIRING/TRACKING role for the initial lock; the two are
complementary — `beacon_track` gets the tag synced, `beacon_sched` keeps it synced across
skips. Do not merge them: the acquisition FSM and the extrapolation math have different
failure modes and separate host tests.

### 9.2 `src/scan_backoff_core.c/h` — coverage probe ladder

Pure; host-tested in `tests/scan_backoff/`.

```c
struct scan_backoff;
void     scan_backoff_reset(struct scan_backoff *b);   /* -> rung 0 */
void     scan_backoff_fail(struct scan_backoff *b);    /* probe found no beacon */
void     scan_backoff_motion(struct scan_backoff *b);  /* activity edge -> rung 0 */
void     scan_backoff_alert(struct scan_backoff *b, bool active); /* pin at rung 0 */
uint32_t scan_backoff_next_ms(const struct scan_backoff *b);
uint8_t  scan_backoff_rung(const struct scan_backoff *b);
```

Test cases: the ladder climbs on consecutive failures and saturates at the terminal rung;
motion resets it from any rung; an active alert pins rung 0 and ignores `fail()`; clearing
the alert releases the pin without jumping rungs.

### 9.3 Changes to existing pure modules

- `src/uwb_net.c` — per-tier `{listen_skip, range_every}` table replacing
  `tier_cadence()`; the §6.2 hysteresis; `UWB_ACT_SLEEP` emitted from `UWB_ST_SCAN`.
  Extend `tests/uwb_net/`.
- `src/uwb_net.h` — the tier parameter struct and the new action semantics.

---

## 10. Verification

### 10.1 Measurement baseline — do this first

Firmware estimates are not measurements, and the nPM1304 fuel-gauge path
(`batt_read_soc()`, `pwr idle`) has nowhere near µA resolution. Use a PPK2 or a source
meter in series with the cell.

Three numbers are needed before any code changes, because they determine whether §8.3 is
worth doing and whether §2's model is right:

1. **Total current, tag idle, DW3000 in SLEEP** (`pwr sleep on`, tag out of coverage after
   backoff). Expect ~1–2 mA today; this is the number the whole plan moves.
2. **LDO2 (1.8 V LNA rail) current with the DW3000 in SLEEP.** If it is not ~0, §8.3 is a
   large free win and should be promoted to the front of the plan.
3. **Total current with the LIS2HH12 held in power-down.** Confirms the 50 µA figure
   against this board rather than against the datasheet.

### 10.2 Acceptance criteria

| # | Criterion | How |
|---|---|---|
| 1 | Stationary, in coverage, IDLE tier: **≤ 60 µA** average over 10 min | PPK2 |
| 2 | Moving, FAST tier, 5 s fixes: **≤ 400 µA** average over 10 min | PPK2 |
| 3 | Out of coverage, stationary, ladder saturated: **≤ 80 µA** | PPK2 |
| 4 | Realistic mixed profile (20% motion, in coverage): **≤ 250 µA** | PPK2, 1 h |
| 5 | Position fixes still land at the configured cadence with `n=4` | `P:` lines over NUS |
| 6 | No `RESCAN miss` in 1 h stationary at IDLE tier | NUS log |
| 7 | HELP press transmits within **2 s** from any tier | anchor/gateway RX log |
| 8 | Motion → FAST transition within **2 s** of the INT1 edge | `P:` cadence change |
| 9 | Re-acquisition after 60 s skip succeeds ≥ 99% of attempts | new `pwr sched` counters |

Criteria 6, 7 and 8 are the ones that catch a too-aggressive design. A tag that meets the
current targets and fails those is worse than the tag we have.

### 10.3 New instrumentation

Extend the `pwr` command family (`tag_cmd.c`), keeping each NUS line ≤19 chars:

```
pwr sched    -> "P 195.02 W 7"      period estimate (ms, 2 dp) + current window (ms)
                "K 300 M 0"         effective skip + consecutive misses
                "OK 412 MS 3"       successful re-syncs / misses since reset
pwr schedrst -> reset those counters
pwr scan     -> "R 4 T 812"         ladder rung + seconds to next probe
```

`pwr rx` and `pwr rxrst` keep their existing meaning and remain the primary way to
confirm the RX-on duty cycle actually fell.

---

## 11. Risks

| Risk | Mitigation |
|---|---|
| Gateway lease contract (§7) is not implemented, capping `listen_skip` at 25 | Ship behind the cap; the tier parameters are runtime-settable so the cap lifts without a reflash |
| LFCLK is the internal RC, halving the usable skip | §4.3; measure early, adjust tier constants, target still met on 250 mAh |
| Long skips destabilise re-acquisition, causing `RESCAN` churn that costs more than the skip saved | Criterion 6 and 9; the miss ladder (§4.4) degrades gracefully rather than dropping straight to SCAN |
| §8.4 timing tightening turns marginal anchors into missing ones | One step, verify `n=4` over hundreds of sweeps, revert if `n` drops |
| Interruptible sleep (§4.5) introduces a race with `uwb_radio_owner` handover | The wake signal only shortens a sleep; it never changes radio ownership. The handover check at the top of `runner_fn` still runs first |
| Alert latency regresses silently as tiers are tuned in the field | Criterion 7 is a hard acceptance test, and the alert pins the scan ladder at rung 0 (§5.3) |

## 12. Open questions

1. **LFCLK source** — crystal or RC? Blocks the §6.1 constants. Schematic check.
2. **LDO2 during DW3000 SLEEP** — is the LNA actually off? Blocks §8.3's priority.
3. **Anchor poll-to-response turnaround** — the real number, for §8.4.
4. **Motion duty cycle in the field** — the 20% assumption in §1.1 is a guess. The tag can
   measure it: count FAST-tier superframes over a day and report it. Worth adding to
   `pwr` before tuning the tier constants against a hypothetical.
