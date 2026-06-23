# Power Saving — Layer 1 (intra-superframe DW3000 sleep + SoC PM) — Design

**Date:** 2026-06-23
**Status:** Design approved, pre-implementation
**Depends on:** the TDMA runner (`src/uwb_net_runner.c`), the DW3000 HAL (`platform/port.c`,
`platform/deca_spi.c`), the fuel gauge (`src/batt.c`), and the calibration record in NVS
(`src/cal.c` — antenna delays re-applied after wake).
**Scope:** Reduce average current during **normal operation** without changing the TDMA
on-air contract or touching the gateway/anchor firmware. Layer 1 = deep-sleep the DW3000
*between the activity of each superframe* (both motion tiers), let the SoC enter PM idle,
keep the LED dark, and add current monitoring. **Out of scope (Layer 2):** sleeping across
multiple superframes when stationary, re-JOIN/keepalive coordination with the gateway, and
beacon-window RX optimization.

---

## 1. Motivation & target

Measured baseline: **~35 mA** with the LED on (read via BQ27421 over BLE on button press).
The dominant fixed cost is the **DW3000 left in IDLE (~12–18 mA) for the full ~200 ms
superframe**, even while moving. PANS 2.0 (DWM1001) shows the right model: the tag is
*asleep by default* and awake only for its slot — 1 TX poll → 4 RX → 1 TX → compute →
sleep — reaching ~12–15 µA deep sleep. Layer 1 brings our tag toward that model **within**
each superframe: awake ≈ beacon window + sweep (~25–30 ms), asleep ≈ ~170 ms of every
200 ms. No change to ranging rate (moving stays 5 Hz).

**Success criteria:** average current during disconnected normal operation drops
substantially vs the 35 mA baseline (target: low single-digit mA average), with **no
increase** in `RESCAN`/lost-sync rate and the BLE debug link surviving sleep cycles.

---

## 2. What changes vs today

`runner_fn` in [`src/uwb_net_runner.c`](../src/uwb_net_runner.c) today loops:
inject tier → RX beacon (re-arm loop) → range in slot → `uwb_radio_sleep_until(t0 +
T_SUPERFRAME_MS)`. During that final slack the **DW3000 sits in IDLE** and the SoC does not
reach deep PM idle.

| Reused as-is | New |
|---|---|
| Beacon RX + re-arm loop (TDMA sync unchanged) | DW3000 enters **SLEEP** during the inter-activity slack |
| Slot scheduling / sweep / keepalive | Timed **wake with guard** before the next beacon RX |
| `uwb_radio_sleep_until()` slack window | SoC **PM idle** (`CONFIG_PM`) during that slack |
| `cal` antenna delays in NVS | Config re-applied on wake (what SLEEP does not retain) |

Removed: the persistent status LED (boot cyan/green, BLE green/blue). LED is button-only
(battery display in `tag_ui`), **except** a fatal DW3000-init failure still latches **red**
(the only signal of a dead tag with no BLE).

---

## 3. Components

### 3.1 DW3000 SLEEP between superframe activity

Core of the saving. After the slot's UWB activity completes (sweep/keepalive done) and
before the inter-superframe slack:

1. **Enter sleep:** `dwt_configuresleep(...)` (preserve config on wake) + `dwt_entersleep()`.
   Use **SLEEP**, not DEEPSLEEP — faster wake (~ms), less reconfiguration, lower risk of
   missing the next beacon.
2. **Sleep the slack:** SoC enters PM idle for `(t0 + T_SUPERFRAME_MS − GUARD_MS − now)`.
3. **Wake with guard:** assert the DW3000 **WAKEUP pin (P0.15)** via the existing HAL in
   [`platform/port.c`](../platform/port.c), wait for IDLE_RC, **verify DEVID**, and
   **re-apply** config that SLEEP does not retain — including the **antenna delays from the
   cal record** (`dwt_setrxantennadelay`/`dwt_settxantennadelay`) and PHY config.
4. Enter the beacon RX as today.

`GUARD_MS` is a tunable constant sized to the measured wake+reconfig latency plus margin.
Bracket the sleep/wake `dwt_*` calls with `decamutexon()/decamutexoff()` per the existing
critical-section pattern.

**Risk:** wake latency pushes beacon RX late → missed beacon → `RESCAN`. Mitigation:
`GUARD_MS` margin + watch the existing `RESCAN` telemetry; tune or fall back to no-sleep via
the toggle (§3.4) if regression appears.

### 3.2 SoC Power Management

- Enable `CONFIG_PM=y` in `prj.conf` so the `k_sleep` slack reaches nRF52833 System ON deep
  idle instead of staying active. Evaluate `CONFIG_PM_DEVICE=y` to suspend the SPI1
  peripheral while the DW3000 sleeps (additional, optional saving).
- **Design invariant (the BLE fear):** SoC PM idle must **not** drop the BLE link — the BLE
  controller wakes the SoC for connection events on its own. Verified in §5.

### 3.3 LED policy

- LED **off** in all normal operation (remove boot and BLE-state LED writes from
  [`src/main.c`](../src/main.c) and the `on_ble_state` LED).
- LED on **only**: button-driven battery display (already in `tag_ui`) and the **fatal
  DW3000-init-failure red latch**.

### 3.4 Monitoring (A + B + PPK2)

- `batt.c`: add `batt_read_current()` reading `SENSOR_CHAN_GAUGE_AVG_CURRENT` from the
  bq274xx driver (negative = discharge; report magnitude in mA).
- **(A) Live deltas over BLE:** report `I:<mA>` alongside the existing `BATT:` line, and add
  a NUS command **`pwr sleep on|off`** to toggle §3.1 at runtime so the DW3000-sleep delta
  can be measured without reflashing.
- **(B) Disconnected-window average:** sample `AverageCurrent` at a low rate (reuse the
  `batt` `k_timer`) into a min/mean/max accumulator that **resets on BLE connect**; on
  reconnect, emit `Idle avg/min/max mA` for the BLE-off window — the real field current.
- **(C) PPK2:** document the procedure to validate the µA floor in SLEEP (the BQ averages
  ~1 s at ~mA resolution and cannot resolve deep-sleep µA).

---

## 4. Files touched

| File | Change |
|---|---|
| [`src/uwb_net_runner.c`](../src/uwb_net_runner.c) | DW3000 SLEEP/wake around the inter-superframe slack; `GUARD_MS`; runtime sleep toggle |
| [`platform/port.c`](../platform/port.c) | Confirm/extend WAKEUP-pin assert + wake-settle/DEVID-verify helper |
| [`src/main.c`](../src/main.c) | Remove boot/BLE status-LED writes; keep red fatal-fail latch |
| [`src/batt.c`](../src/batt.c) / `batt.h` | `batt_read_current()`; disconnected-window accumulator |
| [`src/tag_ui.c`](../src/tag_ui.c) | Append `I:<mA>` to the battery report |
| NUS command handler | `pwr sleep on\|off`, `pwr` status |
| `prj.conf` | `CONFIG_PM=y` (+ evaluate `CONFIG_PM_DEVICE=y`) |
| `docs/` | PPK2 measurement procedure |

---

## 5. Verification (hardware)

1. **Baseline vs sleep:** with a phone connected, toggle `pwr sleep off`/`on` and read live
   `I:` (A) — confirm the DW3000-sleep delta.
2. **Field current:** disconnect, operate, reconnect, read the disconnected-window
   `avg/min/max mA` (B). This is the headline number vs 35 mA.
3. **µA floor:** PPK2 capture across a superframe (C) — confirm SLEEP reaches µA between
   activity.
4. **BLE survives sleep:** hold a NUS session open across many superframes/sleep cycles;
   the link must not drop.
5. **No TDMA regression:** `RESCAN`/lost-sync rate must not rise vs the no-sleep build;
   `P:`/position output cadence unchanged when moving.

Host tests (`tests/uwb_net`, `tests/uwb_frame`, `tests/cal_math`) are unaffected — the TDMA
logic and frame layout do not change; sleep/wake is target-only HAL.

---

## 6. Open risks / notes

- If `GUARD_MS` cannot be made small enough to reliably catch the beacon, the fallback is to
  shorten the sleep (sleep less of the slack) rather than abandon it — still a net win.
- `CONFIG_PM_DEVICE` interactions with the WS2812 SPI3 and I2C0 gauge must be checked (they
  must not be left in a state that blocks idle or corrupts a transfer).
- Antenna-delay re-application after wake is **mandatory** — skipping it silently degrades
  ranging accuracy rather than failing loudly.
