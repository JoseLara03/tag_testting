# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Zephyr RTOS firmware for a custom IoT tracking tag based on the Nordic nRF52833 SoC (Arm Cortex-M4, 128 KB RAM, 512 KB Flash). The tag integrates Ultra-Wideband (DW3000 via SPI1), BLE NUS, NFC Tag Type 4 (T4T), a WS2812 RGB LED, a button, a LIS2HH12 accelerometer (I2C0), and a BQ274xx fuel gauge (I2C0).

Board definition: `boards/Innovaforce/nRF52833_tag/`  
Application entry point: `src/main.c`

## Build & Flash

The user builds and flashes the firmware themselves and will report any errors or issues.

## Source Layout

```
src/
  main.c                — bring-up: LED, BLE NUS, DW3000 init, cal load, UWB ranging start
  ble_log.c/h           — BLE NUS wrapper (TX-only; blocks until notifications enabled; retries on -ENOMEM)
  nfc_tag.c/h           — NFC T4T emulation; default URI https://google.mx; forwards phone writes via ble_log_send()
  uwb.c/h               — DW3000 init with retry logic; exposes uwb_get_dev_id()
  batt.c/h              — BQ274xx periodic read (k_timer 10 s → k_work); sends "BATT: xxmV xx%\n" over BLE NUS
  lis2hh12_if.c/h       — LIS2HH12 I2C callback registration
  uwb_ss_initiator.c/h  — SS-TWR initiator: do_one_range_anchor(), position_publish(), twr_log() + 8-slot msgq
  uwb_net_runner.c/h    — Dynamic anchor selection runner: run_discovery(), anchor_sweep(), uwb_radio_ops impl
  uwb_net.c/h           — MAC FSM: state machine driving DISCOVER / SWEEP / JOIN / KEEPALIVE actions
  uwb_frame_802_15_4z.c/h — Frame builders/parsers: E0 poll, E1 ranging resp, E2 discovery, E4 disc-resp, E5 beacon
  pos_solver.c/h        — 2D trilateration via CMSIS-DSP least-squares; pos_solve() returns x/y in metres + residual_m
  pos_residual.c/h      — RMS range residual for a solved fix; split out because pos_solver.c needs CMSIS and can't compile on the host; host-tested in tests/pos_residual/
  cal.c/h               — Antenna delay calibration: NVS persistence, cal command handler
  cal_math.c/h          — Calibration math: outlier rejection (median/MAD), delay correction
  rx_stats_core.c/h     — Pure beacon-RX duty-cycle accumulator (on-duration + folded arrival offset); host-testable
  rx_stats.c/h          — Zephyr glue for rx_stats_core (k_cycle stamps); feeds the `pwr rx` report
  beacon_track_core.c/h — Pure EMA beacon-period predictor + ACQUIRING/TRACKING FSM for the narrow RX window; host-testable
  tag_cmd.c/h           — BLE NUS command dispatcher: `pwr ...` power commands, else forwards to cal_on_rx
  phy_config.h          — PHY constants: TX_ANT_DLY, RX_ANT_DLY seed (16371), channel/preamble settings
platform/
  port.c/h              — DW3000 GPIO/reset/wakeup HAL; DW3000_IRQ_Pin=30, RST=37, WUP=15
  deca_spi.c/h          — SPI1 driver (4 MHz init → 32 MHz fast); static EasyDMA RX buffer
  deca_probe_interface.c/h — dwt_probe_s function pointers for Decawave library
  deca_mutex.c          — decamutexon/off critical sections around UWB ISR
  deca_sleep.c          — deca_sleep (ms) / deca_usleep (µs) wrappers
drivers/
  lis2hh12-pid/         — ST register-map driver (lis2hh12_reg.c/h)
Shared/
  dwt_uwb_driver/       — Decawave precompiled library (libdwt_uwb_driver-m4-sfp-6.0.7.a) + headers
```

New source files must be added to `CMakeLists.txt` via `target_sources(app PRIVATE ...)`.  
The Decawave library exposes a custom linker section; `dw_drivers.ld` places it in Flash, and `zephyr_ld_options(-Wl,--undefined=dw3000_driver)` in `CMakeLists.txt` pulls the driver registration object into the link (see Key Patterns).

Note: `nfc_tag.c`, `lis2hh12_if.c`, and `drivers/lis2hh12-pid/lis2hh12_reg.c` are still listed in `CMakeLists.txt` and compile into the image, but `main.c` does not invoke them — they are dormant code awaiting reintegration. (`batt.c` is now active: `batt_monitor_start()` runs from `main.c` and feeds the `pwr idle` current-window report.)

## Hardware Peripherals

| Peripheral | Pins | DTS Alias / Node | Purpose |
|---|---|---|---|
| SPI1 | SCK P0.4, MISO P0.5, MOSI P1.9, CS P0.11 | — | DW3000 UWB transceiver |
| SPI3 | MOSI P0.20 | `led-strip` | WS2812 RGB LED (driven by `led_strip` Zephyr driver) |
| I2C0 | SDA P0.2, SCL P0.3 | — | LIS2HH12 accel (addr 0x1E) + BQ274xx fuel gauge (addr 0x55) |
| GPIO P0.17 | — | `button0` | User button (interrupt on both edges; long-press ≥3 s) |
| GPIO P0.30 | — | `led0` | Status LED |
| NFCT | P0.09 (NFC1), P0.10 (NFC2) | `nfct` | NFC T4T antenna (dedicated analog pins, auto-reserved by `CONFIG_NFC_T4T_NRFXLIB=y`) |

Console/UART/logging are **disabled** (`CONFIG_SERIAL=n`, `CONFIG_LOG=n`). BLE NUS is the only debug channel.

## Active Zephyr Configuration (prj.conf)

Key configs currently enabled:
- `CONFIG_BT=y`, `CONFIG_BT_PERIPHERAL=y`, `CONFIG_BT_NUS=y` — BLE peripheral + Nordic UART Service
- `CONFIG_NFC_T4T_NRFXLIB=y`, `CONFIG_NFC_NDEF=y`, `CONFIG_NFC_NDEF_MSG=y`, `CONFIG_NFC_NDEF_RECORD=y`, `CONFIG_NFC_NDEF_URI_REC=y`, `CONFIG_NFC_NDEF_URI_MSG=y`, `CONFIG_NFC_NDEF_PARSER=y` — NFC T4T + NDEF encode/parse
- `CONFIG_LED_STRIP=y`, `CONFIG_WS2812_STRIP_SPI=y` — RGB LED
- `CONFIG_SENSOR=y`, `CONFIG_I2C=y`, `CONFIG_FUEL_GAUGE=y`, `CONFIG_BQ274XX=y` — fuel gauge (BQ274xx is a sensor driver in nCS 3.2.4; requires `CONFIG_SENSOR=y`)
- `CONFIG_ARM_MPU=y`, `CONFIG_HW_STACK_PROTECTION=y`

Board-level defaults live in `boards/Innovaforce/nRF52833_tag/nRF52833_tag_defconfig`.  
To enable a new subsystem, add the `CONFIG_*` line to `prj.conf`.

## Application Behavior (main.c)

Full UWB ranging + dynamic anchor selection system:

1. Get WS2812 RGB LED device; hang on failure.
2. `ble_log_init()` — starts BLE advertising.
3. `ble_log_wait_ready()` — blocks until BLE central enables NUS TX notifications.
4. LED **cyan** — initializing DW3000.
5. `uwb_init(3)` — three retries; sends `probe fail N/3` or `init fail N/3` on each failure, `OK ID=0x........` on success, `all 3 fail` on total failure.
6. On failure: LED **red**, send `DW3000: init failed\n` over NUS, hang.
7. Load calibration from NVS — sends `CAL loaded` or `CAL REQUIRED` (fires before BLE connects; use `cal status` to read after connecting).
8. `uwb_ss_initiator_start()` — starts the SS-TWR thread; also starts `uwb_net_runner_start()` which runs the dynamic anchor selection loop.
9. LED **green** — ranging active. BLE NUS output is `P:x.xx,y.yy\n` once per superframe when ≥3 anchors are visible and calibration is loaded.

NFC, battery, accelerometer, and button handling are not initialised.

## Key Patterns

- **BLE NUS transmit:** `ble_log_init()` → `ble_log_wait_ready()` → `ble_log_send(msg)`. The wait blocks on a semaphore until the central enables notifications; do not call `ble_log_send` before that. `ble_log_send` retries internally on `-ENOMEM` (TX buffer momentarily exhausted by back-to-back sends).
- **NUS 20-byte payload limit:** Default ATT MTU is 23, so a single notification carries at most 20 bytes. `bt_nus_send` does not auto-fragment — payloads >20 bytes fail with `-EMSGSIZE` and are silently dropped. Keep diagnostic strings short (e.g. `"probe fail 1/3\n"`, `"ID=0xDEADBEEF\n"`), or initiate an MTU exchange before transmitting long messages.
- **Linking the precompiled Decawave driver:** `libdwt_uwb_driver-*.a` registers per-chip drivers (DW3000/DW3700/DW3720) into the `.dw_drivers` linker section via static objects. Nothing in app code references those symbols, so the linker would skip the .o files. `zephyr_ld_options(-Wl,--undefined=dw3000_driver)` in `CMakeLists.txt` forces `dw3000_device.c.obj` into the link, populating the section so `dwt_probe()` can find a matching driver. Do **not** use `-Wl,--whole-archive` — CMake 3.21 (nCS 3.2.4) strips the matching `--no-whole-archive` as a duplicate and the flag leaks into picolibc/libgcc, causing multiple-definition errors on `calloc`/`free`/`malloc`/`__retarget_lock_*`/etc.
- **GPIO interrupts:** Use `gpio_pin_interrupt_configure_dt()` with `GPIO_INT_EDGE_BOTH`; never use the button subsystem — GPIO is managed manually to avoid conflicts with the NUS setup.
- **SPI rate switching:** Call `openspi()` / `closespi()` in `deca_spi.c` when changing between init (4 MHz) and fast (32 MHz) modes; the static RX buffer is required for EasyDMA.
- **DW3000 critical sections:** Always bracket Decawave library calls that touch the IRQ with `decamutexon()` / `decamutexoff()`.
- **Adding a peripheral to I2C0:** Add the node under `&i2c0` in `nRF52833_tag.dts`; the bus is already enabled with the correct pins.
- **BQ274xx fuel gauge:** In nCS 3.2.4 the driver lives under `drivers/sensor/ti/bq274xx/` and implements the **sensor API**, not the fuel_gauge API. Use `sensor_sample_fetch(dev)` then `sensor_channel_get(dev, SENSOR_CHAN_GAUGE_VOLTAGE, &val)` and `sensor_channel_get(dev, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE, &val)`. Voltage: `val.val1` = V, `val.val2` = µV fractional → mV = `val1 * 1000 + val2 / 1000`. SoC: `val.val1` = %. The driver requires `CONFIG_SENSOR=y`; `CONFIG_FUEL_GAUGE=y` and `CONFIG_BQ274XX=y` are in the board defconfig. The gauge is not ready without a battery connected — `device_is_ready()` returns false on USB-only power. Battery parameters (DTS node `fuel_gauge: bq274xx@55` under `&i2c0`): design-voltage=4200 mV, design-capacity=400 mAh, taper-current=40 mA, terminate-voltage=3000 mV.
- **NFC T4T:** `nfc_tag_init()` encodes a default URI, registers an RW payload buffer, and starts emulation. The `NFC_T4T_EVENT_NDEF_UPDATED` callback runs in the nrfxlib NFC thread (not ISR) — `ble_log_send()` is safe to call from it. Writes before BLE connects are silently dropped. The NFCT node must be enabled in the board DTS (`&nfct { status = "okay"; }`) for `HAS_HW_NRF_NFCT` to be set, which is required for `NFC_PLATFORM` and `NRFX_NFCT` to build. URI prefix codes follow NFC Forum RTD: 0x03 = `"http://"`, 0x04 = `"https://"` (already includes `://`). Do not pass `"//host"` as the URI string when using these prefix codes.
- **Dynamic anchor selection:** `uwb_net_runner.c` maintains an anchor pool (up to 6 entries) with EMA-filtered CIR quality scores. `run_discovery()` broadcasts an E2 frame and collects E4 DISCOVERY_RESPONSE frames within a `DISCOVERY_WINDOW_MS` window; `anchor_pool_rebuild_selected()` picks the top 3–4 anchors by EMA score into `selected[]`. `anchor_sweep()` ranges only those anchors via SS-TWR. Re-discovery runs every 10 superframes or when fewer than 3 anchors respond. `last_sweep_n` must be primed to the discovery count after the first discovery so the sweep gate (`last_sweep_n < ANCHOR_SELECT_MIN`) does not immediately re-trigger discovery.
- **Discovery window sizing:** The anchor firmware delays its E4 response by `DISC_BASE_UUS + anchor_id × DISC_SLOT_UUS` (currently 2000 + id × 3500 µs). `DISCOVERY_WINDOW_MS` in `uwb_net_runner.c` must exceed the highest anchor's slot: for anchor IDs 0–3 the maximum is 12.5 ms, so `DISCOVERY_WINDOW_MS = 15`. Formula: `ceil((DISC_BASE_UUS + max_id × DISC_SLOT_UUS) / 1000 + 2_ms_margin)`. If anchors with higher IDs are added, increase this constant.
- **BLE message queue budget:** `ss_twr_msgq` has 8 slots (K_NO_WAIT — overflow is silent). A full sweep of 4 anchors produces at most 4 ranging messages + 1 position = 5 slots; stay well under 8. Do not add per-anchor diagnostic lines during sweep — they consume slots and can starve the `P:` position output.
- **Position output:** `position_publish(pos, n_anchors, src_addr)` in `uwb_ss_initiator.c` formats and enqueues `P:x.xx,y.yy\n` via `twr_log()`, and also transmits a `0xEA` POS frame to the gateway (`uwb_frame_pos_build()`), carrying `pos->residual_m` and `batt_soc_cached()`. It takes no `uwb_radio_owner` claim: it runs inside the runner's own sweep, which already owns the radio, and would be claiming from itself otherwise. Called from `uwb_net_runner.c` after a successful `pos_solve()`. Output appears only when ≥3 anchors respond and calibration is loaded. `pos_solve()` uses CMSIS-DSP `arm_mat_inverse_f32`; returns false on singular matrix (anchors collinear or sharing coordinates).
- **Anchor coordinates:** Each anchor stores `anchor_x`, `anchor_y`, `anchor_z` in flash (address `0x1E000`, `flash_record_t` with magic + CRC). Set via anchor console: `set anchor_x <float>`, `set anchor_y <float>`, then `save`. Values are written into E1 ranging response bytes 19–22 (X) and 23–26 (Y) as IEEE 754 float32 LE. Anchors must be non-collinear — if any two share (0,0) or all three are on the same line, `pos_solve` returns false.
- **Multi-tag (validated):** Multiple tags operating simultaneously is implemented and hardware-validated (tested up to 3 tags). Each tag JOINs the gateway, is GRANTed a distinct CFP slot, and ranges **only within its slot** — the TDMA time separation solves RX cross-contamination during ranging by construction (no per-tag frame filtering needed). Two fixes were required: **(1) beacon re-arm loop** (`runner_fn` step 2): `uwb_radio_rx_beacon()` returns on the first frame of ANY type, so with multiple tags another tag's poll/response/keepalive was misread as a beacon miss and `MISS_MAX` in a row bounced the tag to SCAN (`RESCAN miss`). The runner now re-arms RX, discarding non-beacon frames, until the real beacon arrives or the superframe budget elapses. **(2) slot sizing**: `T_SLOT_MS` must be ≥ the measured worst-case sweep (~22 ms 4-anchor, air-time bound at PLEN-1024/850k), so `T_SLOT_MS=24`; the original 15 ms let adjacent 16 ms-spaced slots overlap, causing inter-tag collisions (`n=3`). DISCOVERY (E2/E4) did **not** collide in practice with 3 tags (`DISC n=4` observed consistently), so the previously-feared discovery TX stagger was not the blocker; it may still matter at higher tag counts. **Real capacity ≈ 7 tags** (CFP budget ~180 ms / 25 ms slot spacing), below the protocol hard cap of 12 (`UWB_FRAME_N_CFP` / beacon slot-map). Capacity levers without sacrificing range: PLEN-512 @850k (~9 tags, −3 dB), 3-anchor sweep (~10, no redundancy), or both (~12). The `RESCAN seat`/`RESCAN miss` BLE log is permanent lost-sync telemetry.
- **Power saving — RX duty-cycle (hardware-validated):** Two layers cut consumption from ~66 mA to **~22 mA** (≈64 %). **Layer 1** deep-sleeps the DW3000 (`dw_enter_sleep()`/`dw_wake()` in `uwb_net_runner.c`, gated by `dw_sleep_enabled`, toggled with `pwr sleep on|off`) between activities in SLOW/IDLE tiers (~66→56 mA). **Layer 2 — narrow beacon window** (`beacon_track_core.c`): the open-ended beacon listen kept the receiver on ~91 % of every superframe (`RXon 182/190 ms`, measured via the `pwr rx` instrumentation). The runner now predicts the next beacon from an EMA of the gateway period (~195 ms in the tag's clock, **not** 200 ms — predicting from 200 ms accumulates ~5 ms/superframe of error) and, once `WARMUP_N` clean beacons confirm lock (ACQUIRING→TRACKING), sleeps the radio until just before the prediction and arms only a `±BT_GUARD_MS` (5 ms) window. A miss forces one full-window frame to re-acquire (so a single false miss can never trip `UWB_NET_MISS_MAX`=3 → RESCAN). Result: `RXon` mean ~30 ms (warmup-dominated; the narrow frames themselves are ~5 ms), arrival jitter only ~±0.12 ms once tracking (the tight window also de-jitters the software timestamp path). **Timing is in ms** (`uwb_radio_now_ms()`/`k_uptime`), not k_cycle — the DW3000's hardware timestamp clock stops in SLEEP, so hardware delayed-RX is incompatible with Layer 1; software timing composes cleanly. Tuning constants in `uwb_net_runner.c`: `BT_GUARD_MS`, `BT_WARMUP_N` (8; lowering toward 2 cuts the post-miss warmup penalty since `period_est` is retained across a miss), `BT_EMA_SHIFT`. **Diagnostics:** `pwr rx` reports `RXon mean/max ms`, folded arrival offset min/max µs, and beacon/miss counts; `pwr rxrst` resets the accumulator (use it after the tag locks to measure steady-state only — JOIN/re-discovery skips otherwise inflate the offset). Specs/plans: `spec/2026-06-23-rx-stats-*`, `spec/2026-06-24-narrow-beacon-window-*`, `plan/2026-06-24-narrow-beacon-window.md`. Host tests: `tests/rx_stats/`, `tests/beacon_track/`.
- **DW3000 ownership:** the runner and the calibration thread share one IRQ line and one `irq_sem` through `wait_event()` (`src/uwb_ss_initiator.c`), which is destructive under concurrency — it opens with `k_sem_reset()` and closes with a global `port_DisableEXT_IRQ()`. It is correct only while exactly one thread drives the radio. `src/uwb_radio_owner.c` enforces that by explicit handover: the runner offers the radio once per superframe at the top of `runner_fn`, leaving it awake and idle; calibration claims it for its whole run. The handover is a mutex (`lock`) plus a condition variable (`cv`) over `enum owner_state { OWNER_IDLE, OWNER_REQUESTED, OWNER_HANDED }`, with every transition made under the mutex. `uwb_radio_yield()` re-checks the claim under the lock and returns without handing over if the claimant withdrew in between — this is what stops the runner being stranded in a `K_FOREVER` wait when a claim times out. `uwb_radio_release()` is a no-op unless a handover is actually outstanding, so a stray release cannot make a later `uwb_radio_yield()` return early and put two threads on the radio. `uwb_radio_request()` fails immediately if a claim is already in flight — single claimant thread at a time is a documented contract, not a guarantee the module enforces by queueing. Known residual: a claimant that dies without calling `uwb_radio_release()` still parks the runner in `K_FOREVER` with no diagnostic. `uwb_radio_yield()` returns `bool` — true only if the radio really changed hands; the runner's reacquire (including `beacon_track_reset()`, worth `BT_WARMUP_N`=8 superframes of full-window RX) runs only then. **PHY-state contract:** that reacquire restores only `dwt_setrxaftertxdelay`, `dwt_setrxtimeout`, `dwt_setpreambledetecttimeout`, `dwt_settxantennadelay`, `dwt_setrxantennadelay`; a claimant that changes anything else (`dwt_configure`, `dwt_configuretxrf`, `dwt_setcallbacks`, frame filtering, PAN/short address) must restore it before releasing or runner RX silently dies. Every `dwt_forcetrxoff()` at a handover point is followed by `dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)` — an abort can leave RX-error bits set, and the *other* thread's first `wait_event()` would dispatch them as a spurious `EVT_RXERR`. **Any new radio consumer must go through this handover, not add a third caller of `wait_event()`.** A mutex alone would not work — the runner sleeps inside its own loop. Host test: `tests/uwb_radio_owner/` compiles the real module against a pthread shim for `k_mutex`/`k_condvar` (`shim/zephyr/kernel.h`) and runs two threads against it — Zephyr's `native_sim`/`unit_testing` are POSIX-arch boards and do **not** configure on Windows.
- **UWB antenna delay (calibration):** Per-unit, solved on-device and persisted in NVS — see `src/cal.c`/`src/cal_math.c`. Trigger over BLE NUS with `cal <mm>` (reference distance in millimetres); the initiator (`src/uwb_ss_initiator.c`) collects ~100 ranges, rejects outliers (median/MAD), and corrects the combined TX+RX delay toward the reference by a linear step (~2.34 mm per combined unit) over up to 4 iterations until the residual is ≤15 mm, then stores a CRC-checked, PHY-tagged `cal_record` in NVS (DTS `storage_partition`, resolved via devicetree because this NCS Partition Manager build gives all flash to a single `app` region). Other commands: `cal status`, `cal clear`, `cal selftest` (runs `cal_math_selftest()` → `SELFTEST 0` on success). **NVS is mandatory:** without a valid record for the current PHY the ranging thread reports `CAL REQUIRED` and does not range until a `cal` run succeeds. `TX_ANT_DLY = RX_ANT_DLY = 16371` in `src/phy_config.h` is now only the factory-reference seed/fallback for an uncalibrated unit. Values are PHY-dependent (calibrated for `CONFIG_OPTION_07`: Ch5, PLEN-1024, 850k); `phy_option` in the record invalidates a stored value if the PHY changes. Note: the `CAL loaded`/`CAL REQUIRED` message sent from `main.c` at boot fires before a central connects and is dropped — use `cal status` to read state after connecting. The gate is enforced in the runner via `uwb_net_gate_actions()` (`src/uwb_net.c`), which clears **`UWB_ACT_RUN_SWEEP` only** (`UWB_ACT_RANGING_MASK`) when `cal_is_valid()` is false, while leaving JOIN/KEEPALIVE/DISCOVER/SLEEP intact — an uncalibrated tag keeps its seat and emits no `P:` line. **`UWB_ACT_RUN_DISCOVER` is deliberately excluded**: discovery does no TWR (it broadcasts E2 and collects E4s), so it has no antenna-delay dependence and produces no range to be wrong. Gating it would also pin an uncalibrated tag in `UWB_ST_DISCOVER` forever — the only path to `UWB_ST_RANGING` is `UWB_EV_DISCOVERED`, which the runner emits only inside the gated DISCOVER block — and `UWB_ACT_SLEEP` is emitted from `UWB_ST_RANGING` alone, so `dw_enter_sleep()` would become unreachable and the tag would never deep-sleep the radio (~56 mA back to ~66 mA). That is not just a bench condition: a `phy_option` change invalidates the record for a whole fleet at once. This check lived in `ss_twr_fn` originally and was lost when the runner took over ranging; the documented guarantee was false in between. The gate is applied at two sites in `runner_fn`, not one — once on `uwb_net_handle()`'s return, and again on the GRANT-response re-OR inside the `UWB_ACT_SEND_JOIN` block; the second is defensive (a GRANT currently yields only DISCOVER, which passes the gate anyway), because the re-OR happens after the first gate and any future action it introduced would otherwise bypass it. Contract asserted by `tests/uwb_net/test_uwb_net.c::test_gate_actions()`.

## Flash Memory Layout

| Region | Start | Size |
|---|---|---|
| MCUBoot | 0x00000000 | 48 KB |
| Image-0 (active) | 0x0000c000 | 220 KB |
| Image-1 (update slot) | 0x00043000 | 220 KB |
| Storage | 0x0007a000 | 24 KB |
