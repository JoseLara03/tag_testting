# Tag-Side Network Layer — Design

**Date:** 2026-06-17
**Status:** Design approved, pre-implementation
**Depends on:** `2026-06-17-uwb-mac-protocol-contract.md` (the on-air contract) and the
`feat/uwb-frame-802-15-4z` frame module.
**Scope:** The tag firmware that consumes the MAC contract — discover, join, hold a
seat, range only in that seat, handle loss/handover, sleep between. The gateway and
anchor firmware are out of scope (separate specs); they appear here only as the
counterpart of contract messages.

---

## 1. What changes vs today

Today `ss_twr_fn` in [`src/uwb_ss_initiator.c`](../src/uwb_ss_initiator.c)
**free-runs**: a local `RNG_FAST_MS`/`RNG_SLOW_MS` timer decides when to range, and the
tag keys up whenever it likes. Under TDMA that behaviour is exactly what must stop. The
**beacon drives all timing**; the tag is silent except in CAP (to the gateway) and its
own CFP slot.

The ranging *primitives* the current code already has are kept and reused — only the
thing that *triggers* them changes:

| Reused as-is | New |
|---|---|
| `do_one_range_anchor()` (single-poll exchange) | `uwb_net` MAC / state machine |
| `pos_solve()` (LLS) | beacon RX + slot-timing scheduler |
| `twr_msgq` → BLE sender thread | CAP association traffic (join/keepalive/release) |
| `cal` (bench, unchanged — see §6) | anchor discovery / selection |

Removed: the free-running cadence (`range_tick`, `RNG_FAST_MS`, `RNG_SLOW_MS`,
`ss_moving` as a local cadence switch). `uwb_set_moving()` is **repurposed** to set the
*requested tier* reported in `KEEPALIVE` — motion still drives rate, but now through the
gateway, not a local timer.

---

## 2. Architecture & the host/target seam

**Hard requirement: the MAC logic must be hardware-independent and host-testable.** The
state machine, frame handling, slot-timing math, and lease/tier logic must not call
`dwt_*`, Zephyr kernel APIs, or GPIO directly. They sit above a thin injected
**`radio_ops` interface**; the real DW3000/Zephyr glue is the only target-only code.

```
        host-testable (portable C, no HW)            target-only (thin glue)
  ┌───────────────────────────────────────┐     ┌──────────────────────────────┐
  │  uwb_net  (state machine + scheduler)  │     │  radio_ops_dw3000.c          │
  │  uwb_frame_802_15_4z (build/parse)     │────▶│   dwt_* , SPI, IRQ sem,      │
  │  lease/tier math, slot-offset math     │ via │   k_sleep, deep-sleep,       │
  │  pos_solve (already pure CMSIS-DSP*)    │ ops │   GPIO, monotonic clock      │
  └───────────────────────────────────────┘     └──────────────────────────────┘
```

`radio_ops` (illustrative): `rx_beacon(deadline) → frame|timeout`, `tx_in_cap(frame)`,
`sweep_anchors(anchor_set) → ranges`, `sleep_until(t)`, `now()`. On host tests these are
**mocks** that record intentions and feed scripted events; on target they wrap the
existing DW3000 calls.

*`pos_solve` depends on CMSIS-DSP; for host tests it is either linked against a CMSIS
stub or excluded from the FSM unit and tested separately. The FSM is tested for *when*
it solves, not the LLS math itself.

### Module layout
- `src/uwb_net.c` / `.h` — state machine, scheduler, lease/tier logic (portable).
- `src/uwb_radio_ops.h` — the interface.
- `src/uwb_radio_ops_dw3000.c` — target implementation (owns the DW3000, reuses
  `do_one_range_anchor`, the IRQ/semaphore model, and DW3000 deep-sleep).
- `src/uwb_frame_802_15_4z.*` — extended per the contract (§4 of the contract doc).

---

## 3. State machine

```
        boot
         │
         ▼
   ┌──────────┐  beacon heard (ver OK)   ┌──────────┐  GRANT(my EUI)   ┌───────────┐
   │   SCAN   │ ───────────────────────▶ │ JOINING  │ ───────────────▶ │ DISCOVER  │
   │ RX only, │ ◀─ no beacon for         │ TX join  │ ◀─ backoff /     │ 1 slot:   │
   │ low duty │     T_rescan             │ in CAP   │    no grant       │ pick 4 ax │
   └──────────┘                          └──────────┘                  └─────┬─────┘
        ▲                                                                    │ ≥3 anchors
        │ miss M beacons  OR  my addr gone from slot-map                     │
        │ OR join failed N times                       ┌────────────────────┘
        │                                               ▼
        │                                  ┌──────────────────────────────┐
        └───────────────────────────────  │            RANGING           │
                  re-DISCOVER ◀──────────  │ in-slot (per tier): poll×4 → │
                  (quality drop /          │   pos_solve → publish P:x,y  │
                   <3 anchors repeatedly)  │ in-CAP: KEEPALIVE(tier)      │
                                           │ motion change → change tier  │
                                           └──────────────────────────────┘
```

| State | Enters when | Does | Leaves when |
|---|---|---|---|
| **SCAN** | boot; lease lost; join gave up | RX-only listen for a valid, version-matched beacon; low duty cycle | beacon heard → JOINING |
| **JOINING** | beacon heard, no lease | TX `JOIN_REQ` in a CAP mini-slot; await `GRANT` matching own EUI; Aloha backoff | GRANT → DISCOVER; N fails → SCAN |
| **DISCOVER** | just granted; or re-select trigger | In own slot, broadcast `DISCOVERY`, collect anchor responses, rank by `cir_quality`, pick best ≤4 | ≥3 anchors → RANGING; <3 → retry/report |
| **RANGING** | anchor set chosen | Per-superframe loop (§4); keepalive; publish fixes | lease lost → SCAN; quality drop → DISCOVER |

---

## 4. Per-superframe loop (RANGING)

The radio is a single half-duplex resource; **one thread** serializes it through
`radio_ops`. Each superframe:

1. Wake a guard before the expected beacon; **RX beacon**. The reception instant is
   superframe `t = 0`. Parse `proto_ver`, `frame_counter`, slot map.
2. **Loss checks first** (contract §5.2): if no valid beacon → skip all TX this
   superframe, increment miss counter (→ SCAN at `M`). If own short addr not in the map
   → re-join.
3. If lease is near expiry and it is a CAP turn → **TX `KEEPALIVE`** in an Aloha
   mini-slot, reporting the motion-driven tier.
4. If this superframe is a "my-cadence" frame for the current tier *and* the map names
   me in slot `k` → `sleep_until(slot_start)` then **sweep the chosen anchors**
   (`do_one_range_anchor ×4`), `pos_solve`, enqueue `P:x,y` to the BLE sender.
   - `slot_start = t0 + T_beacon + g + N_CAP·t_minislot + g + k·(T_slot + g)`.
5. **Deep-sleep the DW3000** until step 1 of the next superframe. Radio is off for most
   of every superframe — the primary battery win.

---

## 5. Error / edge handling

| Condition | Tag response |
|---|---|
| Miss 1 beacon | Skip all TX; widen next RX window; do not fire slot on stale timing |
| Miss `M` (=3) beacons | Lease lost → SCAN (handover path) |
| Short addr absent from slot map | Stop using slot immediately → re-JOIN |
| CAP collision (join/keepalive) | Exponential backoff over `N_CAP` mini-slots; after N → SCAN with longer rescan |
| Sweep yields <3 anchors repeatedly | Trigger re-DISCOVER (anchor set stale) |
| Two beacons heard (extended coverage) | Choose stronger `cir_quality`; hysteresis to avoid flapping |
| `proto_ver` mismatch | Reject beacon; stay SCAN |
| Corrupt/duplicate frame | Rejected by `uwb_frame_is_valid()` + seq / `frame_counter` checks |

**Power:** deep-sleep between beacon→slot and slot→next-beacon; wake on a timer a guard
before the expected beacon; re-zero drift on each received beacon; after a miss widen
the RX window rather than transmitting blind.

---

## 6. Calibration coexistence (unchanged)

`cal` (`src/cal.c`, `src/cal_math.c`) stays **entirely outside the MAC**. It ranges
against the legacy unaddressed `WAVE` reference frame (`0xE0/0xE1`) and is a bench /
maintenance operation usable any time, where no beacon/network is present. The addressed
network frames and the `WAVE` cal frames coexist via distinct function codes. No change
to cal is required by this design; `uwb_net` simply does not drive cal, and a `cal` run
is a separate bench mode (network operation is not expected concurrently on the bench).

---

## 7. Test strategy

- **Frame layer (host, WinLibs gcc):** extend `tests/uwb_frame/test_uwb_frame.c` with
  build/parse/validate + exact-byte-layout + round-trip + corruption-rejection for
  `BEACON / JOIN_REQ / GRANT / KEEPALIVE / RELEASE`, and slot-map encode/decode. Same
  standalone-compile pattern the 4z branch already uses (no Zephyr, no DW3000).
- **State machine (host):** unit-test `uwb_net` as a pure FSM over injected events with
  a **mock `radio_ops`**. Examples: `BEACON_RX → JOIN_REQ emitted`; `GRANT(my EUI) →
  DISCOVER`; `BEACON_MISS ×3 → SCAN`; `addr-absent-from-map → re-JOIN`; `MOTION fast→idle
  → req_tier in next KEEPALIVE`; "never TX when last beacon missed."
- **On-device bench:** `cal` still validates the RF path via `WAVE`. Then 1
  gateway-anchor + responders + 1 tag → verify join, slot adherence (logic analyzer /
  sniffer), and `P:x,y` output. Add tags, then **measure real `T_slot`** to confirm /
  tune the contract's v1 timing constants.

---

## 8. Implementation interactions / risks

- **`cal` vs network thread ownership:** today both would want the DW3000; on the bench
  only one mode runs at a time. Ensure `uwb_net` is not started (or is suspended) while
  a bench `cal` run holds the radio.
- **Clock drift across deep-sleep:** the wake-before-beacon guard must exceed the
  DW3000 sleep-clock error over one superframe; validate on hardware and widen the RX
  window after misses.
- **CAP fairness under join storms:** many tags powering on together collide in CAP;
  backoff parameters (`N_CAP`, retry cap) need on-air tuning.
- **`pos_solve` host-testability:** CMSIS-DSP dependency is isolated from the FSM unit
  (§2) so FSM tests stay portable.
