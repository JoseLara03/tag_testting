# UWB TDMA MAC — Protocol Contract (v1)

**Date:** 2026-06-17
**Status:** Design approved, pre-implementation
**Scope:** The on-air contract shared by *all* nodes (gateway, anchors, tags) in the
RTLS network. This document is intentionally implementation-agnostic so it can be
imported verbatim into the anchor/gateway firmware later. The tag-side behaviour
that consumes this contract is specified separately in
`2026-06-17-tag-network-layer-design.md`.

---

## 1. Purpose & context

The RTLS uses **two-way ranging (TWR)**, chosen over TDoA because anchors are spaced
up to **100 m** apart and wirelessly distributing the sub-nanosecond clock TDoA needs
is not viable at that range. TWR measures each distance from a self-contained round
trip (each node uses its own clock + offset correction), so **ranging needs no shared
clock at all**.

The only shared timing this contract requires is **coarse (microsecond-class) TDMA
framing** — "when may a tag transmit." That time base is the **beacon itself**: the
instant a node receives a beacon is its superframe `t = 0`. Guard intervals absorb the
residual drift.

### Topology
- **Gateway** (runs on an anchor for now): emits the beacon, owns the slot map, grants
  and reclaims seats, provides backhaul. Short address `0x0000`.
- **Anchors**: slaves. Listen during active slots and respond to ranging polls. Fixed
  short addresses `0x0001 …`.
- **Tags**: associate to get a seat, then range their chosen anchors only inside that
  seat. Short address assigned on join.
- **One PAN** (single customer `PANID`) spans the whole deployment. Coverage is
  extended by adding anchors; there are no independent cells. The clock is "extended"
  by **beacon coverage** (wired backhaul to multiple beacon emitters, or multi-hop
  beacon relay), not by sync distribution — a few µs of accumulated beacon-time error
  only nudges slot boundaries and is absorbed by guards.

### Two orthogonal concerns (do not conflate)
- **CAP / gateway negotiation answers *WHEN*** a tag may transmit (join / grant /
  keepalive / lease). Tag ↔ gateway.
- **Discovery answers *WHICH*** anchors a tag ranges against. Tag ↔ anchors. The
  gateway never needs to know a tag's anchor set; anchors listen during every active
  slot, so the tag simply polls whichever 4 it selected inside its granted seat.

---

## 2. Superframe structure

A superframe repeats forever and has three regions:

```
|<------------------------ T_superframe (v1: 200 ms) ------------------------>|
[ BEACON ][g][ CAP: N_CAP Aloha mini-slots ][g][ CFP: N_CFP ranging slots ...][g]
  ~1.5ms  0.5         ~6 ms              0.5        N_CFP × (T_slot + g)
```

- **Beacon** (gateway → broadcast): carries the time base + slot map (§4.1).
- **CAP** (Contention Access Period): `N_CAP` slotted-Aloha mini-slots for tag→gateway
  join requests, keepalives, and releases. Collisions resolved by exponential backoff.
- **CFP** (Contention-Free Period): `N_CFP` ranging slots, **one tag per slot per
  superframe**. The tag named in slot *k* by the beacon's slot map owns slot *k* this
  superframe and runs its ranging sweep there.
- **Guard `g` = `T_guard`** precedes the CAP and follows the beacon and every CFP slot;
  it absorbs beacon-clock skew and anchor RX turn-on time.

### 2.1 v1 timing constants

These are fixed by protocol **version** (not carried in the beacon), so `T_superframe`
is constant and the tier→rate mapping in §5 holds. They are the first thing to
re-measure on real hardware and may change before v1 freezes.

| Constant | v1 default | Notes |
|---|---|---|
| `T_superframe` | 200 ms | sets the 5 Hz fast-tier period |
| `T_beacon` | ~1.5 ms | one PLEN-1024 frame @ 100 m |
| `T_guard` | 0.5 ms | beacon-skew + RX turn-on margin |
| `T_slot` | ~15 ms | one **single-poll ×4** sweep (see §6) |
| `t_minislot` | ~1.5 ms | one CAP Aloha mini-slot (one short frame) |
| `N_CFP` | 12 | ranging slots/superframe ⇒ ~12 simultaneous movers |
| `N_CAP` | 4 | Aloha mini-slots for association traffic |

Budget check: `T_beacon + g + N_CAP·t_minislot + g + N_CFP·(T_slot+g)` ≈ `1.5 + 0.5 +
4·1.5 + 0.5 + 12·15.5` ≈ 194.5 ms ≤ 200 ms. ✓

---

## 3. Addressing & framing base

This contract **extends the existing `src/uwb_frame_802_15_4z` module** (on branch
`feat/uwb-frame-802-15-4z`); it does not define a new wire format. New message types
are new function codes in that module, with new builders/parsers/validators in the
same style and the same host-test harness (`tests/uwb_frame/`).

- **Header (bytes 0–9), unchanged from the module:** `FC = 0x41 0x88`, `seq` (byte 2),
  `PANID` (bytes 3–4, customer value — replaces the legacy `0xCADE`), `dest` short
  addr (bytes 5–6, LE), `src` short addr (bytes 7–8, LE), `function code` (byte 9).
- **Addresses are 16-bit short** (`uint16_t`, little-endian). 802.15.4/4z defines only
  *none / 16-bit short / 64-bit extended* — there is no 8-bit mode. 16-bit short is
  what lets the **DW3000 hardware frame filter** drop frames not addressed to the node
  (wrong PAN / wrong dest) before they wake the CPU. `0xFFFF` = broadcast.
- **64-bit EUI** identifies a tag *before* it has a short address (join only); carried
  in the payload of `JOIN_REQ`/`GRANT`, not in the MAC address fields.
- The legacy unaddressed SS-TWR frames (`0xE0/0xE1`, ASCII `WAVE`/`VEWA`) remain
  reserved for **bench calibration only** (`src/cal.c`) and are *not* wire-compatible
  with the addressed network frames by design. They coexist via distinct function
  codes.

### 3.1 Function-code allocation

| Code | Name | Dir | Status |
|---|---|---|---|
| `0xE0` | (legacy SS-TWR poll) | — | reserved, cal/bench only |
| `0xE1` | (legacy SS-TWR resp) | — | reserved, cal/bench only |
| `0xE2` | DISCOVERY | tag → anchors (bcast) | **exists** (4z branch) |
| `0xE3` | MULTI-POLL | tag → anchors | **exists**, deferred (see §6) |
| `0xE4` | RANGE-RESPONSE | anchor → tag | **exists** (carries `cir_power`/`cir_quality`) |
| `0xE5` | BEACON | gateway → broadcast | **new** |
| `0xE6` | JOIN_REQ | tag → gateway | **new** |
| `0xE7` | GRANT | gateway → tag (by EUI) | **new** |
| `0xE8` | KEEPALIVE | tag → gateway | **new** |
| `0xE9` | RELEASE | tag → gateway | **new** |

> v1 in-slot ranging uses the existing **single-poll** path (`0xE0`-style addressed
> poll + `0xE4` response, one exchange per anchor ×4). `0xE3` MULTI-POLL is the
> documented future upgrade (§6); adopting it requires **no contract change**.

---

## 4. New frame layouts

All multi-byte fields are little-endian. Lengths exclude the 2-byte FCS appended by
`dwt_writetxfctrl`. Builders return bytes written or `-EMSGSIZE`/`-EINVAL`; parsers
return `0` or a negative errno; validators return bool.

### 4.1 BEACON `0xE5` (gateway → broadcast)

| Offset | Field | Size | Notes |
|---|---|---|---|
| 0–9 | header | 10 | dest = `0xFFFF`, src = `0x0000` (gateway), type `0xE5` |
| 10 | `proto_ver` | 1 | must match the tag's compiled version |
| 11–14 | `frame_counter` | 4 | monotonic; doubles as superframe sequence + time base ref |
| 15 .. | `slot_map` | `2 × N_CFP` | `uint16` short addr per CFP slot; `0xFFFF` = idle slot |

Length = `15 + 2·N_CFP` (v1: 39 bytes). `N_CFP`/`N_CAP` are version constants, not
carried, so the map length is implied by `proto_ver`.

### 4.2 JOIN_REQ `0xE6` (tag → gateway, in CAP)

| Offset | Field | Size | Notes |
|---|---|---|---|
| 0–9 | header | 10 | dest = `0x0000`, src = `0xFFFE` (unassociated placeholder), type `0xE6` |
| 10–17 | `eui64` | 8 | tag factory EUI |
| 18 | `req_tier` | 1 | requested rate tier (§5) |

Length = 19.

### 4.3 GRANT `0xE7` (gateway → tag, in CAP window, addressed by EUI)

Broadcast at MAC level (`dest = 0xFFFF`) because the tag has no short address yet; the
joining tag accepts the frame **iff** the payload `eui64` matches its own.

| Offset | Field | Size | Notes |
|---|---|---|---|
| 0–9 | header | 10 | dest = `0xFFFF`, src = `0x0000`, type `0xE7` |
| 10–17 | `eui64` | 8 | target tag's EUI (match filter) |
| 18–19 | `short_addr` | 2 | address assigned to the tag |
| 20 | `slot_index` | 1 | CFP slot the tag owns (`0 … N_CFP-1`) |
| 21 | `rate_tier` | 1 | granted tier (may differ from requested) |
| 22–23 | `lease_superframes` | 2 | lease validity in superframes (§5) |

Length = 24.

### 4.4 KEEPALIVE `0xE8` (tag → gateway, in CAP)

| Offset | Field | Size | Notes |
|---|---|---|---|
| 0–9 | header | 10 | dest = `0x0000`, src = tag short addr, type `0xE8` |
| 10 | `req_tier` | 1 | current motion-driven tier request |
| 11 | `slot_index` | 1 | slot the tag believes it holds (sanity/repair) |

Length = 12. Refreshes the lease; the gateway may re-tier and/or re-slot in the next
beacon.

### 4.5 RELEASE `0xE9` (tag → gateway, in CAP)

| Offset | Field | Size | Notes |
|---|---|---|---|
| 0–9 | header | 10 | dest = `0x0000`, src = tag short addr, type `0xE9` |

Length = 10. Graceful leave / power-down; gateway frees the seat immediately.

### 4.6 DISCOVERY / MULTI-POLL / RANGE-RESPONSE

Defined by the existing module (`uwb_frame_discovery_build`,
`uwb_frame_multipoll_build`, `uwb_frame_response_build` and matching parsers). The
RANGE-RESPONSE (`0xE4`) carries `cir_power` + `cir_quality`, which the tag uses for
anchor selection (discovery) and may later use for NLOS gating. The anchor also
self-reports its `(x, y[, z])` coordinates — **z is carried even though the v1 tag
solves in 2D**, so the contract is 3D-ready without a future wire change.

---

## 5. Slot-lease lifecycle & rate tiers

A `GRANT` is a **lease**, not a permanent seat.

### 5.1 Rate tiers (motion → cadence)

| Tier | Value | Cadence | Update rate | Trigger |
|---|---|---|---|---|
| IDLE | 0 | 1 slot / 25 superframes | 0.2 Hz (5 s) | stationary |
| SLOW | 1 | 1 slot / 5 superframes | 1 Hz | recently moved |
| FAST | 2 | 1 slot / superframe | 5 Hz | moving |

A tag transmits its sweep **only** in superframes where the beacon's slot map names it
*and* the tier cadence permits. Lowering a tier hands airtime back — the mechanism that
lets ~12 CFP slots serve 100 tags when only a handful move at once.

### 5.2 Lease maintenance

- The tag sends `KEEPALIVE` in CAP before `lease_superframes` elapses (recommended:
  renew at half the lease). The gateway refreshes the lease and may change tier/slot.
- **Gateway reclamation:** after the lease expires with no keepalive (≈ K missed
  renewals), the gateway frees the seat (`slot_map[k] = 0xFFFF`).
- **Tag-side loss detection (cardinal safety rule — never transmit on unsure timing):**
  | Condition | Tag action |
  |---|---|
  | Miss 1 beacon | Skip *all* TX this superframe; widen next RX window |
  | Miss `M` beacons (v1: 3) | Lease assumed lost → re-scan/join (this is the handover path) |
  | Own short addr absent from slot map | Stop using the slot immediately → re-join |

### 5.3 Lifecycle constants

| Constant | v1 default | Owner |
|---|---|---|
| `lease_superframes` | 50 (~10 s @ 200 ms) | gateway sets, tag honours |
| keepalive renewal | at 50 % of lease | tag |
| `M` (tag beacon-miss → lost) | 3 | tag |
| `K` (gateway missed-renewal → reclaim) | implied by lease expiry | gateway |
| CAP backoff | exponential over `N_CAP` mini-slots, capped retries | both |

---

## 6. Deferred capacity upgrade: MULTI-POLL

v1 ranges with **single-poll ×4** (4 separate poll→response exchanges), `T_slot ≈
15 ms` ⇒ `N_CFP = 12`. The `0xE3` MULTI-POLL frame already exists: one poll names all
4 anchors with staggered `delay_us`, and anchors reply in turn ⇒ one poll preamble
instead of four ⇒ `T_slot ≈ 6–7 ms` ⇒ roughly double `N_CFP` (~24 movers). Switching is
an **in-slot ranging-primitive change only**; the superframe, addressing, and lease
contract are unchanged. Gate the switch on hardware validation of multi-poll timing.

---

## 7. Versioning

`proto_ver` (beacon byte 10) gates compatibility. A tag rejects beacons whose
`proto_ver` it does not implement and stays in SCAN. Changing any v1 timing constant,
`N_CFP`/`N_CAP`, or a frame layout bumps `proto_ver`.

---

## 8. Open items to resolve during anchor/gateway specs

- Gateway scheduling policy (fair-share vs priority) when FAST demand exceeds `N_CFP`.
- Beacon-extension mechanism (wired multi-emitter vs multi-hop relay) and its handover
  hysteresis thresholds.
- Whether anchors need any beacon-derived state beyond "listen during active slots."
- Customer `PANID` provisioning path.
