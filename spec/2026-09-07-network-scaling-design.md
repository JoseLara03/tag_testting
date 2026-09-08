# Network scaling — 100 tags, 100 m, 32 anchors (protocol v3)

**Date:** 2026-09-07
**Status:** Design, pre-implementation
**Scope:** The whole RTLS: tag firmware (`tag_testting`), anchor/gateway firmware
(`ANCLA_ESP32S3`), and the on-air contract both share.
**Supersedes on-air:** `spec/2026-06-17-uwb-mac-protocol-contract.md` (v2 -> **v3**).
Read that contract first; this document states only the deltas and the reasoning.

---

## 1. Targets and non-goals

| Target | Value | Decided |
|---|---|---|
| Simultaneous tags in one PAN | **100** | requirement |
| Per-tag fix rate, stationary | **0.31 Hz** (1 fix / 3.2 s) | chosen 2026-09-07 |
| Per-tag fix rate, moving | **1.25 - 2.5 Hz** (4-8 phases) | chosen 2026-09-07 |
| Ranging anchors supported by firmware | **32** (wire cap 254) | chosen 2026-09-07 |
| Link range, anchor <-> tag | **100 m** | requirement |
| Ranging method | **TWR** (TDoA evaluated and abandoned: precision) | given |

**Non-goals in this design.** Multi-gateway / multi-cell zones, channel reuse
(ch9 as a second colour), sub-100 ms superframes, and TDoA are all out of scope;
§10 records why and what each would cost.

**This is a flag day.** Every change below bumps `proto_ver` to 3. A v3 gateway's
beacon is rejected by a v2 tag and vice versa (`UWB_NET_PROTO_VER`), by design.
The whole fleet — gateway, anchors, tags — is reflashed together. That is
acceptable today because nothing is deployed in the field; it will not be later,
so this is the moment to take every wire change we know we want.

---

## 2. What actually limits the system today

Facts read out of the code at HEAD of both repos, not estimates.

### 2.1 Tag capacity: ~7, not 11

`UWB_FRAME_N_CFP` is 11, but `T_SLOT_MS` is **24** (`src/uwb_net_runner.c:52`),
sized to the measured worst-case 4-anchor single-poll sweep of ~22 ms. The
superframe budget is

```
200 ms - (T_beacon 2 + g 1 + N_CAP 4 x t_minislot 2 + g 1) = 188 ms
188 / (24 + 1) = 7.5  ->  7 usable CFP slots
```

so four of the eleven advertised slots do not exist in wall-clock time. Real
capacity is ~7 simultaneous **movers**, hardware-validated at 3. The 11-entry
slot map is honest about the protocol and dishonest about the clock.

At one fix per superframe per tag, 100 tags need 100 slots. **Nothing in the
current design gets there**: the slot is too long, and the superframe grants
every seat every superframe whether the tag needs it or not.

### 2.2 Anchors: hard-capped at 4 in five independent places

| Limit | Where | Value |
|---|---|---|
| Deployment cap | `ANCLA/src/uwb_config.h` | `UWB_MAX_ANCHORS 4` |
| Console validation | `ANCLA/src/anchor_shell.c` | `anchor id <0..3>` |
| Discovery stagger | `ANCLA/src/disc_schedule.h` | `DISC_BASE 2000 + id x 3500` uus |
| TX completion bound | `ANCLA/src/anchor_respond.c` | `TX_COMPLETE_TIMEOUT_MS 18`, derived from `disc_resp_delay_uus(3)` = 12.5 ms |
| Tag collection window | `src/uwb_net_runner.c` | `DISCOVERY_WINDOW_MS 15` |
| Tag pool / selection | `src/uwb_net_runner.c` | `ANCHOR_POOL_MAX 6`, `ANCHOR_SELECT_MAX 4` |
| Multipoll frame | `src/uwb_frame_802_15_4z.h` | `UWB_FRAME_MAX_ANCHORS 4` |
| Survey | `ANCLA/src/apos_geom.h` | `APOS_MAX_NODES 8`, full-mesh assumption |

The **linear discovery stagger is the structural one**: 32 anchors answering one
broadcast at `2000 + id x 3500` uus needs a 110 ms collection window, which does
not fit in a superframe, let alone a slot. Everything else on that list is a
constant; this one is an algorithm.

Good news, and it decides the addressing plan: the ranging poll's anchor id is
**the low byte of the anchor's short address** (`anchor_respond.c`'s `wire_id`),
so the wire already supports ids 1..254 with no frame change. The cap is
firmware, not format.

### 2.3 Range: 100 m is a link-budget problem, not a MAC problem

At channel 5 (6489.6 MHz, 499.2 MHz bandwidth):

```
FSPL(100 m)     = 32.44 + 20log10(6489.6 MHz) + 20log10(0.1 km) = 88.7 dB
Max legal EIRP  = -41.3 dBm/MHz + 10log10(499.2)                = -14.3 dBm
Rx power @100 m, 0 dBi both ends                                = -103.0 dBm
DW3000 sensitivity, 850 kbps, PLEN-1024 (conservative)          ~ -103 dBm
```

**Margin at 100 m in free space with isotropic antennas: ~0 dB.** Add body loss
on a worn tag (-3 to -6 dB) and any non-LOS and the link does not close. The
missing 10-15 dB has to come from hardware — anchor antenna gain, an RX LNA on
the anchor, and mounting for LOS — because the transmit side is already at the
regulatory ceiling (`txconfig_options` on the tag is `0xffffffff`, i.e. maximum).

Two facts make this worse and must be closed before any range claim:

- **The ANCLA boards measure ~25 dB below free-space expectation** (anchor
  `CLAUDE.md`, "transmit well below free-space expectation", unresolved). At
  -25 dB the same budget closes at ~6 m, which is exactly the 2-3 m bench
  behaviour that was observed. **Nothing about range is measurable until this is
  root-caused.**
- **The link is asymmetric in the wrong direction.** The anchor has a PA and
  **no LNA**; the tag has an LNA and **no PA**. The TWR poll is tag -> anchor, so
  the weak transmitter feeds the weak receiver, and the strong ones (anchor TX,
  tag RX) carry the response that was never the problem. Any LNA/antenna money
  spent goes on the **anchor receive** side first.

By contrast, the MAC needs nothing for distance: 100 m is 0.33 us of flight,
against 1 ms guards and a 2050 us turnaround. Distance shows up in the MAC only
in the **survey**, where anchor pairs 100 m apart may not hear each other at all
(§7.3).

### 2.4 Ranging frames carry no tag identity

The production ranging exchange is still the non-addressed WAVE pair
(`src/uwb_wave_frame.h`): bytes 5-8 are the ASCII `WAVE`/`VEWA` literals, not
`dest`/`src`. An anchor answers whichever tag polled its id, and a tag accepts
any response carrying the id it asked for. TDMA is the *only* thing preventing
cross-talk — which is why multi-tag works today, and why spatial reuse can never
work without changing this. It also means the DW3000 hardware frame filter is
unusable on the hottest path in the system.

### 2.5 CAP and address pool scale with tag count, and both break

- Every tag renews its lease with a `KEEPALIVE` in one of `N_CAP` = 4 Aloha
  mini-slots. At 100 tags and a 50-superframe lease renewed at half, that is
  ~4 keepalives per superframe contending for 4 mini-slots — a collision regime,
  and the failure mode is losing the seat, i.e. a re-JOIN storm on top.
- `alloc_short_addr()` (`ANCLA/src/gw_core.c`) is a bare monotonic counter with
  no reuse. 100 tags rejoining across a day walk the pool upward forever, and
  each rejoin is a new identity to the platform — the same defect the `Tid`
  EUI-hash fix worked around downstream rather than fixing here.

---

## 3. The v3 architecture in one page

Five changes. Each is independently useful; together they multiply.

| # | Change | Buys | Costs |
|---|---|---|---|
| **A** | **Superframe cycle**: a grant is `(slot, phase-set)` over a cycle of `C` = 16 superframes | capacity **x16** | beacon semantics, gateway seat table |
| **B** | **MULTI-POLL in-slot ranging** (`0xE3`, already speced and built on the tag) | slot 24 ms -> **12 ms**, capacity x2 | anchor-side responder, new response frame |
| **C** | **Passive anchor ANNOUNCE + grouped DISCOVERY** | **32 anchors**, bounded windows | 1 beacon byte, 1 new frame, 2 DISCOVERY bytes |
| **D** | **Addressed ranging frames** (real `dest`/`src`, anchor `z`) | correctness, HW frame filter, 3D anchors, future reuse | wire break (already taking one) |
| **E** | **Lease renewed from `0xEA` POS**, declared skip honoured | CAP pressure -> ~0, unblocks power Layer 3 | gateway lease logic |

Resulting capacity, computed in §5: **224 phase-seats**, i.e. 100 stationary tags
at 0.31 Hz with 124 phases left over to give movers 1.25-2.5 Hz.

### A. Superframe cycle — the capacity lever

Today the beacon's slot map names, for *this* superframe, which tag owns each of
`N_CFP` slots, and a tag that owns slot *k* owns it in every superframe. That is
why capacity is `N_CFP` and nothing larger.

In v3 the gateway keeps `C x N_CFP` seats and publishes only the current
superframe's row:

```
phase = frame_counter mod C                 (C = 16, a version constant)
seat  = seats[phase][slot]
beacon.slot_map = seats[phase][0 .. N_CFP-1]      (unchanged format)
```

A tag granted `(slot=3, phases={5})` transmits in slot 3 of every superframe with
`frame_counter mod 16 == 5`, i.e. once per 3.2 s. A mover granted
`phases={1,5,9,13}` gets 1.25 Hz from the same table.

Three properties make this cheap:

- **The beacon does not grow, and its semantics barely change.** A tag that
  simply reads "is my address in the map this superframe" behaves correctly
  without knowing what a phase is. Phases are a *gateway scheduling* concept.
- **It is the gateway-side implementation of `listen_skip`.** The low-power
  design's §7 lease contract ("the tag declares its skip; the gateway sizes the
  lease") becomes trivial: the gateway *assigned* the skip, so it knows it. A
  tag with one phase skips 15 superframes by construction, which is exactly what
  `beacon_sched_core` was written to do — and `UWB_LISTEN_SKIP_CAP` (25) is
  already above `C` = 16, so today's 50-superframe lease keeps working unchanged.
- **Tier changes become re-granting phases**, not re-granting seats. No JOIN
  churn when a tag starts walking; the gateway adds phases in the next beacon's
  maps.

`C = 16` and not 8: 8 x 14 = 112 seats covers 100 tags with 12 spare, which
leaves nothing for movers. 16 gives 224. `C` is a version constant (like
`N_CFP`), not carried in the beacon.

### B. MULTI-POLL — the slot-time lever

The MAC contract already documents this as the deferred capacity upgrade (§6),
the frame (`0xE3`) and its builder/parser already exist and are host-tested on
both sides, and **no anchor implements it**. Four sequential poll->response
exchanges pay four preambles and four 2050 us turnarounds; one multi-poll names
all four anchors with staggered response delays and pays one of each.

At PLEN-1024 / 850 kbps a frame is ~1.05 ms of preamble plus ~9.4 us per byte,
so:

```
single-poll x4 (today)  4 x (1.2 poll + 2.05 turnaround + 1.35 resp) ~ 18.5 ms + overhead ~ 22 ms
multi-poll     (v3)     1.4 poll + 2.05 + 4 x 1.7 stagger            ~ 10.2 ms  ->  T_SLOT_MS = 12
```

The stagger step is 1.7 ms (1.35 ms of response airtime + 0.35 ms margin), and
the whole exchange stays inside one tag's slot, so nothing about TDMA changes.

Second-order lever, worth a task but not a redesign: the anchor's
`POLL_RX_TO_RESP_TX_DLY_UUS` is **2000** uus, sized when the ESP32-S3 SPI bus was
running at 2 MHz. At the 26.67 MHz rate now in use the measured RX-side work is
~180 us, and `disc_schedule.h` records that the gate "only requires holding under
2500 uus". Dropping the turnaround to ~1200 uus removes another ~0.85 ms from
every slot. It must move in lockstep on the tag (`POLL_TX_TO_RESP_RX_DLY_UUS`),
the anchor, and the DWM3001CDK calibration reference node, and it is worth
nothing until measured — hence its own task, gated on hardware.

### C. Anchor discovery that scales — announce, and group

Two mechanisms with different jobs. Do not merge them.

**Passive ANNOUNCE (`0xEC`) — steady state.** The beacon gains one byte,
`announce_id`. The anchor whose id matches transmits a short ANNOUNCE in a
dedicated 2 ms window right after the beacon guard, carrying its short address,
`(x, y, z)` and the CIR quality of the beacon it just heard. Tags collect anchors
passively while they are already awake for the beacon; **tags transmit nothing**,
so this cost is proportional to the number of *anchors*, not tags — the only
property that survives 100 tags.

The schedule is `announce_id = frame_counter mod A` with **`A` prime and coprime
with `C`** (`A = 37` for 32 anchors). This is not decoration: a tag awake only in
superframes `frame_counter = p (mod 16)` would, with `A = 32`, only ever observe
**two** distinct announce ids — the phase and its complement. With
`gcd(A, C) = 1` the Chinese remainder theorem gives that tag every residue, so
every anchor is eventually visible from every phase. Worst case to observe one
specific anchor from one phase is `A x C` = 592 superframes ~ 118 s, which is
fine for pool maintenance and useless for cold start — which is why the second
mechanism exists.

**Grouped DISCOVERY (`0xE2` + 2 bytes) — cold start and forced refresh.** The
DISCOVERY frame gains `group` and `n_groups`. An anchor answers only if
`anchor_id mod n_groups == group`, and staggers by its **rank inside the group**,
`rank = anchor_id / n_groups`:

```
delay_uus = DISC_BASE_UUS + rank x DISC_SLOT_UUS       rank in 0..3
```

With `n_groups = 8` and 32 anchors every group holds at most 4 anchors, so the
window stays 12.5 ms and `DISCOVERY_WINDOW_MS` (15) and `TX_COMPLETE_TIMEOUT_MS`
(18) are **unchanged and still correctly derived** — the grouping is what
preserves both. A tag sweeps `group = round mod n_groups` and sees the whole
deployment in 8 rounds.

Tag-side pool grows `ANCHOR_POOL_MAX` 6 -> **16**; `ANCHOR_SELECT_MAX` stays
**4**, because the slot budget in §5 and `POS_MAX_ANCHORS` both assume 4, and the
existing plan's constraint ("keep 4 anchors, the budget already assumes 4") still
holds. More anchors means a *better* 4, not a bigger sweep.

### D. Addressed ranging, and anchor `z`

The multi-poll response is a new type, `0xED` MPOL_RESP, with a real header
(`dest` = the polling tag's short address, `src` = the anchor's) and the payload
the tag already parses, **plus `z`**:

```
[hdr 0..9][anchor_id 10][poll_rx_ts 11..14][resp_tx_ts 15..18][x 19..22][y 23..26][z 27..30]   len 31
```

Three things fall out. Cross-talk between tags becomes impossible rather than
merely improbable. The DW3000 hardware frame filter becomes usable on the ranging
path. And **`z` closes the open question in the position-filtering spec** — "are
all four anchors at the same height, and if not, per-anchor z has to go into the
frame and anchor firmware lands on the critical path". It is on the critical path
now anyway; adding a float while the format is already broken costs nothing, and
a 32-anchor zone map will certainly not be flat. `pos_cfg`'s single `dz` becomes
the fallback for anchors that report `z` as NaN.

The unaddressed WAVE pair stays exactly as it is, for calibration only. Do not
unify the two paths — that rule from `CLAUDE.md` is unchanged and is now load
bearing in the other direction too: `cal` must keep working against a stock Qorvo
responder that knows nothing about v3.

### E. Lease from POS, and the address pool

- **The gateway refreshes a tag's lease when it decodes that tag's `0xEA` POS
  frame.** It already receives and decodes POS (`uwb_gateway.c` dispatch), and a
  POS frame is proof of liveness strictly stronger than a KEEPALIVE. A tag then
  sends KEEPALIVE only when it has gone `KEEPALIVE_AFTER_N` participations with
  no fix. CAP traffic at 100 tags drops from ~4 frames/superframe to
  approximately zero, and the JOIN storm that CAP congestion would have caused
  never starts.
- **JOIN backoff spreads over up to 64 superframes**, randomised per EUI. 100
  tags rejoining after a gateway restart otherwise collide in 4 mini-slots
  indefinitely.
- **Short addresses are reused by EUI.** `alloc_short_addr()` keeps a small
  EUI->address map so a rejoining tag gets its previous address. This also
  removes the `Tid` phantom-device case the gateway currently papers over
  downstream.

---

## 4. Wire format v3 — the complete delta

`proto_ver` = **3**. Version constants: `N_CFP` = **14**, `N_CAP` = 4,
`C` (cycle) = **16**, `A` (announce cycle) = **37**, `T_superframe` = 200 ms
(unchanged).

| Frame | v2 | v3 | Change |
|---|---|---|---|
| `0xE5` BEACON | 15 + 2x11 = 37 | 16 + 2x14 = **44** | `announce_id` byte at 15; slot map 14 entries |
| `0xE2` DISCOVERY | 14 | **16** | `group`, `n_groups` |
| `0xE4` DISC-RESP | 20 | 20 | unchanged |
| `0xE3` MULTI-POLL | 15 + 4n | 15 + 4n | unchanged (finally used) |
| `0xED` MPOL_RESP | — | **31** | new: addressed ranging response with `z` |
| `0xEC` ANNOUNCE | — | **~24** | new: anchor self-advertisement |
| `0xE7` GRANT | 24 | **26** | 16-bit phase mask |
| `0xE6/E8/E9` | as-is | as-is | unchanged |
| `0xEA` POS | 24 | 24 | unchanged (now also renews the lease) |
| `0xEB` ALERT | 34 | 34 | unchanged |

`UWB_FRAME_MAX_LEN` goes 37 -> **44**. Both firmwares' RX buffers are already 64
bytes (`RX_BUF_LEN` on the anchor, `beacon_buf[UWB_FRAME_MAX_LEN]` on the tag),
so no buffer grows by hand — but `uwb_slave.c`'s `BUILD_ASSERT` tying
`UWB_FRAME_MAX_LEN` to `UWB_FRAME_N_CFP` must be re-derived, not deleted.

`uwb_frame_802_15_4z.c` stays **byte-identical between the two repos**. That rule
is unchanged and this change set is the largest test of it so far.

---

## 5. Timing and capacity budget (v3)

```
T_superframe                                        200.0 ms
  BEACON (44 B)                             2.0
  guard                                     1.0
  ANNOUNCE window (1 anchor, 24 B)          2.0
  guard                                     1.0
  CAP: 4 x t_minislot                       8.0
  guard                                     1.0
                                          -------
  overhead                                 15.0 ms
  CFP available                           185.0 ms
  T_slot 12 + guard 1                      13.0 ms
  N_CFP = floor(185 / 13)                =    14 slots
```

```
seats         = N_CFP x C = 14 x 16   = 224 phase-seats
cycle length  = 16 x 200 ms           = 3.2 s
1 phase                               = 0.31 Hz
100 tags at 1 phase                   = 100 seats used, 124 free
a mover at 4 phases                   = 1.25 Hz
a mover at 8 phases                   = 2.50 Hz
124 free phases                       = 31 concurrent movers at 4 phases, or 15 at 8
```

Sanity check against the requirement: **100 tags fit with 55 % of the phase
budget still free.** The binding constraint is not the tag count, it is how many
of them move at once — which is precisely what the motion tiers already measure.

`T_SLOT_MS` = 12 assumes multi-poll at the current 2000 uus turnaround. If the
turnaround tightening (§3.B) lands, the slot goes to ~11 ms and `N_CFP` to 15
(240 seats). Do not bank it before it is measured.

---

## 6. What this does to power

Better, not worse. A stationary tag participates in 1 superframe out of 16
instead of every one, which is what `beacon_sched_core` and the whole Layer-3
design were built for — except now the gateway *knows*, so the seat cannot be
lost by sleeping and `UWB_LISTEN_SKIP_CAP` stops being a workaround.
`listen_skip` becomes a derived quantity (`C / n_phases`, i.e. 16 for one phase)
rather than a setting the tag guesses at.

The moving case is unchanged from the position-filtering design's finding: a
mover at 4-8 phases is awake 4-8 times per 3.2 s, which is the same
current-per-fix trade already documented there. The EKF's authority while walking
improves, since 1.25-2.5 Hz is where a constant-velocity prediction is still
meaningful (at 0.2 Hz it is not — see that spec's open decision (a)).

---

## 7. Anchors: 4 -> 32

### 7.1 Constants and validation

`UWB_MAX_ANCHORS` 4 -> **32**, `anchor id <0..31>`, `UWB_FRAME_MAX_ANCHORS` stays
**4** (it bounds one multi-poll's slot list, not the deployment).
`apos_node.c`'s `RANGE_CMD` peer check follows `UWB_MAX_ANCHORS` automatically.
The wire cap is **254** (`UWB_ANCHOR_ADDR_BASE + id` must stay below 0x0100 so
the low byte stays unique); 32 is the supported number, 254 is the number the
format cannot exceed — write both down, enforce the first.

### 7.2 Discovery, announce and the two derived timeouts

Per §3.C. The two constants that look unrelated and are not:
`DISCOVERY_WINDOW_MS` (tag) and `TX_COMPLETE_TIMEOUT_MS` (anchor) are both
derived from the **maximum stagger**, and grouping is what keeps that maximum at
`rank 3` = 12.5 ms regardless of deployment size. Any future change to
`n_groups`, `DISC_BASE_UUS` or `DISC_SLOT_UUS` must re-derive both, on both
sides, in the same commit.

### 7.3 The survey (`apos`) is the hardest part of 32 anchors

`APOS_MAX_NODES` 8 -> 32 is the easy half. The hard half is that the survey's
implicit assumption — every ordered pair can range — is false at 100 m spacing:

- **The mesh becomes sparse.** `apos_geom.c` already solves a flat edge list, so
  sparsity is representable; what is missing is a **rigidity check** (a sparse
  framework can be flexible or reflection-ambiguous even with many edges) and a
  **seeding strategy** that grows from the gauge outward instead of assuming a
  full mesh.
- **The run time explodes.** Ordered pairs go from 12 to `32 x 31` = 992, at one
  frame per `apos_gw_step()` call and one step per superframe. A full survey is
  then measured in *hours*. The fix is to range only **candidate** pairs — those
  that heard each other during `apos enum` — and to allow non-overlapping pairs
  to range in the same superframe.
- **`rms_mm` finally becomes meaningful.** The documented degeneracy (4 anchors
  in 3D is isostatic, `rms` identically zero) is a small-N artefact. At 32 nodes
  with a sparse-but-redundant mesh there is real redundancy, so the acceptance
  test can stop being "a tape measure" — provided the rigidity check above exists
  to say when it can be trusted.
- The MQTT anchors payload (`pos_json_anchors()`) grows ~150 B x 32 ~ 4.8 kB and
  will need chunking or a size check against the broker's limit.

### 7.4 One gateway still owns the time base

32 anchors over a 100 m site is one PAN with one beacon, per the contract. If the
site is larger than the beacon's own coverage, the beacon must be extended — the
contract's own open item §8. It is **out of scope here** and §10 records the
options. Do not solve it by adding a second independent gateway: two
unsynchronised beacons on one PAN is not a degraded network, it is two networks
colliding.

---

## 8. PHY: what changes, what deliberately does not

**Keep channel 5, PLEN-1024, PAC32, code 9, 850 kbps, SFD 4z-8, STS off, PDoA
off.** The requirement allows changing the PHY on every device; the analysis says
not to, and the reasons are worth writing down because each is a plausible
mistake:

- **Do not switch to 6.8 Mbps for capacity.** At PLEN-1024 the preamble is
  ~1.05 ms and the payload of a 31-byte frame is ~0.29 ms. Going 8x faster on the
  data saves ~0.25 ms per frame — about 2 % of a slot — and costs 3-4 dB of
  sensitivity, i.e. a third of the range budget, in a design where §2.3 shows the
  budget is already at zero. Capacity comes from §3.A and §3.B, which are free in
  dB terms.
- **Do not lengthen the preamble for range.** PLEN-2048/4096 buys ~1.5-3 dB and
  doubles or quadruples the airtime of every frame in the system, taking the slot
  back over 20 ms and cancelling §3.B. If dB are needed, buy them in antennas and
  an anchor LNA, which cost no airtime.
- **Keep channel 5, not 9.** Channel 9 is 1.8 dB worse in free-space loss before
  any material penalty. Reserve ch9 as the second colour if spatial reuse is ever
  built (§10).
- **Fix the SFD-timeout divergence.** The tag computes `(1024 + 1 + 8 - 8)` =
  1025 (`src/phy_config.c`, whose comment says "PAC 8") while the same struct sets
  `DWT_PAC32`; the anchor computes `(1025 + 8 - 32)` = 1001 with the same PAC32.
  The anchor is right. This is a real divergence in a contract both sides claim
  to share, it has been invisible because an over-long SFD timeout mostly costs
  RX-on time rather than frames, and it must be unified before any range
  measurement is trusted. **This is the one PHY change v3 makes**, and it is a
  bug fix, not a tuning decision.
- **Antenna delay recalibration is required anyway.** `cal` values are
  PHY-specific and the record is invalidated by `phy_option`. Since the SFD
  timeout is inside the PHY struct, treat v3 as a re-calibration event for the
  whole fleet — tag (`cal <mm>`) and anchors (`cal ref` + `cal peer`).

---

## 9. Test strategy — how to claim 100 tags without 100 tags

Three layers, because the claim cannot be made by any one of them:

1. **Host tests, pure C.** The seat table with phases, the phase allocator, the
   grouped-discovery rank function, the announce coprimality, the multi-poll
   builder/parser, the address-reuse pool. All of it is arithmetic and belongs in
   the existing gcc harnesses (`tests/uwb_net/`, `tests/uwb_frame/`,
   `ANCLA/tests/gw_core/`, `ANCLA/tests/disc_schedule/`).
2. **Synthetic full occupancy on real hardware.** A gateway console command
   (`gw seed <n>`) fills the seat table with `n` synthetic seats and lets the real
   beacon publish real maps at full 224-seat occupancy, with no RF from phantom
   tags. This proves the beacon stays on time, the maps are consistent across a
   whole cycle, and the ESP32-S3 holds its `K_PRIO_COOP(0)` deadline with a 3 kB
   table — the three things that actually break at scale.
3. **Real air with the tags that exist.** 3-5 real tags, forced onto adversarial
   phase assignments (adjacent slots, same phase, phase 0 next to the beacon),
   plus a sniffer. This proves slot alignment and the absence of cross-talk; it
   cannot prove capacity, and no one should claim it does.

The number to watch throughout is the one already built: `pwr rx` on the tag and
the gateway's per-superframe JSON.

---

## 10. Explicitly deferred, with the price of each

- **Spatial reuse / zones (>224 seats, or one site larger than one beacon).**
  Needs addressed ranging (§3.D, landing here), a zone id, per-zone superframe
  offsets or a second channel, and tag handover hysteresis. The MAC contract's §8
  open item. Everything in v3 is compatible with it; nothing in v3 implements it.
- **Multi-hop or multi-emitter beacon** for sites beyond one beacon's coverage.
  Same open item. Wired-synchronised emitters are the lower-risk of the two.
- **Sub-200 ms superframe.** Would raise the mover ceiling above 2.5 Hz and halve
  every capacity number in §5. The position-filtering spec already concluded
  10 Hz is unreachable this way and that host-side extrapolation from the EKF's
  published velocity is the right answer.
- **TDoA.** Evaluated and abandoned for precision; nothing here revisits it. Note
  that §3.D's addressed frames plus anchor `z` would be prerequisites for any
  future hybrid anyway.

---

## 11. Blocking unknowns — these gate the plan, not the design

1. **The ANCLA ~25 dB transmit deficit** (§2.3). Until root-caused, no range
   number from this system means anything, and Phase 4 cannot start.
2. **Anchor RX sensitivity as built**, measured, not from the datasheet — the
   anchor has no LNA and it is the receiving end of the weak link.
3. **The real per-slot cost of multi-poll on hardware**, including the tag's SPI
   and processing overhead between responses. §5's `T_SLOT_MS = 12` is derived
   from airtime plus margin, and the current 24 was itself set by a measurement
   that came in far above its own airtime estimate.
4. **The measured anchor turnaround floor** at the 26.67 MHz SPI rate, which
   decides whether §3.B's second-order lever exists.
5. **Whether the ESP32-S3 gateway holds the beacon deadline** with a 224-seat
   table and 14 slots of CFP RX per superframe.
6. **Anchor mounting heights** for the `z` field, and whether the survey can
   reach them (a ceiling anchor 100 m from its neighbour may be in nobody's
   range).
