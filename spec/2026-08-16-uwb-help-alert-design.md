# UWB HELP / CANCEL alert — design

Status: **design only, nothing implemented.** Covers Open Work item 5 (“double button
press should transmit a UWB help/alert frame”).

Goal: a button gesture on the tag raises a HELP condition that reaches the gateway
even when no anchor is in direct range of the tag *and* the tag is in direct range of
no gateway. The gateway **latches** the condition so the platform can advertise it; a
CANCEL clears the latch.

> **Working on the anchor firmware? Start here.** The anchor is a separate codebase, so
> the requirements land as a contract rather than as code in this repo:
>
> - **§6 is the anchor contract** — accept/validate, dedup, local logging, relay
>   decision, relay mechanics, and an explicit list of things an anchor must *not* do.
> - **Anchors do not parse the gateway beacon today**, so there is no learned hop
>   gradient. `hop_to_gw` is a **static value in the anchor's existing flash record** at
>   `0x1E000`, set from the anchor console next to `anchor_x`/`anchor_y` — see §4.1.
>   Unset (`0xFF`) must still relay, bounded by TTL and dedup.
> - **A console override for `hop_to_gw` is required, not optional.** The deployment is
>   single-hop, so the relay path never runs in the field; the override is the only way
>   to exercise it on a bench before it matters.
> - **`src/alert_relay.c` is meant to be compiled into the anchor firmware too.** Do not
>   reimplement the dedup cache, gradient decision, or latch ordering by hand — that is
>   exactly where two independent implementations drift, and the epoch-ordering rules in
>   §3 are the part that is impossible to eyeball.

---

## 1. One frame type, not two

The maintainer's sketch was two message types (HELP and CANCEL). Recommendation:
**one frame type `0xEB` (ALERT) with a `state` byte** (`0x01` = HELP, `0x00` = CANCEL).

Reasons:

- Every anchor-side behaviour — dedup, TTL, gradient relay, backoff, storm
  suppression — is byte-for-byte identical for both. Two types means two copies of
  the relay path in the anchor firmware, and the CANCEL copy is the one nobody
  bench-tests.
- Dedup and latch ordering key on the *same* `(EUI, epoch)` pair for both. Splitting
  the types invites the two caches to disagree.
- A CANCEL that is treated as a lower-priority citizen than the HELP is the classic
  way to end up with a stuck latch. Same type, same path, same priority.

The distinction the platform cares about is one bit; make it one bit.

## 2. Frame layout (`0xEB`)

Follows the existing 10-byte header (`write_hdr`, `src/uwb_frame_802_15_4z.c:55`).
`dest` = `UWB_ADDR_GATEWAY` (0x0000) — the routing target, not the next hop.
`src` = whoever is transmitting *this* copy (the tag, or the relaying anchor).

| Off | Len | Field | Notes |
|---|---|---|---|
| 0–9 | 10 | common header | `type = 0xEB`, `dest = 0x0000` |
| 10 | 1 | `state` | bit0: 1 = HELP, 0 = CANCEL. bits1–7 reserved, must be 0 |
| 11 | 1 | `epoch` | per-tag monotonic alert id, survives reset (NVS) |
| 12 | 1 | `repeat_seq` | 0 on the first TX, +1 per 10 s retransmission |
| 13 | 1 | `sender_hop` | hop distance to gateway of the node transmitting this copy; `0xFF` = unknown/originator |
| 14 | 1 | `ttl` | decremented per relay, dropped at 0. Start at 6 |
| 15–22 | 8 | `orig_eui` | **originator identity** |
| 23–24 | 2 | `orig_addr` | originator short addr, `0xFFFE` if unassociated |
| 25 | 1 | `batt_soc` | `0xFF` = unknown (same sentinel as POS) |
| 26–29 | 4 | `last_x` | float32 LE, NaN if the tag has no fix |
| 30–33 | 4 | `last_y` | float32 LE, NaN if no fix |

`UWB_FRAME_LEN_ALERT = 34`, excl. FCS — under the existing `UWB_FRAME_MAX_LEN` of 37,
so no buffer sizing changes anywhere.

Three points that are easy to get wrong:

- **`orig_eui` is mandatory and must not be “optimised” down to the short address.**
  On relay, `src` becomes the *anchor's* address, so the header can no longer identify
  the tag. And a tag that never joined has `orig_addr = 0xFFFE`, which is not unique —
  precisely the tag most likely to need help.
- **`last_x`/`last_y` are worth the 8 bytes.** A help alert whose only location
  information is “some anchor heard it” is much weaker than one carrying the last
  solved fix. Send NaN when there is none rather than 0,0.
- **`repeat_seq` is what makes each 10 s repeat flood again.** Dedup on
  `(eui, epoch, state)` alone would cause anchors to swallow every repeat after the
  first, defeating the whole point of repeating.

## 3. Epoch and latch semantics (the part that actually matters)

The failure mode to design against is **a stale in-flight HELP re-latching an alert
the user already cancelled** — perfectly possible in a multi-hop flood with per-hop
backoff, where a HELP can arrive seconds after the CANCEL that followed it.

Tag side:
- `epoch` increments on each *new* HELP (button gesture), never on a repeat.
- Persisted to NVS (new storage id 3, alongside cal=1 / NFC name=2) so it stays
  monotonic across a reset. This also lets a tag that resets mid-emergency resume the
  alert instead of going silent — which is the behaviour you want.
- CANCEL is sent with **the epoch being cancelled**, not a new one.

Gateway side, per originator EUI, holds `{active, epoch, last_cancelled_epoch}`:

- `HELP(e)`:
  - `e == last_cancelled_epoch` → **ignore** (stale in-flight, the whole reason epochs exist)
  - `active && e == epoch` → refresh the last-heard timestamp only
  - otherwise, if `e` is newer than `epoch` → latch active with epoch `e`
- `CANCEL(e)`: clear the latch, set `last_cancelled_epoch = e` — **even if no HELP for
  `e` was ever received** (that HELP may simply never have made it out).
- `epoch` is u8 and wraps; compare with serial arithmetic, `(int8_t)(a - b) > 0`.

The latch does **not** auto-expire on silence. A tag going out of range while still in
trouble must not clear the alarm. A lost CANCEL therefore degrades to “the operator
dismisses it in the platform UI” — that human path is required, not optional, and
should be stated in the platform spec.

## 4. Propagation: gradient-biased flood, one hop per superframe

The requirement is “an anchor that hears it carries it toward the gateway.” Full
routing tables are not worth it for a frame that fires once a year. Use a **controlled
flood with a hop gradient**:

- Every node has `hop_to_gw`: gateway = 0, an anchor one hop from it = 1, and so on.
  Unknown = `0xFF`. **Anchors do not parse the gateway beacon today** (confirmed by the
  maintainer), so this value is *not* learned over the air in v1 — see §4.1.
- On receiving an ALERT, an anchor relays it only if `sender_hop > my_hop` — i.e. only
  if relaying moves the frame *closer* to the gateway. It rewrites `sender_hop = my_hop`
  and `ttl -= 1`.
- The originating tag transmits with `sender_hop = 0xFF`, so any anchor will pick it up.
- An anchor with `my_hop == 0xFF` (no gradient yet) relays anyway, leaving `sender_hop`
  at `0xFF`, bounded by TTL and dedup.

**Correctness comes from the dedup cache and TTL, not from the gradient.** The gradient
is a storm suppressor. Do not write anchor code that relies on the hop metric being
correct for loop freedom.

### 4.1 Where `hop_to_gw` comes from (v1: static, in anchor flash)

Anchors ignore the beacon, so there is no over-the-air gradient to learn from. Rather
than build one, **store `hop_to_gw` in the anchor's existing flash record** — the same
`flash_record_t` at `0x1E000` that already holds `anchor_x`/`anchor_y`, set from the
anchor console (`set hop_to_gw <n>`, `save`).

This is the right trade for v1:

- The deployment is single-hop today, so every anchor is simply `1`. Nothing to derive.
- Anchor coordinates are *already* hand-configured per unit; hop distance is one more
  field in a commissioning step that exists.
- It is deterministic and inspectable, which matters for a safety feature — an
  emergency path whose routing depends on a learned metric that nobody can read back
  is hard to trust.

Default when unset: `0xFF` (unknown). Per §6.4 an unknown-hop anchor still relays,
bounded by TTL and dedup, so a forgotten commissioning step degrades to a plain flood
rather than a black hole. That fallback is the reason this is safe to ship static.

When the network does go multi-hop, either extend commissioning or add beacon parsing
on the anchor and derive `hop_to_gw` from it — the relay logic in §6 does not change
either way, only the source of one byte.

**Testability note.** With a single-hop deployment the relay path never executes in
the field. Add a bench override — an anchor console command forcing `hop_to_gw` to an
arbitrary value — so a two-anchor bench setup can be made to relay (anchor A at hop 2,
anchor B at hop 1) and the path is exercised before it is ever load-bearing. Untested
relay code that first runs during an actual emergency is not acceptable for this
feature.

**Relay timing: one hop per superframe, in the CAP.** Anchors defer a relay to the next
superframe's CAP mini-slots (Aloha, random mini-slot, same as JOIN/KEEPALIVE). This:

- never collides with a beacon or a CFP ranging slot, so it cannot perturb positioning;
- naturally paces the flood — a 4-hop network delivers in ≤ 4 × 202 ms ≈ 0.8 s, which
  is far inside human reaction time for a help alert;
- needs no new superframe window and no gateway protocol change.

The cost is CAP contention with JOIN/KEEPALIVE. With 4 mini-slots and alerts measured
in “events per day”, that is fine. If it ever isn't, the fix is a dedicated post-CAP
alert window, not immediate relay.

## 5. Tag side

**Where the frame is sent.** Exactly like `position_publish()`: from the **runner
thread**, inside a block in `runner_fn`, with no separate `uwb_radio_owner` claim
(the runner already owns the radio there) and no new `wait_event()` caller. A
`tag_alert.c` module holds the state and hands the runner a built frame; the runner
transmits it.

**When.** A new `UWB_ACT_SEND_ALERT` emitted by `uwb_net_handle()` from **every state
including `UWB_ST_SCAN`** — an unjoined tag is exactly the case the user described
(“in case no anchor can hear it”). Transport:

- joined → transmit in the tag's own CFP slot, alongside/instead of the POS frame;
- unjoined → random CAP mini-slot, same Aloha path as JOIN.

10 s ≈ 50 superframes, so the alert coexists with deep sleep with no measurable power
cost (one 34-byte frame per 10 s).

**Not cal-gated.** `UWB_ACT_SEND_ALERT` must stay out of `UWB_ACT_RANGING_MASK`
(`src/uwb_net.h:71`). An alert carries no TWR result; an uncalibrated tag must still be
able to shout for help. Add this to `tests/uwb_net/test_uwb_net.c::test_gate_actions()`.

**Repeat policy.**
- HELP repeats every 10 s for as long as the alert is active — indefinitely. v1 has no
  ack, so the tag cannot know it got through; repeating is the only reliability
  mechanism it has.
- CANCEL repeats every 10 s for a bounded number of transmissions (suggest 6 → 60 s),
  then stops. Bounded because a CANCEL that never gets through cannot be fixed by
  repeating forever, and the operator UI is the backstop.

**Gesture and feedback** (`tag_ui.c`). Double press currently just blinks orange. Decided:

- **double press → raise HELP**; LED slow red pulse, persistent, as the standing
  indicator that an alert is active;
- **long press ≥ 3 s → CANCEL**; LED green flash, then dark.

This requires a real change to the button path, not just the gesture decision.
`button_isr` currently discards the release edge (`tag_ui.c:95` returns early unless the
pin reads pressed), so there is no way to measure a hold. Both edges must be
timestamped and pushed to `press_q` with a pressed/released marker, and `ui_fn`'s
gesture decision extended to recognise a hold — while keeping the existing single-press
battery readout and double-press behaviour intact. Two consequences worth planning for:

- the queue entry grows from a bare `uint32_t` to a `{timestamp, edge}` pair, so
  `press_q`'s element size and the existing `k_msgq_put`/`get` call sites all change;
- a hold that exceeds 3 s should fire the CANCEL **on the 3 s mark, not on release**, so
  the user gets LED confirmation while still holding. That means the UI thread waits on
  the queue with a timeout rather than blocking on the release edge.

The asymmetry is deliberate: raising an alert is easy, clearing one is not. An
accidental cancel of a real emergency is the worst outcome this feature can produce.

**Bench commands** in `tag_cmd.c`: `help on`, `help off`, `help` (status: active,
epoch, repeat_seq, last TX). Keep each NUS line under 20 bytes.

## 6. Detailed note: what an anchor must do on receiving `0xEB`

This is the contract for the anchor firmware. It is written to be implementable
without reference to tag internals.

### 6.1 Accept

1. Validate the header: length == 34, PAN `0xCA 0xDE`, `type == 0xEB`. Reject otherwise.
2. Reject if `state` has any reserved bit set (forward compatibility: unknown
   semantics must not be relayed as if understood).
3. Reject if `orig_eui` equals this node's own EUI (should not happen on an anchor, but
   the same rule on a tag prevents it re-processing its own relayed alert).

### 6.2 Deduplicate — before anything else

Keep a cache of **16 entries**, key `(orig_eui, epoch, state, repeat_seq)`, each with a
first-seen timestamp, aged out after **30 s**.

- Key already present → **drop silently, do not relay.** This is what makes the flood
  terminate; it is not an optimisation.
- Key absent → insert, continue.
- Cache full → evict oldest.

The 30 s age must exceed the 10 s repeat interval, so a genuine repeat (`repeat_seq+1`)
is a different key and floods again, while echoes of the *same* copy arriving over
different paths within a hop-time or two are suppressed.

### 6.3 Report locally

Every accepted (non-duplicate) alert should be recorded by the anchor with:
`orig_eui`, `state`, `epoch`, `repeat_seq`, `sender_hop`, RSSI/CIR quality of this
reception, and the anchor's own timestamp. This is the raw material for the platform to
place an alert from a tag that has no fix (trilaterate, or at minimum “nearest anchor”)
— see §8.

### 6.4 Relay decision

Relay if **all** hold:

- `ttl > 0`;
- `sender_hop > my_hop`, **or** `my_hop == 0xFF` (no gradient yet);
- this node is **not** the gateway (the gateway consumes, never relays).

Otherwise drop. In particular, `sender_hop <= my_hop` means the transmitter is already
at least as close to the gateway as this node — relaying would push the frame backwards.

### 6.5 Relay mechanics

- Rewrite `sender_hop = my_hop`, `ttl -= 1`. **Leave `orig_eui`, `orig_addr`, `epoch`,
  `repeat_seq`, `state`, `batt_soc`, `last_x`, `last_y` byte-for-byte untouched.**
- Set the header `src` to this anchor's own short address; keep `dest = 0x0000`.
- Transmit in a **random CAP mini-slot of the next superframe**. Never in a CFP slot,
  never in the beacon window, never immediately.
- **Suppression while queued:** if, before the queued relay goes out, the anchor
  overhears the same dedup key transmitted by a node with `sender_hop <= my_hop`, it
  **cancels its own relay**. Someone closer to the gateway already carried it.
- One relay per dedup key. Never retransmit a relay; the originator's 10 s repeat is
  the retry mechanism.

### 6.6 What an anchor must NOT do

- **No TWR with the alerting tag.** An alert is not a ranging exchange; do not send an
  E1-style response to it.
- **Do not preempt normal duties.** A pending relay never delays a beacon, a scheduled
  E1 response to a ranging poll, or a discovery E4 response. Positioning for every other
  tag in the cell must keep working through an alert.
- **Do not latch.** The latch lives at the gateway/platform only. An anchor holds no
  alert state beyond the 30 s dedup cache and its local log.
- **Do not treat CANCEL as lower priority.** Identical path, identical eagerness. An
  anchor relays a CANCEL even if it never saw the corresponding HELP.
- **Do not change the gradient because of an alert.** `hop_to_gw` is learned from
  beacons, not from alert traffic.

### 6.7 Gateway (hop 0)

Consumes, never relays. Applies the latch state machine of §3 and pushes to the
platform. Deduplicates on the same key so repeats and multi-path copies do not produce
duplicate platform events — only a last-heard refresh.

## 7. Testing

Host-compilable, per the repo's split-the-pure-logic pattern:

- `tests/uwb_frame/` — extend for `uwb_frame_alert_build` / `_parse` / `_is_alert`:
  round-trip, NaN coordinates, truncated frames, reserved-bit rejection.
- **`src/alert_relay.c` — new pure module holding the dedup cache, the gradient relay
  decision, and the latch state machine.** Compile the *same file* into both the tag and
  the anchor firmware. The relay rules above are exactly the kind of thing that drifts
  between two independently written implementations, and the epoch-ordering logic is
  the part that is hard to get right and impossible to eyeball. Host test in
  `tests/alert_relay/`: stale-HELP-after-CANCEL, epoch wrap, dedup eviction, TTL
  expiry, gradient rejection, unknown-hop relay.
- `tests/uwb_net/` — `UWB_ACT_SEND_ALERT` survives `uwb_net_gate_actions()` with
  `cal_valid == false`, and is emitted from `UWB_ST_SCAN`.

## 8. Deferred to v2 (deliberately not in v1)

- **Ack.** Gateway confirms receipt so the tag can stop repeating and show a green LED
  (“help is on its way”), which is a genuine user-experience gain. Cheapest carrier is
  a field in the beacon the tag already listens to every superframe — but the tag may
  be unassociated, so it would have to key on the low 2 bytes of the EUI. Needs a
  gateway protocol change; not worth blocking v1.
- **Anchor witness reports.** An anchor emitting its own frame carrying its coordinates
  and the measured RSSI of the alert, letting the platform localise a tag that has no
  fix of its own. High value for exactly the emergency case; a separate frame type and
  a separate design.
- **Dedicated alert window** if CAP contention ever becomes real.
- **Learned hop gradient** (beacon parsing on the anchor, or a gateway hop
  advertisement) replacing the static flash value of §4.1, when the network goes
  multi-hop in earnest.
- **Remote cancel from the platform.** Deferred by decision. It needs a gateway→tag
  downlink this protocol does not have — an addressed frame routed back down the flood —
  plus authentication, since an unauthenticated downlink that silences an alert is a
  worse failure than no downlink at all.

## 9. Decisions taken (2026-08-16)

Resolved with the maintainer; recorded so they are not re-litigated.

1. **Deployment is single-hop today, multi-hop planned.** Build the full relay path now;
   it is dormant in the field until the network grows. Because it will not execute in
   normal operation, the bench override in §4.1 is **required, not optional** — the relay
   path must be exercised before it is ever depended on.
2. **Anchors do not parse the gateway beacon.** `hop_to_gw` is therefore static, stored
   in the anchor's existing flash record next to `anchor_x`/`anchor_y` (§4.1), with
   `0xFF`/unknown degrading to a TTL- and dedup-bounded flood.
3. **Cancel gesture is a long press ≥ 3 s**, firing on the 3 s mark rather than on
   release. Requires timestamping the button release edge (§5).
4. **Remote cancel is out of scope for v1** — uplink only. The operator dismisses the
   latch in the platform UI; only the on-device gesture stops the tag repeating. This
   makes the platform-side dismissal path a hard requirement, not a nicety.

## 10. Implementation order (suggested)

Each step is independently testable, and the first three need no anchor firmware change.

1. `uwb_frame_alert_build`/`_parse`/`_is_alert` + host tests in `tests/uwb_frame/`.
2. `src/alert_relay.c` (dedup cache, gradient decision, latch FSM) + `tests/alert_relay/`.
   Pure, host-tested, and compiled into both firmwares.
3. Tag: `tag_alert.c`, NVS epoch (storage id 3), `UWB_ACT_SEND_ALERT` in `uwb_net.c`
   with the gate-mask test, runner TX in-slot/CAP, `help on|off` NUS commands. At this
   point a bench gateway can see HELP and CANCEL from a tag with no anchor changes at all.
4. Tag: button release-edge handling and the long-press gesture, LED states.
5. Anchor: accept/dedup/log, `hop_to_gw` in flash + bench override, relay in CAP with
   overheard-copy suppression.
6. Gateway/platform: latch, epoch ordering, operator dismissal in the UI.
