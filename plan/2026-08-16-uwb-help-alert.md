# UWB HELP / CANCEL Alert Implementation Plan

> **For agentic workers:** steps use checkbox (`- [ ]`) syntax for tracking. Implement
> task-by-task, in order; each task ends in a state that builds and tests clean.

**Design doc:** `spec/2026-08-16-uwb-help-alert-design.md` — read it first. This plan
implements it and does not restate the rationale.

**Goal:** A button gesture raises a HELP condition that reaches the gateway even when
the tag is unjoined and out of direct gateway range, repeating every 10 s until
cancelled. The gateway latches it; a CANCEL clears the latch. Anchors relay toward the
gateway.

**Architecture:** A new `0xEB` ALERT frame in the existing frame module. Two pure,
host-tested cores: `alert_relay` (dedup cache + gradient relay decision + gateway latch
ordering — compiled into **both** tag and anchor firmware) and `tag_alert_core` (the
tag's own alert state: epoch, active, repeat scheduling). Zephyr glue in `tag_alert.c`
(NVS-backed epoch, thread-safe accessors). The runner transmits the frame from its own
thread exactly like `position_publish()` — no `uwb_radio_owner` claim, no new
`wait_event()` caller.

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
  for the alert TX — the runner already owns the radio where the TX happens.
- **Do not touch anchor or gateway firmware from this repo.** Task 2 produces the module
  the anchor will compile; the anchor-side integration is a separate effort.
- Tuning constants, all in one place in `tag_alert_core.h`: `ALERT_REPEAT_MS = 10000`,
  `ALERT_CANCEL_REPEATS = 6`, `ALERT_TTL_INIT = 6`, `ALERT_DEDUP_N = 16`,
  `ALERT_DEDUP_AGE_MS = 30000`.

---

### Task 1: `0xEB` ALERT frame builder + parser

**Files:**
- Modify: `src/uwb_frame_802_15_4z.h`, `src/uwb_frame_802_15_4z.c`
- Test: `tests/uwb_frame/test_uwb_frame.c` (extend)

**Interfaces produced:**

```c
#define UWB_FRAME_TYPE_ALERT   0xEB
#define UWB_FRAME_LEN_ALERT    34
#define UWB_ALERT_STATE_CANCEL 0x00
#define UWB_ALERT_STATE_HELP   0x01
#define UWB_ALERT_HOP_UNKNOWN  0xFFu
#define UWB_ALERT_TTL_INIT     6

struct uwb_alert {
    uint8_t  state;        /* bit0 HELP/CANCEL; bits1-7 reserved, must be 0 */
    uint8_t  epoch;
    uint8_t  repeat_seq;
    uint8_t  sender_hop;
    uint8_t  ttl;
    uint8_t  orig_eui[8];
    uint16_t orig_addr;
    uint8_t  batt_soc;
    float    last_x, last_y;   /* NaN when the tag has no fix */
};

int  uwb_frame_alert_build(uint8_t *buf, size_t buf_len, uint16_t src_addr,
                           const struct uwb_alert *a);
bool uwb_frame_is_alert(const uint8_t *buf, size_t len);
int  uwb_frame_parse_alert(const uint8_t *buf, size_t len, struct uwb_alert *a);
```

Field offsets (add next to the existing `OFF_POS_*` block): `OFF_AL_STATE 10`,
`OFF_AL_EPOCH 11`, `OFF_AL_REP 12`, `OFF_AL_HOP 13`, `OFF_AL_TTL 14`, `OFF_AL_EUI 15`,
`OFF_AL_ADDR 23`, `OFF_AL_SOC 25`, `OFF_AL_X 26`, `OFF_AL_Y 30`.

`uwb_frame_alert_build` writes the header with `dest = UWB_ADDR_GATEWAY`, `src = src_addr`
(caller's address — the tag's own, or the relaying anchor's), `type = 0xEB`.

- [x] **Step 1: Tests first.** Extend `tests/uwb_frame/test_uwb_frame.c`:
  - round-trip: build → parse returns every field bit-identical, including a NaN
    `last_x`/`last_y` (compare via `memcmp` on the raw bytes, **not** `==` — NaN never
    compares equal to itself and a naive `CHECK(a.last_x == b.last_x)` fails on a
    correct implementation);
  - `uwb_frame_is_alert` rejects: wrong length (33, 35), wrong type byte, bad PAN;
  - `uwb_frame_parse_alert` rejects a frame with any reserved bit of `state` set;
  - `uwb_frame_alert_build` returns `-EMSGSIZE` for `buf_len = 33`, `-EINVAL` for NULL.
- [x] **Step 2: Implement** in `uwb_frame_802_15_4z.c`, reusing `write_hdr`, `put_u16`,
  `put_f32`, `get_f32`.
- [x] **Step 3: Verify.** Host test passes. `UWB_FRAME_LEN_ALERT` (34) ≤
  `UWB_FRAME_MAX_LEN` (37), so no RX buffer changes anywhere — confirm by grep that no
  buffer is sized on `UWB_FRAME_LEN_POS` as a proxy for "largest tag TX frame".

---

### Task 2: `alert_relay` — pure dedup, relay decision, latch ordering

The module the anchor and gateway will compile too. Pure C, no Zephyr.

**Files:**
- Create: `src/alert_relay.h`, `src/alert_relay.c`
- Test: `tests/alert_relay/test_alert_relay.c`

**Interfaces produced:**

```c
/* --- duplicate suppression --- */
struct alert_dedup_entry { uint8_t eui[8]; uint8_t epoch, state, repeat_seq;
                           uint32_t seen_ms; bool used; };
struct alert_dedup { struct alert_dedup_entry e[ALERT_DEDUP_N]; };

void alert_dedup_reset(struct alert_dedup *d);
/* true = first time seen (caller proceeds); false = duplicate, drop. Inserts on true. */
bool alert_dedup_admit(struct alert_dedup *d, const struct uwb_alert *a, uint32_t now_ms);

/* --- relay decision (anchors only) --- */
bool alert_should_relay(const struct uwb_alert *a, uint8_t my_hop, bool is_gateway);
/* Rewrites sender_hop/ttl in place for the outgoing copy. Call only if should_relay. */
void alert_prepare_relay(struct uwb_alert *a, uint8_t my_hop);

/* --- gateway latch --- */
struct alert_latch_entry { uint8_t eui[8]; uint8_t epoch, last_cancelled;
                           bool active, have_cancelled; uint32_t last_heard_ms; };
struct alert_latch { struct alert_latch_entry e[ALERT_LATCH_N]; };

typedef enum { ALERT_LATCH_IGNORED, ALERT_LATCH_RAISED,
               ALERT_LATCH_REFRESHED, ALERT_LATCH_CLEARED } alert_latch_res_t;

void alert_latch_reset(struct alert_latch *l);
alert_latch_res_t alert_latch_apply(struct alert_latch *l, const struct uwb_alert *a,
                                    uint32_t now_ms);
```

Rules, per design §3 / §6.2 / §6.4:
- `alert_dedup_admit`: key `(eui, epoch, state, repeat_seq)`; entries older than
  `ALERT_DEDUP_AGE_MS` are free for reuse; when full, evict the oldest `seen_ms`.
- `alert_should_relay`: `false` if `is_gateway`; `false` if `ttl == 0`; `true` if
  `my_hop == UWB_ALERT_HOP_UNKNOWN`; else `a->sender_hop > my_hop`.
- `alert_prepare_relay`: `sender_hop = my_hop`, `ttl--`. Touches nothing else.
- `alert_latch_apply`: epoch comparison is **serial arithmetic**, `(int8_t)(a - b) > 0`.
  HELP for `epoch == last_cancelled` → `IGNORED`. HELP matching an active epoch →
  `REFRESHED`. Newer HELP → `RAISED`. CANCEL → `CLEARED`, records `last_cancelled`,
  **even with no prior HELP for that epoch**.

- [x] **Step 1: Tests first** — `tests/alert_relay/test_alert_relay.c`. Required cases:
  - **stale HELP after CANCEL** (the whole reason epochs exist): HELP(5) → RAISED;
    CANCEL(5) → CLEARED; HELP(5) again (a relayed copy arriving late, different
    `repeat_seq` so dedup admits it) → **IGNORED**;
  - **new emergency after a cancel:** HELP(6) after CANCEL(5) → RAISED;
  - **epoch wrap:** `last_cancelled = 250`, HELP(2) → RAISED (2 is newer than 250 under
    serial arithmetic); HELP(250) → IGNORED;
  - **CANCEL with no prior HELP** → CLEARED and `last_cancelled` recorded;
  - dedup: same key twice → admit then reject; `repeat_seq + 1` → admitted; entry older
    than `ALERT_DEDUP_AGE_MS` → admitted again; overflow evicts oldest, not newest;
  - relay: `ttl == 0` → false; gateway → false; `sender_hop <= my_hop` → false;
    `sender_hop > my_hop` → true; `my_hop == 0xFF` → true; `prepare_relay` decrements
    ttl and sets `sender_hop`, leaving `eui`/`epoch`/`state`/`repeat_seq`/coords intact
    (assert with `memcmp` on the untouched fields).
- [x] **Step 2: Implement** `src/alert_relay.c`.
- [x] **Step 3: Verify.** Host test passes with `-Wall -Wextra -Werror`. Note the test
  compiles `src/uwb_frame_802_15_4z.c` too (for `struct uwb_alert`), so add it to the
  test's source list.
- [x] **Step 4:** Add to `CMakeLists.txt`. The tag itself uses only the dedup half (to
  ignore its own relayed alerts); the latch and relay halves are dead code on the tag
  and live code on the gateway/anchor. That is intentional — one implementation, one
  test suite, no drift.

---

### Task 3: `tag_alert_core` — the tag's own alert state (pure)

**Files:**
- Create: `src/tag_alert_core.h`, `src/tag_alert_core.c`
- Test: `tests/tag_alert/test_tag_alert.c`

**Interfaces produced:**

```c
typedef enum { TAG_ALERT_OFF, TAG_ALERT_HELP, TAG_ALERT_CANCELLING } tag_alert_st_t;

struct tag_alert_core {
    tag_alert_st_t state;
    uint8_t  epoch;
    uint8_t  repeat_seq;
    uint8_t  cancels_left;
    uint32_t next_tx_ms;
    bool     armed;        /* a TX is due */
};

void tag_alert_core_init(struct tag_alert_core *c, uint8_t epoch_from_nvs);
/* Returns true if the epoch advanced (caller must persist it). */
bool tag_alert_core_raise(struct tag_alert_core *c, uint32_t now_ms);
void tag_alert_core_cancel(struct tag_alert_core *c, uint32_t now_ms);
/* Called each superframe. true => build and transmit a frame with *out. */
bool tag_alert_core_due(struct tag_alert_core *c, uint32_t now_ms,
                        uint8_t *state_out, uint8_t *epoch_out, uint8_t *rep_out);
/* Call after a successful TX: advances repeat_seq / counts down cancels. */
void tag_alert_core_sent(struct tag_alert_core *c, uint32_t now_ms);
```

Behaviour:
- `raise` from OFF or CANCELLING: `epoch++`, `repeat_seq = 0`, state HELP, TX due
  immediately (`next_tx_ms = now_ms`), returns true. `raise` while already HELP is a
  **no-op returning false** — a second double-press must not burn an epoch or restart
  the repeat count.
- `cancel` from HELP: state CANCELLING, `cancels_left = ALERT_CANCEL_REPEATS`,
  `repeat_seq = 0`, TX due immediately, **epoch unchanged** (the CANCEL names the epoch
  it is cancelling). `cancel` from OFF is a no-op.
- `due`: true when `state != OFF` and `(int32_t)(now_ms - next_tx_ms) >= 0`.
- `sent`: `repeat_seq++`, `next_tx_ms = now_ms + ALERT_REPEAT_MS`; in CANCELLING,
  `--cancels_left == 0` → state OFF.
- All time comparisons signed-difference, so `k_uptime_get_32()` wrap is a non-event.

- [x] **Step 1: Tests first.** Cases: raise advances epoch exactly once; double raise is
  a no-op; HELP repeats forever (loop 100 × 10 s, still due); cancel keeps the epoch and
  stops after exactly `ALERT_CANCEL_REPEATS` sends; cancel from OFF no-op; raise during
  CANCELLING starts a **new** epoch; `now_ms` wrap across `0xFFFFFFFF` still fires.
- [x] **Step 2: Implement.** **Step 3: Verify** host test. **Step 4:** `CMakeLists.txt`.

---

### Task 4: `tag_alert.c` — Zephyr glue (NVS epoch, thread-safe access)

**Files:** Create `src/tag_alert.c`, `src/tag_alert.h`. Modify `src/storage.h` comment
block (document id 3), `src/main.c`, `CMakeLists.txt`.

**Interfaces produced:**

```c
void tag_alert_init(void);                    /* loads epoch from NVS (id 3) */
void tag_alert_raise(void);                   /* from the UI thread */
void tag_alert_cancel(void);
bool tag_alert_active(void);                  /* for the LED */
/* Runner-side: fills *a if a TX is due now. Caller transmits, then calls _sent(). */
bool tag_alert_frame_due(struct uwb_alert *a, uint16_t short_addr, uint32_t now_ms);
void tag_alert_sent(uint32_t now_ms);
```

- Guard the core with a `k_mutex` — `raise`/`cancel` run on the UI thread, `frame_due`
  on the runner thread.
- **NVS id 3** holds `{ uint8_t epoch; uint8_t active; uint16_t crc; }`. Written only on
  a state change (raise / cancel-complete), never per repeat — one flash write per
  emergency, not one per 10 s.
- On boot, `tag_alert_init` restores the epoch **and**, if `active` was set, resumes the
  HELP. A tag that browns out mid-emergency must not come back silent.
- `last_x`/`last_y` come from a position cached by `position_publish()`; add a small
  `pos_last_get(float *x, float *y)` accessor to `uwb_ss_initiator.c` returning false
  when there has never been a fix, in which case the frame carries NaN.
- `batt_soc` from `batt_soc_cached()` — already returns `0xFF` unknown, matching the
  frame sentinel.

- [x] **Step 1:** Implement `tag_alert.c` + `pos_last_get()`.
- [x] **Step 2:** Call `tag_alert_init()` from `main.c` after `storage_init()` and
  before `uwb_net_runner_start()`.
- [x] **Step 3:** `CMakeLists.txt`. **No host test** — this file is NVS + mutex glue;
  the logic under it is Task 3's, already covered.

---

### Task 5: `UWB_ACT_SEND_ALERT` in the pure FSM

**Files:** Modify `src/uwb_net.h`, `src/uwb_net.c`; extend `tests/uwb_net/test_uwb_net.c`.

```c
#define UWB_ACT_SEND_ALERT (1u << 6)
```
and a new event field: `bool alert_pending;` in `struct uwb_net_event`.

**Emission rule** (this is the design decision this task encodes — do not simplify it):

- In `UWB_ST_JOINING`, `UWB_ST_DISCOVER`, `UWB_ST_RANGING`: OR in `UWB_ACT_SEND_ALERT`
  on a **`UWB_EV_BEACON` event only**. The tag has superframe timing there, so the TX
  lands in a known-safe window. Deliberately **not** on `UWB_EV_BEACON_MISS` — without
  the beacon the tag does not know where the CAP is, and a blind TX could land on the
  beacon or in someone's CFP slot. The existing "never TX on a missed beacon" rule in
  `UWB_ST_RANGING` stands.
- In `UWB_ST_SCAN`: OR it in on **both** `UWB_EV_BEACON` and `UWB_EV_BEACON_MISS`. An
  unjoined tag has no sync to protect and no seat to lose — this is exactly the "no
  anchor can hear it" case the feature exists for, so it shouts unsynchronised.

**Gating:** `UWB_ACT_SEND_ALERT` must stay **out** of `UWB_ACT_RANGING_MASK`. An alert
carries no TWR result; an uncalibrated tag must still be able to call for help.

- [x] **Step 1: Tests first** in `tests/uwb_net/test_uwb_net.c`:
  - `alert_pending` + BEACON in each of the four states → action set contains
    `UWB_ACT_SEND_ALERT`, and the state transition is otherwise **unchanged** vs the
    same event with `alert_pending = false` (assert the full action word, so an alert
    cannot accidentally suppress a keepalive or a sweep);
  - BEACON_MISS + `alert_pending` → set in `SCAN`, **not** set in the other three;
  - extend `test_gate_actions()`: `uwb_net_gate_actions(UWB_ACT_SEND_ALERT, false)`
    still contains `UWB_ACT_SEND_ALERT`.
- [x] **Step 2:** Implement. **Step 3:** Verify host test.

---

### Task 6: Runner integration — transmit the frame

**Files:** Modify `src/uwb_net_runner.c`.

- [x] **Step 1:** Before `uwb_net_handle()`, set `ev.alert_pending = tag_alert_active()`.
- [x] **Step 2:** Add an execute block alongside the existing `UWB_ACT_*` blocks:

```c
if (act & UWB_ACT_SEND_ALERT) {
    struct uwb_alert a;
    uint32_t now = uwb_radio_now_ms();
    if (tag_alert_frame_due(&a, ctx.short_addr, now)) {
        uint8_t abuf[UWB_FRAME_LEN_ALERT];
        int alen = uwb_frame_alert_build(abuf, sizeof(abuf), ctx.short_addr, &a);
        if (alen > 0) {
            uint8_t mslot = (uint8_t)(sys_rand32_get() % N_CAP);
            uwb_radio_tx_cap(abuf, (size_t)alen, mslot);
            tag_alert_sent(now);
        }
    }
}
```

  Place it **after** the JOIN/KEEPALIVE blocks and **before** `UWB_ACT_RUN_SWEEP`, so a
  pending alert never delays the sweep's slot-start deadline and vice versa.
- [x] **Step 3:** Ordering check against `UWB_ACT_SLEEP` — the alert block must run
  before the sleep block, or a due alert waits a whole superframe. The action word is a
  bitmask evaluated top to bottom in one pass, so position in the function *is* the
  priority; state that in a comment.
- [ ] **Step 4 (user, on device):** With `help on` over NUS, confirm the gateway receives
  an `0xEB` every ~10 s, that `P:` position output continues uninterrupted, and that
  `pwr rx` numbers are unchanged (the alert must not perturb the beacon window).

---

### Task 7: NUS commands

**Files:** Modify `src/tag_cmd.c`.

- [x] Add before the `pwr` block: `help on` → `tag_alert_raise()`, reply `HELP on e<n>\n`;
  `help off` → `tag_alert_cancel()`, reply `HELP cancel\n`; bare `help` → status,
  `HELP <on|off> e<n>\n`. Keep every line ≤ 19 chars.
- [x] These exist so the whole uplink path is testable **before** Task 8 touches the
  button ISR. Do not defer them.

---

### Task 8: Button long-press + LED states

**Files:** Modify `src/tag_ui.c`.

The riskiest task — it changes a working ISR and gesture path. Do it last.

- [x] **Step 1:** Change the msgq element from `uint32_t` to
  `struct btn_ev { uint32_t t_ms; bool pressed; }`; update `K_MSGQ_DEFINE`, the ISR, and
  every `k_msgq_get`/`put` site. The ISR currently returns early unless the pin reads
  pressed (`tag_ui.c:95`) — it must now queue **both** edges, keeping the 30 ms debounce.
- [x] **Step 2:** Gesture logic in `ui_fn`:
  - press → wait up to `DOUBLE_MS` (350 ms) for a second **press**: as today, double =
    raise HELP, single = battery readout;
  - press with no release within `LONG_MS` (3000 ms) → **cancel**: fire on the 3 s mark
    while the button is still held, so the user gets confirmation without releasing.
    Implement as a bounded wait on the queue, not a blocking wait for the release edge.
  - Releases that arrive while a gesture is resolving are consumed, not treated as
    presses. Keep deciding from timestamps, not arrival order — the existing comment at
    `tag_ui.c:119-121` explains why, and multi-edge queuing makes it more true, not less.
- [x] **Step 3:** LED: alert active → slow red pulse as the standing indicator, taking
  priority over the battery readout and the orange double-press blink (which the raise
  gesture replaces). Cancel → green flash, then dark.
- [ ] **Step 4 (user, on device):** double press raises (red pulse + gateway sees HELP);
  3 s hold cancels (green flash, gateway sees CANCEL, repeats stop after 6); single
  press still shows battery colour; double press still does *not* show battery.

---

### Task 9: Documentation

- [x] Update `CLAUDE.md` Open Work item 5: mark the tag side implemented, keep the anchor
  contract paragraph (that work is still outstanding), and add a Key Patterns entry for
  the alert path — the "no radio claim, runner thread only" rule and the "alert TX only
  on a beacon event except in SCAN" rule are both non-obvious and both easy to break.
- [x] Update `src/storage.h` id list: 1 = cal, 2 = NFC name, **3 = alert state**.
- [x] Add the two new test directories to the CLAUDE.md host-test table
  (`tests/alert_relay/` → `src/alert_relay.c` + `src/uwb_frame_802_15_4z.c`,
  `tests/tag_alert/` → `src/tag_alert_core.c`).

---

## Deliberately out of scope

Per design §8 and the decisions in §9 — do not add these without a fresh decision:

- Any ack from gateway to tag (the tag repeats blindly in v1).
- Remote/downlink cancel from the platform.
- Anchor witness reports.
- A learned hop gradient — `hop_to_gw` is static in anchor flash.
- Any anchor or gateway firmware change; Task 2 delivers the shared module they will use.

## Risk notes

- **Task 8 is the one that can break existing behaviour.** The button path works today
  and the single/double-press gestures are the only local UI. Land Tasks 1–7 first, so
  the alert is fully exercisable over NUS before the ISR is touched; if Task 8 misbehaves
  it can be reverted on its own without losing the feature.
- **The CAP is shared** with JOIN and KEEPALIVE. An alert TX every 10 s against a 202 ms
  superframe is one frame per ~50, so collision risk is low — but if `RESCAN seat`
  appears in the BLE log after this lands, CAP contention starving keepalives is the
  first thing to suspect.
- **`ss_twr_msgq` has 8 slots** and overflow is silent. Do not add per-alert `twr_log()`
  lines to the runner path beyond a single one-shot on raise/cancel.
