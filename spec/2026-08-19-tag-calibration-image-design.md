# A dedicated calibration image for the tag — Design

**Date:** 2026-08-19
**Status:** Design approved, pre-implementation
**Depends on:** the MAC runner (`src/uwb_net_runner.c`), the ranging/calibration thread
(`src/uwb_ss_initiator.c`), the radio handover (`src/uwb_radio_owner.c`), the NVS calibration
record (`src/cal.c`), and the NUS command dispatcher (`src/tag_cmd.c`).
**Scope:** A second firmware image, selected by an `EXTRA_CONF_FILE` overlay, in which the tag
never opens the receiver on its own and never deep-sleeps the DW3000 — it boots idle and does
radio work only when a `cal` command tells it to. Plus two on-demand diagnostics that make the
current `CALp 100 1` failure observable. **Out of scope:** changing the SS-TWR solve itself
(`run_calibration_locked()` is not touched), any on-air frame change, any change to anchor
firmware, and fixing the known SPI-in-ISR latent bug.

---

## 1. Motivation

`cal` does not work. The crash was fixed; the measurement was not. The current symptom is
`CALp 100 1` / `CALx 0 0`: the sampling loop runs its 100 passes, **zero** frames are rejected
for size and **zero** for header, and **99 of 100 exchanges end as RX timeouts**, while the
DWM3001CDK reference node logs `RESP TX OK` throughout.

That failure blocks the whole system, not just the tag. Two range paths exist on this network
and their errors differ by roughly 7×: the anchor-to-anchor survey bisects to a common bias near
**197 mm** (`apos_solve` reporting `rms_mm:31`), while the tag-to-anchor path carries roughly
**1450 mm**. The anchors are the element both paths share, so the difference is the tag. Tag
antenna-delay calibration is therefore the first thing that has to work, ahead of the anchors'
own `cal ref`.

### 1.1 Why a separate image

Antenna-delay calibration is a bench procedure: one tag, one responder, a known distance, a
quiet channel. The production image is the opposite environment and fights it on three fronts.

- **Constant beacon RX.** The runner holds the receiver open for the gateway beacon across most
  of every 202 ms superframe, and ~100 % of it in `UWB_ST_SCAN` — which is where every `cal` run
  lands, since a 3 s absence exceeds `MISS_MAX`. `wait_event()` returns on the **first frame of
  any type**.
- **DW3000 deep sleep.** `dw_enter_sleep()` / `dw_wake()` cycle the radio between activities,
  re-running `dwt_restoreconfig()` and re-applying antenna delays and LNA mode underneath a
  procedure trying to hold the PHY constant across ~100 samples.
- **Radio handovers.** Calibration claims the radio for its whole run, so the claim / yield /
  reacquire machinery is live around the measurement.

### 1.2 What the evidence does and does not support

The beacon-RX explanation above is the one previously recorded in `CLAUDE.md`, and **it is
inconsistent with its own evidence.** If a foreign frame were consuming the post-poll RX window,
`wait_event()` would return `EVT_RXFCG`, the `memcmp` against `rx_resp_msg`
(`uwb_ss_initiator.c` `do_one_range()`) would fail, and the run would report **header rejects**.
`CALx 0 0` reports none. Nothing arrives at all, 99 times in 100. Contention predicts rejects;
the evidence shows an empty window.

Two candidate causes were ruled out while designing this, and are recorded so they are not
re-investigated:

- **The RX window is not misaligned for a 2000 uus peer.** `POLL_TX_TO_RESP_RX_DLY_UUS` = 1000,
  `RESP_RX_TIMEOUT_UUS` = 2000, `PRE_TIMEOUT` = 128 (`uwb_ss_initiator.c:36-38`). The production
  sweep in `uwb_net_runner.c:55-57` uses **the identical three constants** and successfully
  ranged four ANCLA anchors — which turn around at `POLL_RX_TO_RESP_TX_DLY_UUS = 2000` — for 221
  consecutive position fixes. The window is proven against a 2000 uus turnaround.
- **The interrupt mask is not the difference.** `do_one_range()` (`uwb_ss_initiator.c:228`) and
  `do_one_range_anchor()` (`:292`) both apply the same `INT_RX_PHASE` before every exchange.

One structural fact about the peer, however, is new and belongs in the record:

- **`cal` can only ever work against a stock Qorvo SS-TWR responder, never against an ANCLA
  anchor.** The `cal` poll `tx_poll_msg` (`uwb_ss_initiator.c:75`) is **10 bytes** with no
  anchor id, while the production poll `pos_poll_msg` (`:88`) is **11 bytes** with the id at
  index 10. `anchor_respond_wave_poll()` in the anchor firmware opens with
  `if (len < POS_ANCHOR_ID_IDX + 1) return;` — a silent drop of any 10-byte poll. The response
  layouts differ too: `cal` reads timestamps at offsets 10/14 (the stock 18-byte response),
  production at 11/15 (ANCLA's 27-byte response carrying x/y). So `cal` has exactly one valid
  peer, and no diagnostic today can tell "the peer never answered" from "the answer arrived
  outside the window".

That last gap is what §5 exists to close.

**Success criteria.** The image boots and transmits nothing until commanded, verifiable on a
sniffer. `cal <mm>` no longer fails `CAL FAIL busy` with no runner present. `cal probe` reports
either a round-trip time or an unambiguous "no frame", against either peer type, so the next
debugging step is chosen from evidence rather than from a hypothesis.

---

## 2. Architecture

A Kconfig symbol plus a `.conf` overlay, exactly as the anchor project already does with
`CONFIG_ANCLA_CAL_MODE` / `cal.conf`. The difference between the two images is **which threads
`main()` starts**, not a runtime mode and not a second `main()`.

```
                     production                     cal image
                     ----------                     ---------
  main()  --------->  runner thread   (RX, sleep)     (not started)
          --------->  ss_twr thread   (cal solve)     ss_twr thread  (cal solve)
          --------->  ble_tx thread                   ble_tx thread
          --------->  motion / batt / nfc / alert     (not started)
          --------->  tag_ui  (LED = battery SoC)     cal_led  (LED = cal verdict)
                                                      cal_diag thread (listen / probe)

  radio owner state:  IDLE <-> REQUESTED <-> HANDED   unmanaged: IDLE <-> HANDED
```

Three `#ifdef CONFIG_TAG_CAL_MODE` sites in total, each at a natural seam: the NUS dispatcher,
the verdict funnel, and bring-up. Everything else is either a new file compiled only in cal
mode, or a change that is inert in production.

### 2.1 Rejected alternatives

- **A separate `src/main_cal.c` selected in CMakeLists.** Cleaner to read, but it creates two
  bring-up sequences that must stay in step. Divergence between paired constants is a bug class
  this codebase has already paid for. One guarded block beats two files.
- **A runtime `cal mode` command with no new image.** Cheapest, and it fails the requirement:
  the receiver is open from boot until the command lands, Zephyr cannot cleanly kill the runner
  thread, and every piece of handover machinery stays live around the measurement.

---

## 3. Build plumbing

**`Kconfig`** (new — the tag has no project Kconfig today, so this is the file that makes any
`CONFIG_TAG_*` symbol possible at all):

```
menu "Tag firmware"

config TAG_CAL_MODE
	bool "Antenna-delay calibration firmware"
	help
	  Build the calibration image instead of the production tag.

	  main() does not start the MAC runner, so nothing opens the receiver
	  and nothing deep-sleeps the DW3000: the image is idle until a `cal`
	  command arrives over NUS. Adds `cal listen` and `cal probe`.

	  Never ship this. It holds no network seat and reports no position.

endmenu

source "Kconfig.zephyr"
```

`source "Kconfig.zephyr"` must come last, matching the anchor project's file.

**`cal.conf`** (new):

```
CONFIG_TAG_CAL_MODE=y

# A halt during a calibration run must stay halted. task_wdt's hardware
# fallback (TASK_WDT_MIN_TIMEOUT 100 + TASK_WDT_HW_FALLBACK_DELAY 20) arms the
# nRF52833 watchdog at ~120 ms, so the last time `cal` faulted the reset looked
# like an instant reboot and hid the crash entirely.
CONFIG_TASK_WDT=n
CONFIG_WATCHDOG=n
```

Deliberately **not** in `cal.conf`:

- **`CONFIG_ASSERT=y`.** `cal.conf` must never be combined with `debug.conf` for the same
  reason `debug.conf` carries its own warning: `dwt_isr()` runs in the GPIO ISR, and both
  `k_mutex_lock()` and `spi_transceive()` carry an unconditional `__ASSERT(!arch_is_in_isr())`.
  With asserts on, every DW3000 interrupt panics the kernel. The `cal <mm>` solve loop goes
  through `wait_event()`, so it needs that ISR to work.
- **NFC / battery / sensor Kconfig symbols.** Not calling `nfc_tag_init()` and
  `batt_monitor_start()` is the mechanism; switching libraries off would only save flash, and
  the exact symbol names have not been verified. Out of scope.

**`CMakeLists.txt`** — append, using the Zephyr idiom the anchor project already uses:

```cmake
target_sources_ifdef(CONFIG_TAG_CAL_MODE app PRIVATE
    src/cal_diag.c
    src/cal_led.c
)
```

Nothing is **removed** from the source list. `src/uwb_net_runner.c` stays compiled — see §4.2.

**Build command** (the user builds and flashes; this project is not built here):

```
west build -b nRF52833_tag --pristine -d build_cal -- -DEXTRA_CONF_FILE=cal.conf
west flash -d build_cal
```

---

## 4. Bring-up

### 4.1 What starts

One guarded block in `main()`. `ble_log_init()`, `storage_init()`, `uwb_init(3)`, `cal_init()`,
`tag_cmd_init()` and `uwb_ss_initiator_start()` are common to both images and stay where they
are.

| call | production | cal image |
|---|---|---|
| `uwb_net_runner_start(eui)` | yes | **no** |
| `motion_init()` | yes | **no** |
| `batt_monitor_start()` | yes | **no** |
| `nfc_tag_init()` | yes | **no** |
| `tag_alert_init()` | yes | **no** |
| `tag_ui_init()` | yes | **no** — replaced by `cal_led_init(strip)` |
| `tag_wdt_start_boot_guard()` | yes | **no** |
| `tag_wdt_run_feeder()` | yes | **no** |
| `uwb_radio_owner_set_unmanaged()` | no | **yes** |
| `cal_diag_start()` | no | **yes** |

`uwb_radio_owner_set_unmanaged()` must be called **before** `uwb_ss_initiator_start()`, so no
claimant thread can be running when the mode is set.

The boot guard is dropped along with the feeder because `wdt.c` compiles the guard out when
`CONFIG_TASK_WDT=n`. `tag_reset_reason()` and `tag_fault_get()` are unaffected and stay
available over NUS as `rst` and `fault` — both are wanted in this image.

### 4.2 Why not starting the runner is sufficient, and verifiable

Both requirements fall out of one omission, and neither needs a flag:

- **No active RX.** The only `dwt_rxenable()` in the application today is
  `uwb_net_runner.c:274`, reachable only from `runner_fn`.
- **No deep sleep.** `dw_enter_sleep()`, `dw_wake()` and `dw_sleep_enabled`
  (`uwb_net_runner.c:125-128`, `:445-490`) are all static to that file and called only by the
  runner. There is no sleep path to disable; it is unreachable.

`uwb_net_runner.c` therefore stays in the build. Dropping it would demand stubs for
`uwb_net_runner_wake()` (called by `tag_alert.c`), `uwb_radio_sleep_enabled()` and the tier
accessors (called by `tag_cmd.c`) for no behavioural gain, since an unstarted thread executes
nothing.

`cal_diag.c` adds the second `dwt_rxenable()` in the tree, so the invariant to hold in review
becomes: **`dwt_rxenable()` appears in exactly two files, and the `cal_diag.c` one is reachable
only from a queued command.** That is still a grep.

### 4.3 Accepted knock-on effects

Documented rather than fixed, because each fix would cost an `#ifdef` inside a production file
for no measurement benefit:

- `pwr tier` and `pwr scan` still link and answer, with the runner's never-advanced state — so
  `pwr scan` reads `R 0 T 0`. The `pwr sleep` toggle likewise reports a flag nothing consumes.
- `ss_twr_fn` polls at 100 ms rather than blocking when a valid calibration is already stored
  (`uwb_ss_initiator.c:642` takes the `cal_wait_request()` branch only when `!ranging`). It
  touches no radio; it is a 100 ms sleep loop.

---

## 5. Radio ownership

### 5.1 The problem

`run_calibration()` opens with `uwb_radio_request(CAL_RADIO_WAIT)` — 2 seconds
(`uwb_ss_initiator.c:488`). The grant comes from the runner reaching `uwb_radio_yield()` at the
top of its superframe loop. **With no runner, no grant ever arrives, and every `cal <mm>` run in
the new image would die `CAL FAIL busy` after 2 s.**

### 5.2 The change

`uwb_radio_owner` learns that it has no runner, rather than the caller learning to skip it:

```c
/* Declare that no MAC runner will ever offer the radio: this image has no
 * runner thread, so a claim can be granted immediately instead of waiting for
 * a yield that will never come. Call once from bring-up, before any claimant
 * thread starts. */
void uwb_radio_owner_set_unmanaged(void);
```

Implementation is a `static bool unmanaged;` plus one branch in `uwb_radio_request()`: under the
same mutex, when `unmanaged` and the state is `OWNER_IDLE`, go straight to `OWNER_HANDED` and
return true.

Every existing invariant survives untouched:

| function | behaviour when unmanaged |
|---|---|
| `uwb_radio_request()` | grants immediately from `IDLE`; **still fails** if a claim is in flight, so the single-claimant contract holds and a second claimant is still surfaced rather than hidden |
| `uwb_radio_release()` | unchanged — `HANDED -> IDLE` plus broadcast already covers this |
| `uwb_radio_request_pending()` | unchanged, and correctly always false: the state never rests in `REQUESTED` |
| `uwb_radio_yield()` | unchanged and never called; if it were, the state is `IDLE` or `HANDED`, not `REQUESTED`, so it returns false without parking |

Roughly six lines and one `bool` in BSS. **This is the only file production both links and
executes that this design changes**, and it is inert there because nothing calls the setter —
which greps to a single call site under the flag. That containment is the reason this approach
was chosen over `#ifdef`-ing the request out of the measurement path: `run_calibration()` is not
touched at all.

The PHY-state contract in `uwb_radio_owner.h` needs no change. It constrains what a claimant
must restore before releasing; with no runner there is nothing to restore it *for*, and honoring
it anyway keeps one contract instead of two.

---

## 6. Diagnostics

`src/cal_diag.{c,h}`, compiled only in cal mode. One thread, its own request queue, and two
commands. `cal_diag_on_rx()` parses, enqueues and returns immediately — the BT RX thread must
never block.

### 6.1 Why it polls instead of using `wait_event()`

`cal_diag.c` disables interrupts (`dwt_setinterrupt(0xFFFFFFFF, 0xFFFFFFFF, DWT_DISABLE_INT)`)
and polls `SYS_STATUS_LO`, exactly as the anchor project's `src/ss_initiator.c` does. Two
reasons:

- It keeps `uwb_ss_initiator.c` **completely untouched**. Using `wait_event()` would mean adding
  a third caller to a function documented as destructive under concurrency, or threading a new
  request kind through `cal_take_request()` — a production signature change.
- It sidesteps the `k_sem_reset()` / `port_DisableEXT_IRQ()` pair that `uwb_radio_owner.h`
  exists to protect.

It claims the radio through `uwb_radio_request()` like any other claimant — instant under §5 —
and restores the five PHY items named in the handover contract before releasing, so the same
code would remain correct if the image ever gained a runner.

### 6.2 `cal listen [ms]`

Default 3000 ms, clamped to 10000 ms — the image has no watchdog (§3), so the bound exists
to keep a mistyped argument from parking the diagnostic thread, not to protect a feeder.
Claims the radio, opens RX with **no** timeouts
(`dwt_setrxtimeout(0)`, `dwt_setpreambledetecttimeout(0)`), and for each frame reports its
length and type byte, then re-arms. Ends with a tally.

This is the RX-health check: aimed at a gateway beacon it proves the receiver, the PHY
configuration and the LNA are alive end to end, independently of SS-TWR.

**It reads zero frames against a stock SS-TWR responder, and that is correct, not a fault** — a
stock responder is silent until polled. The operator documentation must say so, or a zero will
be misread.

### 6.3 `cal probe [id]`

The command that answers `CALx 0 0`. One poll, then RX **wide open** — no window at all — and it
reports the round-trip plus the response's type byte and length, or an unambiguous "no frame"
after **50 ms**. 50 ms is ~16× the widest legitimate answer (a 2000 uus turnaround plus a
PLEN_1024 frame is ~3.2 ms), so anything the peer could plausibly send lands inside it while a
dead peer still reports promptly.

The reported round-trip is
`(dwt_readrxtimestamplo32() - dwt_readtxtimestamplo32()) / UUS_TO_DWT_TIME`, in uus — the same
two registers `do_one_range()` already reads, so the number is directly comparable to the
`[1000, 3000]` uus window rather than being a new quantity.

`id` selects the peer, and this is the point of the command:

- omitted: the 10-byte stock poll, for the DWM3001CDK
- given: the 11-byte addressed poll, for an ANCLA anchor

`id` is the **wire** id, 1..4, not the anchor's console `anchor id` of 0..3. The anchor compares
the polled byte against `uwb_config_short_addr(cfg) & 0xFF`, i.e. `1 + anchor_id`, and the tag
already takes it from the low byte of `src_addr` in the DISCOVERY response. `cal probe` must use
the same value or the anchor drops the poll as "not ours" — which would look identical to no
answer at all, defeating the command's whole purpose. Reject anything outside 1..4 at parse time.

Same command, two peers, one number — a direct A/B on hardware already on the bench:

| `cal probe` (DWM3001CDK) | `cal probe <id>` (ANCLA) | conclusion |
|---|---|---|
| no frame | plausible RTD | the tag is exonerated; the reference node is the fault |
| no frame | no frame | tag or environment, and this image is where to chase it |
| RTD outside 1000–3000 uus | — | window misalignment; the fix is a constant, not an image |
| foreign frames in-window | — | contention after all, and §1.1 was right |

`cal probe` **reports only**. It never writes NVS and never changes the active antenna delays.

### 6.4 Output format

Every NUS reply is at most **19 characters plus NUL**. This is not a style rule: it already bit
`cal`, where a 23-byte error string returned `-EMSGSIZE` and answered a mistyped command with
silence (`cal.c`, `cal_on_rx()`). Planned lines:

```
F t=2054 E1 20      one frame: round-trip in uus, type byte, length
F none              nothing arrived within the bounded wait
LSN n=12 e=0        listen tally: frames, errors
```

### 6.5 One refactor: a single home for the WAVE byte pattern

The WAVE poll and response patterns are hand-rolled `static` arrays in `uwb_ss_initiator.c`
(`:75-76`, `:88-89`); `uwb_frame_802_15_4z.h` has no builder for them. `cal_diag.c` needs the
same bytes, and this codebase already treats a forked wire format as a bug waiting to happen —
`uwb_frame_802_15_4z.c` is kept byte-identical to the anchor's copy for exactly that reason.

So: a small new header `src/uwb_wave_frame.h` holding the byte patterns as macro initializers
plus the field offsets, included by both files. The arrays stay per-translation-unit (the
sequence number is written into the poll buffer, so it cannot be `const`), but the byte pattern
and every offset exist **once**.

`uwb_ss_initiator.c` changes only its four initializers and drops the duplicated offset
defines. Behaviour in production is identical, and that identity is checkable by inspection.

---

## 7. Status LED

`src/cal_led.{c,h}`, roughly 30 lines, driven from `cal_set_last_result()` in `cal.c` — already
the single funnel every verdict passes through, including the `"CAL running"` written when a run
starts. One `#ifdef` there and no new plumbing, and it covers the solve **and** the §6
diagnostics for free.

| state | LED |
|---|---|
| idle, calibration stored | dim green |
| idle, no calibration | dim amber |
| run in progress | blue |
| `CAL OK` | green |
| `CAL FAIL ...` | red |

`tag_ui.c` is deliberately not reused: its LED reports **battery SoC**, and it pulls `batt.h`
and `tag_alert.h` back into the link. In a calibration image the status worth seeing is the run.

`cal_led_init()` takes the `led_strip` device from `main()`, which already holds it for the
fatal-DW3000-init red path. The two writers are mutually exclusive in time (one is a terminal
error path) and `tag_ui` is not running, so no lock is needed. `cal_set_last_result()` is called
from the SS-TWR thread at iteration boundaries, never mid-exchange, so a ~1 ms strip update
costs nothing measurable.

---

## 8. Testing

**Host test** — extend `tests/uwb_radio_owner/`, which already compiles the real module against
a pthread shim:

1. unmanaged: `uwb_radio_request()` grants immediately with no yielding thread present
2. unmanaged: a second concurrent claim still fails, contract intact
3. unmanaged: `release()` returns the state to idle, so a later claim succeeds
4. unmanaged: `uwb_radio_yield()` returns false and does **not** park — the regression that
   would deadlock a future image that mixed the two modes
5. managed: every existing test still passes unchanged

**Static checks:**

- `dwt_rxenable()` appears in exactly two files; the `cal_diag.c` one is reachable only from a
  queued command
- the production build is unchanged in behaviour: `uwb_wave_frame.h` is a pure move, and
  `uwb_radio_owner_set_unmanaged()` has no production call site

**On hardware,** in order:

1. Flash the cal image. Point the DWM3001CDK sniffer at the tag and confirm it transmits
   **nothing** — no poll, no join, no keepalive — until a command arrives. This is the real test
   of "no active RX"; no unit test can give it.
2. `cal listen` with a gateway on air: frames counted. This proves the receive chain before any
   SS-TWR conclusion is drawn from silence.
3. `cal probe` and `cal probe <id>`, then read §6.3's table.
4. Only then `cal <mm>`, and compare `CALp` / `CALx` against the production image's numbers.

---

## 9. Deliverables

New:

- `Kconfig`, `cal.conf`
- `src/cal_diag.{c,h}`, `src/cal_led.{c,h}`, `src/uwb_wave_frame.h`

Changed:

- `CMakeLists.txt` — `target_sources_ifdef` block
- `src/main.c` — one guarded bring-up block
- `src/tag_cmd.c` — one guarded dispatch line
- `src/cal.c` — one guarded LED call in `cal_set_last_result()`
- `src/uwb_radio_owner.{c,h}` — the unmanaged state
- `src/uwb_ss_initiator.c` — frame initializers only, no behaviour change
- `tests/uwb_radio_owner/` — five cases
- `CLAUDE.md` — the image, the build command, the `debug.conf` incompatibility, and §1.2's two
  ruled-out causes plus the 10-byte/11-byte poll fact

## 10. What this design does not claim

It does not claim to fix `cal`. It removes the three environmental confounds named in §1.1 and
adds the two measurements that make the failure legible. If `cal probe` reports a plausible RTD
against both peers and `cal <mm>` still returns `CALp 100 1`, then the cause is neither the
environment nor the peer, and §6.3's table says so explicitly rather than leaving another
hypothesis unfalsified.
