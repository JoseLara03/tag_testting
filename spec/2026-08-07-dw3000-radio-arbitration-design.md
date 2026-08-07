# DW3000 radio arbitration and the calibration gate — Design

**Date:** 2026-08-07
**Status:** Design approved, pre-implementation
**Depends on:** the MAC runner (`src/uwb_net_runner.c`), the ranging/calibration thread
(`src/uwb_ss_initiator.c`), the shared event path (`wait_event()`), and the NVS calibration
record (`src/cal.c`).
**Scope:** Give the DW3000 a single owner at any instant, by explicit handover between the
runner and the calibration run; and restore the guarantee that a tag does not range until its
antenna delays are calibrated. **Out of scope:** why the anchor does not answer the calibration
poll (see §2), and any change to the on-air protocol or the anchor firmware.

---

## 1. Motivation

Two independent defects, both about who may drive the radio and under what precondition.

**The radio has two drivers and no arbitration.** `runner_fn` and `ss_twr_fn` both run at
priority 2 (`uwb_net_runner.c:84`, `uwb_ss_initiator.c:173`) and share one binary semaphore and
one interrupt line through `wait_event()` (`uwb_ss_initiator.c:140-159`). That function is not
merely unsynchronised, it is destructive when two threads use it at once: it opens with
`k_sem_reset(&irq_sem)`, which discards an event the other thread was waiting for; it writes a
single shared `last_evt`; and it closes with `port_DisableEXT_IRQ()`, switching off the interrupt
line while the other thread may still be blocked waiting for it.

**A tag will range without calibration.** The runner applies antenna delays unconditionally
(`uwb_net_runner.c:422-425`), and `cal_get_ant_dly()` returns `active.tx_ant_dly` /
`active.rx_ant_dly` without consulting `active_valid` (`cal.c:87-91`). `active` is a static
record, so with no valid NVS record a tag ranges with **antenna delays of zero** — not the
factory seed, zero, an error of metres. `CLAUDE.md` still documents that the ranging thread
"does not range until a cal run succeeds"; that was true while `ss_twr_fn` owned ranging, and
the check did not move when ranging moved to the runner.

**Success criteria:** calibration and the runner never drive the radio at the same time, the
runner recovers sync cleanly after a calibration run, and a tag with no valid calibration keeps
its seat but emits no ranging result.

---

## 2. What this does not fix, and why it is still worth doing

`cal 300` currently reports `CAL FAIL no-resp`. Arbitration was the leading hypothesis and **an
over-the-air capture disproved it**: during a calibration run the tag's 100 polls all reach the
air cleanly at −73 dBm (`41 88 xx CA DE 57 41 56 45 E0`) and not one response comes back. The
tag transmits correctly; the anchor does not answer. That is an anchor-side problem — most likely
DW3000 frame filtering, since the poll is addressed to `0x4157` (the `'WA'` bytes of the legacy
Decawave example) while the anchor is `0x0000` on this network.

So this design fixes a real latent hazard, not the symptom that exposed it. Recording that
distinction matters: after this work, `cal 300` will still fail until the anchor answers.

---

## 3. A rendezvous, not a general mutex

A mutex held across the runner's loop would deadlock calibration, because the runner **sleeps
inside its own loop** — `uwb_radio_sleep_until()` in the beacon-window planner and
`dw_enter_sleep()` in the tail. Holding the radio while asleep would starve any other claimant.

Since calibration takes the radio for its whole run, what is needed is an explicit handover, in
a new module with one purpose — `src/uwb_radio_owner.c` / `.h`:

```c
void uwb_radio_request(void);            /* calibration: block until the runner hands over */
void uwb_radio_release(void);            /* calibration: hand the radio back */
bool uwb_radio_yield_if_requested(void); /* runner: hand over and wait; true if it yielded */
```

The runner calls `uwb_radio_yield_if_requested()` **once per superframe, at the top of the
loop** — the only point where the radio is in a known state with no exchange in flight. There it
hands over and blocks until calibration finishes. No thread is ever suspended mid-operation,
which is what made the earlier throwaway experiment unsafe.

**Handover contract:** before handing over, the runner leaves the radio **awake and idle**
(`dwt_forcetrxoff()`, waking it first if it was deep-asleep). Calibration therefore needs to know
nothing about the power state, and `dw_wake()` stays private to the runner.

With exclusive ownership guaranteed, `wait_event()` becomes correct as written — its
`k_sem_reset()` and global IRQ disable are only destructive under concurrency. No rewrite of the
event path is required, and the beacon timing path is untouched.

---

## 4. Reacquiring the radio

When `uwb_radio_yield_if_requested()` returns true — and only then, so the common path costs
nothing — the runner must re-establish three things, in this order:

1. Its own RX parameters (`rxaftertxdelay`, `rxtimeout`, `preambledetecttimeout`) — calibration
   sets its own and `uwb_radio_rx_beacon()` uses `rxtimeout = 0`.
2. The antenna delays, which calibration may have **changed**.
3. `beacon_track_reset()`, returning the tracker to ACQUIRING.

Step 3 is the one that is easy to omit and would undo the narrow-beacon-window work: after
several seconds off the air the arrival prediction is stale, and a narrow window aimed at a
stale instant would miss repeatedly and trip `RESCAN miss`. Losing sync during calibration is
accepted by design; failing to re-acquire cleanly afterwards is not.

---

## 5. The calibration gate

In the runner, when `cal_is_valid()` is false: keep the beacon cycle, keep the seat, keep sending
keepalives, and **skip the ranging sweep**. No `P:` output. This restores the behaviour
`CLAUDE.md` already describes, so that document becomes true again rather than being edited to
match a regression.

Separately, `cal_get_ant_dly()` stops handing out zeros: with no valid record it returns the
factory seed from `phy_config.h`, matching what `active_total_seed()` already does. The gate
means nothing should range uncalibrated anyway, but silently returning zero is a trap for the
next caller.

---

## 6. Cleanup

Removed — temporary diagnostics from the `CAL FAIL no-resp` investigation, which have served
their purpose:

- the `CALd ok/to/er/bd/txf` counters and their reporting in `run_calibration`;
- `uwb_net_runner_suspend()` / `uwb_net_runner_resume()` and their declarations.

Kept:

- the `dwt_starttx()` return check in `do_one_range()`. Discarding it was a real defect — it made
  "rejected before reaching the air" indistinguishable from "transmitted, no answer", which is
  what made this investigation take three refuted hypotheses. The positioning path
  (`do_one_range_anchor()`) already checks it.
- the `run_calibration_locked()` / `run_calibration()` split, which now carries the
  request/release pair so that all three early returns hand the radio back.

---

## 7. Testing

Honest about the limits: the rendezvous is built on kernel primitives and does not fit the pure
host-test pattern used by `tests/`. What is host-testable is the gate decision, extracted as a
pure predicate:

- ranging is skipped when the calibration record is invalid;
- ranging proceeds when it is valid;
- the decision does not depend on anything else the runner tracks.

Everything else is a build plus on-hardware checks, which are yours to run:

1. `cal clear`, then reboot: the tag must take a seat and hold it, with no `P:` lines.
2. `cal 300` on a calibrated tag: the runner must stop for the duration and resume afterwards,
   re-acquiring the beacon within a few superframes. A single `RESCAN` on return is expected.
3. `cal 300` interrupted by each of the three early-return paths in `run_calibration_locked()`:
   the runner must never be left waiting for a radio that is not coming back.

Check 3 is the one that matters most — a missed release is a permanently dead tag, and it is the
failure this design is most exposed to.
