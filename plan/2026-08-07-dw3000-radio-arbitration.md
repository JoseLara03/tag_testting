# DW3000 Radio Arbitration and Calibration Gate — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the DW3000 exactly one owner at any instant via an explicit handover between the MAC runner and the calibration run, and stop a tag from ranging before its antenna delays are calibrated.

**Architecture:** A small rendezvous module (`uwb_radio_owner`) lets the calibration thread claim the radio; the runner checks for a pending claim once per superframe, at the top of its loop, and hands over there — the only point with no exchange in flight. The calibration gate is a pure bitmask filter over the FSM's action word, so no state-machine changes are needed.

**Tech Stack:** Zephyr (nRF Connect SDK v3.2.4), nRF52833, DW3000 via the precompiled Decawave driver, C99. Host tests are plain C compiled with gcc.

## Global Constraints

- Spec: `spec/2026-08-07-dw3000-radio-arbitration-design.md`. Read it before starting.
- Target build (used verbatim in every build step; run from `C:\ncs\v3.2.4`):
  ```
  west build -p always -b nRF52833_tag -d C:\Users\JoseAntonioLaraPerez\Documents\tag_testting\build_npm1304 C:\Users\JoseAntonioLaraPerez\Documents\tag_testting -- -DBOARD_ROOT="C:/Users/JoseAntonioLaraPerez/Documents/tag_testting"
  ```
  with the toolchain env from `C:\ncs\toolchains\fd21892d0f` and `ZEPHYR_BASE=C:\ncs\v3.2.4\zephyr`.
- Host tests: `gcc -Wall -Wextra -Isrc -o <out>.exe <test.c> <src.c>` then run. Zero failures required.
- Work on branch `feat/npm1304-battery`. Do not touch `platform/port.c` beyond what a task states.
- Every new pure module gets a host test under `tests/<module>/`, following the existing `CHECK` macro style (see `tests/batt_window/test_batt_window.c`).
- Do not modify `src/uwb_net.c`'s state machine transitions. The gate must work without them.

---

## File Structure

| File | Responsibility |
|---|---|
| `src/uwb_radio_owner.c` / `.h` | **New.** Rendezvous: claim, yield, release of the DW3000. No radio calls of its own. |
| `src/uwb_net.c` / `.h` | Add `uwb_net_gate_actions()` — pure bitmask filter clearing ranging actions when uncalibrated. |
| `src/uwb_net_runner.c` | Yield point at the top of the superframe loop; reacquire sequence; apply the action gate. |
| `src/uwb_ss_initiator.c` | Calibration claims and releases the radio; remove temporary diagnostics. |
| `src/cal.c` | `cal_get_ant_dly()` falls back to the factory seed instead of returning zeros. |
| `tests/uwb_net/test_uwb_net.c` | Extend with the action-gate tests. |
| `CMakeLists.txt` | Add `src/uwb_radio_owner.c`. |

---

## Task 1: Clean the working tree to a known base

The tree carries uncommitted diagnostics from the `CAL FAIL no-resp` investigation. One is a real fix and stays; the rest go before new work starts.

**Files:**
- Modify: `src/uwb_ss_initiator.c`
- Modify: `src/uwb_net_runner.c`, `src/uwb_net_runner.h`

**Interfaces:**
- Consumes: nothing.
- Produces: a tree where `run_calibration()` is a wrapper around `run_calibration_locked(uint32_t ref_mm)` containing only `dwt_forcetrxoff()` and the three RX-parameter calls. Task 6 replaces that body.

- [ ] **Step 1: Remove the diagnostic counters**

In `src/uwb_ss_initiator.c`, delete the block declaring `cal_n_ok, cal_n_to, cal_n_err, cal_n_bad, cal_n_txf` together with `cal_dbg_reset()` and `cal_dbg_report()`, and delete the `cal_dbg_reset();` and `cal_dbg_report();` calls inside `run_calibration_locked()`.

In `do_one_range()`, remove the counter increments only. The failure branch becomes:

```c
    if (evt != EVT_RXFCG) {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return false;
    }
```

and the frame-mismatch branch becomes:

```c
    if (memcmp(rx_buf, rx_resp_msg, ALL_MSG_COMMON_LEN) != 0) {
        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
        return false;
    }
```

and the success path drops `cal_n_ok++;`, keeping `return true;`.

- [ ] **Step 2: Keep the dwt_starttx check, drop the diagnostic comment**

In `do_one_range()`, the TX block becomes exactly:

```c
    dwt_writetxfctrl(sizeof(tx_poll_msg) + FCS_LEN, 0, 1);
    /* The positioning path checks this too. A poll rejected before it reaches
     * the air must not look like a poll that got no answer. */
    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        frame_seq_nb++;
        return false;
    }
    frame_seq_nb++;
```

- [ ] **Step 3: Remove the suspend/resume experiment**

In `src/uwb_net_runner.h`, delete the `TEMPORARY EXPERIMENT` comment block and the declarations of `uwb_net_runner_suspend()` and `uwb_net_runner_resume()`.

In `src/uwb_net_runner.c`, delete the `TEMPORARY EXPERIMENT` comment block and both function definitions.

In `src/uwb_ss_initiator.c`, `run_calibration()` becomes:

```c
static void run_calibration(uint32_t ref_mm)
{
    dwt_forcetrxoff();
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    run_calibration_locked(ref_mm);
}
```

- [ ] **Step 4: Build**

Run the target build from Global Constraints.
Expected: no errors; no warnings mentioning `uwb_ss_initiator.c` or `uwb_net_runner.c`.

- [ ] **Step 5: Commit**

```bash
git add src/uwb_ss_initiator.c src/uwb_net_runner.c src/uwb_net_runner.h
git commit -m "fix(cal): check dwt_starttx return; drop investigation scaffolding

do_one_range() discarded the TX return code while the positioning path checks
it, so a poll rejected before reaching the air was indistinguishable from one
that got no answer. Remove the temporary CALd counters and the runner
suspend/resume experiment, both of which have served their purpose."
```

---

## Task 2: The calibration action gate

A pure bitmask filter, so the FSM is untouched. The property worth locking down is not that ranging stops — it is that **stopping it does not also stop seat maintenance**.

**Files:**
- Modify: `src/uwb_net.h`, `src/uwb_net.c`
- Test: `tests/uwb_net/test_uwb_net.c`

**Interfaces:**
- Consumes: the existing `UWB_ACT_*` bit definitions in `src/uwb_net.h`.
- Produces: `uint32_t uwb_net_gate_actions(uint32_t act, bool cal_valid)` and `UWB_ACT_RANGING_MASK`. Task 5 calls this.

- [ ] **Step 1: Write the failing test**

Append to `tests/uwb_net/test_uwb_net.c`, before `main()`:

```c
static void test_gate_actions(void)
{
    const uint32_t ranging = UWB_ACT_RUN_DISCOVER | UWB_ACT_RUN_SWEEP;
    const uint32_t housekeeping = UWB_ACT_SEND_JOIN | UWB_ACT_SEND_KEEPALIVE
                                | UWB_ACT_SLEEP | UWB_ACT_TO_SCAN;

    /* Calibrated: every action passes through untouched. */
    CHECK(uwb_net_gate_actions(ranging | housekeeping, true)
          == (ranging | housekeeping));

    /* Uncalibrated: both ranging actions are cleared. */
    CHECK((uwb_net_gate_actions(ranging, false) & UWB_ACT_RUN_DISCOVER) == 0);
    CHECK((uwb_net_gate_actions(ranging, false) & UWB_ACT_RUN_SWEEP) == 0);

    /* Uncalibrated: everything that keeps the seat survives. This is the
     * property that matters -- a tag that stops ranging must not also stop
     * renewing its lease, or it silently drops off the network. */
    CHECK(uwb_net_gate_actions(ranging | housekeeping, false) == housekeeping);

    /* Nothing in, nothing out, either way. */
    CHECK(uwb_net_gate_actions(UWB_ACT_NONE, true) == UWB_ACT_NONE);
    CHECK(uwb_net_gate_actions(UWB_ACT_NONE, false) == UWB_ACT_NONE);
}
```

Add `test_gate_actions();` to `main()` alongside the existing test calls.

- [ ] **Step 2: Run the test to verify it fails**

```bash
gcc -Wall -Wextra -Isrc -o /tmp/un.exe tests/uwb_net/test_uwb_net.c src/uwb_net.c
```
Expected: FAIL to compile with `implicit declaration of function 'uwb_net_gate_actions'`.

- [ ] **Step 3: Write the implementation**

In `src/uwb_net.h`, after the `UWB_ACT_TO_SCAN` definition:

```c
/* Actions that put the radio on the air for ranging. Cleared when the tag has
 * no valid antenna calibration -- see uwb_net_gate_actions(). */
#define UWB_ACT_RANGING_MASK    (UWB_ACT_RUN_DISCOVER | UWB_ACT_RUN_SWEEP)

/* Filter an action word from uwb_net_handle() against calibration validity.
 * Without a valid record the antenna delays are meaningless, so ranging is
 * suppressed while everything that holds the tag's seat is left alone. Pure. */
uint32_t uwb_net_gate_actions(uint32_t act, bool cal_valid);
```

`src/uwb_net.h` already includes `<stdbool.h>` (line 5) and `<stdint.h>` (line 4); no include changes are needed.

In `src/uwb_net.c`, at the end of the file:

```c
uint32_t uwb_net_gate_actions(uint32_t act, bool cal_valid)
{
    if (cal_valid) {
        return act;
    }
    return act & ~UWB_ACT_RANGING_MASK;
}
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
gcc -Wall -Wextra -Isrc -o /tmp/un.exe tests/uwb_net/test_uwb_net.c src/uwb_net.c && /tmp/un.exe
```
Expected: `OK`, exit 0.

- [ ] **Step 5: Commit**

```bash
git add src/uwb_net.h src/uwb_net.c tests/uwb_net/test_uwb_net.c
git commit -m "feat(uwb_net): add uwb_net_gate_actions for the calibration gate

Pure bitmask filter clearing RUN_DISCOVER and RUN_SWEEP when the antenna
calibration is invalid, leaving JOIN, KEEPALIVE, SLEEP and TO_SCAN intact so an
uncalibrated tag keeps its seat instead of dropping off the network. The state
machine is unchanged."
```

---

## Task 3: Stop handing out zero antenna delays

**Files:**
- Modify: `src/cal.c:87-91`

**Interfaces:**
- Consumes: `TX_ANT_DLY` / `RX_ANT_DLY` from `src/phy_config.h` (both `16371`). `src/cal.c` already includes `phy_config.h` at line 4, so no include change is needed.
- Produces: `cal_get_ant_dly()` with unchanged signature; never writes zeros.

- [ ] **Step 1: Change the accessor**

Replace `cal_get_ant_dly()` (currently `src/cal.c:87-91`):

```c
void cal_get_ant_dly(uint16_t *tx, uint16_t *rx)
{
    if (!active_valid) {
        /* No stored record: hand back the factory reference, as
         * active_total_seed() already does. `active` lives in BSS, so the old
         * behaviour was to return zero -- an ~8 m bias that looked like a
         * plausible reading. Callers that must not range uncalibrated are
         * gated by uwb_net_gate_actions(); this is the backstop for the rest. */
        *tx = TX_ANT_DLY;
        *rx = RX_ANT_DLY;
        return;
    }

    *tx = active.tx_ant_dly;
    *rx = active.rx_ant_dly;
}
```

- [ ] **Step 2: Build**

Run the target build.
Expected: no errors, no new warnings.

- [ ] **Step 3: Commit**

```bash
git add src/cal.c
git commit -m "fix(cal): return the factory seed instead of zeros when uncalibrated

cal_get_ant_dly() ignored active_valid and read a BSS-resident record, so an
uncalibrated tag was handed antenna delays of zero rather than the ~16371
factory reference."
```

---

## Task 4: The rendezvous module

**Files:**
- Create: `src/uwb_radio_owner.h`, `src/uwb_radio_owner.c`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: Zephyr kernel primitives only. No DW3000 calls.
- Produces:
  - `bool uwb_radio_request(k_timeout_t timeout);` — true if the radio was handed over.
  - `void uwb_radio_release(void);`
  - `bool uwb_radio_request_pending(void);`
  - `void uwb_radio_yield(void);`

  Tasks 5 and 6 call these.

- [ ] **Step 1: Write the header**

Create `src/uwb_radio_owner.h`:

```c
#ifndef UWB_RADIO_OWNER_H_
#define UWB_RADIO_OWNER_H_

#include <zephyr/kernel.h>
#include <stdbool.h>

/*
 * Explicit handover of the DW3000 between the MAC runner and a claimant.
 *
 * The runner and the calibration thread share one interrupt line and one event
 * semaphore through wait_event(), which is destructive under concurrency: it
 * opens with k_sem_reset() and closes with a global port_DisableEXT_IRQ(). That
 * code is correct as long as exactly one thread drives the radio at a time,
 * which is what this module guarantees.
 *
 * This is not a mutex. The runner sleeps inside its own superframe loop, so a
 * lock held across the loop would starve every other claimant. Instead the
 * runner offers the radio at one safe point per superframe.
 */

/* Claimant: ask for the radio and block until the runner hands it over.
 * Returns false if the runner did not yield within `timeout`, in which case the
 * claim is withdrawn and the radio was NOT acquired. */
bool uwb_radio_request(k_timeout_t timeout);

/* Claimant: hand the radio back. Must be called on every path out of a
 * successful uwb_radio_request(), or the runner blocks forever. */
void uwb_radio_release(void);

/* Runner: is a claimant waiting? Cheap, no blocking. */
bool uwb_radio_request_pending(void);

/* Runner: hand the radio over and block until it is returned. Only call when
 * uwb_radio_request_pending() is true and the radio has been left awake and
 * idle, per the handover contract. */
void uwb_radio_yield(void);

#endif /* UWB_RADIO_OWNER_H_ */
```

- [ ] **Step 2: Write the implementation**

Create `src/uwb_radio_owner.c`:

```c
#include "uwb_radio_owner.h"

static K_SEM_DEFINE(handover_sem, 0, 1);   /* runner -> claimant: it is yours */
static K_SEM_DEFINE(returned_sem, 0, 1);   /* claimant -> runner: it is back  */
static atomic_t requested = ATOMIC_INIT(0);

bool uwb_radio_request(k_timeout_t timeout)
{
    atomic_set(&requested, 1);

    if (k_sem_take(&handover_sem, timeout) != 0) {
        /* The runner never reached its yield point. Withdraw, so a later
         * yield does not hand the radio to a claimant that gave up. */
        atomic_set(&requested, 0);
        return false;
    }

    return true;
}

void uwb_radio_release(void)
{
    atomic_set(&requested, 0);
    k_sem_give(&returned_sem);
}

bool uwb_radio_request_pending(void)
{
    return atomic_get(&requested) != 0;
}

void uwb_radio_yield(void)
{
    k_sem_give(&handover_sem);
    k_sem_take(&returned_sem, K_FOREVER);
}
```

- [ ] **Step 3: Add to the build**

In `CMakeLists.txt`, immediately after the `src/uwb_net_runner.c` line, add:

```
    src/uwb_radio_owner.c
```

- [ ] **Step 4: Build**

Run the target build.
Expected: `Building C object CMakeFiles/app.dir/src/uwb_radio_owner.c.obj` appears; no errors, no warnings for that file.

- [ ] **Step 5: Commit**

```bash
git add src/uwb_radio_owner.c src/uwb_radio_owner.h CMakeLists.txt
git commit -m "feat(uwb): add uwb_radio_owner, an explicit DW3000 handover

wait_event() is correct only while exactly one thread drives the radio. A mutex
would deadlock, because the runner sleeps inside its own superframe loop; this
offers the radio at one safe point per superframe instead. The request side
takes a timeout so a runner that never yields fails the claim rather than
hanging the caller."
```

---

## Task 5: Wire the runner — yield point, reacquire, gate

**Files:**
- Modify: `src/uwb_net_runner.c` (includes; top of the `while (1)` loop in `runner_fn`; the `UWB_ACT_*` dispatch)

**Interfaces:**
- Consumes: `uwb_radio_request_pending()`, `uwb_radio_yield()` (Task 4); `uwb_net_gate_actions()` (Task 2); `cal_is_valid()`, `cal_get_ant_dly()` (Task 3).
- Produces: nothing new for later tasks.

- [ ] **Step 1: Add the includes**

In `src/uwb_net_runner.c`, next to the existing `#include "pos_solver.h"`, add one line:

```c
#include "uwb_radio_owner.h"
```

`cal.h` is already included at line 20 — do not add it again.

- [ ] **Step 2: Insert the yield point**

In `runner_fn`, the `while (1) {` line is immediately followed by the comment `/* 1. Inject any pending tier change before the beacon window. */`. Insert before that comment:

```c
        /* 0. Offer the radio to a waiting claimant. This is the only point in
         * the superframe with no exchange in flight, and the handover contract
         * says we leave the radio awake and idle. */
        if (uwb_radio_request_pending()) {
            if (radio_asleep) { dw_wake(); radio_asleep = false; }
            dwt_forcetrxoff();

            uwb_radio_yield();   /* blocks until the claimant releases */

            /* Seconds may have passed and the antenna delays may have changed.
             * Re-establish everything the claimant could have disturbed. */
            dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
            dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
            dwt_setpreambledetecttimeout(PRE_TIMEOUT);

            uint16_t rtx, rrx;
            cal_get_ant_dly(&rtx, &rrx);
            dwt_settxantennadelay(rtx);
            dwt_setrxantennadelay(rrx);

            /* The arrival prediction is stale after that long off the air. A
             * narrow window aimed at a dead instant would miss repeatedly and
             * trip RESCAN, so go back to ACQUIRING. */
            beacon_track_reset(&bt, T_SUPERFRAME_MS, BT_GUARD_MS,
                               BT_WARMUP_N, BT_EMA_SHIFT);
        }

```

- [ ] **Step 3: Apply the action gate**

`src/uwb_net_runner.c:546` reads:

```c
        uint32_t act = uwb_net_handle(&ctx, &ev);
```

It is not `const`, so no declaration change is needed. Insert immediately after that line, before the first `if (act & ...)`:

```c
        /* Suppress ranging while the antenna delays are uncalibrated. Seat
         * maintenance survives, so the tag stays on the network and is ready
         * for a `cal <mm>` command. */
        act = uwb_net_gate_actions(act, cal_is_valid());
```

- [ ] **Step 4: Build**

Run the target build.
Expected: no errors; no new warnings for `uwb_net_runner.c`.

- [ ] **Step 5: Verify the gate reaches the binary**

```bash
grep -c "uwb_net_gate_actions" src/uwb_net_runner.c
```
Expected: `1`.

- [ ] **Step 6: Commit**

```bash
git add src/uwb_net_runner.c
git commit -m "feat(runner): yield the radio once per superframe; gate ranging on calibration

The runner offers the DW3000 at the top of its loop, the only point with no
exchange in flight, leaving it awake and idle per the handover contract. On
return it re-applies its RX parameters and antenna delays and resets the beacon
tracker to ACQUIRING -- after seconds off the air the arrival prediction is
stale, and a narrow window aimed at it would trip RESCAN.

Ranging actions are filtered against cal_is_valid(). An uncalibrated tag keeps
its seat and its keepalives but emits no position, restoring the guarantee
CLAUDE.md already documents."
```

---

## Task 6: Wire the calibration claim

**Files:**
- Modify: `src/uwb_ss_initiator.c` (`run_calibration()`)

**Interfaces:**
- Consumes: `uwb_radio_request()`, `uwb_radio_release()` (Task 4).
- Produces: nothing for later tasks.

- [ ] **Step 1: Add the include**

In `src/uwb_ss_initiator.c`, next to the existing includes:

```c
#include "uwb_radio_owner.h"
```

- [ ] **Step 2: Claim and release around the whole run**

Replace `run_calibration()` from Task 1 with:

```c
/* Calibration owns the radio for its whole run: consistent conditions across
 * all samples matter more than keeping beacon sync, and this is a bench
 * operation. The runner reacquires and re-locks afterwards.
 *
 * The wrapper exists so that every exit path of run_calibration_locked() --
 * three of them are early returns -- releases the radio. A missed release
 * blocks the runner forever. */
#define CAL_RADIO_WAIT  K_SECONDS(2)

static void run_calibration(uint32_t ref_mm)
{
    if (!uwb_radio_request(CAL_RADIO_WAIT)) {
        twr_log("CAL FAIL busy\n");
        return;
    }

    dwt_forcetrxoff();
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    run_calibration_locked(ref_mm);

    dwt_forcetrxoff();
    uwb_radio_release();
}
```

`CAL_RADIO_WAIT` is 2 s because the runner yields once per superframe (~200 ms); ten superframes without reaching the yield point means the runner is wedged, and failing the command beats hanging the thread that services it.

- [ ] **Step 3: Verify every exit path releases**

```bash
grep -n "return" src/uwb_ss_initiator.c | sed -n '/run_calibration_locked/,$p'
```
Read `run_calibration_locked()` and confirm all three early returns (`CAL FAIL no-resp`, the `CAL OK` / `CAL FAIL nvs` pair, and the trailing `CAL FAIL res`) are **inside** the function, not in the wrapper. Confirm `run_calibration()` has exactly one `uwb_radio_release()` on its success path and one early `return` before the claim succeeds.

- [ ] **Step 4: Build**

Run the target build.
Expected: no errors, no new warnings.

- [ ] **Step 5: Report sizes**

From the build output, record `FLASH` and `RAM` used, and compare against the previous commit's figures (187012 B flash / 35328 B RAM at `eefd377`).

- [ ] **Step 6: Commit**

```bash
git add src/uwb_ss_initiator.c
git commit -m "feat(cal): claim the radio for the whole calibration run

Calibration takes the DW3000 exclusively from first poll to last: consistent
conditions across samples matter more than beacon sync, and calibrating is a
bench operation. The claim has a 2 s timeout so a wedged runner fails the
command instead of hanging the thread that services it, and the wrapper
guarantees a release on every exit path of run_calibration_locked()."
```

---

## Task 7: Update CLAUDE.md

`CLAUDE.md` documents the calibration guarantee that Task 5 restores, and says nothing about radio ownership. It is gitignored in this repo, so this task changes a local file only and produces no commit.

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: Correct the calibration note**

Find the bullet beginning `**UWB antenna delay (calibration):**`. Its claim that the tag "does not range until a `cal` run succeeds" was untrue between the move of ranging to the runner and this work. Append to that bullet:

```
The gate is enforced in the runner via `uwb_net_gate_actions()` (`src/uwb_net.c`), which clears `UWB_ACT_RUN_DISCOVER`/`UWB_ACT_RUN_SWEEP` when `cal_is_valid()` is false while leaving JOIN/KEEPALIVE intact — an uncalibrated tag keeps its seat and emits no `P:` line. This check lived in `ss_twr_fn` originally and was lost when the runner took over ranging; the documented guarantee was false in between.
```

- [ ] **Step 2: Document radio ownership**

Add a new bullet after the power-saving one:

```
- **DW3000 ownership:** the runner and the calibration thread share one IRQ line and one `irq_sem` through `wait_event()` (`src/uwb_ss_initiator.c`), which is destructive under concurrency — it opens with `k_sem_reset()` and closes with a global `port_DisableEXT_IRQ()`. It is correct only while exactly one thread drives the radio. `src/uwb_radio_owner.c` enforces that by explicit handover: the runner offers the radio once per superframe at the top of `runner_fn`, leaving it awake and idle; calibration claims it for its whole run. **Any new radio consumer must go through this handover, not add a third caller of `wait_event()`.** A mutex would not work — the runner sleeps inside its own loop.
```

- [ ] **Step 3: No commit**

`CLAUDE.md` matches `.gitignore`. Confirm with `git status --short` that it does not appear, and do not force-add it.

---

## Self-Review

**Spec coverage.** §3 rendezvous → Tasks 4, 5, 6. §4 reacquire (RX params, antenna delays, `beacon_track_reset`) → Task 5 Step 2. §5 gate → Task 2 and Task 5 Step 3; `cal_get_ant_dly()` fallback → Task 3. §6 cleanup → Task 1. §7 testing → Task 2's host test plus the hardware checks below. §2 (out of scope) needs no task.

**Deliberate deviation from the spec.** §3 sketched one function, `uwb_radio_yield_if_requested()`. The plan splits it into `uwb_radio_request_pending()` + `uwb_radio_yield()` so the runner can satisfy the handover contract — wake the radio and force the transceiver off — only when a claim is actually pending, keeping the common path free. `dw_wake()` stays private to the runner, as §3 requires.

**Second deviation.** §7 proposed a pure predicate for the gate. A function returning its own boolean argument would be a tautology test, so the plan tests `uwb_net_gate_actions()` instead: a bitmask filter whose worthwhile property is that suppressing ranging does not suppress seat maintenance.

**Type consistency.** `uwb_net_gate_actions(uint32_t, bool) -> uint32_t` is defined in Task 2 and called in Task 5 Step 3. `uwb_radio_request(k_timeout_t) -> bool`, `uwb_radio_release(void)`, `uwb_radio_request_pending(void) -> bool`, `uwb_radio_yield(void)` are defined in Task 4 and called in Tasks 5 and 6 with those exact names. `cal_get_ant_dly(uint16_t*, uint16_t*)` keeps its existing signature in Task 3.

---

## Hardware verification (after all tasks — yours to run)

Compilation proves none of this. Three checks, in order of what they would catch:

1. **`cal 300` on a calibrated tag.** The runner must stop for the run and resume afterwards, re-acquiring the beacon within a few superframes. One `RESCAN` on return is expected. Watch for the tag never resuming — that is a missed release, the failure this design is most exposed to.
2. **`cal clear`, then reboot.** The tag must take a seat and hold it (visible in the beacon slot map from the sniffer) while emitting no `P:` lines. If it drops off the network instead, the gate cleared too many action bits.
3. **`cal 300` twice in a row.** Confirms the claim/release pair is re-entrant across runs and that `CAL FAIL busy` does not appear spuriously.

Note that `cal 300` will still report `CAL FAIL no-resp` until the anchor answers the poll — spec §2. These checks verify handover and gating, not that calibration completes.
