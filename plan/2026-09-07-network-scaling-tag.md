# Network scaling (protocol v3) — TAG implementation plan

> **For agentic workers:** steps use checkbox (`- [ ]`) syntax for tracking. Implement
> task-by-task, in order; each task ends in a state that builds and tests clean.

**Design doc:** `spec/2026-09-07-network-scaling-design.md` — read it first. This plan
implements the tag half and does not restate the rationale.
**Anchor/gateway half:** `../ANCLA_ESP32S3/docs/superpowers/plans/2026-09-07-network-scaling-anchor.md`.
The two plans are phase-locked: a phase is only air-testable when both sides have
finished it.

**Goal:** 100 tags in one PAN at 0.31 Hz stationary / 1.25-2.5 Hz moving, 32 anchors,
100 m links. Today: ~7 tags, 4 anchors, ~6 m of demonstrated range.

**Tech stack:** C99, Zephyr (nCS 3.2.4), nRF52833, DW3000. Host unit tests with WinLibs
GCC.

## Global constraints

- **Host test build command** (gcc is NOT on PATH — see the header of any file in
  `plan/` for the full path on this machine):
  ```
  GCC="/c/Users/.../WinLibs.../mingw64/bin/gcc.exe"
  "$GCC" -std=c99 -Wall -Wextra -Werror -Isrc -o /tmp/t.exe tests/<dir>/test_<x>.c src/<x>.c && /tmp/t.exe
  ```
- **The user builds and flashes the firmware** and reports results. Every on-device step
  is performed by the user.
- **`src/uwb_frame_802_15_4z.{c,h}` must stay byte-identical to the anchor's copy.**
  Every task that touches it ends with the copy step *and* re-running the anchor repo's
  `tests/uwb_frame/` against the new file. A divergence here is a wire bug that shows up
  as an RF fault.
- **`proto_ver` is bumped to 3 exactly once**, in Task 12, after every frame change has
  landed on both sides. Bumping it early makes the bench un-testable; bumping it late
  lets a half-migrated node look like it works.
- **Do not touch the WAVE calibration path** (`src/uwb_wave_frame.h`, `cal_run.c`,
  `cal_diag.c`). It must keep working against a stock Qorvo responder.
- **`ANCHOR_SELECT_MAX` stays 4.** The slot budget and `POS_MAX_ANCHORS` both assume it.
  A bigger pool means a better 4, not a bigger sweep.
- **NUS lines stay <= 19 chars + NUL** (`twr_log()` truncates at 20, `bt_nus_send` drops
  over 20 silently).
- **Do not add a third `wait_event()` caller** and do not take a `uwb_radio_owner` claim
  inside the runner's own loop.
- **The `dbg` raw-range log must be removed before Phase 3** — see the removal checklist
  in `spec/2026-08-22-position-filtering-design.md`. Phase 3 changes the sweep cadence,
  which invalidates any capture taken before it and any capture taken after it is only
  meaningful at the new rate.

---

## Phase 0 — PHY unification and baselines (blocking)

### Task 1: Fix the SFD-timeout divergence

- [ ] In `src/phy_config.c`, `CONFIG_OPTION_07`: change the SFD timeout from
      `(1024 + 1 + 8 - 8)` to `(1024 + 1 + 8 - 32)` = 1001, matching `DWT_PAC32` (which
      the same struct already selects) and the anchor's `uwb_phy.h`.
- [ ] Fix the stale `PAC 8` text in that block's comment and in `phy_config.h`'s
      option-07 description; add a one-line comment that the term is `- PAC`, so the
      next PAC change moves it.
- [ ] Grep every other enabled `CONFIG_OPTION_*` block for the same `- 8` with a
      non-`PAC8` struct; fix or leave a comment saying it is unused.
- [ ] **Step 4 (user, hardware):** reflash, run `cal 1000` to convergence on one tag,
      and confirm `CAL OK`. **The whole fleet must be recalibrated after this task** —
      tag `cal <mm>`, anchors `cal ref`. Record the before/after delay values.

### Task 2: Baseline measurements (user, hardware — no code)

- [ ] Record today's 4-anchor sweep duration from a sniffer capture (the 22 ms figure
      `T_SLOT_MS` is built on) and the per-exchange breakdown: poll airtime, turnaround,
      response airtime, tag-side gap between anchors.
- [ ] Record RSSI at 1 m, 5 m and the maximum distance that still ranges, tag -> anchor
      and gateway -> tag, with the sniffer at a fixed reference point. This is the
      baseline the ANCLA 25 dB deficit (anchor plan Task 1) is measured against.
- [ ] Record `pwr rx` steady-state on a locked tag for the current cadence.

---

## Phase 1 — 32 anchors (tag side)

### Task 3: DISCOVERY gains `group` / `n_groups`

- [ ] `src/uwb_frame_802_15_4z.h`: `UWB_FRAME_LEN_DISC` 14 -> 16; add
      `UWB_FRAME_DISC_N_GROUPS_MAX` (8) and a comment tying it to
      `ceil(UWB_MAX_ANCHORS / 4)` on the anchor.
- [ ] `uwb_frame_discovery_build()` takes `group` and `n_groups`; add
      `uwb_frame_parse_discovery()` (the anchor needs it; the tag builds only).
- [ ] Reject `n_groups == 0`, `group >= n_groups`, `n_groups > _MAX` with `-EINVAL`.
- [ ] `tests/uwb_frame/`: round-trip, every rejection, and a byte-exact vector for
      `group=3, n_groups=8` that the anchor's copy of the test can share.
- [ ] Copy the file pair to the anchor repo; run **both** repos' `tests/uwb_frame/`.

### Task 4: ANNOUNCE (`0xEC`) parser

- [ ] Add `UWB_FRAME_TYPE_ANNOUNCE 0xEC`, `UWB_FRAME_LEN_ANNOUNCE`, and
      `struct uwb_announce { uint16_t addr; float x, y, z; int32_t cir_power;
      uint16_t cir_quality; }`.
- [ ] `uwb_frame_announce_build()` (anchor uses it) and
      `uwb_frame_parse_announce()` / `uwb_frame_is_announce()` (tag uses these).
- [ ] `tests/uwb_frame/`: round-trip including a NaN `z`, a truncated frame, and a
      wrong-type frame.
- [ ] Copy to the anchor repo; run both suites.

### Task 5: Anchor pool grows and learns passively

- [ ] `src/uwb_net_runner.c`: `ANCHOR_POOL_MAX` 6 -> 16. Leave `ANCHOR_SELECT_MAX` at 4
      and add a comment saying why (design §3.C).
- [ ] `anchor_entry_t` gains `float x, y, z; bool have_pos; uint32_t last_seen_ms;`.
      Announce-learned coordinates seed the pool so a tag can rank anchors by geometry,
      not only by CIR.
- [ ] **Pool entries expire.** Add `ANCHOR_ENTRY_STALE_MS` (300000) and drop entries not
      seen since. Today the pool never invalidates an entry, which is what let the tag
      report three anchors while emitting no fix at all (`CLAUDE.md`, sweep-gate entry).
      At 16 entries that failure gets easier, not harder.
- [ ] In the beacon RX path, after a valid beacon, keep the receiver armed for the
      `T_ANNOUNCE_MS` (2 ms) window and feed any `0xEC` to the pool. Do **not** extend
      the window when no announce arrives — the whole point is that it is bounded.
- [ ] Host-test the pool logic by moving it into a pure `anchor_pool_core.c` with
      `tests/anchor_pool/`: insert, EMA update, staleness, top-4 selection, and the
      "never report more anchors than are actually live" property.

### Task 6: Grouped discovery rounds in the runner

- [ ] Add `disc_group` state to the runner; each discovery round uses
      `group = disc_round++ % UWB_DISC_N_GROUPS` and passes it to the builder.
- [ ] `DISCOVERY_WINDOW_MS` stays 15. Add a `BUILD_ASSERT` (or a comment with the
      arithmetic) that 15 ms >= `DISC_BASE_UUS + 3 * DISC_SLOT_UUS` in ms, so the next
      person who edits the stagger sees the dependency.
- [ ] `uwb_sweep_gate_discovered()` must count anchors found **across the group cycle**,
      not within one round: a round that finds 2 anchors of its group is not a short
      sweep. Add `n_groups`-aware accounting to `struct uwb_sweep_gate` and a regression
      test alongside `test_sweep_gate_recovers_after_short_sweep()` — the latching
      failure this gate exists to prevent is exactly the shape grouping could reintroduce.
- [ ] `tests/uwb_net/`: group cycling covers every group; a full cycle with 32 anchors
      spread over 8 groups reaches `UWB_NET_MIN_ANCHORS` and does not latch.
- [ ] **Step 4 (user, hardware, needs anchor Phase 1):** with 5+ anchors on air, confirm
      the tag discovers all of them within `n_groups` rounds, and that `P:` output
      continues throughout.

---

## Phase 2 — multi-poll ranging (slot 24 ms -> 12 ms)

### Task 7: MPOL_RESP (`0xED`) with anchor `z`

- [ ] Add `UWB_FRAME_TYPE_MPOL_RESP 0xED`, `UWB_FRAME_LEN_MPOL_RESP` (31), builder,
      parser, validator, per design §3.D's byte layout.
- [ ] `z` is a float and **may be NaN**, meaning "anchor did not report a height";
      the parser must pass NaN through rather than rejecting it.
- [ ] `tests/uwb_frame/`: round-trip, NaN `z`, wrong dest address, short frame.
- [ ] Copy to the anchor repo; run both suites.

### Task 8: Multi-poll sweep on the tag

- [ ] New `uwb_multipoll_sweep(struct pos_meas *out, size_t max)` in
      `src/uwb_ss_initiator.c`: build `0xE3` naming `selected[0..n-1]` with
      `delay_us = MPOL_BASE_UUS + k * MPOL_SLOT_UUS`, transmit once, then keep the
      receiver armed for the whole response window, matching frames by `src_addr` and
      `anchor_id`.
- [ ] **Arm the receiver for the full remaining window, never in fixed slices.** This is
      the same bug that made anchor id 2 invisible in `run_discovery()`
      (`CLAUDE.md`, discovery-slice entry); a multi-poll response window sliced at a
      fixed period will lose whichever anchor's stagger lands on a boundary.
- [ ] Compute distance per response with the existing clock-offset correction; a missing
      response is a missing `pos_meas`, not a failure.
- [ ] Keep `do_one_range_anchor()` compiled — it is the fallback for a single-anchor
      bench check and the `cal` path's structural sibling — but the runner no longer
      calls it in the sweep.
- [ ] `MPOL_BASE_UUS` / `MPOL_SLOT_UUS` live next to each other with the airtime
      arithmetic in a comment (design §3.B), because the anchor derives its own TX
      deadline from the same numbers.
- [ ] **Step 4 (user, hardware, needs anchor Phase 2):** sniffer-confirm one poll and up
      to four staggered responses inside one slot; measure the real slot occupancy and
      write the number into the design's §11 item 3.

### Task 9: Slot and frame constants follow the measurement

- [ ] Only after Task 8 Step 4: `T_SLOT_MS` 24 -> 12 (or the measured value + 2 ms
      guard), `UWB_FRAME_N_CFP` 11 -> 14, `UWB_FRAME_MAX_LEN` 37 -> 44.
- [ ] Re-derive every `BUILD_ASSERT` that ties those together on both sides; the anchor's
      `uwb_slave.c` has one that will fail loudly, which is the desired behaviour.
- [ ] Confirm `beacon_buf[UWB_FRAME_MAX_LEN]` and every RX path still fits (anchor
      buffers are 64 B; the tag's beacon buffer is sized by the macro).
- [ ] `tests/uwb_net/`: the existing `test_proto_ver_matches_frame_module` style pin, plus
      a new one asserting `T_SLOT_MS * N_CFP + overhead <= T_SUPERFRAME_MS`. A budget
      that no longer closes must fail a host test, not a bench session.

### Task 10: Anchor `z` reaches the solver

- [ ] `struct pos_meas` gains `float z` (already 3D-modelled internally via `pos_cfg`'s
      `dz`); `pos_solver.c` / `pos_residual.c` / `pos_ekf.c` use the per-anchor `z` when
      it is finite and fall back to `pos_cfg`'s single `dz` when it is NaN.
- [ ] `pos z` NUS command keeps setting the fallback; add one line to its reply saying
      how many of the last sweep's anchors reported a real `z`.
- [ ] Extend `tests/pos_solver/`, `tests/pos_residual/`, `tests/pos_ekf/` with a mixed
      case: two anchors reporting `z`, two NaN. Mutation-test as the existing suites are.

---

## Phase 3 — superframe cycle (capacity x16)

### Task 11: Phase-aware participation in `uwb_net.c`

- [ ] `UWB_NET_CYCLE_C` = 16 in `uwb_net.h`, with the §5 budget arithmetic in the
      comment.
- [ ] `struct uwb_net_ctx` gains `uint16_t phase_mask`. `UWB_EV_BEACON` carries
      `frame_counter`; participation requires `phase_mask & (1u << (frame_counter % C))`
      **and** `in_map`. Both, not either: the mask is what the gateway granted, the map
      is what it is publishing right now, and a disagreement means re-JOIN.
- [ ] GRANT parse: length 24 -> 26, new `phase_mask` field. `uwb_frame_grant_build()`
      gains the parameter (the gateway builds it).
- [ ] `listen_skip` becomes derived: the runner sleeps to the next set bit in the mask
      instead of reading the tier table. Keep `pwr tier` as an override for bench work,
      clamped to the mask.
- [ ] `lease_age()` already subtracts elapsed superframes — verify it still holds when a
      tag is awake only 1 superframe in 16, and extend
      `test_lease_ages_by_elapsed()` accordingly. This is the change most likely to
      produce `RESCAN seat`.
- [ ] `tests/uwb_net/`: single-phase tag participates exactly once per 16; 4-phase mover
      participates 4 times; a mask that disagrees with the map forces re-JOIN; wrap of
      `frame_counter` across `2^32` does not skip a phase.

### Task 12: Runner wake planning, keepalive suppression, `proto_ver` 3

- [ ] `beacon_sched_core` plans the wake for the **next set phase bit**, not for a fixed
      skip. Its long-baseline estimator already takes an arbitrary superframe delta; the
      change is in what the runner asks for.
- [ ] KEEPALIVE is sent only after `KEEPALIVE_AFTER_N` (default 4) participations that
      produced no `0xEA` POS frame. With POS renewing the lease (anchor Task 20), a
      healthy tag never enters CAP after JOIN.
- [ ] `UWB_NET_PROTO_VER` and `UWB_PROTO_VER` -> **3**, in the same commit, on both
      repos. Re-run `test_proto_ver_matches_frame_module`.
- [ ] `pwr sched` reports the phase mask and the derived skip; `pwr tier` prints the mask
      it is clamped by. Keep every line <= 19 chars — split across lines rather than
      widening.
- [ ] **Step 4 (user, hardware):** one tag joins, receives a 1-phase grant, and holds it
      for 30 minutes with no `RESCAN`; `pwr rx` shows RX-on falling by roughly the
      cycle factor against the Task 2 baseline.

---

## Phase 4 — range (blocked on hardware, anchor plan Phase 4)

### Task 13: Link validation at distance

- [ ] **Step 4 (user, hardware):** with the ANCLA transmit deficit closed and whatever
      antenna/LNA change it implies, walk a tag out from a gateway + 4 anchors and record
      the distance at which `n_anchors` drops below 3, the RSSI at that point, and the
      residual trend.
- [ ] Only if the link closes short of 100 m and hardware has no more headroom: reopen
      the PHY decision in design §8 with the measurement attached. Do not pre-emptively
      lengthen the preamble — it costs the whole of Phase 2.

### Task 14: Documentation

- [ ] Update `CLAUDE.md`: the capacity numbers, the anchor cap, `proto_ver` 3, the new
      frames, the grouped-discovery invariant (the two derived timeouts), the phase
      mask, and the fact that `UWB_LISTEN_SKIP_CAP` is now superseded by the grant.
- [ ] Update `spec/2026-06-17-uwb-mac-protocol-contract.md` to v3, or mark it superseded
      by the scaling design — one of the two, not neither.
- [ ] Fold the measured numbers from Tasks 2, 8 and 13 back into the design's §5 and §11.
