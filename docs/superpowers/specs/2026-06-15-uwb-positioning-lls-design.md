# 2D Tag Positioning via Multi-Anchor SS-TWR + Linear Least Squares

**Date:** 2026-06-15
**Branch:** `feat/uwb-positioning-lls` (off `feat/tag-workflow`)
**Status:** Approved design

## Goal

Compute the tag's 2D position by ranging to up to 4 self-describing anchors with
SS-TWR, then solving a linear least-squares (LLS) trilateration. When a position
is valid (≥3 anchors measured), queue it for transmission over BLE. The output
queue is structured so a future UWB-to-master sender can replace the BLE sender
without reworking the ranging/solver path.

## Locked decisions

- **Addressing:** add an `anchor_id` byte to the SS poll/response frames; anchors
  reply only when addressed (mirrors the DS-TWR reference).
- **Solver:** 2D `(x, y)` linear least squares; requires ≥3 anchors. `z` assumed
  fixed/known (single floor).
- **Anchor coordinates:** self-describing — each anchor reports its own `(x, y)`
  inside its SS response payload. No coordinate table on the tag.
- **Anchor list:** static compile-time array of anchor IDs (≤4).
- **Output:** BLE now (`P:x.xx,y.yy\n`), structured for UWB-to-master later. Do
  not build UWB TX in this work (YAGNI).
- **Units:** `float32` metres for anchor coordinates and solved position; ranges
  converted to metres for the solver.
- **Cadence:** unchanged — 0.2 s moving / 1 s static, driven by `uwb_set_moving()`.
- **Math library:** the solver uses CMSIS-DSP (`arm_math.h`) matrix functions
  with the hardware FPU. Requires `CONFIG_CMSIS_DSP=y`,
  `CONFIG_CMSIS_DSP_MATRICES=y`, `CONFIG_FPU=y` in `prj.conf`. Consequence: the
  solver depends on a target-only library, so it is **verified on hardware**,
  not via the host gcc harness (no `pos_solver` host self-test).

## Components

| Unit | Responsibility | Deps | Tested |
|---|---|---|---|
| `pos_solver.c/.h` (new) | 2D LLS trilateration via CMSIS-DSP. In: array of `{x,y,range}` + count. Out: `{x,y,valid}`. | `arm_math.h` (CMSIS-DSP) | on-hardware |
| `uwb_ss_initiator.c` (modified) | Anchor-addressed SS exchange, parse anchor self-coords, per-cycle multi-anchor loop → solver → publish. | DW3000, `pos_solver` | on-hardware |
| `position_publish()` (in initiator) | Format `P:x.xx,y.yy\n` → existing `ss_twr_msgq` → BLE sender thread. Single swap point for future UWB-to-master TX. | `ble_log` | — |

`pos_solver` is still isolated behind a clean interface (`pos_solve`), but its
CMSIS-DSP dependency means it cannot run under the host gcc harness, so it is
verified on hardware via the BLE position output rather than a host self-test.

## Frame contract (anchor responder firmware must match)

These addressed frames are **positioning-only**. The existing non-addressed
calibration frames are unchanged (see "Calibration interaction"). Anchor-side
firmware is updated separately to honor this contract; it is outside the
tag-firmware scope of this work, but the layout below is the spec for it.

**Poll** (tag → anchor), `WAVE` magic + anchor_id:
```
[hdr 0..9][anchor_id @10]                                       = 11 B (+FCS)
```

**Response** (anchor → tag), `VEWA` magic, self-describing:
```
[hdr 0..9][anchor_id @10][poll_rx_ts @11..14][resp_tx_ts @15..18]
         [x f32 @19..22][y f32 @23..26]                         = 27 B (+FCS)
```

Tag-side changes:
- Positioning uses its own poll/response message arrays and field indices
  (`poll_rx_ts @11`, `resp_tx_ts @15`, `x @19`, `y @23`); the calibration frames
  keep the current layout (`poll_rx_ts @10`, `resp_tx_ts @14`, no anchor_id).
- `RX_BUF_LEN` 20 → 32 (shared buffer; harmless for calibration, which reads
  only `flen` bytes).
- Validate `rx_buf[10] == polled aid`; reject a reply from any other anchor.

## LLS math (`pos_solver`)

Circle equations `(x−xᵢ)² + (y−yᵢ)² = rᵢ²`, linearized by subtracting a reference
anchor `k` (anchor index 0 of the supplied set):
```
A_i = [ 2(x_k − x_i),  2(y_k − y_i) ]     (one row per anchor i ≠ k)
b_i = (r_i² − r_k²) + (x_k² + y_k² − x_i² − y_i²)
```
Solve `p = [x, y]` via the normal equations `p = (AᵀA)⁻¹ Aᵀ b`, computed with
CMSIS-DSP `float32_t` matrices:
`arm_mat_trans_f32` → `arm_mat_mult_f32` (form `AᵀA`, `Aᵀb`) →
`arm_mat_inverse_f32` (2×2) → `arm_mat_mult_f32` (final `p`). With `A` of
`(n−1)×2` (≤3×2) and the FPU enabled, the buffers are small fixed-size stack
arrays. Return `valid = false` when fewer than 3 anchors are supplied or
`arm_mat_inverse_f32` reports `ARM_MATH_SINGULAR` (collinear / degenerate
geometry).

## Data flow per ranging cycle

In the existing `ss_twr_fn` loop, after the calibration gate:
```
n = 0
for aid in ANCHOR_IDS[]:                      // static, ≤4
    if do_one_range_anchor(aid, &range_m, &ax, &ay): // SS exchange + parse self-coords
        meas[n++] = { ax, ay, range_m }
    sleep(INTER_ANCHOR_DELAY_MS)
if n >= 3 and pos_solve(meas, n, &pos):       // LLS
    position_publish(pos.x, pos.y)            // → BLE queue
k_sem_take(&range_tick, moving ? 200ms : 1000ms)   // cadence unchanged
```

Positioning uses a new primitive `do_one_range_anchor(uint8_t aid, float
*range_m, float *ax, float *ay)`: it writes `aid` into the poll, validates the
echoed `aid`, parses the anchor's `(x, y)` from the response, and returns the
range in metres. The SS-exchange/ToF math mirrors the existing `do_one_range`;
the small duplication is deliberate, to isolate the new path from the
hardware-verified calibration code. (If DRY is preferred, the shared inner
exchange can be factored into one helper — noted as an option for the plan.)

## Calibration interaction

Calibration is a **bench-only** procedure run before production to calibrate a
unit against a reference device at a known distance. It is not tied to any
deployment anchor or `anchor_id`. The existing `do_one_range(int32_t *out_mm)`
primitive and its non-addressed `WAVE`/`VEWA` frames are **left untouched**, and
`run_calibration()` is unchanged. After this work, `do_one_range` is called only
by calibration; positioning uses `do_one_range_anchor`.

Ranging stays gated on `cal_is_valid()` (`CAL REQUIRED` until a valid record
exists).

## What stays the same

- Cal gating, NVS calibration workflow, IRQ-driven `wait_event`.
- The BLE sender thread and `ss_twr_msgq`.
- Motion-driven cadence (`uwb_set_moving`, `range_tick`).
- No new threads.

## Output / logging

- Position only: `P:x.xx,y.yy\n` (fits the 20-byte NUS limit; signed values
  handled like the existing `D:` formatter).
- Per-anchor `D:` distance logs are dropped. (Can be retained behind a debug
  flag if desired — not part of this scope.)

## Configuration (prj.conf)

Add for CMSIS-DSP matrix math on the FPU:
```
CONFIG_FPU=y
CONFIG_CMSIS_DSP=y
CONFIG_CMSIS_DSP_MATRICES=y
```

## Testing & success criteria

- `pos_solver` is verified **on hardware** (CMSIS-DSP is target-only; no host
  self-test). Success: with ≥3 anchors flashed to the responder contract and a
  valid calibration, the tag emits plausible `P:x.xx,y.yy` positions at the
  correct cadence; with <3 anchors responding, no position is published;
  collinear/degenerate geometry yields no `P:` line (solver returns invalid).

## Out of scope

- UWB-to-master position TX (frame + responder) — future work behind the
  `position_publish()` seam.
- Runtime/NVS-configurable anchor list or anchor coordinates.
- 3D positioning.
