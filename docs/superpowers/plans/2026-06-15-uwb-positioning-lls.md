# 2D Tag Positioning (Multi-Anchor SS-TWR + LLS) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Compute the tag's 2D position by ranging up to 4 self-describing anchors over SS-TWR and solving a CMSIS-DSP linear least-squares trilateration, publishing valid positions (≥3 anchors) over BLE.

**Architecture:** A new `pos_solver` module does the 2D LLS math using CMSIS-DSP (`arm_math.h`) matrix functions on the hardware FPU. The existing SS-TWR initiator gains an anchor-addressed ranging primitive (`do_one_range_anchor`) and a per-cycle loop that ranges each anchor, feeds `{x,y,range}` to the solver, and publishes the result through a single `position_publish()` seam (BLE today, UWB-to-master later). The hardware-verified calibration path is left untouched.

**Tech Stack:** Zephyr RTOS, nRF52833 (Cortex-M4F), DW3000 (Decawave driver), CMSIS-DSP, C99.

**Spec:** `docs/superpowers/specs/2026-06-15-uwb-positioning-lls-design.md`

**Verification note:** Per project convention the user builds/flashes (`west build`) and reports results. The solver depends on CMSIS-DSP (target-only), so there is **no host self-test** — the solver and the radio code are verified together on hardware in Task 4.

---

## File Structure

| File | Responsibility |
|---|---|
| `prj.conf` (modify) | Enable FPU + CMSIS-DSP matrix support |
| `src/pos_solver.h` (new) | Solver types + `pos_solve()` declaration |
| `src/pos_solver.c` (new) | 2D LLS trilateration via CMSIS-DSP matrices |
| `CMakeLists.txt` (modify) | Add `src/pos_solver.c` to the build |
| `src/uwb_ss_initiator.c` (modify) | Positioning frames, `do_one_range_anchor`, multi-anchor cycle, `position_publish` |

---

## Task 1: Enable FPU + CMSIS-DSP

**Files:**
- Modify: `prj.conf`

- [ ] **Step 1: Add the configs**

Append to `prj.conf`:

```
CONFIG_FPU=y
CONFIG_FP_SOFTABI=y
CONFIG_CMSIS_DSP=y
CONFIG_CMSIS_DSP_MATRIX=y
```

`CONFIG_FP_SOFTABI=y` is required: `CONFIG_FPU=y` alone defaults to `FP_HARDABI`,
which makes the image use VFP register arguments and fails to link against the
soft-float precompiled DW3000 driver (`libdwt_uwb_driver-m4-sfp`). The matrix
option is `CMSIS_DSP_MATRIX` (singular), not `CMSIS_DSP_MATRICES`.

- [ ] **Step 2: Verify (build)**

Ask the user to run `west build`. Expected: clean configure + build (the
nRF52833 is a Cortex-M4F, so `CONFIG_FPU=y` is valid; CMSIS-DSP is provided by
the Zephyr module). No new errors.

- [ ] **Step 3: Commit**

```bash
git add prj.conf
git commit -m "build(pos): enable FPU and CMSIS-DSP matrix support"
```

---

## Task 2: `pos_solver` module (CMSIS-DSP)

**Files:**
- Create: `src/pos_solver.h`
- Create: `src/pos_solver.c`
- Modify: `CMakeLists.txt:6-25` (the `target_sources(app PRIVATE ...)` list)

- [ ] **Step 1: Write the solver header**

Create `src/pos_solver.h`:

```c
#ifndef POS_SOLVER_H
#define POS_SOLVER_H

#include <stddef.h>
#include <stdbool.h>

/* Maximum anchors per solve (matches the static anchor list). 2D needs >=3. */
#define POS_MAX_ANCHORS  4

/* One anchor measurement: anchor position (metres) + measured range (metres). */
struct pos_meas {
    float x;
    float y;
    float range_m;
};

/* Solved 2D position. valid == false means no solution was produced. */
struct pos_result {
    float x;
    float y;
    bool  valid;
};

/* 2D linear least-squares trilateration (CMSIS-DSP).
 *
 * Linearizes the circle equations by subtracting anchor index 0 (the
 * reference), then solves the 2x2 normal equations p = (At*A)^-1 * (At*b).
 * Needs 3 <= n <= POS_MAX_ANCHORS. Sets out->valid = false and returns false
 * when n is out of range or the geometry is degenerate (collinear anchors ->
 * singular normal matrix). */
bool pos_solve(const struct pos_meas *m, size_t n, struct pos_result *out);

#endif /* POS_SOLVER_H */
```

- [ ] **Step 2: Write the solver implementation**

Create `src/pos_solver.c`:

```c
#include "pos_solver.h"
#include <arm_math.h>

/* Largest A is (POS_MAX_ANCHORS - 1) x 2. */
#define POS_MAX_ROWS  (POS_MAX_ANCHORS - 1)

bool pos_solve(const struct pos_meas *m, size_t n, struct pos_result *out)
{
    out->valid = false;
    if (n < 3 || n > POS_MAX_ANCHORS) {
        return false;
    }

    const uint16_t rows = (uint16_t)(n - 1);

    /* Fixed-size scratch (max sizes; only the first `rows` are used). */
    float32_t A_d[POS_MAX_ROWS * 2];
    float32_t b_d[POS_MAX_ROWS * 1];
    float32_t At_d[2 * POS_MAX_ROWS];
    float32_t AtA_d[2 * 2];
    float32_t Atb_d[2 * 1];
    float32_t inv_d[2 * 2];
    float32_t p_d[2 * 1];

    /* Reference anchor = index 0. For each other anchor i:
     *   A_i = [ 2(xk - xi), 2(yk - yi) ]
     *   b_i = (ri^2 - rk^2) + (xk^2 + yk^2 - xi^2 - yi^2) */
    const float32_t xk = m[0].x, yk = m[0].y, rk = m[0].range_m;
    const float32_t ck = xk * xk + yk * yk;

    for (uint16_t i = 0; i < rows; i++) {
        const struct pos_meas *mi = &m[i + 1];
        float32_t ci = mi->x * mi->x + mi->y * mi->y;

        A_d[i * 2 + 0] = 2.0f * (xk - mi->x);
        A_d[i * 2 + 1] = 2.0f * (yk - mi->y);
        b_d[i] = (mi->range_m * mi->range_m - rk * rk) + (ck - ci);
    }

    arm_matrix_instance_f32 A, b, At, AtA, Atb, inv, p;
    arm_mat_init_f32(&A,   rows, 2, A_d);
    arm_mat_init_f32(&b,   rows, 1, b_d);
    arm_mat_init_f32(&At,  2, rows, At_d);
    arm_mat_init_f32(&AtA, 2, 2, AtA_d);
    arm_mat_init_f32(&Atb, 2, 1, Atb_d);
    arm_mat_init_f32(&inv, 2, 2, inv_d);
    arm_mat_init_f32(&p,   2, 1, p_d);

    arm_mat_trans_f32(&A, &At);       /* At = A^T            */
    arm_mat_mult_f32(&At, &A, &AtA);  /* AtA = A^T A (2x2)   */
    arm_mat_mult_f32(&At, &b, &Atb);  /* Atb = A^T b (2x1)   */

    /* arm_mat_inverse_f32 returns ARM_MATH_SINGULAR on a non-invertible matrix
     * (collinear/degenerate anchor geometry). It may modify AtA in place. */
    if (arm_mat_inverse_f32(&AtA, &inv) != ARM_MATH_SUCCESS) {
        return false;
    }

    arm_mat_mult_f32(&inv, &Atb, &p); /* p = inv(AtA) * Atb  */

    out->x = p_d[0];
    out->y = p_d[1];
    out->valid = true;
    return true;
}
```

- [ ] **Step 3: Register the source in the build**

In `CMakeLists.txt`, add `src/pos_solver.c` to the `target_sources` list,
directly after the `src/cal.c` line:

```cmake
    src/cal_math.c
    src/cal.c
    src/pos_solver.c
    src/uwb_ss_initiator.c
```

- [ ] **Step 4: Verify (build)**

Ask the user to `west build`. Expected: clean build; `pos_solver.c` compiles
and links against CMSIS-DSP (`arm_mat_*` symbols resolve). `pos_solve` is unused
until Task 4 — if the build treats unused statics/functions as errors this is
expected; otherwise it links cleanly. Behaviour is validated on hardware in
Task 4.

- [ ] **Step 5: Commit**

```bash
git add src/pos_solver.h src/pos_solver.c CMakeLists.txt
git commit -m "feat(pos): 2D linear least-squares trilateration via CMSIS-DSP"
```

---

## Task 3: Anchor-addressed ranging primitive

**Files:**
- Modify: `src/uwb_ss_initiator.c` (includes; frames block ~lines 47-58; add new function after `do_one_range`, ~line 216)

Radio firmware — verified by build here and on-hardware in Task 4.

- [ ] **Step 1: Add the `pos_solver` include**

In `src/uwb_ss_initiator.c`, after `#include "cal_math.h"`:

```c
#include "pos_solver.h"
```

- [ ] **Step 2: Enlarge the RX buffer and add positioning frame definitions**

The positioning response is 27 bytes; the shared `rx_buf` must hold it. Change
`RX_BUF_LEN` from 20 to 32:

```c
#define RX_BUF_LEN               32
```

Then, immediately after the existing `rx_resp_msg` definition (leave the
calibration frames unchanged), add the positioning frames and field indices:

```c
/* ---- Positioning frames (addressed; anchor self-reports its (x,y)) --------
 * Poll : [hdr 0..9][anchor_id @10]
 * Resp : [hdr 0..9][anchor_id @10][poll_rx_ts @11..14][resp_tx_ts @15..18]
 *        [x f32 @19..22][y f32 @23..26]
 * Distinct from the non-addressed calibration frames above. */
#define POS_ANCHOR_ID_IDX        10
#define POS_POLL_RX_TS_IDX       11
#define POS_RESP_TX_TS_IDX       15
#define POS_ANCHOR_X_IDX         19
#define POS_ANCHOR_Y_IDX         23

static uint8_t pos_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0 };
static uint8_t pos_resp_ref[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1 };
```

- [ ] **Step 3: Add the `do_one_range_anchor` primitive**

Insert this function immediately after `do_one_range` (after its closing brace,
before `apply_total_dly`). It mirrors `do_one_range` but addresses one anchor
and parses the anchor's self-reported coordinates.

```c
/*
 * Run a single addressed SS-TWR exchange against anchor `aid`. On a valid,
 * id-matched response, writes the range in metres to *range_m and the anchor's
 * self-reported coordinates to *ax/*ay, then returns true. Returns false on
 * timeout, RX error, wrong magic, or an anchor_id mismatch. Relies on the
 * rx-after-tx delay / timeout / antenna delay configured by ss_twr_fn.
 */
static bool do_one_range_anchor(uint8_t aid, float *range_m, float *ax, float *ay)
{
    dwt_setinterrupt(INT_RX_PHASE, 0, DWT_ENABLE_INT_ONLY);

    pos_poll_msg[ALL_MSG_SN_IDX]    = frame_seq_nb;
    pos_poll_msg[POS_ANCHOR_ID_IDX] = aid;
    dwt_writetxdata(sizeof(pos_poll_msg), pos_poll_msg, 0);
    dwt_writetxfctrl(sizeof(pos_poll_msg) + FCS_LEN, 0, 1);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    frame_seq_nb++;

    irq_evt_t evt = wait_event(K_MSEC(20));

    if (evt != EVT_RXFCG) {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return false;
    }

    uint16_t flen = dwt_getframelength();
    if (flen > RX_BUF_LEN) {
        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
        return false;
    }
    dwt_readrxdata(rx_buf, flen, 0);
    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

    rx_buf[ALL_MSG_SN_IDX] = 0;
    if (memcmp(rx_buf, pos_resp_ref, ALL_MSG_COMMON_LEN) != 0) {
        return false;
    }
    if (rx_buf[POS_ANCHOR_ID_IDX] != aid) {
        return false;   /* response from a different anchor */
    }

    uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
    uint32_t resp_rx_ts = dwt_readrxtimestamplo32();
    double clock_offset_ratio =
        ((double)dwt_readclockoffset()) / (uint32_t)(1 << 26);
    uint32_t poll_rx_ts = get_ts_4b(&rx_buf[POS_POLL_RX_TS_IDX]);
    uint32_t resp_tx_ts = get_ts_4b(&rx_buf[POS_RESP_TX_TS_IDX]);

    int32_t rtd_init = (int32_t)(resp_rx_ts - poll_tx_ts);
    int32_t rtd_resp = (int32_t)(resp_tx_ts - poll_rx_ts);

    double tof = ((rtd_init - rtd_resp * (1 - clock_offset_ratio)) / 2.0)
                 * DWT_TIME_UNITS;
    *range_m = (float)(tof * SPEED_OF_LIGHT);

    memcpy(ax, &rx_buf[POS_ANCHOR_X_IDX], sizeof(float));
    memcpy(ay, &rx_buf[POS_ANCHOR_Y_IDX], sizeof(float));
    return true;
}
```

- [ ] **Step 4: Verify (build)**

`do_one_range_anchor` is unused until Task 4 — with `-Werror` on unused statics
this won't build standalone, so fold this build check into Task 4 Step 4. Commit
the code regardless (next step).

- [ ] **Step 5: Commit**

```bash
git add src/uwb_ss_initiator.c
git commit -m "feat(uwb): addressed SS-TWR primitive parsing anchor self-coords"
```

---

## Task 4: Multi-anchor cycle, position publish, wire-in

**Files:**
- Modify: `src/uwb_ss_initiator.c` (anchor list + `position_publish` near the timing defines; rework the ranging branch in `ss_twr_fn` ~lines 329-337)

- [ ] **Step 1: Add the anchor list and inter-anchor delay**

After the timing defines block (after `#define RNG_SLOW_MS 1000U`), add:

```c
/* Anchors to range each cycle (static; <=4). Positioning needs >=3 of these
 * to respond in a cycle to produce a fix. Edit and rebuild to change the set. */
static const uint8_t ANCHOR_IDS[] = { 1, 2, 3, 4 };
#define POS_NUM_ANCHORS  ARRAY_SIZE(ANCHOR_IDS)

/* Settle time between anchors within one cycle (radio turnaround margin). */
#define INTER_ANCHOR_DELAY_MS  10U
```

- [ ] **Step 2: Add the `position_publish` seam**

Add these two functions just before `ss_twr_fn`. `fmt_coord` avoids `%f`
(the nano printf has no float support) by splitting into integer centimetres,
matching the existing `D:` formatter style.

```c
/* Format a metre value as a signed "x.xx" string (centimetre resolution),
 * without relying on %f. */
static void fmt_coord(char *buf, size_t len, float v)
{
    int cm = (int)(v * 100.0f);     /* truncates toward zero */
    const char *sign = (cm < 0) ? "-" : "";

    if (cm < 0) {
        cm = -cm;
    }
    snprintf(buf, len, "%s%d.%02d", sign, cm / 100, cm % 100);
}

/*
 * Single output seam for a solved position. Today: format "P:x.xx,y.yy\n"
 * (<=20 bytes for the NUS limit) and enqueue to the BLE sender. A future
 * UWB-to-master sender replaces only this function body.
 */
static void position_publish(float x, float y)
{
    char xs[12], ys[12];

    fmt_coord(xs, sizeof(xs), x);
    fmt_coord(ys, sizeof(ys), y);
    twr_log("P:%s,%s\n", xs, ys);
}
```

- [ ] **Step 3: Replace the single-range branch in `ss_twr_fn` with the cycle**

In `ss_twr_fn`, replace this existing block:

```c
        int32_t mm;
        if (do_one_range(&mm)) {
            int32_t v = mm;
            const char *sign = (v < 0) ? "-" : "";
            if (v < 0) {
                v = -v;
            }
            twr_log("D:%s%d.%02dm\n", sign, v / 1000, (v % 1000) / 10);
        }
```

with the multi-anchor positioning cycle:

```c
        struct pos_meas meas[POS_NUM_ANCHORS];
        size_t n = 0;

        for (size_t i = 0; i < POS_NUM_ANCHORS; i++) {
            float r, ax, ay;
            if (do_one_range_anchor(ANCHOR_IDS[i], &r, &ax, &ay)) {
                meas[n].x = ax;
                meas[n].y = ay;
                meas[n].range_m = r;
                n++;
            }
            k_sleep(K_MSEC(INTER_ANCHOR_DELAY_MS));
        }

        struct pos_result pos;
        if (n >= 3 && pos_solve(meas, n, &pos)) {
            position_publish(pos.x, pos.y);
        }
```

The cadence wait below this block (`k_sem_take(&range_tick, K_MSEC(wait_ms))`)
is unchanged. `do_one_range` and `run_calibration` remain, still used by the
calibration path.

- [ ] **Step 4: Verify (build + on-hardware)**

Ask the user to `west build`, flash, and observe over BLE NUS:
- Expected build: clean (no unused-function warning now — `do_one_range_anchor`
  and `pos_solve` are referenced).
- With ≥3 anchors flashed to the response contract and a valid calibration:
  plausible `P:x.xx,y.yy` lines arrive at ~0.2 s when moving / ~1 s when still.
- With <3 anchors responding: no `P:` line is emitted.
- Degenerate (collinear) anchor geometry: no `P:` line (solver returns invalid).
- If responses time out unexpectedly, the longer (27-byte) response may exceed
  `RESP_RX_TIMEOUT_UUS`; bump it (e.g. to 3000) as an on-hardware tuning step.

- [ ] **Step 5: Commit**

```bash
git add src/uwb_ss_initiator.c
git commit -m "feat(uwb): multi-anchor ranging cycle -> LLS -> BLE position publish"
```

---

## Self-Review notes

- **Spec coverage:** FPU+CMSIS-DSP config (Task 1) · CMSIS-DSP solver + build wiring (Task 2) · addressed frames, primitive, anchor_id validation, self-coord parse (Task 3) · static anchor list, ≥3 gate, cadence preserved, `position_publish` seam, `D:`→`P:` (Task 4). Calibration left untouched (no task edits `do_one_range`/`run_calibration`).
- **Type consistency:** `pos_meas{x,y,range_m}`, `pos_result{x,y,valid}`, `pos_solve(const struct pos_meas*, size_t, struct pos_result*)`, `do_one_range_anchor(uint8_t, float*, float*, float*)`, `POS_MAX_ANCHORS` used identically across Tasks 2–4.
- **Testing:** no host self-test (CMSIS-DSP is target-only, per the approved decision); solver correctness is validated on hardware via the BLE `P:` output in Task 4.
- **Known build-order caveat:** `do_one_range_anchor` is unused between Task 3 and Task 4; with `-Werror` on unused-static, Task 3 won't build standalone — fold its build check into Task 4.
```
