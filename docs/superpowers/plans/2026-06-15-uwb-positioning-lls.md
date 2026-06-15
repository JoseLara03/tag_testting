# 2D Tag Positioning (Multi-Anchor SS-TWR + LLS) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Compute the tag's 2D position by ranging up to 4 self-describing anchors over SS-TWR and solving a linear least-squares trilateration, publishing valid positions (≥3 anchors) over BLE.

**Architecture:** A new pure, host-tested `pos_solver` module does the 2D LLS math. The existing SS-TWR initiator gains an anchor-addressed ranging primitive (`do_one_range_anchor`) and a per-cycle loop that ranges each anchor, feeds `{x,y,range}` to the solver, and publishes the result through a single `position_publish()` seam (BLE today, UWB-to-master later). The hardware-verified calibration path is left untouched.

**Tech Stack:** Zephyr RTOS, nRF52833, DW3000 (Decawave driver), C99. Host unit tests built with WinLibs gcc.

**Spec:** `docs/superpowers/specs/2026-06-15-uwb-positioning-lls-design.md`

---

## File Structure

| File | Responsibility |
|---|---|
| `src/pos_solver.h` (new) | Solver types + `pos_solve()` + `pos_solver_selftest()` declarations |
| `src/pos_solver.c` (new) | 2D LLS trilateration + built-in self-test vectors |
| `tests/pos_solver/test_pos_solver.c` (new) | Host runner: calls `pos_solver_selftest()` |
| `src/uwb_ss_initiator.c` (modify) | Positioning frames, `do_one_range_anchor`, multi-anchor cycle, `position_publish` |
| `CMakeLists.txt` (modify) | Add `src/pos_solver.c` to the build |

Conventions mirror the existing `src/cal_math.c` / `tests/cal_math/test_cal_math.c` pair.

---

## Task 1: `pos_solver` pure module + host test

**Files:**
- Create: `src/pos_solver.h`
- Create: `tests/pos_solver/test_pos_solver.c`
- Create: `src/pos_solver.c`

- [ ] **Step 1: Write the solver header**

Create `src/pos_solver.h`:

```c
#ifndef POS_SOLVER_H
#define POS_SOLVER_H

#include <stddef.h>
#include <stdbool.h>

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

/* 2D linear least-squares trilateration.
 *
 * Linearizes the circle equations by subtracting anchor index 0 (the
 * reference), then solves the 2x2 normal equations. Needs n >= 3. Sets
 * out->valid = false and returns false when n < 3 or the geometry is
 * degenerate (collinear anchors -> singular normal matrix). */
bool pos_solve(const struct pos_meas *m, size_t n, struct pos_result *out);

/* Run built-in assertion vectors. Returns the number of failed checks
 * (0 == all pass). */
int pos_solver_selftest(void);

#endif /* POS_SOLVER_H */
```

- [ ] **Step 2: Write the host test runner**

Create `tests/pos_solver/test_pos_solver.c`:

```c
#include "../../src/pos_solver.h"
#include <stdio.h>

int main(void)
{
    int fails = pos_solver_selftest();
    printf("pos_solver_selftest: %d failure(s)\n", fails);
    return fails == 0 ? 0 : 1;
}
```

- [ ] **Step 3: Write the solver with a stub `pos_solve` + real self-test vectors**

Create `src/pos_solver.c`. The self-test vectors below encode the expected
behaviour; `pos_solve` is a stub for now so the test fails first.

```c
#include "pos_solver.h"

bool pos_solve(const struct pos_meas *m, size_t n, struct pos_result *out)
{
    (void)m;
    (void)n;
    out->valid = false;
    return false;   /* stub: implemented in the next step */
}

/* |a - b| <= tol */
static bool approx(float a, float b, float tol)
{
    float d = a - b;
    if (d < 0) {
        d = -d;
    }
    return d <= tol;
}

int pos_solver_selftest(void)
{
    int fails = 0;
    struct pos_result p;

    /* 1. Exact 3-anchor solution. Anchors (0,0),(4,0),(0,3); tag at (1,1). */
    {
        struct pos_meas m[3] = {
            { 0.0f, 0.0f, 1.41421356f },   /* sqrt(2)  */
            { 4.0f, 0.0f, 3.16227766f },   /* sqrt(10) */
            { 0.0f, 3.0f, 2.23606798f },   /* sqrt(5)  */
        };
        if (!pos_solve(m, 3, &p) || !p.valid ||
            !approx(p.x, 1.0f, 1e-3f) || !approx(p.y, 1.0f, 1e-3f)) {
            fails++;
        }
    }

    /* 2. Exact 4-anchor (overdetermined). Square corners; tag at (2,2). */
    {
        struct pos_meas m[4] = {
            { 0.0f, 0.0f, 2.82842712f },   /* sqrt(8)  */
            { 5.0f, 0.0f, 3.60555128f },   /* sqrt(13) */
            { 0.0f, 5.0f, 3.60555128f },   /* sqrt(13) */
            { 5.0f, 5.0f, 4.24264069f },   /* sqrt(18) */
        };
        if (!pos_solve(m, 4, &p) || !p.valid ||
            !approx(p.x, 2.0f, 1e-3f) || !approx(p.y, 2.0f, 1e-3f)) {
            fails++;
        }
    }

    /* 3. Overdetermined with noise: LLS should land near the truth (2,2). */
    {
        struct pos_meas m[4] = {
            { 0.0f, 0.0f, 2.81f },
            { 5.0f, 0.0f, 3.63f },
            { 0.0f, 5.0f, 3.59f },
            { 5.0f, 5.0f, 4.26f },
        };
        if (!pos_solve(m, 4, &p) || !p.valid ||
            !approx(p.x, 2.0f, 0.2f) || !approx(p.y, 2.0f, 0.2f)) {
            fails++;
        }
    }

    /* 4. Collinear anchors -> degenerate -> invalid. */
    {
        struct pos_meas m[3] = {
            { 0.0f, 0.0f, 2.0f },
            { 1.0f, 0.0f, 1.5f },
            { 2.0f, 0.0f, 2.0f },
        };
        if (pos_solve(m, 3, &p) || p.valid) {
            fails++;
        }
    }

    /* 5. Fewer than 3 anchors -> invalid. */
    {
        struct pos_meas m[2] = {
            { 0.0f, 0.0f, 1.0f },
            { 2.0f, 0.0f, 1.0f },
        };
        if (pos_solve(m, 2, &p) || p.valid) {
            fails++;
        }
    }

    return fails;
}
```

- [ ] **Step 4: Run the test and verify it FAILS**

```bash
GCC="/c/Users/JoseAntonioLaraPerez/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe/mingw64/bin/gcc.exe"
"$GCC" -std=c99 -Wall -Wextra -Werror -I src tests/pos_solver/test_pos_solver.c src/pos_solver.c -o /tmp/test_pos_solver.exe && /tmp/test_pos_solver.exe
```
Expected: builds, then `pos_solver_selftest: 3 failure(s)` and exit code 1. Vectors 1–3 (which expect a valid solution) fail against the always-invalid stub; vectors 4 and 5 (which expect `valid == false`) pass against the stub.

- [ ] **Step 5: Implement `pos_solve`**

Replace the stub body in `src/pos_solver.c` with the real implementation:

```c
bool pos_solve(const struct pos_meas *m, size_t n, struct pos_result *out)
{
    out->valid = false;
    if (n < 3) {
        return false;
    }

    /* Reference anchor = index 0. For each other anchor i, subtracting the
     * reference circle equation gives a linear row:
     *   A_i = [ 2(xk - xi), 2(yk - yi) ]
     *   b_i = (ri^2 - rk^2) + (xk^2 + yk^2 - xi^2 - yi^2)
     * Accumulate the 2x2 normal equations (AtA) and 2-vector (Atb). */
    const double xk = m[0].x, yk = m[0].y, rk = m[0].range_m;
    const double ck = xk * xk + yk * yk;

    double AtA00 = 0, AtA01 = 0, AtA11 = 0;
    double Atb0 = 0, Atb1 = 0;

    for (size_t i = 1; i < n; i++) {
        double a0 = 2.0 * (xk - m[i].x);
        double a1 = 2.0 * (yk - m[i].y);
        double ci = (double)m[i].x * m[i].x + (double)m[i].y * m[i].y;
        double bi = ((double)m[i].range_m * m[i].range_m - rk * rk) + (ck - ci);

        AtA00 += a0 * a0;
        AtA01 += a0 * a1;
        AtA11 += a1 * a1;
        Atb0  += a0 * bi;
        Atb1  += a1 * bi;
    }

    double det = AtA00 * AtA11 - AtA01 * AtA01;
    if (det > -1e-6 && det < 1e-6) {
        return false;   /* singular: collinear / degenerate geometry */
    }

    double inv = 1.0 / det;
    out->x = (float)(( AtA11 * Atb0 - AtA01 * Atb1) * inv);
    out->y = (float)((-AtA01 * Atb0 + AtA00 * Atb1) * inv);
    out->valid = true;
    return true;
}
```

- [ ] **Step 6: Run the test and verify it PASSES**

```bash
GCC="/c/Users/JoseAntonioLaraPerez/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe/mingw64/bin/gcc.exe"
"$GCC" -std=c99 -Wall -Wextra -Werror -I src tests/pos_solver/test_pos_solver.c src/pos_solver.c -o /tmp/test_pos_solver.exe && /tmp/test_pos_solver.exe
```
Expected: `pos_solver_selftest: 0 failure(s)` and exit code 0.

- [ ] **Step 7: Commit**

```bash
git add src/pos_solver.h src/pos_solver.c tests/pos_solver/test_pos_solver.c
git commit -m "feat(pos): 2D linear least-squares trilateration solver + host test"
```

---

## Task 2: Register `pos_solver.c` in the build

**Files:**
- Modify: `CMakeLists.txt:6-25` (the `target_sources(app PRIVATE ...)` list)

- [ ] **Step 1: Add the source file**

In `CMakeLists.txt`, add `src/pos_solver.c` to the `target_sources` list, directly after the `src/cal.c` line:

```cmake
    src/cal_math.c
    src/cal.c
    src/pos_solver.c
    src/uwb_ss_initiator.c
```

- [ ] **Step 2: Verify (build)**

The user builds/flashes the firmware. Ask them to run `west build` and confirm the image links with `src/pos_solver.c` compiled in and no errors.
Expected: clean build (no new warnings/errors).

- [ ] **Step 3: Commit**

```bash
git add CMakeLists.txt
git commit -m "build(pos): compile pos_solver.c into the image"
```

---

## Task 3: Anchor-addressed ranging primitive

**Files:**
- Modify: `src/uwb_ss_initiator.c` (frame definitions block ~lines 47-58; add new function after `do_one_range`, ~line 216)

This is radio firmware — not host-testable. Verification is a clean build plus the on-hardware check in Task 4.

- [ ] **Step 1: Add the `pos_solver` include**

In `src/uwb_ss_initiator.c`, add to the project includes (after `#include "cal_math.h"`):

```c
#include "pos_solver.h"
```

- [ ] **Step 2: Enlarge the RX buffer and add positioning frame definitions**

The positioning response is 27 bytes; the shared `rx_buf` must hold it. In the
frames block, change `RX_BUF_LEN` from 20 to 32:

```c
#define RX_BUF_LEN               32
```

Then, immediately after the existing `rx_resp_msg` definition (the calibration
frames — leave them unchanged), add the positioning frames and field indices:

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

Insert this function immediately after `do_one_range` (i.e. after its closing
brace, before `apply_total_dly`). It mirrors `do_one_range` but addresses one
anchor and parses the anchor's self-reported coordinates.

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

    pos_poll_msg[ALL_MSG_SN_IDX]   = frame_seq_nb;
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

Ask the user to `west build`. Expected: clean build. (`do_one_range_anchor` is
unused until Task 4 — `static` + unused will warn; if the build uses `-Werror`
this is expected to fail until Task 4 wires it in. In that case, combine the
build verification with Task 4 Step 4 rather than building here.)

- [ ] **Step 5: Commit**

```bash
git add src/uwb_ss_initiator.c
git commit -m "feat(uwb): addressed SS-TWR primitive parsing anchor self-coords"
```

---

## Task 4: Multi-anchor cycle, position publish, wire-in

**Files:**
- Modify: `src/uwb_ss_initiator.c` (add anchor list + `position_publish` near the timing defines; rework the ranging branch in `ss_twr_fn` ~lines 329-337)

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
(the nano printf in this build has no float support) by splitting into integer
centimetres, matching the existing `D:` formatter style.

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
is unchanged. `do_one_range` and `run_calibration` remain in the file, still
used by the calibration path.

- [ ] **Step 4: Verify (build + on-hardware)**

Ask the user to `west build`, flash, and observe over BLE NUS:
- Expected build: clean (no unused-function warning now — `do_one_range_anchor`
  is referenced).
- With ≥3 anchors flashed to the response contract and a valid calibration:
  `P:x.xx,y.yy` lines arrive at ~0.2 s when moving / ~1 s when still.
- With <3 anchors responding: no `P:` line is emitted.
- If responses time out unexpectedly, the longer (27-byte) response may exceed
  `RESP_RX_TIMEOUT_UUS`; bump it (e.g. to 3000) as an on-hardware tuning step.

- [ ] **Step 5: Commit**

```bash
git add src/uwb_ss_initiator.c
git commit -m "feat(uwb): multi-anchor ranging cycle -> LLS -> BLE position publish"
```

---

## Self-Review notes

- **Spec coverage:** solver (Task 1) · build wiring (Task 2) · addressed frames + primitive + anchor_id validation + self-coord parse (Task 3) · static anchor list, ≥3 gate, cadence preserved, `position_publish` seam, `D:`→`P:` (Task 4). Calibration left untouched (Tasks touch only `do_one_range_anchor`, never `do_one_range`/`run_calibration`).
- **Type consistency:** `pos_meas{x,y,range_m}`, `pos_result{x,y,valid}`, `pos_solve(const struct pos_meas*, size_t, struct pos_result*)`, `do_one_range_anchor(uint8_t, float*, float*, float*)` used identically across Tasks 1, 3, 4.
- **Known build-order caveat:** `do_one_range_anchor` is unused between Task 3 and Task 4; with `-Werror` on unused-static, Task 3 won't build standalone. Documented in Task 3 Step 4 — fold its build check into Task 4 if so.
```
