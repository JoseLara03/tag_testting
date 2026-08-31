# Position Filtering & Precision — Design

**Date:** 2026-08-22
**Branch:** feat/pos-frame
**Status:** Design for review — not implemented
**Related:** `spec/2026-08-16-low-power-duty-cycle-design.md` (the tier/skip machinery this design has to negotiate with)

## Problem

`pos_solve()` produces visible jumps of well over a metre for a tag that has
barely moved. The maintainer's proposal is an EKF fused with the on-board
LIS2HH12 accelerometer.

The proposal is half right. An EKF is the correct structure, but the
accelerometer cannot contribute inertial data on this hardware, and — more
importantly — **the current estimator has systematic errors large enough that
filtering it would only produce a smooth wrong answer.** The bias fixes come
first; the filter comes second.

## What the current estimator actually does

`src/pos_solver.c` is a single-shot linear least squares. It linearizes the
circle equations by subtracting anchor index 0, then solves the 2×2 normal
equations. `struct pos_meas` is `{x, y, range_m}` — there is no z anywhere in
the pipeline.

Four defects, ranked by expected contribution to the jumps:

### 1. Slant range fed into a planar model (largest)

The anchors are ceiling-mounted; the tag is worn at waist height. The E1 ranging
response carries only X (bytes 19–22) and Y (23–26), even though the anchor's
flash record already holds `anchor_z`. So a *slant* range is fed into a planar
equation.

With anchors at 2.7 m and the tag at 1.1 m, Δz = 1.6 m:

| true horizontal | measured slant | error |
|---|---|---|
| 0.0 m | 1.60 m | +1.60 m |
| 1.0 m | 1.89 m | +0.89 m |
| 3.0 m | 3.40 m | +0.40 m |
| 6.0 m | 6.21 m | +0.21 m |

This is not an offset that averages out. It is position-dependent and
nonlinear, and it makes the four circles **mutually inconsistent** — no point in
the plane satisfies all of them. An inconsistent least-squares problem has a
flat, ill-conditioned minimum, and the solution *slides* along that valley in
response to millimetres of range noise. That is precisely the reported symptom:
big jumps from small motion. It also inflates `residual_m`, which is why the
residual has never looked healthy.

### 2. The linearization has a privileged anchor

Subtracting anchor 0 puts anchor 0's range error into **every** row of `A`.
The unweighted normal equations do not model that correlation, so one bad range
on the reference anchor corrupts the entire fix rather than one row of it.

Worse: `selected[]` is re-ranked by EMA CIR score in `uwb_net_runner.c`, so
**which anchor occupies index 0 can change between sweeps**. When it does, the
estimator itself changes and the fix jumps with zero physical motion.

### 3. No outlier rejection

With n=4 and two unknowns there are two degrees of freedom — enough to *detect*
a bad range, but plain LS smears it across the solution instead. NLOS (a body
between tag and anchor is the common case) adds a positive-only 0.3–2 m bias.
A single NLOS hit on one anchor is indistinguishable, to the current solver,
from the tag having moved.

### 4. All ranges weighted equally

Signal quality varies by tens of dB across a room, and antenna delay was
calibrated at exactly one distance, i.e. one RX level. The residual RX-level
range bias is tens of centimetres and is another source of circle inconsistency.

**Already correct and not to be touched:** the SS-TWR clock-offset correction is
applied in both range paths (`uwb_ss_initiator.c:269-278`, `:331-340`). That is
the single largest classic SS-TWR error term and it is handled.

## Why the accelerometer cannot do dead reckoning here

Three independent blockers:

**No heading.** The LIS2HH12 is a 3-axis accelerometer. Gravity yields roll and
pitch; yaw is unobservable. Without full attitude there is no rotation from body
frame to room frame, so `a → v → p` is not merely noisy — it is not observable.
Accelerometer-only pedestrian navigation is step counting, and step counting
still needs a magnetometer for direction.

**Error budget.** 1° of attitude error leaks `g·sin(1°) = 0.17 m/s²`.
Double-integrated over a fix interval that is `0.5 × 0.17 × dt²`:

| dt | drift from 1° tilt error alone |
|---|---|
| 0.2 s | 3.4 mm |
| 1 s | 8.5 cm |
| 5 s | 2.1 m |

**It is not streaming.** `motion.c` runs the LIS2HH12 at 10 Hz with the
activity/inactivity engine; only the INT1 edge reaches the MCU. Inertial work
needs 50–100 Hz samples read over I²C, waking the MCU 50–100 times per second —
directly opposed to the 250 µA target, to buy data that blocker 1 says is
unusable anyway.

### What the accelerometer *should* contribute

All three are cheap, and the plumbing (`uwb_set_moving()`, INT1) already exists:

- **ZUPT.** When INT1 reports inactive, apply a zero-velocity pseudo-measurement
  with small R. This is the single largest visual improvement available, because
  a stationary tag is the common case and stationary jitter is the most visible
  artefact.
- **Process-noise scheduling.** σ_a ≈ 0.05 m/s² still, 1.2 m/s² walking.
- **Speed gate.** Reject any update implying > 2 m/s.

The accelerometer is a **mode discriminator**, not an inertial sensor. This is
strictly better than an IMM's model-probability estimate, because it is a direct
physical measurement of the mode rather than an inference from the residuals.

## Update rate: the 0.1 s target is not reachable, and does not need to be

The maintainer wants a 0.1 s fix interval at the fastest tier.

**The MAC ceiling is one fix per superframe ≈ 202 ms (~5 Hz).** The tag ranges
inside its own CFP slot, one slot per superframe. `T_SLOT_MS` is 24 ms against a
measured ~22 ms four-anchor sweep, so a second sweep does not fit in the slot
either. 10 Hz measurement would require a 100 ms superframe, which halves the
CFP budget (capacity ~7 tags → ~3) and doubles the beacon-RX duty cycle. **Not
recommended.**

**This does not matter, because the EKF decouples output rate from measurement
rate.** The filter state is `[x, y, vx, vy]`. Publishing velocity alongside
position lets the consumer extrapolate to any display rate for free — no radio,
no power, no MAC change. A 10 Hz visualization from 5 Hz measurements is not an
approximation; it is what the filter is for.

**Recommendation: publish `x, y, vx, vy` in the POS frame and up-sample on the
platform side.** Zero tag cost.

### Rate ladder

The maintainer's framing — "5 s and above, the system only needs to know where
it is in space; precision starts when the assets walk" — is exactly right, and
the filter mathematics agrees with it.

| Tier | `listen_skip` | fix interval | σ from prediction alone | EKF role |
|---|---|---|---|---|
| FAST (walking) | 1 | 0.20 s | 2.4 cm | **Full authority.** Prediction is 5× tighter than the measurement, so the filter smooths hard and gates outliers tightly. |
| SLOW | 5 | 1.0 s | 60 cm | Partial. Comparable to measurement noise; gating still works, smoothing is mild. |
| IDLE (still) | 75–300 | 15–60 s | — (ZUPT) | **Full authority again**, via a different mechanism — see below. |

(σ from prediction = `0.5·σ_a·dt²` at σ_a = 1.2 m/s². Measurement σ ≈ 12 cm.)

The non-obvious result is the third row. A **stationary** tag has near-zero
process noise, so the filter averages successive fixes instead of chasing them:
σ converges as `σ_meas/√N`. Ten fixes at 0.2 Hz — 50 s of standing still — give
roughly 4 cm. **Precision and power are only in conflict while the tag is
moving.** A still tag gets *better* with a lower fix rate, not worse.

The one genuinely unrecoverable cell is moving-at-5 s, which is the cell the
maintainer has already said does not need precision.

**Consequence for tuning:** FAST must move from `listen_skip = 25` to `1`. This
returns the moving tag to roughly the pre-Layer-3 current (~22 mA, scaled model
— no PPK2 measurement exists yet), paid only while walking. The FAST→SLOW
hysteresis (`UWB_TIER_HOLD_FAST_MS` = 30 s) bounds how long that is paid after
motion stops.

## Architecture

### Range model: 3D, everywhere, no pre-projection

Every range model in the system becomes

```
h_i(x, y) = sqrt( (x - x_i)^2 + (y - y_i)^2 + dz_i^2 )
```

with `dz_i = z_i - z_tag`, and the Jacobian

```
dh/dx = (x - x_i) / h_i          dh/dy = (y - y_i) / h_i
```

This is deliberately **not** a "project the range down to horizontal" step.
Pre-projection (`r_h = sqrt(r² - dz²)`) needs a clamp when `r < dz`, and its
error blows up as `dz/r_h` near an anchor, requiring an R-inflation hack to stay
consistent. Putting `dz` inside the measurement model instead makes the EKF
handle the near-anchor dilution correctly and automatically — the Jacobian
shrinks exactly where the horizontal information does. It is also less code.

**Getting `z_i` without touching anchor firmware.** All anchors are
ceiling-mounted at one height, so v1 uses a single tag-side pair of constants —
anchor height and assumed tag height — set over NUS (`pos z <anchor_cm>
<tag_cm>`) and persisted to NVS (**new storage id 4**). Per-anchor z via two
extra bytes in the E1 response is the eventual answer if anchors ever end up at
differing heights, but it needs anchor firmware that is not in this repo, and it
is not needed for a single-room deployment.

### Snapshot solver: Gauss-Newton replaces the reference-anchor linearization

`pos_solve()` becomes 2–3 Gauss-Newton iterations on the raw nonlinear residual,
seeded from the previous fix (or from the existing linear solution on the first
call):

```
repeat 2-3x:
    r_i  = h_i(p) - range_i          # residual
    J    = [dh_i/dx, dh_i/dy]        # n x 2
    dp   = -(J^T W J)^-1 J^T W r     # same 2x2 inverse already in use
    p   += dp
```

Same CMSIS-DSP 2×2 inverse, same cost class. What it buys:

- **No privileged anchor** — defect 2 disappears entirely, including the
  jump-on-reordering.
- **No correlated noise** — each row carries only its own anchor's error.
- **It minimizes the residual the code already reports**, so `residual_m`
  becomes a meaningful goodness-of-fit rather than a number describing a point
  the solver was not trying to find.
- `W` is the hook for per-range weighting (defect 4) and for IRLS.

Retain the linear solver as the cold-start seed and as the fallback when
Gauss-Newton fails to converge.

### Robust rejection

**As implemented this diverged from the original sketch, because the sketch was
wrong.** Both are recorded — the reasoning is reusable.

The plan was a subset search comparing the full-set RMS residual against the
best 3-anchor subset's, gated by a floor and a ratio. Measured over 200k clean
fixes and 50k single-NLOS fixes (8×6 m room, corner anchors, σ = 0.12 m), that
test **discarded a good anchor on 3.8 % of clean fixes** while catching only
79 % of 1 m NLOS hits, and picked the wrong anchor a fifth of the time.

Two independent reasons, both worth remembering:

1. **RMS is not comparable across anchor counts.** A clean 3-anchor fit has one
   degree of freedom against the 4-anchor fit's two, so its RMS is ~0.82× the
   4-anchor value *for identical noise*. A ratio between them measures the
   degree-of-freedom change as much as it measures the outlier.
2. **Least squares hides the outlier it is meant to reveal.** The fit absorbs
   part of a bad range by moving the fix. The absorbed fraction is the leverage
   `h_ii = J_i (JᵀJ)⁻¹ J_iᵀ`; with 4 anchors and 2 unknowns the leverages sum to
   2, so `h_ii` averages 0.5 and a 1 m bias surfaces as only ~0.5 m of residual.

What shipped is the standard **studentized-residual** test: divide each residual
by `sqrt(1 − h_ii)`, which undoes that shrinkage exactly and makes the residuals
~N(0, σ), then apply one absolute threshold in σ units. An anchor is dropped
only when removing it brings the rest back inside the threshold — dropping must
*explain* the inconsistency, not merely reduce it. The 2×2 normal matrix is
already in hand, so the leverage costs almost nothing.

At `K = 3.8` σ, same harness:

| | clean false-drop | +0.5 m NLOS | +1.0 m NLOS |
|---|---|---|---|
| planned RMS-ratio | 3.81 % | 67 % (47 % correct) | 99 % (79 % correct) |
| shipped, studentized | **0.057 %** | 21 % (16 %) | **95 % (76 %)** |

A 67× cut in false drops for 4 points of 1 m sensitivity. Conservative is the
right bias here: the EKF gates every range at 3σ of its own innovation
covariance, so a missed outlier gets a second chance downstream, while a wrongly
dropped anchor is information nothing can recover. `K` is meaningful rather than
tuned — the expected false-drop rate is `4 × P(|N(0,1)| > K)`.

**σ = 0.12 m is the load-bearing assumption.** This threshold and the EKF's `R`
both derive from it, and it is the design's estimate, not a measurement.

### Degeneracy: `det(JᵀJ)` does not detect collinear anchors

Worth stating plainly, because it is the obvious thing to reach for and the
first implementation did reach for it.

With the tag **off** the anchor line the direction cosines still fan out, so the
normal matrix stays well conditioned while the problem has two mirror solutions
of identical residual. Measured on the room model: collinear anchors reach
`det(JᵀJ) = 2.23` against a legitimate-geometry **minimum of 1.89** — not
separable by that metric in either direction.

Collinearity is a property of the anchors, so it is tested on the anchors: the
largest twice-triangle-area any three span, rejected below 0.5 m².

This mattered more than it first looked. The original code passed its
collinearity test only by accident — the cold-start seed happens to land on the
anchor line, where the Jacobian genuinely does collapse. **The runner now seeds
every solve from the filter**, so the seeded path is the normal path, and under
a seed Gauss-Newton converged contentedly to whichever mirror the seed was
nearer and returned `valid = true` for a reflection of the truth.

### Convergence is judged on the gradient, not the line search

A stalled line search is not a stationary point: the Gauss-Newton direction can
be poor while the gradient is still large. Treating "no step improved the fit"
as "converged" let `pos_solve()` return the caller's own seed with
`valid = true` — up to 113 m of residual on adversarial input, which for a
seeded solver means handing the previous fix back as a new one.

The final gate is `||Jᵀr|| < 5 mm`. A large *residual* is fine and is the
caller's signal that the ranges disagree; a large *gradient* means the ranges
were never fitted at all.

Two traps found while testing it, both worth not re-learning:

- The obvious hand-built pathology — contradictory ranges seeded at the room
  centre — is not one. The centre of a symmetric anchor set genuinely **is** the
  least-squares solution, large residual and all. The invariant is stationarity,
  so it must be asserted as stationarity, over random adversarial input.
- After a rejection the reported point is the optimum of the **subset**, so the
  full-set gradient there is non-zero by construction. Any such check has to
  skip the dropped anchor.

The backtracking line search was added beyond the original sketch and is
measured as worth keeping, though only just visibly: on realistic room geometry
it changes nothing (100 % accurate solves with or without it, even from seeds
60 m outside the room), but on adversarial geometry it takes convergence from
**60.4 % to 86.9 %**.

Once the EKF exists, **innovation gating complements this** for the tracking
case: any range whose innovation exceeds 3σ of `HPHᵀ + R` is rejected. The
snapshot robustness still matters for cold start and for re-acquisition.

### The filter: tightly-coupled EKF over raw ranges

**State:** `[x, y, vx, vy]`, constant velocity with white-noise acceleration.

**Measurements: the individual ranges, not the solved position.** This is the
central design decision. Filtering the LS *output* discards the geometry: the
output error is non-Gaussian, strongly correlated with GDOP, and changes
character every time the anchor set or count changes (n=3 vs n=4 happens
routinely in `anchor_sweep()`). A filter downstream of LS cannot know any of
that. A filter over raw ranges gets it all for free, and gets the z model,
per-range weighting and per-range gating for free with it.

**Sequential scalar updates.** Each range is applied one at a time: a 4×1 gain
and one scalar division, no matrix inverse at any point. This is *less* code
than the current CMSIS-DSP path, and per-range gating falls out naturally.

```
predict:   F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
           Q from the CV white-noise-acceleration model, sigma_a by tier
per range: h = sqrt(dx^2 + dy^2 + dz_i^2)
           H = [dx/h, dy/h, 0, 0]
           S = H P H^T + R_i           (scalar)
           if (z - h)^2 > 9 * S: skip this range        # 3-sigma gate
           K = P H^T / S               (4x1)
           x += K (z - h);  P = (I - K H) P
ZUPT:      if !moving, apply vx = 0 and vy = 0 as two scalar updates, small R
```

**Parameters:**

| Parameter | Initial value | Source |
|---|---|---|
| `σ_a` still | 0.05 m/s² | LIS2HH12 INT1 |
| `σ_a` walking | 1.2 m/s² | LIS2HH12 INT1 |
| `R_i` | (0.12 m)² | to be measured — see instrumentation |
| ZUPT R | (0.02 m/s)² | tuned |
| gate | 3σ | standard |
| `dt` | actual elapsed ms | varies with tier and skips; must not be assumed |

**Divergence protection.** Count consecutive fixes in which every range was
gated out. After N (start at 3), reset the filter to the snapshot solution and
reinflate P — the kidnapped-tag case, and the case where the tag was carried
while the accelerometer said still.

**Module split follows the existing repo convention.** `src/pos_ekf.c/h` is pure
C with no Zephyr dependency, host-tested in `tests/pos_ekf/` — the same split as
`cal_math.c`, `pos_residual.c` and `beacon_sched_core.c`. It must not need
CMSIS-DSP: with sequential scalar updates there is no matrix inverse, so this is
achievable and it is the reason to prefer sequential updates over a batch update.

### Rejected alternatives

**UKF.** The range nonlinearity is mild once linearized about a Gauss-Newton
point, and the prediction step at 0.2 s is nearly linear. Nine sigma points for
no measurable gain.

**Particle filter.** Only earns its cost with a floor plan or wall constraints,
and 128 KB of RAM plus a 250 µA budget rules it out. If a room polygon ever
exists, a simple "reject fixes outside the polygon" constraint captures most of
the benefit for almost nothing.

**IMM (constant-velocity + stationary).** The textbook answer for this exact
symptom, but the tag has a *hardware* motion detector. Motion-gated Q plus ZUPT
gets the same behaviour deterministically, without the mode-probability
bookkeeping or its lag at mode transitions.

**Constant-acceleration model.** Worse than CV for pedestrian motion; two extra
states that noise excites.

**EMA / α-β on the solved x,y.** What gets tried first everywhere. It lags real
motion in proportion to speed, rejects nothing, and discards geometry. Smoother
*and* less accurate. Explicitly rejected.

**Sliding-window batch least squares (MHE).** Strictly better than snapshot LS,
but the EKF is its recursive form at a fraction of the cost.

**Low-pass on individual ranges.** Introduces lag proportional to speed, and the
EKF does the same job with the geometry included.

## Implementation order

Order matters: filtering a biased estimator produces a smooth wrong answer.

1. ~~**3D range model + z constants**~~ — done (`pos_cfg.c`, NVS id 4, `pos z`).
2. ~~**Gauss-Newton snapshot solver**~~ — done, plus the gradient gate and the
   anchor-spread degeneracy test above.
3. ~~**Outlier rejection**~~ — done, as the studentized test above rather than
   the RMS-ratio sketch.
4. ~~**Temporary raw-range log**~~ — built (`pos_dbg.c`), but **the capture
   campaign it existed for was never run**, and it was removed 2026-08-30
   (Task 8 of `docs/superpowers/plans/2026-08-25-fase3-tdoa.md`) per its own
   removal checklist below, without ever collecting a trace. Not superseded by
   running it — superseded by the project moving position-solving off the tag
   entirely (Fase 3, TDoA): the tag now emits a BLINK and the gateway solves,
   so there is no longer a tag-side EKF whose tuning this log was for. Steps 5
   and 6 below inherit the same status, for the same reason, not because they
   ran.
5. ~~**`pos_ekf.c` + host tests**~~ — written, host-tested, and **left wired
   into the runner's TWR fallback path** (`blink off`) — it was never tuned
   against captured data (step 4 never ran) and is not expected to be, now
   that TDoA is the primary path. `pos_solver`/`pos_residual` were ported
   verbatim to the gateway for TDoA's own solve (Fase 3 Task 2); `pos_ekf.c`
   was not, since TDoA's measurement model isn't range-based (see the "EKF
   queda fuera" note in `docs/superpowers/plans/2026-08-25-fase3-tdoa.md`).
6. **Per-range R from DW3000 diagnostics + RX-level bias correction** — not
   started, and now unlikely to be: it tunes the same TWR/EKF path step 4/5
   describe, which TDoA has replaced as the product direction.
7. **Re-tune FAST `listen_skip`** and publish `vx, vy` — not started;
   `listen_skip` deliberately left alone at the maintainer's direction.

**None of this has run on hardware.** Every figure above is a host measurement
or a model.

**Publishing criteria are deliberately unchanged.** A fix still requires ≥3
anchors and a converged snapshot solve; the filter changes the *value*, not when
one is emitted. Publishing a pure prediction is a separate decision that should
be made against captured data rather than assumed — and holding the criteria
fixed means this change set cannot emit *more* bad fixes than the code it
replaces, only better-valued ones. `residual_m` likewise stays the snapshot's:
it measures range consistency, not filter confidence.

Steps 1–3 are worth doing on their own merits and should measurably reduce the
jumps before any filter exists.

---

## Temporary instrumentation: raw-range log over BLE

> **This log is a tuning instrument with a defined lifetime. It exists to
> capture the data needed to set `R`, `σ_a` and the gate threshold, and it must
> be removed once the EKF is tuned.** Removal checklist at the end of this
> section. See also CLAUDE.md Open Work.

### Why BLE and not RTT

The tag has no serial port (`CONFIG_SERIAL=n`). RTT via a debugger would work,
but a tethered tag cannot be walked, and walking is the entire experiment. BLE
NUS is therefore the only viable channel, which imposes the **20-byte
notification limit** (default ATT MTU 23; `bt_nus_send` does not fragment and
payloads over 20 bytes fail `-EMSGSIZE` silently). The MTU is deliberately left
alone — renegotiating it is a config change with its own risk, and the records
below fit in 20 bytes.

Records are therefore **binary**, and start with a non-ASCII magic byte so a host
can demultiplex them from the text lines (`P:`, `BATT:`, cal verdicts) sharing
the characteristic.

### SWEEP record — 20 bytes, one per sweep, magic `0xA5`

| off | size | field | notes |
|---|---|---|---|
| 0 | 1 | `0xA5` | magic |
| 1 | 1 | `seq` | uint8, ++ per sweep; gaps identify dropped notifications |
| 2 | 2 | `dt_ms` | uint16 LE, ms since previous SWEEP (0xFFFF = ≥65.535 s) |
| 4 | 1 | `epoch` | uint8, matches the SET record that defines the slots |
| 5 | 1 | `flags` | b0–3 respond mask, b4–5 tier, b6 moving, b7 solved |
| 6 | 2 | `r0` | int16 LE, cm — signed: close-in ranges can go negative |
| 8 | 2 | `r1` | |
| 10 | 2 | `r2` | |
| 12 | 2 | `r3` | |
| 14 | 4 | `q0..q3` | uint8 each, quality; zero unless `dbg q on` |
| 18 | 2 | `res` | uint16 LE, `residual_m` in cm; 0xFFFF when not solved |

`dt_ms` is generated on the tag and is authoritative — host arrival timestamps
carry BLE connection-interval jitter and must not be used for the filter's `dt`.

### SET record — 13 bytes, magic `0xA6`

Emitted at log start and whenever `selected[]` changes; two records cover four
anchors. Anchor coordinates arrive over the air in E1 and can be misconfigured,
so the host needs them recorded rather than assumed.

| off | size | field |
|---|---|---|
| 0 | 1 | `0xA6` |
| 1 | 1 | `epoch` — ++ on every `selected[]` change |
| 2 | 1 | `base` — 0 or 2, which slot pair this record carries |
| 3 | 1 | `aid[base+0]` — 0xFF for an empty slot |
| 4 | 2 | `x[base+0]` int16 LE, cm |
| 6 | 2 | `y[base+0]` int16 LE, cm |
| 8 | 1 | `aid[base+1]` |
| 9 | 2 | `x[base+1]` |
| 11 | 2 | `y[base+1]` |

Splitting anchor identity and coordinates into a low-rate record is what keeps
the per-sweep record inside 20 bytes.

### Quality byte — off by default

`q0..q3` encode `round(2 × (RX_level_dB − first_path_level_dB))`, clamped to
0..255 — the standard Qorvo NLOS indicator in 0.5 dB steps, where a value above
~12 (6 dB) suggests NLOS. It requires `dwt_readdiagnostics()` after each RX.

**Hazard:** four extra diagnostic reads inside the CFP slot add SPI time to a
~22 ms sweep in a 24 ms slot. Overrunning the slot causes exactly the inter-tag
collisions `T_SLOT_MS = 24` was sized to prevent. Quality is therefore **off by
default** (`dbg q on`) and **single-tag bench use only**. It is not needed for
EKF tuning (step 5); it is needed for per-range weighting (step 6).

### Plumbing

- `ble_log_send_raw(const uint8_t *buf, uint16_t len)` added to `ble_log.c`;
  `ble_log_send()` becomes a one-line wrapper over it. Required because the
  existing path uses `strlen()`, which truncates binary at the first NUL.
- `struct twr_msg` gains a `uint8_t len`; `twr_log()` sets it from `strlen`, a
  new `twr_log_raw()` sets it from its argument. The BLE TX thread passes
  `m.len` through.
- `ss_twr_msgq` depth 8 → 16. Load is ~5 records/s plus the `P:` line; depth 8
  would work but has no margin, and `k_msgq_put` drops silently.
- Suppress the `P:` text line while logging — the host recomputes position from
  the ranges, and it halves the notification rate.
- Everything else lives in one new file, `src/pos_dbg.c/h`, so removal is one
  deletion plus the four edits above.
- NUS commands in `tag_cmd.c`: `dbg on|off`, `dbg q on|off`, `dbg mark`
  (inserts a `0xA7` marker record when the operator passes a surveyed point).

**Rate:** 20 B × 5 Hz = 100 B/s. A 10-minute walk is ~3000 records, ~60 kB.
Trivial for BLE and for a phone or laptop capture.

### Ground truth without a motion-capture rig

- **Static soak.** Tag on a tripod at a tape-measured point, several hundred
  fixes. Gives per-anchor σ (→ `R`), per-anchor bias vs. true range (→ the
  RX-level correction), and the stationary jitter the ZUPT has to remove.
- **Straight taped line.** Walk a taped line at constant pace. Lateral scatter
  about the line is a direct precision measure and **needs no time
  synchronization with ground truth**, which is what makes it the best cheap
  test for the walking case.
- **Closed loop.** Walk a loop back to the start. End-to-start error plus the
  visual shape of the path is strong qualitative evidence and catches scale and
  heading-like distortions the straight line misses.
- **`dbg mark`** inserts a marker record; the operator taps it from nRF Connect
  when passing a surveyed point.

Filter development happens **offline in Python** against the captured data.
Tuning a Kalman filter by reflashing firmware is how weeks disappear. The tuned
constants are then ported into `pos_ekf.c`, whose host test replays a captured
trace and asserts the expected output — which also makes the campaign data a
permanent regression fixture.

### Removal checklist

- [x] Delete `src/pos_dbg.c/h` and its `target_sources` line in `CMakeLists.txt`
      (2026-08-30)
- [x] Remove `dbg` commands from `tag_cmd.c` (2026-08-30)
- [x] Revert `struct twr_msg` `len` field and `twr_log_raw()` in
      `uwb_ss_initiator.c` (2026-08-30)
- [x] Revert `ss_twr_msgq` depth 16 → 8 (2026-08-30)
- [x] Restore the `P:` line if it was suppressed — checked: it never was (no
      call site gated it on `pos_dbg_enabled()`), so there was nothing to
      restore (2026-08-30)
- [x] Keep `ble_log_send_raw()` on its own merits — kept, per this checklist's
      own note (2026-08-30)
- [ ] Remove the Open Work entry and the Key Patterns note from CLAUDE.md —
      **not done**. Deferred: this session's operator instructed that no
      `CLAUDE.md` be edited from this machine ("not the main developer unit").

This must be done **before** Open Work item 7 (dropping NUS output for battery
life), since that work removes the channel the log depends on.

## Open questions

1. **Anchor and tag height.** The z constants are the highest-value input in
   this design and nobody has measured them yet. Anchor mounting height and a
   representative worn height, to ±5 cm.
2. **Are all four anchors at the same height?** If not, v1's single-constant
   approach is wrong and per-anchor z in the E1 frame becomes step 1 — which
   pulls anchor firmware, not in this repo, onto the critical path.
3. **Current at `listen_skip = 1`.** Modelled at ~22 mA from the Layer-1/2
   measurement; unverified, and it sets the battery cost of the FAST tier.
4. **Measured range σ per anchor.** Assumed 12 cm. The static soak answers it,
   and `R` depends on it directly.
5. **Anchor geometry and GDOP.** Four anchors in one room: are they in the
   corners of a rectangle, and how bad is GDOP near the walls? This bounds
   achievable precision independently of any filter, and may argue for moving an
   anchor rather than improving the algorithm.
