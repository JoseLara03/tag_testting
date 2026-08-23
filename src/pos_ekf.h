#ifndef POS_EKF_H
#define POS_EKF_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "pos_solver.h"   /* struct pos_meas */

/*
 * Tightly-coupled EKF over raw UWB ranges.
 *
 * State is [x, y, vx, vy], constant velocity with white-noise acceleration.
 * Measurements are the individual ranges, NOT the solved position: filtering
 * the least-squares output discards the geometry, whose error is non-Gaussian,
 * correlated with GDOP, and changes character every time the anchor count or
 * set changes (n=3 vs n=4 happens routinely in anchor_sweep()). Fusing ranges
 * directly also gets the dz model and per-range gating for free.
 *
 * Updates are sequential and scalar -- one range at a time, a 4x1 gain and one
 * scalar divide. There is no matrix inverse anywhere, which is why this module
 * needs neither CMSIS-DSP nor Zephyr and is fully host-testable.
 *
 * The accelerometer contributes ZUPT and process-noise scheduling only. The
 * LIS2HH12 is a 3-axis accelerometer, so yaw is unobservable and body->room
 * rotation is impossible; it is a mode discriminator, not an inertial sensor.
 *
 * See spec/2026-08-22-position-filtering-design.md.
 */

struct pos_ekf_cfg {
    float sigma_a_still;  /* process noise accel std dev, still   (m/s^2) */
    float sigma_a_move;   /* process noise accel std dev, walking (m/s^2) */
    float r_range;        /* range measurement std dev            (m)     */
    float r_zupt;         /* zero-velocity pseudo-measurement std dev (m/s) */
    float gate_k;         /* innovation gate, in sigmas (e.g. 3.0)        */
    float v_max;          /* speed clamp                          (m/s)   */
    uint8_t reset_after;  /* consecutive all-gated fixes before a reset   */
};

struct pos_ekf {
    float   x[4];        /* x, y, vx, vy */
    float   P[16];       /* row-major 4x4 covariance */
    bool    init;        /* false until pos_ekf_seed() */
    uint8_t gate_streak; /* consecutive fixes in which every range was gated */
};

/* Compiled-in starting point; tuned against captured data (see the `dbg` log
 * in the design). Callers may override any field. */
void pos_ekf_cfg_defaults(struct pos_ekf_cfg *c);

/* Drop all state. The filter is uninitialised until the next pos_ekf_seed(). */
void pos_ekf_reset(struct pos_ekf *f);

/* Initialise position from a snapshot fix, zero velocity, inflated covariance.
 * Also used for the divergence recovery path. */
void pos_ekf_seed(struct pos_ekf *f, float x, float y);

/* Constant-velocity prediction over dt_s seconds. `moving` selects
 * sigma_a_move vs sigma_a_still. dt_s must be the ACTUAL elapsed time -- it
 * varies with tier and with skipped superframes and must never be assumed. */
void pos_ekf_predict(struct pos_ekf *f, const struct pos_ekf_cfg *c,
                     float dt_s, bool moving);

/*
 * Fuse `n` ranges as sequential scalar updates, each gated at gate_k sigma of
 * (H P H^T + R). Returns the number of ranges accepted.
 *
 * When every range is gated the internal streak counter advances; once it
 * reaches cfg->reset_after the caller should re-seed from a snapshot solve
 * (query with pos_ekf_needs_reseed()). This is the kidnapped-tag case, and the
 * case where the tag was carried while the accelerometer reported still.
 */
int pos_ekf_update_ranges(struct pos_ekf *f, const struct pos_ekf_cfg *c,
                          const struct pos_meas *m, size_t n);

/* Zero-velocity update: two scalar pseudo-measurements vx = 0, vy = 0. Apply
 * when the LIS2HH12 reports inactive. This is the single largest visual
 * improvement available, because a stationary tag is the common case. */
void pos_ekf_zupt(struct pos_ekf *f, const struct pos_ekf_cfg *c);

/* True once the all-gated streak has reached cfg->reset_after. */
bool pos_ekf_needs_reseed(const struct pos_ekf *f, const struct pos_ekf_cfg *c);

/* Current estimate. Returns false if the filter has never been seeded; the
 * out-pointers are then untouched. Any of them may be NULL. */
bool pos_ekf_get(const struct pos_ekf *f, float *x, float *y,
                 float *vx, float *vy);

/* Position standard deviation, sqrt(P[0,0] + P[1,1]) -- a scalar spread figure
 * for diagnostics and for deciding whether a fix is worth publishing.
 *
 * Returns 0.0f for an unseeded filter, because P is zeroed until the first
 * seed. That reads as *maximal* confidence, so a bare `if (sigma < thresh)
 * publish()` would publish garbage from a filter that has never had a fix.
 * Always gate on pos_ekf_get()'s return value first. */
float pos_ekf_pos_sigma(const struct pos_ekf *f);

#endif /* POS_EKF_H */
