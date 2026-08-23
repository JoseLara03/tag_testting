#include "pos_ekf.h"
#include "pos_solver.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

/* Anchors at the corners of an 8x8 m room, ceiling-mounted 1.6 m above the
 * tag -- the same dz the design doc's worked example uses. Non-collinear,
 * so trilateration is well-posed for every test below. */
static const float ANCHOR_XY[4][2] = {
    { 0.0f, 0.0f }, { 8.0f, 0.0f }, { 0.0f, 8.0f }, { 8.0f, 8.0f },
};
#define ANCHOR_DZ  1.6f

/* Mirrors POS_EKF_DT_MAX, which is file-local to pos_ekf.c. If that
 * constant moves, test_dt_guards() fails loudly rather than silently
 * stopping testing the clamp. */
#define POS_EKF_DT_MAX_EXPECTED  120.0f

/* Build exact (noise-free) ranges from the 3D slant model for a true (x, y).
 * Exact ranges let each test assert a tight numeric bound instead of a fuzzy
 * one, while still exercising the real dz-carrying measurement model. */
static void make_ranges(float x, float y, struct pos_meas *out, size_t n)
{
    for (size_t i = 0; i < n && i < 4; i++) {
        float dx = x - ANCHOR_XY[i][0];
        float dy = y - ANCHOR_XY[i][1];

        out[i].x       = ANCHOR_XY[i][0];
        out[i].y       = ANCHOR_XY[i][1];
        out[i].dz      = ANCHOR_DZ;
        out[i].range_m = sqrtf(dx * dx + dy * dy + ANCHOR_DZ * ANCHOR_DZ);
    }
}

static int all_finite_state(const struct pos_ekf *f)
{
    for (int i = 0; i < 4; i++) {
        if (!isfinite(f->x[i])) return 0;
    }
    for (int i = 0; i < 16; i++) {
        if (!isfinite(f->P[i])) return 0;
    }
    return 1;
}

/* -------------------------------------------------------------------- */

static void test_converges_static_from_poor_seed(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);

    /* True position (4.0, 3.0); poor seed ~3.6 m away. */
    pos_ekf_seed(&f, 1.0f, 1.0f);

    for (int i = 0; i < 60; i++) {
        pos_ekf_predict(&f, &c, 0.2f, false);
        make_ranges(4.0f, 3.0f, m, 4);
        pos_ekf_update_ranges(&f, &c, m, 4);
    }

    float x, y;
    CHECK(pos_ekf_get(&f, &x, &y, NULL, NULL));
    CHECK(fabsf(x - 4.0f) < 0.05f);
    CHECK(fabsf(y - 3.0f) < 0.05f);
    CHECK(all_finite_state(&f));
}

static void test_tracks_constant_velocity(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];
    const float dt = 0.2f;
    const float vx_true = 0.5f;
    const float vy_true = 0.3f;
    float tx = 1.0f, ty = 1.0f;

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, tx, ty);

    /* 100 steps @ 0.2 s = 20 s of straight-line walking. */
    for (int i = 0; i < 100; i++) {
        tx += vx_true * dt;
        ty += vy_true * dt;

        pos_ekf_predict(&f, &c, dt, true);
        make_ranges(tx, ty, m, 4);
        pos_ekf_update_ranges(&f, &c, m, 4);
    }

    float x, y, vx, vy;
    CHECK(pos_ekf_get(&f, &x, &y, &vx, &vy));
    CHECK(fabsf(x - tx) < 0.15f);
    CHECK(fabsf(y - ty) < 0.15f);
    CHECK(fabsf(vx - vx_true) < 0.1f);
    CHECK(fabsf(vy - vy_true) < 0.1f);
    CHECK(all_finite_state(&f));
}

static void test_outlier_gated(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 3.9f, 3.1f); /* close seed */

    /* Converge P tight with 30 good, exact fixes at the true point (4, 3). */
    for (int i = 0; i < 30; i++) {
        pos_ekf_predict(&f, &c, 0.2f, false);
        make_ranges(4.0f, 3.0f, m, 4);
        pos_ekf_update_ranges(&f, &c, m, 4);
    }

    float x_before, y_before;
    pos_ekf_get(&f, &x_before, &y_before, NULL, NULL);

    /* One fix with anchor 0's range inflated by exactly 1 m. */
    pos_ekf_predict(&f, &c, 0.2f, false);
    make_ranges(4.0f, 3.0f, m, 4);
    m[0].range_m += 1.0f;
    int accepted = pos_ekf_update_ranges(&f, &c, m, 4);

    CHECK(accepted == 3); /* anchor 0 gated, three others accepted */

    float x_after, y_after;
    pos_ekf_get(&f, &x_after, &y_after, NULL, NULL);

    /* The estimate barely moves -- well under the size of the outlier. */
    CHECK(fabsf(x_after - x_before) < 0.05f);
    CHECK(fabsf(y_after - y_before) < 0.05f);
    CHECK(fabsf(x_after - 4.0f) < 0.05f);
    CHECK(fabsf(y_after - 3.0f) < 0.05f);
}

/* Shared walk-then-stop scenario for isolating ZUPT's own contribution.
 * `apply_zupt` selects whether pos_ekf_zupt() is called during the stopped
 * phase; everything else (seed, walk, dt, stopped-phase ranges) is
 * identical between the two runs so any velocity difference at the end is
 * attributable to ZUPT alone, not to the range updates converging on their
 * own. `speed_before_out` (walking-phase exit speed) may be NULL. */
static void run_walk_then_stop(bool apply_zupt, float *speed_before_out,
                                float *speed_after_out)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];
    float tx = 1.0f, ty = 1.0f;

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, tx, ty);

    /* Walk for a bit so velocity is genuinely nonzero. */
    for (int i = 0; i < 40; i++) {
        tx += 0.4f * 0.2f;
        ty += 0.2f * 0.2f;
        pos_ekf_predict(&f, &c, 0.2f, true);
        make_ranges(tx, ty, m, 4);
        pos_ekf_update_ranges(&f, &c, m, 4);
    }

    if (speed_before_out) {
        float vx, vy;
        pos_ekf_get(&f, NULL, NULL, &vx, &vy);
        *speed_before_out = sqrtf(vx * vx + vy * vy);
    }

    /* Now the tag stops: static ranges every fix, still-mode process noise,
     * and (only in the apply_zupt run) the ZUPT pseudo-measurement. */
    for (int i = 0; i < 40; i++) {
        pos_ekf_predict(&f, &c, 0.2f, false);
        make_ranges(tx, ty, m, 4);
        pos_ekf_update_ranges(&f, &c, m, 4);
        if (apply_zupt) {
            pos_ekf_zupt(&f, &c);
        }
    }

    CHECK(all_finite_state(&f));

    float vx, vy;
    pos_ekf_get(&f, NULL, NULL, &vx, &vy);
    *speed_after_out = sqrtf(vx * vx + vy * vy);
}

static void test_zupt_zeroes_velocity(void)
{
    float speed_before, speed_with_zupt, speed_without_zupt;

    run_walk_then_stop(true, &speed_before, &speed_with_zupt);
    CHECK(speed_before > 0.1f); /* sanity: motion actually built up velocity */
    CHECK(speed_with_zupt < 0.02f);

    run_walk_then_stop(false, NULL, &speed_without_zupt);

    /* Isolate ZUPT's own contribution: with byte-for-byte identical inputs
     * otherwise, applying the ZUPT pseudo-measurement must drive residual
     * velocity down well past what the range updates alone achieve. If
     * pos_ekf_zupt()'s body were deleted, the two runs would be numerically
     * identical and this comparison would fail. */
    CHECK(speed_with_zupt < 0.5f * speed_without_zupt);
}

static void test_speed_clamp_engages(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];

    pos_ekf_cfg_defaults(&c);
    c.v_max = 1.0f;
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 4.0f, 3.0f);

    /* Force an outrageous velocity directly into the state -- far past
     * v_max -- then run one range update matching the current position
     * exactly (so the correction itself barely touches x/y and, since P is
     * still diagonal from the seed, leaves vx/vy completely alone) purely
     * to exercise ekf_clamp_speed() at the end of the batch. */
    f.x[2] = 10.0f;
    f.x[3] = 0.0f;

    make_ranges(4.0f, 3.0f, m, 4);
    pos_ekf_update_ranges(&f, &c, m, 4);

    float vx, vy;
    pos_ekf_get(&f, NULL, NULL, &vx, &vy);
    float speed = sqrtf(vx * vx + vy * vy);

    /* Clamped to exactly v_max (10.0 * (v_max/10.0)), not left at 10 and
     * not zeroed -- direction (pure +x) is preserved too. */
    CHECK(fabsf(speed - c.v_max) < 1e-3f);
    CHECK(fabsf(vx - c.v_max) < 1e-3f);
    CHECK(fabsf(vy) < 1e-4f);
}

static void test_stationary_variance_falls(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 4.3f, 2.6f); /* seeded near, not at, the true point */

    float sigma_first = pos_ekf_pos_sigma(&f);

    float sigma_at_5 = 0.0f, sigma_at_50 = 0.0f;
    for (int i = 1; i <= 50; i++) {
        pos_ekf_predict(&f, &c, 0.2f, false);
        make_ranges(4.0f, 3.0f, m, 4);
        pos_ekf_update_ranges(&f, &c, m, 4);
        pos_ekf_zupt(&f, &c);

        if (i == 5)  { sigma_at_5  = pos_ekf_pos_sigma(&f); }
        if (i == 50) { sigma_at_50 = pos_ekf_pos_sigma(&f); }
    }

    /* The averaging effect: sigma shrinks from the seed, and keeps shrinking
     * (or holds at its still-mode floor) as more stationary fixes arrive. */
    CHECK(sigma_at_5 < sigma_first);
    CHECK(sigma_at_50 < sigma_at_5);
    CHECK(sigma_at_50 < 0.05f);
}

static void test_pos_sigma_uses_both_diag_terms(void)
{
    struct pos_ekf f;

    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 0.0f, 0.0f);

    /* Asymmetric position variance -- P[0][0] != P[1][1] -- so an
     * implementation that only reads P[0][0] (e.g. sqrtf(P[0][0]) alone) is
     * numerically distinguishable from the documented
     * sqrt(P[0,0] + P[1,1]). */
    f.P[0 * 4 + 0] = 4.0f;
    f.P[1 * 4 + 1] = 9.0f;

    float sigma = pos_ekf_pos_sigma(&f);
    CHECK(fabsf(sigma - sqrtf(13.0f)) < 1e-4f);
    CHECK(fabsf(sigma - 2.0f) > 1e-3f);  /* what sqrtf(P[0][0]) alone gives */
    CHECK(fabsf(sigma - 3.0f) > 1e-3f);  /* what sqrtf(P[1][1]) alone gives */
}

static void test_gate_streak_and_reseed(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 4.0f, 3.0f);

    /* A couple of good fixes first, to look like a running filter. */
    for (int i = 0; i < 5; i++) {
        pos_ekf_predict(&f, &c, 0.2f, false);
        make_ranges(4.0f, 3.0f, m, 4);
        pos_ekf_update_ranges(&f, &c, m, 4);
    }
    CHECK(!pos_ekf_needs_reseed(&f, &c));

    /* Now feed garbage: ranges consistent with a point 50 m away, on every
     * anchor, for reset_after fixes running. */
    for (uint8_t i = 0; i < c.reset_after; i++) {
        pos_ekf_predict(&f, &c, 0.2f, false);
        make_ranges(54.0f, 53.0f, m, 4);
        int accepted = pos_ekf_update_ranges(&f, &c, m, 4);
        CHECK(accepted == 0);
    }

    CHECK(pos_ekf_needs_reseed(&f, &c));

    /* One good fix clears the streak. */
    pos_ekf_predict(&f, &c, 0.2f, false);
    make_ranges(4.0f, 3.0f, m, 4);
    int accepted = pos_ekf_update_ranges(&f, &c, m, 4);
    CHECK(accepted > 0);
    CHECK(!pos_ekf_needs_reseed(&f, &c));
}

static void test_gate_streak_saturates(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 4.0f, 3.0f);

    /* Drive the counter to just below the wrap point directly rather than by
     * running 255 all-gated fixes. That longer route does not actually work,
     * and the reason is worth recording: with nothing ever accepted, Q keeps
     * inflating P every predict, so S = HPH^T + R grows without bound and the
     * 3-sigma gate eventually reopens and admits a range. That is CORRECT --
     * it is how the filter recovers from divergence when no caller reseeds it
     * -- so a test must not depend on the gate staying shut indefinitely.
     * What is under test here is only the counter's saturation. */
    f.gate_streak = UINT8_MAX - 5u;

    for (int i = 0; i < 20; i++) {
        pos_ekf_predict(&f, &c, 0.2f, false);
        make_ranges(54.0f, 53.0f, m, 4);   /* ~70 m away: gated */
        (void)pos_ekf_update_ranges(&f, &c, m, 4);
    }

    /* Saturated, not wrapped. Without the < UINT8_MAX guard the counter rolls
     * through 0 and back up through every small value, so needs_reseed()
     * would falsely clear for reset_after fixes after each wrap -- the tag
     * would stop asking for the reseed that is its only way out. */
    CHECK(f.gate_streak == UINT8_MAX);
    CHECK(pos_ekf_needs_reseed(&f, &c));

    /* And one accepted range still clears it. */
    make_ranges(4.0f, 3.0f, m, 4);
    CHECK(pos_ekf_update_ranges(&f, &c, m, 4) > 0);
    CHECK(f.gate_streak == 0u);
    CHECK(!pos_ekf_needs_reseed(&f, &c));
}

static void test_reset_after_zero_never_reseeds(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];

    pos_ekf_cfg_defaults(&c);
    c.reset_after = 0;
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 4.0f, 3.0f);

    /* A freshly seeded, perfectly healthy filter has gate_streak == 0. With
     * reset_after == 0 that must NOT be read as "streak has reached the
     * threshold" -- 0 means "never auto-reseed" (the safe reading for a
     * caller that zeroed/memset the config), not "reseed immediately". */
    CHECK(!pos_ekf_needs_reseed(&f, &c));

    /* Even a long all-gated run must never report needing a reseed. */
    for (int i = 0; i < 10; i++) {
        pos_ekf_predict(&f, &c, 0.2f, false);
        make_ranges(54.0f, 53.0f, m, 4);
        int accepted = pos_ekf_update_ranges(&f, &c, m, 4);
        CHECK(accepted == 0);
        CHECK(!pos_ekf_needs_reseed(&f, &c));
    }
}

static void test_q_position_and_cross_terms(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    const float dt = 0.5f;

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 0.0f, 0.0f);

    /* Start from a clean diagonal P so the whole of the resulting
     * position-velocity covariance comes from Q, not from the seed. */
    memset(f.P, 0, sizeof(f.P));

    pos_ekf_predict(&f, &c, dt, true);

    const float q  = c.sigma_a_move * c.sigma_a_move;
    const float pp = q * dt * dt * dt * dt / 4.0f;   /* dt^4/4 */
    const float pv = q * dt * dt * dt / 2.0f;        /* dt^3/2 */
    const float vv = q * dt * dt;                    /* dt^2   */

    /* Pins the dt^4/4 scale: a mutation to dt^4 quadruples this. */
    CHECK(fabsf(f.P[0 * 4 + 0] - pp) < 1e-6f);
    CHECK(fabsf(f.P[1 * 4 + 1] - pp) < 1e-6f);

    /* Pins the dt^3/2 cross terms, which are what make position and velocity
     * co-vary. Deleting them leaves the diagonal correct and the filter
     * quietly worse, so assert them directly rather than via behaviour. */
    CHECK(f.P[0 * 4 + 2] > 0.0f);
    CHECK(f.P[1 * 4 + 3] > 0.0f);
    CHECK(fabsf(f.P[0 * 4 + 2] - pv) < 1e-6f);
    CHECK(fabsf(f.P[2 * 4 + 0] - pv) < 1e-6f);
    CHECK(fabsf(f.P[1 * 4 + 3] - pv) < 1e-6f);
    CHECK(fabsf(f.P[3 * 4 + 1] - pv) < 1e-6f);

    CHECK(fabsf(f.P[2 * 4 + 2] - vv) < 1e-6f);
    CHECK(fabsf(f.P[3 * 4 + 3] - vv) < 1e-6f);

    /* No cross-axis coupling: x must not co-vary with vy. */
    CHECK(fabsf(f.P[0 * 4 + 3]) < 1e-9f);
    CHECK(fabsf(f.P[1 * 4 + 2]) < 1e-9f);
}

static void test_dt_guards(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f, ref;

    pos_ekf_cfg_defaults(&c);

    /* Non-positive dt must be a no-op, not a negative-time propagation.
     * Reachable in practice from a wrapped or equal clock read. */
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 2.0f, 5.0f);
    f.x[2] = 0.7f;
    f.x[3] = -0.4f;
    ref = f;

    pos_ekf_predict(&f, &c, 0.0f, true);
    CHECK(memcmp(&f, &ref, sizeof(f)) == 0);

    pos_ekf_predict(&f, &c, -3.0f, true);
    CHECK(memcmp(&f, &ref, sizeof(f)) == 0);

    /* An absurd dt must be clamped, not propagated. Without the clamp the
     * dt^4/4 term grows as the fourth power, so 10x the limit is 10000x the
     * position variance -- assert against what the clamp permits. */
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 2.0f, 5.0f);
    memset(f.P, 0, sizeof(f.P));
    pos_ekf_predict(&f, &c, 1200.0f, true);

    const float q      = c.sigma_a_move * c.sigma_a_move;
    const float capped = q * POS_EKF_DT_MAX_EXPECTED * POS_EKF_DT_MAX_EXPECTED
                           * POS_EKF_DT_MAX_EXPECTED * POS_EKF_DT_MAX_EXPECTED
                           / 4.0f;

    CHECK(fabsf(f.P[0] - capped) < capped * 1e-4f);
    CHECK(all_finite_state(&f));
}

static void test_dt_variation_stable(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, 3.5f, 3.5f);

    /* Alternate a fast-tier dt (0.2 s) with a deep-skip dt (5 s) at a fixed
     * true point; the filter must not destabilise either way. */
    for (int i = 0; i < 40; i++) {
        float dt = (i % 2 == 0) ? 0.2f : 5.0f;
        pos_ekf_predict(&f, &c, dt, false);
        make_ranges(4.0f, 3.0f, m, 4);
        pos_ekf_update_ranges(&f, &c, m, 4);
        CHECK(all_finite_state(&f));
    }

    float x, y;
    pos_ekf_get(&f, &x, &y, NULL, NULL);
    CHECK(fabsf(x - 4.0f) < 0.2f);
    CHECK(fabsf(y - 3.0f) < 0.2f);
}

static void test_covariance_symmetric_positive_diag(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];
    float tx = 1.0f, ty = 1.0f;

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, tx, ty);

    for (int i = 0; i < 500; i++) {
        bool moving = (i % 7) < 4;
        float dt = 0.2f + 0.1f * (float)(i % 5);

        if (moving) {
            tx += 0.3f * dt;
            ty += 0.15f * dt;
        }

        pos_ekf_predict(&f, &c, dt, moving);
        make_ranges(tx, ty, m, 4);

        /* Every 11th fix, corrupt one anchor to exercise the gate path too. */
        if ((i % 11) == 0) {
            m[i % 4].range_m += 2.0f;
        }
        pos_ekf_update_ranges(&f, &c, m, 4);

        if (!moving) {
            pos_ekf_zupt(&f, &c);
        }

        for (int r = 0; r < 4; r++) {
            for (int col = 0; col < 4; col++) {
                /* Tight on purpose: 1e-3 was looser than the asymmetry
                 * 500 sequential scalar updates accumulate in float, so
                 * it passed with ekf_symmetrize() deleted. P must be
                 * symmetric to rounding, not merely nearly so. */
                float diff = fabsf(f.P[r * 4 + col] - f.P[col * 4 + r]);
                CHECK(diff <= 1e-7f * (1.0f + fabsf(f.P[r * 4 + col])));
            }
            CHECK(f.P[r * 4 + r] > 0.0f);
        }
        CHECK(all_finite_state(&f));
    }
}

static void test_get_before_seed_and_null_pointers(void)
{
    struct pos_ekf f;
    pos_ekf_reset(&f);

    float x = -999.0f, y = -999.0f, vx = -999.0f, vy = -999.0f;
    bool ok = pos_ekf_get(&f, &x, &y, &vx, &vy);

    CHECK(!ok);
    /* Out-pointers must be left untouched before the first seed. */
    CHECK(x == -999.0f);
    CHECK(y == -999.0f);
    CHECK(vx == -999.0f);
    CHECK(vy == -999.0f);

    /* NULL out-pointers must be accepted, seeded or not. */
    CHECK(pos_ekf_get(&f, NULL, NULL, NULL, NULL) == false);

    pos_ekf_seed(&f, 2.0f, 2.0f);
    CHECK(pos_ekf_get(&f, NULL, NULL, NULL, NULL) == true);
    CHECK(pos_ekf_get(&f, &x, NULL, NULL, NULL) == true);
    CHECK(fabsf(x - 2.0f) < 1e-6f);
}

static void test_no_nan_inf_long_run(void)
{
    struct pos_ekf_cfg c;
    struct pos_ekf f;
    struct pos_meas m[4];
    float tx = 0.5f, ty = 7.5f;

    pos_ekf_cfg_defaults(&c);
    pos_ekf_reset(&f);
    pos_ekf_seed(&f, tx, ty);

    for (int i = 0; i < 1000; i++) {
        bool moving = ((i / 20) % 2) == 0;
        float dt = 0.1f + 0.05f * (float)(i % 100); /* sweeps 0.1..5.1 s */

        if (moving) {
            tx += 0.25f * dt;
            ty -= 0.10f * dt;
            if (tx > 7.5f) tx = 0.5f;
            if (ty < 0.5f) ty = 7.5f;
        }

        pos_ekf_predict(&f, &c, dt, moving);

        size_t n = (size_t)3 + (size_t)(i % 2); /* alternate n=3/n=4 */
        make_ranges(tx, ty, m, 4);
        if ((i % 13) == 0) {
            m[0].range_m += 3.0f; /* occasional gross outlier */
        }
        pos_ekf_update_ranges(&f, &c, m, n);

        if (!moving && (i % 3) == 0) {
            pos_ekf_zupt(&f, &c);
        }

        if (pos_ekf_needs_reseed(&f, &c)) {
            /* Exercise the caller-driven reseed path itself. */
            pos_ekf_seed(&f, tx, ty);
        }

        CHECK(all_finite_state(&f));
    }
}

int main(void)
{
    test_converges_static_from_poor_seed();
    test_tracks_constant_velocity();
    test_outlier_gated();
    test_zupt_zeroes_velocity();
    test_speed_clamp_engages();
    test_stationary_variance_falls();
    test_pos_sigma_uses_both_diag_terms();
    test_gate_streak_and_reseed();
    test_gate_streak_saturates();
    test_reset_after_zero_never_reseeds();
    test_dt_variation_stable();
    test_q_position_and_cross_terms();
    test_dt_guards();
    test_covariance_symmetric_positive_diag();
    test_get_before_seed_and_null_pointers();
    test_no_nan_inf_long_run();

    if (g_fail) { printf("%d FAILURES\n", g_fail); return 1; }
    printf("ALL TESTS PASSED\n");
    return 0;
}
