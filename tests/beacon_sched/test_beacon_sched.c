/*
 * Host test for beacon_sched_core.
 *
 * The property under test is the one the module exists for: the period
 * estimate's error scales as 1/n with the baseline length in *superframes*,
 * rather than sitting at the fixed residual an EMA over consecutive arrivals
 * converges to. Everything else here defends that property.
 *
 * Arrival times are modelled the way the runner produces them: a true arrival
 * instant in microseconds, plus a bounded jitter, truncated to a whole
 * millisecond by k_uptime_get_32(). That truncation is deliberate -- it is the
 * dominant endpoint error and the design's §4.2 arithmetic omits it.
 */

#include "beacon_sched_core.h"
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c); g_fail++; } } while(0)

#define NOMINAL_MS      200u
#define P_TRUE_US   195020u    /* the ~195.02 ms the tag actually measures */
#define JIT_US         120u    /* +/- measured arrival jitter */

static uint32_t p_true_q16(void)
{
    /* 195020 us in ms, Q16.16. */
    return (uint32_t)((((uint64_t)P_TRUE_US) << 16) / 1000u);
}

/* Deterministic pseudo-jitter in [-JIT_US, +JIT_US]; a fixed sequence so a
 * failure is reproducible. */
static int32_t jitter_us(uint32_t k)
{
    uint32_t h = k * 2654435761u;

    h ^= h >> 15;
    return (int32_t)(h % (2u * JIT_US + 1u)) - (int32_t)JIT_US;
}

/* Arrival of superframe `fc`, in the tag's truncated millisecond clock.
 * `t_base_ms` lets a test place the sequence anywhere, including across the
 * uint32 wrap. */
static uint32_t arrival_ms(uint32_t t_base_ms, uint32_t fc0, uint32_t fc)
{
    uint64_t rel_us = (uint64_t)(fc - fc0) * P_TRUE_US;
    int64_t  us     = (int64_t)rel_us + jitter_us(fc);

    if (us < 0) {
        us = 0;
    }
    return t_base_ms + (uint32_t)((uint64_t)us / 1000u);
}

/* Feed `n_obs` observations spaced `step` superframes apart. */
static void feed(struct beacon_sched *s, uint32_t t_base_ms, uint32_t fc_base,
                 uint32_t n_obs, uint32_t step)
{
    for (uint32_t i = 0; i < n_obs; i++) {
        uint32_t fc = fc_base + i * step;

        beacon_sched_observe(s, arrival_ms(t_base_ms, fc_base, fc), fc);
    }
}

static uint32_t abs_diff(uint32_t a, uint32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

/* Estimate error expressed in microseconds of period. */
static uint32_t err_us(const struct beacon_sched *s)
{
    uint32_t d_q16 = abs_diff(beacon_sched_period_q16(s), p_true_q16());

    return (uint32_t)(((uint64_t)d_q16 * 1000u) >> 16);
}

/* ------------------------------------------------------------------ */

static void test_converges(void)
{
    struct beacon_sched s;

    memset(&s, 0, sizeof(s));
    beacon_sched_reset(&s, NOMINAL_MS);

    /* Seeded at the nominal 200 ms, which is 5 ms wrong on purpose -- that is
     * the mistake beacon_track_core was written to avoid and this module must
     * not reintroduce. */
    CHECK(beacon_sched_period_q16(&s) == (NOMINAL_MS << 16));
    CHECK(!beacon_sched_have_ref(&s));

    feed(&s, 1000u, 0u, 40u, 25u);

    CHECK(beacon_sched_have_ref(&s));
    /* Converged to the true period, not to the seed. */
    CHECK(err_us(&s) < 10u);
    CHECK(beacon_sched_baseline_sf(&s) == 39u * 25u);
}

/* The whole point of the module: error <= endpoint_uncertainty / n. */
static void test_error_scales_as_one_over_n(void)
{
    static const uint32_t spans[] = { 1u, 10u, 100u, 1000u };
    uint32_t prev_bound = 0;

    for (unsigned i = 0; i < sizeof(spans) / sizeof(spans[0]); i++) {
        struct beacon_sched s;
        uint32_t span = spans[i];

        memset(&s, 0, sizeof(s));
        beacon_sched_reset(&s, NOMINAL_MS);
        /* Two observations `span` superframes apart is the shortest way to
         * pin the relationship; more observations only move the endpoints. */
        feed(&s, 5000u, 100u, 2u, span);

        CHECK(beacon_sched_baseline_sf(&s) == span);

        /* Endpoint uncertainty: 2*JIT_US of jitter across the pair plus up to
         * 1000 us of millisecond truncation on the difference. +1 us covers
         * the Q16.16 rounding of the comparison itself. */
        uint32_t bound = ((2u * JIT_US + 1000u) / span) + 1u;

        CHECK(err_us(&s) <= bound);
        if (i > 0) {
            CHECK(bound < prev_bound);   /* the bound really is tightening */
        }
        prev_bound = bound;
    }

    /* Concretely: over a 1000-superframe baseline the estimate is better than
     * a couple of microseconds, i.e. two orders of magnitude better than the
     * per-arrival jitter an EMA is limited by. */
    {
        struct beacon_sched s;

        memset(&s, 0, sizeof(s));
        beacon_sched_reset(&s, NOMINAL_MS);
        feed(&s, 5000u, 100u, 2u, 1000u);
        CHECK(err_us(&s) <= 2u);
    }
}

/* Hand-computed window widths. drift_us = 2*eff*P*ppm/1000 with P = 195 ms;
 * est_us = 2*eff*(JITTER+QUANT)/span; window = ceil(sum/1000) + margin,
 * floored at BSCHED_WINDOW_MIN_MS. */
static void test_window_widths(void)
{
    struct beacon_sched s;

    memset(&s, 0, sizeof(s));
    beacon_sched_reset(&s, NOMINAL_MS);
    feed(&s, 1000u, 0u, 100u, 25u);           /* span = 99 * 25 = 2475 sf */

    uint32_t span = beacon_sched_baseline_sf(&s);
    uint32_t p_ms = beacon_sched_period_q16(&s) >> 16;

    CHECK(span == 2475u);
    CHECK(p_ms == 195u);

    static const uint32_t skips[] = { 1u, 25u, 75u, 300u };

    for (unsigned i = 0; i < sizeof(skips) / sizeof(skips[0]); i++) {
        uint32_t skip = skips[i];
        uint32_t drift_us = (2u * skip * p_ms * BSCHED_PPM_TOTAL) / 1000u;
        uint32_t est_us   = (2u * skip * (BSCHED_JITTER_US + BSCHED_QUANT_US)) / span;
        uint32_t want     = ((drift_us + est_us + 999u) / 1000u) + BSCHED_WINDOW_MARGIN_MS;
        uint32_t arm = 0, win = 0, eff = 0;

        if (want < BSCHED_WINDOW_MIN_MS) {
            want = BSCHED_WINDOW_MIN_MS;
        }

        beacon_sched_plan(&s, skip, &arm, &win, &eff);
        CHECK(eff == skip);
        CHECK(win == want);

        /* The window is centred on the prediction, and arm_ms is its start --
         * the same contract beacon_track_plan() uses. */
        uint32_t span_ms = (uint32_t)(((uint64_t)skip *
                                       beacon_sched_period_q16(&s)) >> 16);
        CHECK(arm == s.t_last_ms + span_ms - win / 2u);
    }
}

/*
 * Plan Step 8's gate: a 300-superframe skip must plan a window <= 10 ms.
 *
 * This holds for any baseline that itself spans a few hundred superframes,
 * which is the only situation a 300-superframe skip can arise in -- the
 * baseline is measured in superframes elapsed, not in observations taken, so a
 * tag running at a deep skip accumulates span 300x faster than it takes
 * observations.
 *
 * It does NOT hold for a 100-observation baseline taken at skip 1 (span 99),
 * which plans ~14 ms. That combination is not reachable in operation, but it
 * is worth pinning: the gap is the millisecond truncation of the arrival
 * timestamp (BSCHED_QUANT_US), which the design's §4.2 arithmetic omits and
 * which is 8x the arrival jitter it does account for. If a deep skip ever has
 * to work off a short baseline, feed this module microseconds, not
 * milliseconds.
 */
static void test_deep_skip_window(void)
{
    struct beacon_sched s;
    uint32_t win = 0, eff = 0;

    /* 100 observations at skip 25 -- what a tag running the SLOW tier has. */
    memset(&s, 0, sizeof(s));
    beacon_sched_reset(&s, NOMINAL_MS);
    feed(&s, 1000u, 0u, 100u, 25u);
    beacon_sched_plan(&s, 300u, NULL, &win, &eff);
    CHECK(eff == 300u);
    CHECK(win <= 10u);

    /* Even a 100-observation baseline at skip 4 (span 396) clears it. */
    memset(&s, 0, sizeof(s));
    beacon_sched_reset(&s, NOMINAL_MS);
    feed(&s, 1000u, 0u, 100u, 4u);
    beacon_sched_plan(&s, 300u, NULL, &win, &eff);
    CHECK(win <= 10u);

    /* The skip-1 baseline case, recorded rather than asserted away. */
    memset(&s, 0, sizeof(s));
    beacon_sched_reset(&s, NOMINAL_MS);
    feed(&s, 1000u, 0u, 100u, 1u);
    beacon_sched_plan(&s, 300u, NULL, &win, &eff);
    CHECK(beacon_sched_baseline_sf(&s) == 99u);
    CHECK(win > 10u && win <= 16u);

    /* At the shipped UWB_LISTEN_SKIP_CAP of 25 the same short baseline is
     * comfortable, which is why this is not a shipping problem. */
    beacon_sched_plan(&s, 25u, NULL, &win, &eff);
    CHECK(win <= 5u);
}

/* A freshly locked tag (beacon_track needs 8 clean beacons) has a span of 7
 * superframes and must widen its own window rather than aim a 4 ms slot with a
 * one-superframe period estimate. */
static void test_short_baseline_widens(void)
{
    struct beacon_sched s;
    uint32_t win_short = 0, win_long = 0;

    memset(&s, 0, sizeof(s));
    beacon_sched_reset(&s, NOMINAL_MS);
    feed(&s, 1000u, 0u, 8u, 1u);
    CHECK(beacon_sched_baseline_sf(&s) == 7u);
    beacon_sched_plan(&s, 25u, NULL, &win_short, NULL);

    feed(&s, 1000u, 8u, 60u, 25u);
    beacon_sched_plan(&s, 25u, NULL, &win_long, NULL);

    CHECK(win_short > win_long);
    CHECK(win_long == BSCHED_WINDOW_MIN_MS);
}

static void test_miss_ladder(void)
{
    struct beacon_sched s;
    uint32_t win = 0, eff = 0;

    memset(&s, 0, sizeof(s));
    beacon_sched_reset(&s, NOMINAL_MS);
    feed(&s, 1000u, 0u, 100u, 25u);

    uint32_t win0 = 0;
    beacon_sched_plan(&s, 24u, NULL, &win0, &eff);
    CHECK(eff == 24u);

    uint32_t span_before = beacon_sched_baseline_sf(&s);
    uint32_t period_before = beacon_sched_period_q16(&s);

    /* 1 miss: half the skip, double the window, phase reference gone. */
    beacon_sched_miss(&s);
    CHECK(!beacon_sched_have_ref(&s));
    beacon_sched_plan(&s, 24u, NULL, &win, &eff);
    CHECK(eff == 12u);
    {
        uint32_t drift_us = (2u * 12u * 195u * BSCHED_PPM_TOTAL) / 1000u;
        uint32_t est_us   = (2u * 12u * (BSCHED_JITTER_US + BSCHED_QUANT_US))
                          / span_before;
        uint32_t want     = (((drift_us + est_us + 999u) / 1000u)
                             + BSCHED_WINDOW_MARGIN_MS) * 2u;

        if (want < BSCHED_WINDOW_MIN_MS) {
            want = BSCHED_WINDOW_MIN_MS;
        }
        CHECK(win == want);
    }
    CHECK(win > win0);   /* halving the skip must not narrow the window */

    /* The baseline survives a miss: the arrival that was missed says nothing
     * about the arrivals that were seen. */
    CHECK(beacon_sched_baseline_sf(&s) == span_before);
    CHECK(beacon_sched_period_q16(&s) == period_before);

    /* 2 misses: give up on skipping and listen for a whole superframe. */
    beacon_sched_miss(&s);
    beacon_sched_plan(&s, 24u, NULL, &win, &eff);
    CHECK(eff == 1u);
    CHECK(win == NOMINAL_MS + BSCHED_WINDOW_MARGIN_MS);

    /* 3 misses: the module still only counts -- escalating to UWB_ST_SCAN is
     * the caller's state machine. */
    beacon_sched_miss(&s);
    CHECK(s.misses == 3u);
    beacon_sched_plan(&s, 24u, NULL, &win, &eff);
    CHECK(eff == 1u);

    /* A success restores the tier's value and the tight window. */
    uint32_t fc = s.fc_last + 3u;
    beacon_sched_observe(&s, arrival_ms(1000u, 0u, fc), fc);
    CHECK(beacon_sched_have_ref(&s));
    CHECK(s.misses == 0u);
    beacon_sched_plan(&s, 24u, NULL, &win, &eff);
    CHECK(eff == 24u);
    CHECK(win == win0);

    /* Counters: 3 misses, and one observation more than we fed. */
    CHECK(s.miss_count == 3u);
    CHECK(s.ok_count == 101u);
    beacon_sched_stats_reset(&s);
    CHECK(s.miss_count == 0u && s.ok_count == 0u);
    /* stats_reset must not touch the estimate. */
    CHECK(beacon_sched_have_ref(&s));
    CHECK(err_us(&s) < 10u);
}

/* A skip of 1 must never be halved below 1, and skip 0 is treated as 1. */
static void test_skip_floor(void)
{
    struct beacon_sched s;
    uint32_t eff = 0;

    memset(&s, 0, sizeof(s));
    beacon_sched_reset(&s, NOMINAL_MS);
    feed(&s, 1000u, 0u, 20u, 5u);

    beacon_sched_plan(&s, 0u, NULL, NULL, &eff);
    CHECK(eff == 1u);

    beacon_sched_miss(&s);
    beacon_sched_plan(&s, 1u, NULL, NULL, &eff);
    CHECK(eff == 1u);
}

static void test_ms_clock_wrap(void)
{
    struct beacon_sched s;
    struct beacon_sched ref;

    /* Same sequence placed just before the uint32 millisecond wrap and well
     * away from it must produce the same estimate. */
    memset(&s, 0, sizeof(s));
    beacon_sched_reset(&s, NOMINAL_MS);
    feed(&s, 0xFFFFF000u, 0u, 60u, 25u);

    memset(&ref, 0, sizeof(ref));
    beacon_sched_reset(&ref, NOMINAL_MS);
    feed(&ref, 1000u, 0u, 60u, 25u);

    CHECK(beacon_sched_period_q16(&s) == beacon_sched_period_q16(&ref));
    CHECK(beacon_sched_baseline_sf(&s) == beacon_sched_baseline_sf(&ref));
    CHECK(err_us(&s) < 10u);

    /* arm_ms wraps with the clock and stays consistent with t_last. */
    uint32_t arm = 0, win = 0;
    beacon_sched_plan(&s, 25u, &arm, &win, NULL);
    uint32_t span_ms = (uint32_t)(((uint64_t)25u * beacon_sched_period_q16(&s)) >> 16);
    CHECK(arm == (uint32_t)(s.t_last_ms + span_ms - win / 2u));
}

static void test_frame_counter_wrap(void)
{
    struct beacon_sched s;
    struct beacon_sched ref;

    memset(&s, 0, sizeof(s));
    beacon_sched_reset(&s, NOMINAL_MS);
    feed(&s, 1000u, 0xFFFFF000u, 60u, 25u);

    memset(&ref, 0, sizeof(ref));
    beacon_sched_reset(&ref, NOMINAL_MS);
    feed(&ref, 1000u, 0u, 60u, 25u);

    CHECK(beacon_sched_baseline_sf(&s) == 59u * 25u);
    CHECK(beacon_sched_baseline_sf(&ref) == 59u * 25u);
    CHECK(err_us(&s) < 10u);
}

/* A gateway restart resets its frame counter. The baseline must be abandoned,
 * not turned into a nonsense period that aims every window at nothing. The
 * *estimate* is kept: the gateway's period did not change, and the window
 * widens on its own because the span drops to zero. */
static void test_gateway_restart(void)
{
    struct beacon_sched s;

    memset(&s, 0, sizeof(s));
    beacon_sched_reset(&s, NOMINAL_MS);
    feed(&s, 1000u, 5000u, 40u, 25u);
    CHECK(beacon_sched_baseline_sf(&s) > 0u);
    uint32_t good = beacon_sched_period_q16(&s);

    /* Counter goes backwards. */
    beacon_sched_observe(&s, 500000u, 3u);
    CHECK(beacon_sched_baseline_sf(&s) == 0u);
    CHECK(beacon_sched_period_q16(&s) == good);

    /* And forward again by an implausible amount -- an absurd period must be
     * rejected rather than stored. */
    memset(&s, 0, sizeof(s));
    beacon_sched_reset(&s, NOMINAL_MS);
    feed(&s, 1000u, 0u, 40u, 25u);
    good = beacon_sched_period_q16(&s);
    beacon_sched_observe(&s, s.t_last_ms + 200u, s.fc_last + 100000u);
    CHECK(beacon_sched_period_q16(&s) == good);   /* nonsense rejected */
    CHECK(beacon_sched_baseline_sf(&s) == 0u);    /* baseline restarted */

    /* The next observation off the new baseline re-converges. */
    uint32_t fc0 = s.fc_last;
    for (uint32_t i = 1; i <= 40u; i++) {
        uint32_t fc = fc0 + i * 25u;

        beacon_sched_observe(&s, s.t0_ms + (uint32_t)(((uint64_t)(fc - fc0)
                                                       * P_TRUE_US) / 1000u), fc);
    }
    CHECK(err_us(&s) < 10u);
}

/* The baseline rolls onto its own midpoint at BSCHED_BASELINE_MAX so a slow
 * change in the gateway's clock is tracked instead of averaged away -- and the
 * span never collapses to nothing when it does. */
static void test_baseline_roll(void)
{
    struct beacon_sched s;

    memset(&s, 0, sizeof(s));
    beacon_sched_reset(&s, NOMINAL_MS);

    feed(&s, 1000u, 0u, BSCHED_BASELINE_MAX - 1u, 1u);
    CHECK(s.n_obs == BSCHED_BASELINE_MAX - 1u);
    uint32_t span_before = beacon_sched_baseline_sf(&s);

    /* The observation that trips the roll. */
    uint32_t fc = BSCHED_BASELINE_MAX - 1u;
    beacon_sched_observe(&s, arrival_ms(1000u, 0u, fc), fc);

    CHECK(s.n_obs == BSCHED_BASELINE_MAX - (BSCHED_BASELINE_MAX / 2u));
    /* Half the span survives -- never a restart from a single superframe. */
    uint32_t span_after = beacon_sched_baseline_sf(&s);
    CHECK(span_after >= span_before / 2u - 1u);
    CHECK(span_after < span_before);
    CHECK(err_us(&s) < 200u);   /* still a usable estimate right after the roll */
}

int main(void)
{
    test_converges();
    test_error_scales_as_one_over_n();
    test_window_widths();
    test_deep_skip_window();
    test_short_baseline_widens();
    test_miss_ladder();
    test_skip_floor();
    test_ms_clock_wrap();
    test_frame_counter_wrap();
    test_gateway_restart();
    test_baseline_roll();
    printf("beacon_sched_core: %d failure(s)\n", g_fail);
    return g_fail ? 1 : 0;
}
