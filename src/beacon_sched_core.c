#include "beacon_sched_core.h"

/* Accept a period estimate only inside [nominal/2, nominal*2]. A gateway that
 * restarts resets its frame counter, and a counter that jumps forward (rather
 * than backwards, which the signed diff below already catches) would otherwise
 * produce an absurd period and aim every subsequent window at nothing. */
static bool period_sane(const struct beacon_sched *s, uint32_t period_q16)
{
    uint32_t lo = (s->nominal_ms << 16) / 2u;
    uint32_t hi = (s->nominal_ms << 16) * 2u;

    return period_q16 >= lo && period_q16 <= hi;
}

/* Start (or restart) the baseline at a single observation. period_q16 is
 * deliberately left alone: a gateway that restarts its frame counter has not
 * changed its superframe period, so the last good estimate is a far better
 * prediction than falling back to the nominal seed (which is ~5 ms wrong --
 * the exact mistake beacon_track_core was written to avoid). The window
 * widens on its own anyway, because the baseline span drops to zero. */
static void baseline_seed(struct beacon_sched *s, uint32_t t_ms, uint32_t frame_ctr)
{
    s->t0_ms    = t_ms;
    s->fc0      = frame_ctr;
    s->t_mid_ms = t_ms;
    s->fc_mid   = frame_ctr;
    s->t_last_ms = t_ms;
    s->fc_last   = frame_ctr;
    s->n_obs     = 1u;
}

void beacon_sched_reset(struct beacon_sched *s, uint32_t nominal_period_ms)
{
    s->nominal_ms = nominal_period_ms;
    s->t0_ms      = 0;
    s->fc0        = 0;
    s->t_mid_ms   = 0;
    s->fc_mid     = 0;
    s->t_last_ms  = 0;
    s->fc_last    = 0;
    s->period_q16 = nominal_period_ms << 16;
    s->n_obs      = 0;
    s->misses     = 0;
    s->have_ref   = false;
    /* ok_count / miss_count intentionally untouched -- see the header. */
}

void beacon_sched_stats_reset(struct beacon_sched *s)
{
    s->ok_count   = 0;
    s->miss_count = 0;
}

void beacon_sched_observe(struct beacon_sched *s, uint32_t t_ms, uint32_t frame_ctr)
{
    s->ok_count++;
    s->misses   = 0;
    s->have_ref = true;

    if (s->n_obs == 0u) {
        baseline_seed(s, t_ms, frame_ctr);
        return;
    }

    /* Signed differences: both the ms clock and the gateway frame counter are
     * uint32 and both wrap inside the tag's target battery life. */
    int32_t dfc = (int32_t)(frame_ctr - s->fc0);
    int32_t dt  = (int32_t)(t_ms - s->t0_ms);

    if (dfc <= 0 || dt <= 0) {
        /* Counter went backwards (gateway restart) or the clocks disagree:
         * the old baseline describes a different gateway epoch. */
        baseline_seed(s, t_ms, frame_ctr);
        return;
    }

    uint32_t p_q16 = (uint32_t)((((uint64_t)(uint32_t)dt) << 16) / (uint32_t)dfc);

    if (!period_sane(s, p_q16)) {
        baseline_seed(s, t_ms, frame_ctr);
        return;
    }

    s->period_q16 = p_q16;
    s->t_last_ms  = t_ms;
    s->fc_last    = frame_ctr;

    if (s->n_obs < BSCHED_BASELINE_MAX) {
        s->n_obs++;
    }

    /* Capture the midpoint on the way up, then roll the baseline onto it when
     * the cap is reached. Rolling (rather than freezing t0 forever) is what
     * lets a slow drift in the gateway's clock be tracked; keeping the drop to
     * the midpoint (rather than restarting from scratch) keeps half the
     * baseline length, so the estimate error never jumps back to a single
     * superframe's worth. */
    if (s->n_obs == (BSCHED_BASELINE_MAX / 2u)) {
        s->t_mid_ms = t_ms;
        s->fc_mid   = frame_ctr;
    } else if (s->n_obs >= BSCHED_BASELINE_MAX) {
        s->t0_ms = s->t_mid_ms;
        s->fc0   = s->fc_mid;
        s->n_obs = BSCHED_BASELINE_MAX - (BSCHED_BASELINE_MAX / 2u);
    }
}

void beacon_sched_miss(struct beacon_sched *s)
{
    s->miss_count++;
    if (s->misses < 0xFFu) {
        s->misses++;
    }
    /* The phase is gone -- we do not know which superframe we are in any more,
     * so no narrow window can be aimed. The baseline stays: the arrivals that
     * were seen are still valid measurements of the period. */
    s->have_ref = false;
}

bool beacon_sched_have_ref(const struct beacon_sched *s)
{
    return s->have_ref;
}

uint32_t beacon_sched_period_q16(const struct beacon_sched *s)
{
    return s->period_q16;
}

uint32_t beacon_sched_baseline_sf(const struct beacon_sched *s)
{
    if (s->n_obs < 2u) {
        return 0u;
    }
    return (uint32_t)(int32_t)(s->fc_last - s->fc0);
}

void beacon_sched_plan(const struct beacon_sched *s, uint32_t skip,
                       uint32_t *arm_ms, uint32_t *window_ms,
                       uint32_t *effective_skip)
{
    uint32_t eff   = (skip == 0u) ? 1u : skip;
    uint32_t widen = 1u;
    uint32_t w_ms;

    /* Miss ladder (design §4.4): rungs, not a cliff. A miss is evidence the
     * prediction was wrong, so the next attempt must be cheaper to get right,
     * not a repeat of the same bet at the same width. */
    if (s->misses == 1u) {
        eff   = (eff / 2u) ? (eff / 2u) : 1u;
        widen = 2u;
    } else if (s->misses >= 2u) {
        eff = 1u;
    }

    if (s->misses >= 2u) {
        w_ms = s->nominal_ms + BSCHED_WINDOW_MARGIN_MS;
    } else {
        uint32_t p_ms = s->period_q16 >> 16;

        /* Clock drift over the skip, microseconds:
         *     2 * eff * P[ms] * ppm / 1e6  seconds  ->  * 1e6 us
         * which reduces to 2 * eff * P * ppm / 1000. */
        uint32_t drift_us = (2u * eff * p_ms * BSCHED_PPM_TOTAL) / 1000u;

        /* Period-estimate error over the skip. eps_P = endpoint uncertainty
         * divided by the baseline span in superframes -- this is the 1/n term
         * the module exists for. A baseline of one superframe (or none) gives
         * the full endpoint uncertainty, so a freshly reset scheduler widens
         * its own window automatically. */
        uint32_t span_sf = beacon_sched_baseline_sf(s);
        if (span_sf == 0u) {
            span_sf = 1u;
        }
        uint32_t est_us = (2u * eff * (BSCHED_JITTER_US + BSCHED_QUANT_US)) / span_sf;

        uint32_t tot_us = drift_us + est_us;

        w_ms = ((tot_us + 999u) / 1000u) + BSCHED_WINDOW_MARGIN_MS;
        w_ms *= widen;
        if (w_ms < BSCHED_WINDOW_MIN_MS) {
            w_ms = BSCHED_WINDOW_MIN_MS;
        }
        if (w_ms > s->nominal_ms + BSCHED_WINDOW_MARGIN_MS) {
            /* Never plan wider than a full superframe listen -- past that the
             * narrow window costs more than it saves and the caller may as
             * well re-acquire. */
            w_ms = s->nominal_ms + BSCHED_WINDOW_MARGIN_MS;
        }
    }

    if (window_ms) {
        *window_ms = w_ms;
    }
    if (effective_skip) {
        *effective_skip = eff;
    }
    if (arm_ms) {
        /* 64-bit product: eff * period_q16 overflows uint32 at large skips
         * (300 * 200 ms in Q16.16 is already 3.9e9). */
        uint32_t span_ms = (uint32_t)(((uint64_t)eff * (uint64_t)s->period_q16) >> 16);

        *arm_ms = s->t_last_ms + span_ms - (w_ms / 2u);
    }
}
