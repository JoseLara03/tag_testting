#ifndef BEACON_SCHED_CORE_H_
#define BEACON_SCHED_CORE_H_

#include <stdint.h>
#include <stdbool.h>

/* Pure (Zephyr-free) long-baseline beacon scheduler for superframe skipping.
 * See spec/2026-08-16-low-power-duty-cycle-design.md §4.
 *
 * Complementary to beacon_track_core, NOT a replacement for it: beacon_track
 * runs the ACQUIRING->TRACKING acquisition FSM and gets the tag locked;
 * beacon_sched keeps it locked across skips of tens or hundreds of
 * superframes. The two have different failure modes and separate host tests.
 *
 * Why a long baseline rather than beacon_track's EMA: the window a skip needs
 * is dominated by the *period estimate error*, multiplied by the skip factor.
 * An EMA over consecutive arrivals converges to a fixed residual of roughly
 * the arrival jitter, so skipping K superframes costs K x that residual. A
 * two-point estimate over a baseline of `n` superframes,
 *
 *     P = (t_last - t0) / (fc_last - fc0)
 *
 * has an error of jitter/n instead, so the window shrinks as the tag stays
 * locked rather than growing with the skip. That 1/n scaling is the entire
 * reason this module exists; a change that breaks it breaks deep skipping.
 *
 * All arithmetic is Q16.16 fixed point. The firmware is CONFIG_FP_SOFTABI and
 * this runs on every superframe wake, so no float appears here.
 *
 * All time is milliseconds in the caller's monotonic clock (k_uptime_get_32()
 * on the target) and all differences are taken signed, so both the ms clock
 * and the gateway's frame counter may wrap freely. */

/* Combined tag + gateway crystal tolerance, ppm. 40 assumes a 32.768 kHz
 * crystal at both ends (+/-20 ppm each); raise to ~500 if the board turns out
 * to run the internal RC (design §4.3, open question 1). */
#define BSCHED_PPM_TOTAL         40u

/* Never plan a window narrower than this: below it the DW3000's own preamble
 * acquisition time and the software timestamp path dominate. */
#define BSCHED_WINDOW_MIN_MS      4u

/* Fixed slack added to every computed window. */
#define BSCHED_WINDOW_MARGIN_MS   2u

/* Superframes' worth of observations retained before the baseline is rolled
 * forward onto its own midpoint. Bounding it is what lets a slow change in the
 * gateway's clock be tracked instead of averaged away over the tag's lifetime. */
#define BSCHED_BASELINE_MAX     128u

/* Per-endpoint arrival jitter, microseconds. 120 us is the figure measured
 * through `pwr rx`'s folded arrival offset once the narrow window is tracking. */
#define BSCHED_JITTER_US        120u

/* Truncation error the millisecond clock contributes to the *difference*
 * between the two baseline endpoints. k_uptime_get_32() truncates, so each
 * endpoint carries 0..1 ms of error and the difference carries up to +/-1 ms.
 *
 * This term is not in the design's §4.2 arithmetic and it dominates
 * BSCHED_JITTER_US by 8x. It is why a skip of several hundred superframes
 * needs a baseline that itself spans hundreds of superframes -- which it
 * naturally does, since the baseline is measured in superframes elapsed and
 * not in observations taken. See the note in tests/beacon_sched/. */
#define BSCHED_QUANT_US        1000u

struct beacon_sched {
    uint32_t nominal_ms;    /* seed period, also the fallback full window */
    uint32_t t0_ms;         /* first arrival of the current baseline */
    uint32_t fc0;           /* frame counter at t0 */
    uint32_t t_mid_ms;      /* baseline-roll checkpoint (see _observe) */
    uint32_t fc_mid;
    uint32_t t_last_ms;     /* most recent arrival: the phase reference */
    uint32_t fc_last;
    uint32_t period_q16;    /* estimated superframe period, ms in Q16.16 */
    uint32_t ok_count;      /* successful re-syncs   } diagnostics, survive */
    uint32_t miss_count;    /* missed re-syncs       } beacon_sched_reset()  */
    uint8_t  n_obs;         /* observations in the current baseline */
    uint8_t  misses;        /* consecutive missed re-syncs */
    bool     have_ref;      /* t_last_ms/fc_last usable as a phase reference */
};

/* Drop the estimate and the phase reference, seeding the period with the
 * nominal superframe length. ok_count/miss_count are deliberately preserved:
 * the runner calls this on a radio handover and on a tier change, and wiping
 * the re-sync statistics there would make `pwr sched` useless. Use
 * beacon_sched_stats_reset() for `pwr schedrst`. */
void beacon_sched_reset(struct beacon_sched *s, uint32_t nominal_period_ms);

/* Zero the ok/miss counters only, leaving the estimate intact. */
void beacon_sched_stats_reset(struct beacon_sched *s);

/* Feed an observed beacon: arrival time in the tag's clock and the gateway's
 * frame counter, which gives the true elapsed superframe count even across a
 * skip. Updates the long-baseline period estimate and clears the miss ladder. */
void beacon_sched_observe(struct beacon_sched *s, uint32_t t_ms, uint32_t frame_ctr);

/* Feed a missed re-sync: steps down the skip ladder (see beacon_sched_plan)
 * and invalidates the phase reference. The *baseline* is retained -- the
 * arrival that was missed does not invalidate the arrivals that were seen. */
void beacon_sched_miss(struct beacon_sched *s);

/* Plan the next wake.
 *   `skip`            the tier's configured listen_skip (0 is treated as 1).
 *   *arm_ms           absolute ms at which to arm RX, i.e. the *start* of the
 *                     window -- the caller subtracts its own wake guard, the
 *                     same contract beacon_track_plan() uses.
 *   *window_ms        RX-on span to allow.
 *   *effective_skip   the skip actually planned; the miss ladder may return
 *                     less than `skip`.
 * Any out pointer may be NULL. Pure read; does not mutate state.
 *
 * Miss ladder (design §4.4): 0 misses -> the tier's skip; 1 miss -> half the
 * skip and double the window; 2+ -> skip 1 and a full-superframe window. The
 * module only counts; escalating to UWB_ST_SCAN at UWB_NET_MISS_MAX is the
 * caller's state machine, not this one's.
 *
 * *arm_ms is meaningless while beacon_sched_have_ref() is false (no arrival
 * has been observed, or the last re-sync missed). The caller must listen for a
 * full superframe from `now` in that case. */
void beacon_sched_plan(const struct beacon_sched *s, uint32_t skip,
                       uint32_t *arm_ms, uint32_t *window_ms,
                       uint32_t *effective_skip);

/* True once an arrival has been observed and the last re-sync did not miss. */
bool beacon_sched_have_ref(const struct beacon_sched *s);

/* Estimated period, ms in Q16.16 -- diagnostics (`pwr sched`) and tests. */
uint32_t beacon_sched_period_q16(const struct beacon_sched *s);

/* Superframes spanned by the current baseline; 0 before the second
 * observation. This is the `n` the estimate error scales as 1/n. */
uint32_t beacon_sched_baseline_sf(const struct beacon_sched *s);

#endif /* BEACON_SCHED_CORE_H_ */
