#ifndef BEACON_TRACK_CORE_H_
#define BEACON_TRACK_CORE_H_

#include <stdint.h>
#include <stdbool.h>

/* Pure (Zephyr-free) beacon-arrival predictor for the narrow-window RX scheme.
 * The caller supplies beacon arrival times in milliseconds (any monotonic ms
 * clock); the core maintains an EMA of the gateway beacon period and a two-state
 * machine: ACQUIRING (listen full superframe) until warmup_n clean beacons in a
 * row, then TRACKING (listen only in a narrow window around the prediction).
 * A miss drops back to ACQUIRING but retains the period estimate. */
struct beacon_track {
    uint32_t period_est_ms;   /* EMA of the beacon period, in ms */
    uint32_t last_beacon_ms;  /* arrival time of the last caught beacon */
    bool     have_last;       /* false until first beacon / after a miss */
    uint16_t warmup_count;    /* consecutive clean beacons */
    uint32_t guard_ms;        /* half-width of the narrow window, in ms */
    uint16_t warmup_n;        /* clean beacons required before TRACKING */
    uint8_t  ema_shift;       /* EMA smoothing: period += err >> ema_shift */
    bool     tracking;        /* false = ACQUIRING, true = TRACKING */
};

/* period_seed_ms = nominal beacon period (e.g. 200); guard_ms, warmup_n,
 * ema_shift = tuning constants. */
void beacon_track_reset(struct beacon_track *c, uint32_t period_seed_ms,
                        uint32_t guard_ms, uint16_t warmup_n, uint8_t ema_shift);

/* Plan the next listen window.
 *   *narrow     = true only in TRACKING with a known reference.
 *   *arm_at_ms  = absolute ms to arm RX (narrow only) = last + period_est - guard.
 *   *window_ms  = RX-on span to allow (narrow only) = 2 * guard.
 * When *narrow is false the caller uses the full-superframe listen. Any out
 * pointer may be NULL. Pure read; does not mutate state. */
void beacon_track_plan(const struct beacon_track *c,
                       bool *narrow, uint32_t *arm_at_ms, uint32_t *window_ms);

/* Record a caught beacon at now_ms: update the EMA (skipped on the first beacon
 * or the first beacon after a miss, to avoid poisoning it across a gap), advance
 * the reference, advance warmup, and enter TRACKING once warmup_n is reached. */
void beacon_track_beacon(struct beacon_track *c, uint32_t now_ms);

/* Record a missed beacon: drop to ACQUIRING, reset warmup, clear the reference,
 * keep period_est. */
void beacon_track_miss(struct beacon_track *c);

/* Current period estimate (ms) — diagnostics / tests. */
uint32_t beacon_track_period_ms(const struct beacon_track *c);

#endif /* BEACON_TRACK_CORE_H_ */
