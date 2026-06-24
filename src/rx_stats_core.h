#ifndef RX_STATS_CORE_H_
#define RX_STATS_CORE_H_

#include <stdint.h>
#include <stdbool.h>
#include "batt_window.h"

/* Pure (Zephyr-free) accumulator core for beacon RX duty-cycle instrumentation.
 * Operates on raw cycle-counter values supplied by the caller, so it is fully
 * host-testable. The Zephyr glue (rx_stats.c) captures the cycle stamps. */
struct rx_stats_core {
    struct batt_window on_win;     /* RX-on duration, microseconds */
    struct batt_window off_win;    /* beacon arrival offset, microseconds (signed) */
    uint32_t prev_beacon_cyc;      /* cycle stamp of the previous beacon */
    bool     prev_valid;           /* false until first beacon / after a miss */
    uint32_t miss_count;
    uint32_t nominal_sf_cyc;       /* nominal superframe period, in cycles */
    uint32_t cyc_per_sec;          /* cycle-counter frequency (Hz), for cyc->us */
};

/* Reset all state. nominal_sf_cyc = nominal superframe in cycles;
 * cyc_per_sec = cycle-counter frequency (Hz). */
void rx_stats_core_reset(struct rx_stats_core *c,
                         uint32_t nominal_sf_cyc, uint32_t cyc_per_sec);

/* Record a received beacon. arm_cyc = cycle stamp when the listen window was
 * armed; now_cyc = cycle stamp at beacon arrival. Records on-duration
 * (now-arm) and, if a previous beacon is known, arrival offset
 * (now - (prev + nominal)). Advances the phase reference to now_cyc. */
void rx_stats_core_beacon(struct rx_stats_core *c,
                          uint32_t arm_cyc, uint32_t now_cyc);

/* Record a missed beacon: invalidate the phase reference and count it. */
void rx_stats_core_miss(struct rx_stats_core *c);

/* Read stats. Returns 1 if >=1 beacon recorded (out params written), else 0.
 * on_* in milliseconds, off_* in microseconds. Any out param may be NULL. */
int rx_stats_core_get(const struct rx_stats_core *c,
                      int *on_mean_ms, int *on_max_ms,
                      int *off_min_us, int *off_max_us,
                      uint32_t *count, uint32_t *misses);

#endif /* RX_STATS_CORE_H_ */
