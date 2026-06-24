#ifndef RX_STATS_H_
#define RX_STATS_H_

#include <stdint.h>

/* Beacon RX duty-cycle instrumentation (singleton). The runner calls
 * arm/beacon/miss around the beacon-listen window; `pwr rx` reads it. */

/* Reset all accumulators and phase state. Call once at startup. */
void rx_stats_reset(void);

/* Mark the start of a beacon-listen window (RX about to be armed). */
void rx_stats_arm(void);

/* A real beacon was received: record on-duration and arrival offset. */
void rx_stats_beacon(void);

/* The beacon was missed this superframe. */
void rx_stats_miss(void);

/* Read stats: on_* in ms, off_* in us. Returns 1 if >=1 beacon recorded. */
int rx_stats_get(int *on_mean_ms, int *on_max_ms,
                 int *off_min_us, int *off_max_us,
                 uint32_t *count, uint32_t *misses);

#endif /* RX_STATS_H_ */
