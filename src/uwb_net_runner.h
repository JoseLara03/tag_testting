#ifndef UWB_NET_RUNNER_H_
#define UWB_NET_RUNNER_H_

#include <stdint.h>
#include "uwb_net.h"   /* uwb_tier_t */

/* Start the MAC runner thread. Call after uwb_init() and uwb_ss_initiator_start(). */
void uwb_net_runner_start(const uint8_t eui[8]);

/* Minimum bytes ever left unused on the runner thread stack. The runner's
 * deepest path is pos_solve() (CMSIS-DSP matrix inversion, float matrices in
 * locals). Requires CONFIG_INIT_STACKS; returns 0 when unavailable. */
size_t uwb_net_runner_stack_unused(void);

/* Set the tier directly, bypassing the motion hysteresis; the runner feeds
 * UWB_EV_MOTION on the next superframe. Safe from ISR or any thread. Note that
 * uwb_net_tier_filter() re-evaluates every superframe, so this is an override
 * for one superframe rather than a latch -- motion wins on the next pass. */
void uwb_net_set_tier(uwb_tier_t t);

/* Report the raw LIS2HH12 motion state. The runner runs it through
 * uwb_net_tier_filter() (design §6.2 hysteresis) and, on an activity edge,
 * drops the coverage-probe ladder back to rung 0. Safe from ISR. */
void uwb_net_set_moving(bool moving);

/* Cut short whatever sleep the runner is in and force a full-window beacon
 * re-sync on the next pass. Safe from ISR and from any thread.
 *
 * Needed because the runner sleeps through whole superframes: without it a
 * motion edge would take up to a full skip (60 s at the IDLE tier) to raise
 * the tier, and a HELP press the same to reach the air. */
void uwb_net_runner_wake(void);

#include <stdbool.h>

/* Layer-1 power saving: enable/disable DW3000 SLEEP between superframes.
 * Default enabled. Toggled at runtime via the `pwr sleep on|off` NUS command. */
void uwb_radio_set_sleep_enabled(bool en);
bool uwb_radio_sleep_enabled(void);

/* `pwr sched` -- beacon scheduler state. Returns false before the first
 * re-sync of either kind. Any out pointer may be NULL. Read without a lock
 * from the BT RX thread; the values are advisory. */
bool uwb_net_runner_sched_get(uint32_t *period_q16, uint32_t *window_ms,
                              uint32_t *eff_skip, uint32_t *misses,
                              uint32_t *ok_count, uint32_t *miss_count);

/* `pwr schedrst` -- zero the ok/miss counters, leaving the estimate alone. */
void uwb_net_runner_sched_reset_stats(void);

/* `pwr scan` -- coverage-probe ladder rung and the interval it implies. */
void uwb_net_runner_scan_get(uint8_t *rung, uint32_t *next_ms);

#endif /* UWB_NET_RUNNER_H_ */
