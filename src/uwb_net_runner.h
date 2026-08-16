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

/* Set a new motion-driven tier; the runner feeds UWB_EV_MOTION on the next
 * superframe. Safe to call from ISR or any thread context. */
void uwb_net_set_tier(uwb_tier_t t);

#include <stdbool.h>

/* Layer-1 power saving: enable/disable DW3000 SLEEP between superframes.
 * Default enabled. Toggled at runtime via the `pwr sleep on|off` NUS command. */
void uwb_radio_set_sleep_enabled(bool en);
bool uwb_radio_sleep_enabled(void);

#endif /* UWB_NET_RUNNER_H_ */
