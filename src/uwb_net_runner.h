#ifndef UWB_NET_RUNNER_H_
#define UWB_NET_RUNNER_H_

#include <stdint.h>
#include "uwb_net.h"   /* uwb_tier_t */

/* Start the MAC runner thread. Call after uwb_init() and uwb_ss_initiator_start(). */
void uwb_net_runner_start(const uint8_t eui[8]);

/* Set a new motion-driven tier; the runner feeds UWB_EV_MOTION on the next
 * superframe. Safe to call from ISR or any thread context. */
void uwb_net_set_tier(uwb_tier_t t);

/* B4.2 test helper — broadcasts DISCOVERY (0xE2) every 500 ms.
 * Use instead of uwb_net_runner_start() while verifying anchor discovery replies.
 * Revert after B4.2 is confirmed. */
void uwb_disc_test_start(void);

#endif /* UWB_NET_RUNNER_H_ */
