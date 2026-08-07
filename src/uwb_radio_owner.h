#ifndef UWB_RADIO_OWNER_H_
#define UWB_RADIO_OWNER_H_

#include <zephyr/kernel.h>
#include <stdbool.h>

/*
 * Explicit handover of the DW3000 between the MAC runner and a claimant.
 *
 * The runner and the calibration thread share one interrupt line and one event
 * semaphore through wait_event(), which is destructive under concurrency: it
 * opens with k_sem_reset() and closes with a global port_DisableEXT_IRQ(). That
 * code is correct as long as exactly one thread drives the radio at a time,
 * which is what this module guarantees.
 *
 * This is not a mutex. The runner sleeps inside its own superframe loop, so a
 * lock held across the loop would starve every other claimant. Instead the
 * runner offers the radio at one safe point per superframe.
 */

/* Claimant: ask for the radio and block until the runner hands it over.
 * Returns false if the runner did not yield within `timeout`, in which case the
 * claim is withdrawn and the radio was NOT acquired. */
bool uwb_radio_request(k_timeout_t timeout);

/* Claimant: hand the radio back. Must be called on every path out of a
 * successful uwb_radio_request(), or the runner blocks forever. */
void uwb_radio_release(void);

/* Runner: is a claimant waiting? Cheap, no blocking. */
bool uwb_radio_request_pending(void);

/* Runner: hand the radio over and block until it is returned. Only call when
 * uwb_radio_request_pending() is true and the radio has been left awake and
 * idle, per the handover contract. */
void uwb_radio_yield(void);

#endif /* UWB_RADIO_OWNER_H_ */
