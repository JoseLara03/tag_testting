#ifndef UWB_RADIO_OWNER_H_
#define UWB_RADIO_OWNER_H_

#include <zephyr/kernel.h>
#include <stdbool.h>

/*
 * Explicit handover of the DW3000 between the MAC runner and a claimant.
 *
 * This module exists to enforce the single-claimant PHY-state handover
 * contract below: dwt_setrxaftertxdelay/dwt_setrxtimeout/
 * dwt_setpreambledetecttimeout/dwt_settxantennadelay/dwt_setrxantennadelay
 * are shared, mutable DW3000 state, and two threads driving them
 * concurrently would corrupt each other's ranging. This module guarantees
 * exactly one thread drives the radio at a time for callers that follow the
 * contract below.
 *
 * Historically this also protected wait_event()'s shared interrupt line and
 * event semaphore (opened with k_sem_reset(), closed with a global
 * port_DisableEXT_IRQ()) between the runner and the calibration thread. The
 * calibration engine (src/cal_run.c) no longer calls wait_event() at all --
 * it polls SYS_STATUS_LO directly and never enables the DW3000 IRQ. That
 * sharing now applies only between the runner and do_one_range_anchor()
 * (production ranging), not calibration -- but the PHY-state handover
 * contract above is still the real reason this module is needed.
 *
 * Single-claimant contract: at most one claimant thread may hold an
 * outstanding uwb_radio_request() at a time. A second, concurrent
 * uwb_radio_request() fails immediately -- it does not queue behind the
 * first -- because this firmware never needs more than one claimant and
 * queueing would hide a violation of that assumption instead of surfacing it.
 *
 * This is not a mutex. The runner sleeps inside its own superframe loop, so a
 * lock held across the loop would starve every other claimant. Instead the
 * runner offers the radio at one safe point per superframe.
 *
 * PHY-state contract: on reacquire the runner restores only
 * dwt_setrxaftertxdelay, dwt_setrxtimeout, dwt_setpreambledetecttimeout,
 * dwt_settxantennadelay and dwt_setrxantennadelay. Everything else it assumes
 * unchanged. A claimant that touches anything outside that list -- dwt_configure,
 * dwt_configuretxrf, dwt_setcallbacks, frame filtering, PAN/short address --
 * must restore it before uwb_radio_release(), or it silently breaks runner RX
 * with no error anywhere: the tag simply stops hearing beacons.
 */

/* Claimant: ask for the radio and block until the runner hands it over.
 * Returns false if the runner did not yield within `timeout`, in which case the
 * claim is withdrawn and the radio was NOT acquired. */
bool uwb_radio_request(k_timeout_t timeout);

/* Claimant: hand the radio back. Must be called on every path out of a
 * successful uwb_radio_request(), or the runner blocks forever. */
void uwb_radio_release(void);

/* Runner: is a claimant waiting? Takes the internal mutex for the duration of
 * one comparison -- uncontended in the common case, but not lock-free, so do
 * not call this from an ISR. */
bool uwb_radio_request_pending(void);

/* Runner: hand the radio over and block until it is returned. Call when
 * uwb_radio_request_pending() is true and the radio has been left awake and
 * idle, per the handover contract. It is also safe to call if the claim was
 * withdrawn between that poll and this call -- the withdrawal is caught under
 * the same lock and uwb_radio_yield() returns immediately without handing
 * over anything.
 *
 * Returns true if the radio was actually handed over and has now come back,
 * false if the claim had already been withdrawn and nothing changed hands. Run
 * the reacquire sequence only when it returns true: after a false the radio was
 * never touched by anyone else, and rebuilding state the claimant never
 * disturbed costs real airtime. */
bool uwb_radio_yield(void);

/* Declare that no MAC runner will ever offer the radio: this image has no
 * runner thread, so a claim can be granted immediately instead of waiting for
 * a yield that will never come. Call once from bring-up, before any claimant
 * thread starts. */
void uwb_radio_owner_set_unmanaged(void);

#endif /* UWB_RADIO_OWNER_H_ */
