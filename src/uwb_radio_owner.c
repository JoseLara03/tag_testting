#include "uwb_radio_owner.h"

/* OWNER_IDLE      -- nobody wants the radio; the runner drives it.
 * OWNER_REQUESTED -- a claimant is waiting for the runner to reach its yield
 *                    point.
 * OWNER_HANDED    -- the claimant drives the radio; the runner is parked
 *                    inside uwb_radio_yield().
 *
 * Every transition happens under `lock`, which is what makes the handover
 * safe: the runner can no longer commit to a yield in the instant after a
 * claimant has given up waiting for it. */
enum owner_state { OWNER_IDLE, OWNER_REQUESTED, OWNER_HANDED };

static K_MUTEX_DEFINE(lock);
static K_CONDVAR_DEFINE(cv);
static enum owner_state state = OWNER_IDLE;

bool uwb_radio_request(k_timeout_t timeout)
{
    k_mutex_lock(&lock, K_FOREVER);

    if (state != OWNER_IDLE) {
        /* A claim is already in flight. This firmware has exactly one
         * claimant, so reaching here means the contract was broken -- fail
         * the claim rather than corrupt the one in progress. */
        k_mutex_unlock(&lock);
        return false;
    }

    state = OWNER_REQUESTED;

    while (state == OWNER_REQUESTED) {
        if (k_condvar_wait(&cv, &lock, timeout) != 0) {
            break;
        }
    }

    if (state != OWNER_HANDED) {
        /* Timed out with the claim still pending. Withdrawing it under the
         * lock is what guarantees the runner never yields into the void. */
        state = OWNER_IDLE;
        k_mutex_unlock(&lock);
        return false;
    }

    k_mutex_unlock(&lock);
    return true;
}

void uwb_radio_release(void)
{
    k_mutex_lock(&lock, K_FOREVER);

    if (state == OWNER_HANDED) {
        state = OWNER_IDLE;
        k_condvar_broadcast(&cv);
    }
    /* Otherwise there was no successful claim to release. Doing nothing is
     * deliberate: a stray release must not arm the next handover to complete
     * instantly, which would put two threads on the DW3000. */

    k_mutex_unlock(&lock);
}

bool uwb_radio_request_pending(void)
{
    k_mutex_lock(&lock, K_FOREVER);
    bool pending = (state == OWNER_REQUESTED);
    k_mutex_unlock(&lock);

    return pending;
}

bool uwb_radio_yield(void)
{
    k_mutex_lock(&lock, K_FOREVER);

    if (state != OWNER_REQUESTED) {
        /* The claim was withdrawn between the runner's poll and this call.
         * Handing over now would park us forever waiting for a claimant that
         * already gave up. Nothing was handed over, so the caller must not pay
         * the cost of reacquiring a radio it never lost. */
        k_mutex_unlock(&lock);
        return false;
    }

    state = OWNER_HANDED;
    k_condvar_broadcast(&cv);

    while (state == OWNER_HANDED) {
        k_condvar_wait(&cv, &lock, K_FOREVER);
    }

    k_mutex_unlock(&lock);
    return true;
}
