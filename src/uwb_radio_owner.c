#include "uwb_radio_owner.h"

static K_SEM_DEFINE(handover_sem, 0, 1);   /* runner -> claimant: it is yours */
static K_SEM_DEFINE(returned_sem, 0, 1);   /* claimant -> runner: it is back  */
static atomic_t requested = ATOMIC_INIT(0);

bool uwb_radio_request(k_timeout_t timeout)
{
    atomic_set(&requested, 1);

    if (k_sem_take(&handover_sem, timeout) != 0) {
        /* The runner never reached its yield point. Withdraw, so a later
         * yield does not hand the radio to a claimant that gave up. */
        atomic_set(&requested, 0);
        return false;
    }

    return true;
}

void uwb_radio_release(void)
{
    atomic_set(&requested, 0);
    k_sem_give(&returned_sem);
}

bool uwb_radio_request_pending(void)
{
    return atomic_get(&requested) != 0;
}

void uwb_radio_yield(void)
{
    k_sem_give(&handover_sem);
    k_sem_take(&returned_sem, K_FOREVER);
}
