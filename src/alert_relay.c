#include "alert_relay.h"
#include <string.h>

/* ---- duplicate suppression ------------------------------------------- */

void alert_dedup_reset(struct alert_dedup *d)
{
    memset(d, 0, sizeof(*d));
}

static bool dedup_key_eq(const struct alert_dedup_entry *e, const struct uwb_alert *a)
{
    return memcmp(e->eui, a->orig_eui, 8) == 0 &&
           e->epoch == a->epoch &&
           e->state == a->state &&
           e->repeat_seq == a->repeat_seq;
}

bool alert_dedup_admit(struct alert_dedup *d, const struct uwb_alert *a, uint32_t now_ms)
{
    int free_idx   = -1;
    int oldest_idx = 0;
    int32_t oldest_age = -1;

    for (int i = 0; i < ALERT_DEDUP_N; i++) {
        struct alert_dedup_entry *e = &d->e[i];

        if (!e->used) {
            if (free_idx < 0) {
                free_idx = i;
            }
            continue;
        }

        int32_t age = (int32_t)(now_ms - e->seen_ms);
        bool aged_out = age >= (int32_t)ALERT_DEDUP_AGE_MS;

        if (dedup_key_eq(e, a) && !aged_out) {
            return false;   /* duplicate: drop */
        }
        if (aged_out && free_idx < 0) {
            free_idx = i;   /* stale slot, free for reuse */
        }
        if (age > oldest_age) {
            oldest_age = age;
            oldest_idx = i;
        }
    }

    int idx = (free_idx >= 0) ? free_idx : oldest_idx;
    struct alert_dedup_entry *ne = &d->e[idx];

    memcpy(ne->eui, a->orig_eui, 8);
    ne->epoch      = a->epoch;
    ne->state      = a->state;
    ne->repeat_seq = a->repeat_seq;
    ne->seen_ms    = now_ms;
    ne->used       = true;
    return true;
}

/* ---- relay decision (anchors only) ------------------------------------ */

bool alert_should_relay(const struct uwb_alert *a, uint8_t my_hop, bool is_gateway)
{
    if (is_gateway) {
        return false;
    }
    if (a->ttl == 0) {
        return false;
    }
    if (my_hop == UWB_ALERT_HOP_UNKNOWN) {
        return true;
    }
    return a->sender_hop > my_hop;
}

void alert_prepare_relay(struct uwb_alert *a, uint8_t my_hop)
{
    a->sender_hop = my_hop;
    a->ttl--;
}

/* ---- gateway latch ----------------------------------------------------- */

void alert_latch_reset(struct alert_latch *l)
{
    memset(l, 0, sizeof(*l));
}

static struct alert_latch_entry *latch_find_or_alloc(struct alert_latch *l,
                                                      const uint8_t eui[8],
                                                      uint32_t now_ms)
{
    int free_idx   = -1;
    int oldest_idx = 0;
    uint32_t oldest_ms = 0xFFFFFFFFu;

    for (int i = 0; i < ALERT_LATCH_N; i++) {
        struct alert_latch_entry *e = &l->e[i];

        if (e->used && memcmp(e->eui, eui, 8) == 0) {
            return e;
        }
        if (!e->used && free_idx < 0) {
            free_idx = i;
        }
        if (e->used && e->last_heard_ms < oldest_ms) {
            oldest_ms  = e->last_heard_ms;
            oldest_idx = i;
        }
    }

    int idx = (free_idx >= 0) ? free_idx : oldest_idx;
    struct alert_latch_entry *ne = &l->e[idx];

    memset(ne, 0, sizeof(*ne));
    memcpy(ne->eui, eui, 8);
    ne->used          = true;
    ne->last_heard_ms = now_ms;
    return ne;
}

alert_latch_res_t alert_latch_apply(struct alert_latch *l, const struct uwb_alert *a,
                                    uint32_t now_ms)
{
    struct alert_latch_entry *e = latch_find_or_alloc(l, a->orig_eui, now_ms);
    bool is_help = (a->state & UWB_ALERT_STATE_HELP) != 0;

    if (!is_help) {
        /* CANCEL: always clears, even with no prior HELP for this epoch. */
        e->active         = false;
        e->epoch          = a->epoch;
        e->have_epoch     = true;
        e->last_cancelled = a->epoch;
        e->have_cancelled = true;
        e->last_heard_ms  = now_ms;
        return ALERT_LATCH_CLEARED;
    }

    if (e->have_cancelled && a->epoch == e->last_cancelled) {
        /* Stale in-flight HELP for an already-cancelled epoch. */
        return ALERT_LATCH_IGNORED;
    }

    if (e->active && a->epoch == e->epoch) {
        e->last_heard_ms = now_ms;
        return ALERT_LATCH_REFRESHED;
    }

    if (!e->have_epoch || (int8_t)(a->epoch - e->epoch) > 0) {
        e->active        = true;
        e->epoch         = a->epoch;
        e->have_epoch    = true;
        e->last_heard_ms = now_ms;
        return ALERT_LATCH_RAISED;
    }

    /* Older/stale epoch that is neither the active one nor the cancelled
     * one: drop rather than move the latch backwards. */
    return ALERT_LATCH_IGNORED;
}
