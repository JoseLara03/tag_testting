#include "uwb_net.h"
#include <string.h>

/* Design §6.1 defaults. Runtime-settable so the constants can be tuned on
 * hardware (against a measured LFCLK tolerance, see design §4.3) rather than
 * by reflashing. Indexed by uwb_tier_t: IDLE = 0, SLOW = 1, FAST = 2. */
static const struct uwb_tier_params tier_defaults[UWB_TIER_COUNT] = {
    { 300u, 1u },   /* IDLE: 60 s re-sync */
    {  75u, 1u },   /* SLOW: 15 s */
    {  25u, 1u },   /* FAST:  5 s */
};

static struct uwb_tier_params tier_params[UWB_TIER_COUNT] = {
    { 300u, 1u },
    {  75u, 1u },
    {  25u, 1u },
};

void uwb_net_reset_tier_params(void)
{
    memcpy(tier_params, tier_defaults, sizeof(tier_params));
}

void uwb_net_set_tier_params(uwb_tier_t t, const struct uwb_tier_params *p)
{
    if ((unsigned)t >= UWB_TIER_COUNT || p == NULL) {
        return;
    }
    tier_params[t] = *p;
}

void uwb_net_get_tier_params(uwb_tier_t t, struct uwb_tier_params *out)
{
    if (out == NULL) {
        return;
    }
    /* An unknown tier reads back FAST -- the safe direction: shortest skip,
     * every participation ranges. */
    *out = ((unsigned)t < UWB_TIER_COUNT) ? tier_params[t]
                                          : tier_params[UWB_TIER_FAST];

    if (out->listen_skip > UWB_LISTEN_SKIP_CAP) {
        out->listen_skip = UWB_LISTEN_SKIP_CAP;
    }
    if (out->listen_skip == 0u) {
        out->listen_skip = 1u;
    }
    if (out->range_every == 0u) {
        out->range_every = 1u;
    }
}

uwb_tier_t uwb_net_tier_filter(struct uwb_net_ctx *c, bool moving, uint32_t now_ms)
{
    if (moving) {
        /* Promotion is never held: a tag that has started moving should be
         * ranging at the moving cadence now, not in 30 s. */
        c->last_active_ms = now_ms;
        if (c->filt_tier != UWB_TIER_FAST) {
            c->filt_tier     = UWB_TIER_FAST;
            c->tier_since_ms = now_ms;
        }
        return c->filt_tier;
    }

    /* Demotions are held. Signed differences so the ms clock may wrap. */
    if (c->filt_tier == UWB_TIER_FAST) {
        if ((int32_t)(now_ms - c->last_active_ms) >= (int32_t)UWB_TIER_HOLD_FAST_MS) {
            c->filt_tier     = UWB_TIER_SLOW;
            c->tier_since_ms = now_ms;
        }
    } else if (c->filt_tier == UWB_TIER_SLOW) {
        if ((int32_t)(now_ms - c->tier_since_ms) >= (int32_t)UWB_TIER_HOLD_SLOW_MS) {
            c->filt_tier     = UWB_TIER_IDLE;
            c->tier_since_ms = now_ms;
        }
    }
    return c->filt_tier;
}

void uwb_net_init(struct uwb_net_ctx *c, const uint8_t eui[8])
{
    memset(c, 0, sizeof(*c));
    memcpy(c->eui, eui, 8);
    c->state     = UWB_ST_SCAN;
    c->tier      = UWB_TIER_FAST;
    c->req_tier  = UWB_TIER_FAST;
    c->filt_tier = UWB_TIER_FAST;
}

static uint32_t uwb_net_handle_core(struct uwb_net_ctx *c, const struct uwb_net_event *ev);

/* Age the seat's lease by the superframes that actually elapsed since the last
 * beacon we saw, not by one.
 *
 * The gateway ages every lease once per superframe whether or not the tag was
 * listening. Decrementing by one per *received* beacon was correct only while
 * the tag participated in every superframe; once it skips, the renewal
 * threshold (LEASE_SF/2) arrives listen_skip times too late -- at
 * listen_skip = 25 the tag would renew after 625 superframes against a lease
 * the gateway reclaims at 50, so it would lose its seat on the very first
 * skip and pay a full JOIN + GRANT + re-discovery to get it back. That costs
 * far more than the skip saves, and it is invisible in the FSM's own state:
 * the tag simply sees `in_map == false` and reports RESCAN seat.
 *
 * Must be called before c->frame_counter is advanced. */
static void lease_age(struct uwb_net_ctx *c, const struct uwb_net_event *ev)
{
    /* Wrap-safe: both counters are uint32 and the gateway's wraps. A backwards
     * jump (gateway restart) produces a huge value and zeroes the lease, which
     * forces a keepalive -- the right answer for a gateway that just rebooted. */
    uint32_t elapsed = ev->frame_counter - c->frame_counter;

    if (c->lease_remaining > elapsed) {
        c->lease_remaining -= (uint16_t)elapsed;
    } else {
        c->lease_remaining = 0;
    }
}

/* UWB_ACT_SEND_ALERT emission rule (design §5): synced states (JOINING /
 * DISCOVER / RANGING) only fire on an actual UWB_EV_BEACON -- never on a
 * miss, because without the beacon the tag does not know where the CAP is
 * and a blind TX could land on the beacon or in someone's CFP slot. SCAN has
 * no sync to protect and no seat to lose, so it fires on both BEACON and
 * BEACON_MISS. `st` is the state the event was handled in (captured before
 * uwb_net_handle() lets the switch below mutate c->state). */
static uint32_t alert_action(uwb_net_state_t st, const struct uwb_net_event *ev)
{
    if (!ev->alert_pending) {
        return UWB_ACT_NONE;
    }
    if (st == UWB_ST_SCAN) {
        if (ev->kind == UWB_EV_BEACON || ev->kind == UWB_EV_BEACON_MISS) {
            return UWB_ACT_SEND_ALERT;
        }
        return UWB_ACT_NONE;
    }
    if (ev->kind == UWB_EV_BEACON) {
        return UWB_ACT_SEND_ALERT;
    }
    return UWB_ACT_NONE;
}

uint32_t uwb_net_handle(struct uwb_net_ctx *c, const struct uwb_net_event *ev)
{
    uwb_net_state_t st0 = c->state;
    uint32_t act = uwb_net_handle_core(c, ev);

    return act | alert_action(st0, ev);
}

static uint32_t uwb_net_handle_core(struct uwb_net_ctx *c, const struct uwb_net_event *ev)
{
    /* Motion updates the requested tier and applies it locally so the ranging
     * cadence changes immediately without waiting for a gateway re-grant. */
    if (ev->kind == UWB_EV_MOTION) {
        c->req_tier = (uwb_tier_t)ev->req_tier;
        c->tier     = (uwb_tier_t)ev->req_tier;
        return UWB_ACT_NONE;
    }

    switch (c->state) {
    case UWB_ST_SCAN:
        if (ev->kind == UWB_EV_BEACON && ev->proto_ver == UWB_NET_PROTO_VER) {
            c->frame_counter = ev->frame_counter;
            c->miss_count = 0;
            c->join_retries = 0;
            c->state = UWB_ST_JOINING;
            return UWB_ACT_SEND_JOIN;
        }
        /* Out of coverage: sleep the radio until the next scheduled probe.
         * This state used to return UWB_ACT_NONE on everything, so an unjoined
         * tag held RX open across ~100% of every superframe and never deep-
         * slept -- straight back to ~66 mA, making out-of-coverage the tag's
         * *worst* power state (Open Work item 3). In SCAN the action means
         * "sleep until the next probe" rather than "until the next
         * superframe": the interval comes from scan_backoff_core, which the
         * runner owns. */
        return UWB_ACT_SLEEP;

    case UWB_ST_JOINING:
        if (ev->kind == UWB_EV_GRANT) {
            c->short_addr      = ev->g_short_addr;
            c->slot_index      = ev->g_slot;
            c->tier            = (uwb_tier_t)ev->g_tier;
            c->lease_remaining = ev->g_lease;
            c->miss_count      = 0;
            c->state           = UWB_ST_DISCOVER;
            return UWB_ACT_RUN_DISCOVER;
        }
        if (ev->kind == UWB_EV_GRANT_MISS) {
            if (++c->join_retries >= UWB_NET_JOIN_RETRY_MAX) {
                c->state = UWB_ST_SCAN;
                return UWB_ACT_NONE;
            }
            return UWB_ACT_SEND_JOIN;
        }
        if (ev->kind == UWB_EV_BEACON &&
            ev->proto_ver == UWB_NET_PROTO_VER) {
            c->miss_count = 0;
            c->frame_counter = ev->frame_counter;
            return UWB_ACT_SEND_JOIN;   /* retry each beacon until GRANT */
        }
        return UWB_ACT_NONE;

    case UWB_ST_DISCOVER:
        if (ev->kind == UWB_EV_DISCOVERED) {
            c->n_anchors = ev->n_anchors;
            if (ev->n_anchors >= UWB_NET_MIN_ANCHORS) {
                c->state = UWB_ST_RANGING;
                return UWB_ACT_NONE;
            }
            return UWB_ACT_RUN_DISCOVER;    /* retry */
        }
        if (ev->kind == UWB_EV_BEACON_MISS) {
            if (++c->miss_count >= UWB_NET_MISS_MAX) {
                c->state = UWB_ST_SCAN; c->miss_count = 0;
                return UWB_ACT_TO_SCAN;
            }
            return UWB_ACT_NONE;
        }
        if (ev->kind == UWB_EV_BEACON) {
            c->miss_count = 0;
            lease_age(c, ev);
            c->frame_counter = ev->frame_counter;
            if (!ev->in_map) {             /* gateway reclaimed our seat */
                c->state = UWB_ST_SCAN;
                return UWB_ACT_TO_SCAN;
            }
            c->slot_index = ev->map_slot;

            /* The seat's lease ages every superframe on the gateway, including
             * while we are still discovering.  Renew it here too — otherwise a
             * tag that needs more than the lease to gather >= MIN_ANCHORS is
             * reclaimed and bounced back to SCAN before it can ever range. */
            uint32_t act = UWB_ACT_RUN_DISCOVER;   /* retry until >= MIN_ANCHORS */
            if (c->lease_remaining <= (UWB_NET_LEASE_SF / 2)) {
                act |= UWB_ACT_SEND_KEEPALIVE;
                c->lease_remaining = UWB_NET_LEASE_SF;   /* optimistic renew */
            }
            return act;
        }
        return UWB_ACT_NONE;

    case UWB_ST_RANGING:
        if (ev->kind == UWB_EV_BEACON_MISS) {
            if (++c->miss_count >= UWB_NET_MISS_MAX) {
                c->state = UWB_ST_SCAN; c->miss_count = 0;
                return UWB_ACT_TO_SCAN;
            }
            return UWB_ACT_NONE;            /* never TX on a missed beacon */
        }
        if (ev->kind == UWB_EV_SWEPT) {
            if (ev->n_anchors < UWB_NET_MIN_ANCHORS) {
                c->state = UWB_ST_DISCOVER;
                return UWB_ACT_RUN_DISCOVER;
            }
            return UWB_ACT_NONE;
        }
        if (ev->kind == UWB_EV_BEACON) {
            c->miss_count = 0;
            lease_age(c, ev);
            c->frame_counter = ev->frame_counter;
            if (!ev->in_map) {              /* lease reclaimed */
                c->state = UWB_ST_SCAN;
                return UWB_ACT_TO_SCAN;
            }
            c->slot_index = ev->map_slot;

            uint32_t act = 0;
            if (c->lease_remaining <= (UWB_NET_LEASE_SF / 2)) {
                act |= UWB_ACT_SEND_KEEPALIVE;
                c->lease_remaining = UWB_NET_LEASE_SF;   /* optimistic renew */
            }
            /* Count participations, not frame counters -- see part_count in
             * uwb_net.h for why the modulo test had to go. */
            struct uwb_tier_params tp;

            uwb_net_get_tier_params(c->tier, &tp);
            c->part_count++;
            if (c->part_count >= tp.range_every) {
                c->part_count = 0;
                act |= UWB_ACT_RUN_SWEEP;
            } else {
                act |= UWB_ACT_SLEEP;
            }
            return act;
        }
        return UWB_ACT_NONE;

    default:
        return UWB_ACT_NONE;
    }
}

uint32_t uwb_net_gate_actions(uint32_t act, bool cal_valid)
{
    if (cal_valid) {
        return act;
    }
    return act & ~UWB_ACT_RANGING_MASK;
}
