#include "uwb_net.h"
#include <string.h>

/* Design §6.1 defaults. Runtime-settable so the constants can be tuned on
 * hardware (against a measured LFCLK tolerance, see design §4.3) rather than
 * by reflashing. Indexed by uwb_tier_t: IDLE = 0, SLOW = 1, FAST = 2. */
/* The default listen_skip of every tier IS that tier's period -- not a separate
 * number that happens to be near it. See the long comment on
 * UWB_LISTEN_SKIP_CAP in uwb_net.h: under contract v3 the gateway reserves
 * airtime per tier and schedules the tag once per tier period, so a skip longer
 * than the period sleeps through slots reserved for that tag and wastes
 * capacity other tags are now competing for.
 *
 * The design's values (300 / 75 / 1) were wrong twice over and the read-clamp
 * to UWB_LISTEN_SKIP_CAP hid half of it: IDLE's 300 clamped to 25, which
 * happens to equal the IDLE period, so IDLE looked fine by accident. SLOW's 75
 * also clamped to 25 -- against a SLOW period of 5, i.e. awake for one in every
 * five slots the gateway had reserved. A clamp is not agreement; deriving the
 * value from the period is.
 *
 * FAST was already 1 for a bench-measured reason that still holds: at a skip of
 * 25 the tag spent its participations on seat maintenance and emitted nothing
 * else. That reason is now the general rule rather than a FAST-only exception.
 *
 * `pwr tier` still overwrites these at runtime for experiments; these are the
 * values a tag boots and `pwr tier reset`s to. tests/uwb_net pins every default
 * against uwb_net_tier_listen_skip() so the two cannot drift apart. */
static const struct uwb_tier_params tier_defaults[UWB_TIER_COUNT] = {
    { UWB_NET_PERIOD_IDLE, 1u },   /* 25 superframes -> 5 s re-sync */
    { UWB_NET_PERIOD_SLOW, 1u },   /* 5 superframes -> 1 s */
    { UWB_NET_PERIOD_FAST, 1u },   /* every superframe */
};

static struct uwb_tier_params tier_params[UWB_TIER_COUNT] = {
    { UWB_NET_PERIOD_IDLE, 1u },
    { UWB_NET_PERIOD_SLOW, 1u },
    { UWB_NET_PERIOD_FAST, 1u },
};

uint16_t uwb_net_tier_period(uwb_tier_t tier)
{
    switch (tier) {
    case UWB_TIER_FAST: return UWB_NET_PERIOD_FAST;
    case UWB_TIER_SLOW: return UWB_NET_PERIOD_SLOW;
    case UWB_TIER_IDLE: return UWB_NET_PERIOD_IDLE;
    default:            return UWB_NET_PERIOD_IDLE;
    }
}

uint16_t uwb_net_tier_listen_skip(uwb_tier_t tier)
{
    uint16_t p = uwb_net_tier_period(tier);

    if (p > UWB_LISTEN_SKIP_CAP) p = UWB_LISTEN_SKIP_CAP;
    if (p == 0u) p = 1u;
    return p;
}

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

/* ---- sweep gate (see uwb_net.h) ---- */

void uwb_sweep_gate_init(struct uwb_sweep_gate *g)
{
    if (g == NULL) {
        return;
    }
    g->last_discover_ms = 0u;
    g->last_sweep_n     = 0u;
    g->ever_discovered  = false;
}

bool uwb_sweep_gate_rediscover_due(const struct uwb_sweep_gate *g,
                                   uint32_t now_ms)
{
    if (g == NULL) {
        return true;
    }
    /* Signed difference so the ms clock may wrap freely. */
    return !g->ever_discovered
           || ((int32_t)(now_ms - g->last_discover_ms)
               >= (int32_t)UWB_NET_REDISCOVER_INTERVAL_MS)
           || (g->last_sweep_n < UWB_NET_MIN_ANCHORS);
}

void uwb_sweep_gate_discovered(struct uwb_sweep_gate *g, uint32_t now_ms,
                               uint8_t n_found)
{
    if (g == NULL) {
        return;
    }
    g->last_discover_ms = now_ms;
    g->ever_discovered  = true;
    /* THE FIX. Without this the gate latches: see struct uwb_sweep_gate. */
    g->last_sweep_n     = n_found;
}

void uwb_sweep_gate_swept(struct uwb_sweep_gate *g, uint8_t n_ranged)
{
    if (g == NULL) {
        return;
    }
    g->last_sweep_n = n_ranged;
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
            c->seat_id         = ev->g_slot;
            c->tier            = (uwb_tier_t)ev->g_tier;
            c->lease_remaining = ev->g_lease;
            c->miss_count      = 0;
            /* Start the unscheduled-gap clock now. Left at 0 it would already
             * look like a 4-billion-superframe gap against any live gateway
             * counter and drop the tag straight back to SCAN. */
            c->last_sched_fc   = c->frame_counter;
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
            if (ev->in_map) {
                c->tx_slot       = ev->map_slot;
                c->last_sched_fc = ev->frame_counter;
            } else if ((uint32_t)(ev->frame_counter - c->last_sched_fc) >
                       UWB_NET_SCHED_GAP_MAX) {
                c->state = UWB_ST_SCAN;
                return UWB_ACT_TO_SCAN;
            }

            /* The seat's lease ages every superframe on the gateway, including
             * while we are still discovering.  Renew it here too — otherwise a
             * tag that needs more than the lease to gather >= MIN_ANCHORS is
             * reclaimed and bounced back to SCAN before it can ever range.
             *
             * KEEPALIVE is CAP traffic and owes nothing to the CFP schedule, so
             * it must still go out in a superframe we were NOT scheduled in --
             * otherwise a tag at a slow tier stops renewing exactly while it
             * waits to be scheduled, and the gateway reclaims the seat it is
             * waiting on. */
            uint32_t act = 0;
            if (c->lease_remaining <= (UWB_NET_LEASE_SF / 2)) {
                act |= UWB_ACT_SEND_KEEPALIVE;
                c->lease_remaining = UWB_NET_LEASE_SF;   /* optimistic renew */
            }
            if (!ev->in_map) {
                return act | UWB_ACT_SLEEP;
            }
            return act | UWB_ACT_RUN_DISCOVER;   /* retry until >= MIN_ANCHORS */
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
            if (ev->in_map) {
                c->tx_slot       = ev->map_slot;
                c->last_sched_fc = ev->frame_counter;
            } else if ((uint32_t)(ev->frame_counter - c->last_sched_fc) >
                       UWB_NET_SCHED_GAP_MAX) {
                c->state = UWB_ST_SCAN;
                return UWB_ACT_TO_SCAN;
            }

            uint32_t act = 0;
            if (c->lease_remaining <= (UWB_NET_LEASE_SF / 2)) {
                act |= UWB_ACT_SEND_KEEPALIVE;
                c->lease_remaining = UWB_NET_LEASE_SF;   /* optimistic renew */
            }
            /* Not our turn this superframe: sleep rather than sweep. Under
             * contract v3 this is the ordinary case for every tier but FAST,
             * and it is what lets 11 slots serve 100 tags. */
            if (!ev->in_map) {
                return act | UWB_ACT_SLEEP;
            }
            /* Count participations, not frame counters -- see part_count in
             * uwb_net.h for why the modulo test had to go. */
            struct uwb_tier_params tp;

            uwb_net_get_tier_params(c->tier, &tp);
            c->part_count++;
            if (c->part_count >= tp.range_every) {
                c->part_count = 0;
                /* One cadence slot, two possible emissions -- never both.
                 * TWR sweep by default; a BLINK when the tag has been put in
                 * TDoA mode. */
                act |= c->blink_mode ? UWB_ACT_SEND_BLINK : UWB_ACT_RUN_SWEEP;
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
