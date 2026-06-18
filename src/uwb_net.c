#include "uwb_net.h"
#include <string.h>

static uint32_t tier_cadence(uwb_tier_t t)
{
    switch (t) {
    case UWB_TIER_FAST: return 1u;
    case UWB_TIER_SLOW: return 5u;
    default:            return 25u;   /* IDLE */
    }
}

bool uwb_tier_due(uwb_tier_t tier, uint32_t frame_counter)
{
    return (frame_counter % tier_cadence(tier)) == 0u;
}

void uwb_net_init(struct uwb_net_ctx *c, const uint8_t eui[8])
{
    memset(c, 0, sizeof(*c));
    memcpy(c->eui, eui, 8);
    c->state    = UWB_ST_SCAN;
    c->tier     = UWB_TIER_FAST;
    c->req_tier = UWB_TIER_FAST;
}

uint32_t uwb_net_handle(struct uwb_net_ctx *c, const struct uwb_net_event *ev)
{
    /* Motion is orthogonal: it only updates the requested tier. */
    if (ev->kind == UWB_EV_MOTION) {
        c->req_tier = (uwb_tier_t)ev->req_tier;
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
        return UWB_ACT_NONE;

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
            c->frame_counter = ev->frame_counter;
            if (!ev->in_map) {             /* gateway reclaimed our seat */
                c->state = UWB_ST_SCAN;
                return UWB_ACT_TO_SCAN;
            }
            return UWB_ACT_RUN_DISCOVER;   /* retry until >= MIN_ANCHORS */
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
            c->frame_counter = ev->frame_counter;
            if (!ev->in_map) {              /* lease reclaimed */
                c->state = UWB_ST_SCAN;
                return UWB_ACT_TO_SCAN;
            }
            c->slot_index = ev->map_slot;
            if (c->lease_remaining > 0) c->lease_remaining--;

            uint32_t act = 0;
            if (c->lease_remaining <= (UWB_NET_LEASE_SF / 2)) {
                act |= UWB_ACT_SEND_KEEPALIVE;
                c->lease_remaining = UWB_NET_LEASE_SF;   /* optimistic renew */
            }
            if (uwb_tier_due(c->tier, c->frame_counter)) {
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
