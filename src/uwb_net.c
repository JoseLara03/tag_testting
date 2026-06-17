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

/* uwb_net_handle is implemented incrementally in Tasks 6-8. Temporary stub: */
uint32_t uwb_net_handle(struct uwb_net_ctx *c, const struct uwb_net_event *ev)
{
    (void)c; (void)ev;
    return UWB_ACT_NONE;
}
