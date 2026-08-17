#include "scan_backoff_core.h"

/* Rung 0 is deliberately short: a lost beacon is most often transient (a
 * doorway, a body block) and re-acquiring within 10 s is worth ~200 uA for the
 * few seconds it lasts. Rung 2 is the maintainer's "no beacon for ~2 min"
 * instinct. Rungs 4 and 5 are a tag in a drawer -- at 15/30 minutes the probe
 * costs less than the accelerometer does. */
static const uint32_t rung_ms[SCAN_BACKOFF_RUNGS] = {
    10u * 1000u,
    30u * 1000u,
    2u  * 60u * 1000u,
    5u  * 60u * 1000u,
    15u * 60u * 1000u,
    30u * 60u * 1000u,
};

void scan_backoff_reset(struct scan_backoff *b)
{
    b->rung = 0;
    /* alert_pinned is a standing condition owned by the alert state, not
     * something a coverage event gets to clear. */
}

void scan_backoff_fail(struct scan_backoff *b)
{
    if (b->alert_pinned) {
        return;
    }
    if (b->rung < SCAN_BACKOFF_RUNGS - 1u) {
        b->rung++;
    }
}

void scan_backoff_motion(struct scan_backoff *b)
{
    b->rung = 0;
}

void scan_backoff_alert(struct scan_backoff *b, bool active)
{
    b->alert_pinned = active;
    if (active) {
        b->rung = 0;
    }
}

uint32_t scan_backoff_next_ms(const struct scan_backoff *b)
{
    uint8_t r = (b->rung < SCAN_BACKOFF_RUNGS) ? b->rung
                                               : (uint8_t)(SCAN_BACKOFF_RUNGS - 1u);

    return rung_ms[r];
}

uint8_t scan_backoff_rung(const struct scan_backoff *b)
{
    return b->rung;
}
