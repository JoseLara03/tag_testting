#include "tag_alert_core.h"
#include <string.h>

void tag_alert_core_init(struct tag_alert_core *c, uint8_t epoch_from_nvs)
{
    memset(c, 0, sizeof(*c));
    c->state = TAG_ALERT_OFF;
    c->epoch = epoch_from_nvs;
}

bool tag_alert_core_raise(struct tag_alert_core *c, uint32_t now_ms)
{
    if (c->state == TAG_ALERT_HELP) {
        return false;   /* no-op: do not burn an epoch or restart repeats */
    }

    c->epoch++;
    c->repeat_seq = 0;
    c->state      = TAG_ALERT_HELP;
    c->next_tx_ms = now_ms;
    c->armed      = true;
    return true;
}

void tag_alert_core_cancel(struct tag_alert_core *c, uint32_t now_ms)
{
    if (c->state != TAG_ALERT_HELP) {
        return;   /* no-op from OFF or already CANCELLING */
    }

    c->state        = TAG_ALERT_CANCELLING;
    c->cancels_left = ALERT_CANCEL_REPEATS;
    c->repeat_seq   = 0;
    c->next_tx_ms   = now_ms;
    c->armed        = true;
    /* epoch intentionally unchanged -- the CANCEL names the epoch it cancels. */
}

bool tag_alert_core_due(struct tag_alert_core *c, uint32_t now_ms,
                        uint8_t *state_out, uint8_t *epoch_out, uint8_t *rep_out)
{
    if (c->state == TAG_ALERT_OFF) {
        c->armed = false;
        return false;
    }

    bool due = (int32_t)(now_ms - c->next_tx_ms) >= 0;
    c->armed = due;
    if (!due) {
        return false;
    }

    if (state_out) {
        *state_out = (c->state == TAG_ALERT_HELP) ? UWB_ALERT_STATE_HELP
                                                   : UWB_ALERT_STATE_CANCEL;
    }
    if (epoch_out) {
        *epoch_out = c->epoch;
    }
    if (rep_out) {
        *rep_out = c->repeat_seq;
    }
    return true;
}

void tag_alert_core_sent(struct tag_alert_core *c, uint32_t now_ms)
{
    c->repeat_seq++;
    c->next_tx_ms = now_ms + ALERT_REPEAT_MS;

    if (c->state == TAG_ALERT_CANCELLING) {
        if (--c->cancels_left == 0) {
            c->state = TAG_ALERT_OFF;
        }
    }
}
