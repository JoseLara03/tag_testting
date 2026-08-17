#ifndef TAG_ALERT_CORE_H
#define TAG_ALERT_CORE_H

/*
 * Pure tag-side alert state: epoch, active/cancelling, repeat scheduling.
 * No Zephyr, no NVS, no radio -- see src/tag_alert.c for the glue that wraps
 * this with a k_mutex and persists the epoch. Design: see
 * spec/2026-08-16-uwb-help-alert-design.md §3/§5.
 */

#include <stdint.h>
#include <stdbool.h>
#include "uwb_frame_802_15_4z.h"   /* UWB_ALERT_TTL_INIT */

/* Tuning constants (design §5 / plan Global Constraints), all in one place. */
#define ALERT_REPEAT_MS       10000u
#define ALERT_CANCEL_REPEATS  6u
#define ALERT_TTL_INIT        UWB_ALERT_TTL_INIT
#define ALERT_DEDUP_N         16
#define ALERT_DEDUP_AGE_MS    30000u

typedef enum { TAG_ALERT_OFF, TAG_ALERT_HELP, TAG_ALERT_CANCELLING } tag_alert_st_t;

struct tag_alert_core {
    tag_alert_st_t state;
    uint8_t  epoch;
    uint8_t  repeat_seq;
    uint8_t  cancels_left;
    uint32_t next_tx_ms;
    bool     armed;        /* a TX is due */
};

void tag_alert_core_init(struct tag_alert_core *c, uint8_t epoch_from_nvs);
/* Returns true if the epoch advanced (caller must persist it). A raise while
 * already in HELP is a no-op returning false -- a second double-press must
 * not burn an epoch or restart the repeat count. */
bool tag_alert_core_raise(struct tag_alert_core *c, uint32_t now_ms);
/* No-op unless currently in HELP. Epoch is left unchanged -- the CANCEL
 * names the epoch it is cancelling. */
void tag_alert_core_cancel(struct tag_alert_core *c, uint32_t now_ms);
/* Called each superframe. true => build and transmit a frame with *out. */
bool tag_alert_core_due(struct tag_alert_core *c, uint32_t now_ms,
                        uint8_t *state_out, uint8_t *epoch_out, uint8_t *rep_out);
/* Call after a successful TX: advances repeat_seq / counts down cancels. */
void tag_alert_core_sent(struct tag_alert_core *c, uint32_t now_ms);

#endif /* TAG_ALERT_CORE_H */
