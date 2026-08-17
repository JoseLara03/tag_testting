#ifndef ALERT_RELAY_H
#define ALERT_RELAY_H

/*
 * Pure dedup cache, gradient relay decision, and gateway latch ordering for
 * the 0xEB ALERT frame. See spec/2026-08-16-uwb-help-alert-design.md §3/§6.
 *
 * This module is meant to be compiled into BOTH the tag and the anchor/
 * gateway firmware unchanged -- the epoch-ordering rules in §3 and the relay
 * rules in §6 are exactly the kind of thing that drifts between two
 * independently written implementations. Do not hand-reimplement any of this
 * on the anchor.
 */

#include <stdint.h>
#include <stdbool.h>
#include "uwb_frame_802_15_4z.h"   /* struct uwb_alert */

/* Dedup cache sizing (design §6.2). */
#define ALERT_DEDUP_N       16
#define ALERT_DEDUP_AGE_MS  30000u

/* Gateway latch table sizing: one entry per originator EUI the gateway has
 * ever seen an alert from. Not part of the design's tuning-constant list
 * (that list is tag-side); sized generously for a bench/small-fleet gateway. */
#define ALERT_LATCH_N       32

/* --- duplicate suppression --- */
struct alert_dedup_entry {
    uint8_t  eui[8];
    uint8_t  epoch, state, repeat_seq;
    uint32_t seen_ms;
    bool     used;
};
struct alert_dedup {
    struct alert_dedup_entry e[ALERT_DEDUP_N];
};

void alert_dedup_reset(struct alert_dedup *d);
/* true = first time seen (caller proceeds); false = duplicate, drop. Inserts
 * on true. Key is (eui, epoch, state, repeat_seq); entries older than
 * ALERT_DEDUP_AGE_MS are free for reuse; a full cache evicts the oldest. */
bool alert_dedup_admit(struct alert_dedup *d, const struct uwb_alert *a, uint32_t now_ms);

/* --- relay decision (anchors only) --- */
bool alert_should_relay(const struct uwb_alert *a, uint8_t my_hop, bool is_gateway);
/* Rewrites sender_hop/ttl in place for the outgoing copy. Call only if
 * alert_should_relay() returned true. Touches nothing else in *a. */
void alert_prepare_relay(struct uwb_alert *a, uint8_t my_hop);

/* --- gateway latch --- */
struct alert_latch_entry {
    uint8_t  eui[8];
    uint8_t  epoch, last_cancelled;
    bool     active, have_cancelled;
    uint32_t last_heard_ms;
    bool     used;        /* slot allocated */
    bool     have_epoch;  /* epoch has been set by at least one HELP/CANCEL --
                            * without this, a brand-new entry's first HELP at
                            * a high epoch value would lose the serial-newer
                            * compare against the implicit epoch==0 default. */
};
struct alert_latch {
    struct alert_latch_entry e[ALERT_LATCH_N];
};

typedef enum {
    ALERT_LATCH_IGNORED,
    ALERT_LATCH_RAISED,
    ALERT_LATCH_REFRESHED,
    ALERT_LATCH_CLEARED,
} alert_latch_res_t;

void alert_latch_reset(struct alert_latch *l);
/* Epoch comparison is serial arithmetic, (int8_t)(a - b) > 0 -- see design
 * §3. HELP for epoch == last_cancelled -> IGNORED. HELP matching an active
 * epoch -> REFRESHED (last-heard timestamp only). A newer HELP -> RAISED.
 * CANCEL -> CLEARED and records last_cancelled, even with no prior HELP for
 * that epoch. */
alert_latch_res_t alert_latch_apply(struct alert_latch *l, const struct uwb_alert *a,
                                    uint32_t now_ms);

#endif /* ALERT_RELAY_H */
