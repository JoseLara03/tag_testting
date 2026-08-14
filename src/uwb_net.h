#ifndef UWB_NET_H
#define UWB_NET_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Lifecycle/config constants (contract v2). */
#define UWB_NET_LEASE_SF        50   /* lease length, superframes */
#define UWB_NET_MISS_MAX        3    /* M: consecutive beacon misses -> lost */
#define UWB_NET_JOIN_RETRY_MAX  4    /* N: join attempts -> back to scan */
#define UWB_NET_MIN_ANCHORS     3    /* need >=3 for a 2D fix */
/* Must equal UWB_PROTO_VER in uwb_frame_802_15_4z.h -- the gateway stamps beacon
 * byte 10 with that one and the tag drops any beacon that does not match this
 * one. Bumping only the frame module leaves the tag deaf in SCAN with no
 * diagnostic. Pinned by tests/uwb_net/test_proto_ver_matches_frame_module. */
#define UWB_NET_PROTO_VER       2

typedef enum { UWB_TIER_IDLE = 0, UWB_TIER_SLOW = 1, UWB_TIER_FAST = 2 } uwb_tier_t;
typedef enum { UWB_ST_SCAN = 0, UWB_ST_JOINING, UWB_ST_DISCOVER, UWB_ST_RANGING } uwb_net_state_t;

typedef enum {
    UWB_EV_BEACON = 0,   /* valid beacon received this superframe */
    UWB_EV_BEACON_MISS,  /* no valid beacon this superframe */
    UWB_EV_GRANT,        /* a grant addressed to us (EUI matched) */
    UWB_EV_GRANT_MISS,   /* CAP window elapsed, no grant */
    UWB_EV_DISCOVERED,   /* discovery sweep completed */
    UWB_EV_SWEPT,        /* a ranging sweep completed */
    UWB_EV_MOTION        /* motion module reports a tier request */
} uwb_ev_kind_t;

struct uwb_net_event {
    uwb_ev_kind_t kind;
    /* BEACON */
    uint8_t  proto_ver;
    uint32_t frame_counter;
    bool     in_map;     /* our short addr present in the slot map */
    uint8_t  map_slot;   /* slot index we occupy (valid iff in_map) */
    /* GRANT */
    uint16_t g_short_addr;
    uint8_t  g_slot;
    uint8_t  g_tier;
    uint16_t g_lease;
    /* DISCOVERED / SWEPT */
    uint8_t  n_anchors;
    /* MOTION */
    uint8_t  req_tier;
};

/* Action bit-flags returned by uwb_net_handle (a superframe may need >1). */
#define UWB_ACT_NONE            0u
#define UWB_ACT_SEND_JOIN       (1u << 0)
#define UWB_ACT_SEND_KEEPALIVE  (1u << 1)
#define UWB_ACT_RUN_DISCOVER    (1u << 2)
#define UWB_ACT_RUN_SWEEP       (1u << 3)
#define UWB_ACT_SLEEP           (1u << 4)
#define UWB_ACT_TO_SCAN         (1u << 5)   /* lease lost this superframe */

/* Actions whose result depends on the antenna delay. Cleared when the tag has
 * no valid antenna calibration -- see uwb_net_gate_actions().
 *
 * UWB_ACT_RUN_DISCOVER is deliberately NOT in the mask. Discovery does no TWR
 * at all — it broadcasts E2 and collects E4s — so it never touches the antenna
 * delay and produces no range to be wrong. Gating it would also pin an
 * uncalibrated tag in UWB_ST_DISCOVER forever, because the only path to
 * UWB_ST_RANGING is UWB_EV_DISCOVERED, which the runner only emits inside the
 * gated DISCOVER block. That in turn makes UWB_ACT_SLEEP unreachable (it is
 * emitted only from UWB_ST_RANGING), so the radio would never deep-sleep: a
 * regression from ~56 mA back to ~66 mA for every uncalibrated tag — and a
 * phy_option change invalidates the record for a whole fleet at once. */
#define UWB_ACT_RANGING_MASK    (UWB_ACT_RUN_SWEEP)

/* Filter an action word from uwb_net_handle() against calibration validity.
 * Without a valid record the antenna delays are meaningless, so ranging is
 * suppressed while everything that holds the tag's seat is left alone. Pure. */
uint32_t uwb_net_gate_actions(uint32_t act, bool cal_valid);

struct uwb_net_ctx {
    uwb_net_state_t state;
    uint8_t   eui[8];
    uint16_t  short_addr;
    uint8_t   slot_index;
    uwb_tier_t tier;            /* granted tier */
    uwb_tier_t req_tier;        /* motion-driven request */
    uint16_t  lease_remaining;  /* superframes until expiry */
    uint8_t   miss_count;       /* consecutive beacon misses */
    uint8_t   join_retries;
    uint32_t  frame_counter;    /* last beacon's counter (cadence ref) */
    uint8_t   n_anchors;        /* selected anchors after discovery */
};

void     uwb_net_init(struct uwb_net_ctx *c, const uint8_t eui[8]);
bool     uwb_tier_due(uwb_tier_t tier, uint32_t frame_counter);
uint32_t uwb_net_handle(struct uwb_net_ctx *c, const struct uwb_net_event *ev);

#endif /* UWB_NET_H */
