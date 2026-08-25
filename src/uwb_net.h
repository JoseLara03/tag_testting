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

/* Superframes without appearing in the beacon's slot map after which the seat
 * is presumed gone.
 *
 * Contract v3 makes the slot map a SCHEDULE rather than an ownership table, so
 * absence is normal: a tag at the IDLE tier is legitimately absent from 24 of
 * every 25 maps. Treating one absence as a reclaim -- which is what v2 did --
 * would bounce every non-FAST tag to SCAN on its very first unscheduled
 * superframe.
 *
 * UWB_NET_LEASE_SF is the right threshold rather than an invented one: it is
 * exactly the window in which the gateway reclaims an unrenewed seat, so if the
 * gateway still held our seat it would have scheduled us inside it. It also
 * clears the slowest legitimate cadence (IDLE, 25 superframes) by 2x.
 *
 * This exists because `lease_remaining` is NOT a seat-loss detector, contrary
 * to what the design spec first claimed. Its only consumers are the keepalive
 * threshold -- which resets it to full OPTIMISTICALLY, whether or not the
 * gateway ever received the keepalive -- and the GRANT. Nothing drives a state
 * change off it, so the `!in_map` test was v2's ONLY way to notice a lost
 * seat, and deleting it without this counter would leave the tag sleeping and
 * self-renewing forever against a gateway that had long since forgotten it. */
#define UWB_NET_SCHED_GAP_MAX   UWB_NET_LEASE_SF
/* Must equal UWB_PROTO_VER in uwb_frame_802_15_4z.h -- the gateway stamps beacon
 * byte 10 with that one and the tag drops any beacon that does not match this
 * one. Bumping only the frame module leaves the tag deaf in SCAN with no
 * diagnostic. Pinned by tests/uwb_net/test_proto_ver_matches_frame_module. */
#define UWB_NET_PROTO_VER       2

typedef enum { UWB_TIER_IDLE = 0, UWB_TIER_SLOW = 1, UWB_TIER_FAST = 2 } uwb_tier_t;
#define UWB_TIER_COUNT  3

/* Per-tier duty cycle (design §6.1). These are two *different* cadences that
 * used to be one number, and conflating them is why the tier setting saved so
 * little: tier_cadence() gated UWB_ACT_RUN_SWEEP only, so the dominant costs --
 * the beacon RX and the DW3000 wake, paid on every superframe regardless of
 * tier -- were invisible to it.
 *
 *   listen_skip  superframes slept between beacon re-syncs. This is where the
 *                power is.
 *   range_every  participations between position fixes. This is where the
 *                position update rate is.
 *
 * range_every defaults to 1 in every tier: with listen_skip doing the work
 * there is no longer a reason to wake, re-sync and then not range -- the
 * re-sync is the expensive part and the sweep is already paid for once you are
 * awake. It stays for the case where the fix rate must be decoupled from the
 * seat-maintenance rate. */
struct uwb_tier_params {
    uint16_t listen_skip;
    uint16_t range_every;
};

/* Hard cap applied to listen_skip on read, until the gateway implements the
 * lease contract in design §7.
 *
 * UWB_NET_LEASE_SF is 50 superframes (10 s) and is renewed at half that, so a
 * tag that sleeps for more than 25 superframes cannot renew its lease: it
 * loses its seat on every skip and pays a full JOIN + GRANT + re-discovery
 * cycle each time, which costs more than the skip saves. Until the gateway
 * sizes the lease from a declared skip factor, no tier's design value works
 * against today's gateway: a skip of exactly 25 lands every renewal on its own
 * deadline, and on the bench a tag at "pwr tier f 25 1" emitted keepalives and
 * no position fixes at all. That is why the FAST default is 1 rather than the
 * design's 25 -- see tier_defaults in uwb_net.c.
 *
 * Clamped on read, not on write, so lifting this cap is a one-line edit that
 * does not require rewriting any stored value. */
#define UWB_LISTEN_SKIP_CAP  25u

/* Wall-clock interval between forced re-discovery rounds, milliseconds. Was
 * counted in participations, which becomes ~10 minutes once the IDLE tier
 * participates once a minute -- the anchor pool would go stale exactly where
 * the tag moves least and matters most. */
#define UWB_NET_REDISCOVER_INTERVAL_MS  60000u

/* Whether a participation must re-run discovery instead of sweeping, plus the
 * state that decision reads.
 *
 * Pure and here rather than inline in uwb_net_runner.c because inline it
 * LATCHED. The rediscover branch ran discovery but never refreshed the
 * sweep count it was itself gated on, so one sweep that came back short --
 * every anchor power-cycled, say -- left the tag re-discovering every
 * superframe forever: no sweep, therefore no UWB_EV_SWEPT, therefore the FSM
 * never fell back to UWB_ST_DISCOVER, therefore the only other writer of the
 * count was unreachable. UWB_ST_RANGING became terminal and only a tag reboot
 * cleared it. The runner's own comment described a recovery through
 * UWB_EV_SWEPT that its control flow made impossible.
 *
 * The threshold is UWB_NET_MIN_ANCHORS, deliberately the same symbol the FSM
 * compares n_anchors against: a private copy of "3" here is what let the gate
 * and the FSM disagree about whether the tag was making progress. */
struct uwb_sweep_gate {
    uint32_t last_discover_ms;
    uint8_t  last_sweep_n;
    bool     ever_discovered;
};

/* Fresh state: the first participation always discovers. */
void uwb_sweep_gate_init(struct uwb_sweep_gate *g);

/* True if this participation should discover rather than sweep. Pure read. */
bool uwb_sweep_gate_rediscover_due(const struct uwb_sweep_gate *g,
                                   uint32_t now_ms);

/* Record a discovery round that found n_found anchors. Refreshing the count
 * here is what keeps the gate from latching: a round that finds enough anchors
 * must let the very next participation sweep again. */
void uwb_sweep_gate_discovered(struct uwb_sweep_gate *g, uint32_t now_ms,
                               uint8_t n_found);

/* Record a completed sweep that ranged n_ranged anchors. */
void uwb_sweep_gate_swept(struct uwb_sweep_gate *g, uint8_t n_ranged);

/* Set/get a tier's parameters. The getter always applies UWB_LISTEN_SKIP_CAP
 * and never returns a range_every of 0. An out-of-range tier reads back the
 * FAST defaults rather than indexing off the end. Pure; not thread-safe --
 * the table is written from the command handler and read from the runner, and
 * both are aligned 16-bit fields. */
void uwb_net_set_tier_params(uwb_tier_t t, const struct uwb_tier_params *p);
void uwb_net_get_tier_params(uwb_tier_t t, struct uwb_tier_params *out);

/* Restore the compiled-in defaults (`pwr tier def`). */
void uwb_net_reset_tier_params(void);

/* Tier hysteresis (design §6.2). The LIS2HH12's own ACT_DUR gives a ~5 s
 * inactivity window, but the *activity* edge is immediate, so without this a
 * person shifting in a chair flaps the tag into FAST and back. */
#define UWB_TIER_HOLD_FAST_MS  30000u
#define UWB_TIER_HOLD_SLOW_MS  60000u
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
    uint8_t  map_slot;   /* CFP slot we were scheduled in (valid iff in_map) */
    /* GRANT */
    uint16_t g_short_addr;
    uint8_t  g_slot;
    uint8_t  g_tier;
    uint16_t g_lease;
    /* DISCOVERED / SWEPT */
    uint8_t  n_anchors;
    /* MOTION */
    uint8_t  req_tier;
    /* Set by the caller every superframe from tag_alert_active()-equivalent
     * state: true whenever the tag has a HELP/CANCEL frame it wants sent.
     * See UWB_ACT_SEND_ALERT's emission rule below. */
    bool     alert_pending;
};

/* Action bit-flags returned by uwb_net_handle (a superframe may need >1). */
#define UWB_ACT_NONE            0u
#define UWB_ACT_SEND_JOIN       (1u << 0)
#define UWB_ACT_SEND_KEEPALIVE  (1u << 1)
#define UWB_ACT_RUN_DISCOVER    (1u << 2)
#define UWB_ACT_RUN_SWEEP       (1u << 3)
#define UWB_ACT_SLEEP           (1u << 4)
#define UWB_ACT_TO_SCAN         (1u << 5)   /* lease lost this superframe */
/* Emitted per the design's alert emission rule (spec/2026-08-16-uwb-help-alert-
 * design.md §5): on UWB_EV_BEACON in JOINING/DISCOVER/RANGING (synced TX
 * window only -- never on a missed beacon there, since the tag would not
 * know where the CAP is); on BOTH UWB_EV_BEACON and UWB_EV_BEACON_MISS in
 * SCAN, where the tag has no sync to protect and no seat to lose. Deliberately
 * outside UWB_ACT_RANGING_MASK -- an uncalibrated tag must still call for
 * help. */
#define UWB_ACT_SEND_ALERT      (1u << 6)

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
    /* Seat identity, from the GRANT. Stable for the life of the seat, echoed
     * in KEEPALIVE so the gateway can sanity-check. NOT a slot index under
     * contract v3 -- GW_MAX_SEATS is 128, so this can exceed N_CFP. */
    uint8_t   seat_id;
    /* The CFP slot the beacon scheduled us in THIS superframe. Valid only for
     * the superframe it was read from, and the only value the runner may use
     * for TX timing -- using seat_id there would compute a slot offset far
     * outside the superframe once seat ids exceed N_CFP. */
    uint8_t   tx_slot;
    /* Frame counter of the last beacon that scheduled us, for
     * UWB_NET_SCHED_GAP_MAX. Compared with unsigned difference, so it is safe
     * across the gateway counter's 2^32 wrap. */
    uint32_t  last_sched_fc;
    uwb_tier_t tier;            /* granted tier */
    uwb_tier_t req_tier;        /* motion-driven request */
    uint16_t  lease_remaining;  /* superframes until expiry */
    uint8_t   miss_count;       /* consecutive beacon misses */
    uint8_t   join_retries;
    uint32_t  frame_counter;    /* last beacon's counter (last beacon seen) */
    uint8_t   n_anchors;        /* selected anchors after discovery */
    /* Participations since the last sweep. Counted, not derived from
     * frame_counter % cadence: once whole superframes are skipped the frame
     * counter is no longer a usable cadence reference -- it advances while the
     * tag is asleep, so a modulo test would fire on whichever superframe the
     * tag happened to wake in, or never. */
    uint16_t  part_count;
    /* uwb_net_tier_filter() state. Separate from `tier`, which is the tier the
     * gateway granted / the FSM is running; the filter's output is fed back in
     * as UWB_EV_MOTION by the caller. */
    uwb_tier_t filt_tier;
    uint32_t  last_active_ms;   /* last activity edge */
    uint32_t  tier_since_ms;    /* when filt_tier was entered */
};

void     uwb_net_init(struct uwb_net_ctx *c, const uint8_t eui[8]);
uint32_t uwb_net_handle(struct uwb_net_ctx *c, const struct uwb_net_event *ev);

/* Motion state -> the tier the runner should actually use. Call once per
 * superframe with the raw INT1-derived motion state and a monotonic ms clock;
 * `moving` true is an activity edge and always returns FAST immediately. The
 * demotions are held: FAST -> SLOW only once UWB_TIER_HOLD_FAST_MS has passed
 * since the last activity edge, and SLOW -> IDLE only after a further
 * UWB_TIER_HOLD_SLOW_MS of continued stillness. Wrap-safe in now_ms. */
uwb_tier_t uwb_net_tier_filter(struct uwb_net_ctx *c, bool moving, uint32_t now_ms);

#endif /* UWB_NET_H */
