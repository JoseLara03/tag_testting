#ifndef UWB_NET_H
#define UWB_NET_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Lifecycle/config constants (contract v2). */
/* Lease length, superframes. Raised 50 -> 75 on 2026-08-25 to match the
 * gateway's GW_LEASE_SF; the GRANT carries this value, so the two must agree.
 * See gw_core.h for the derivation -- in short, the lease, the tier period and
 * listen_skip are one parameter, and 50 put the slowest tier exactly on its own
 * renewal deadline. */
#define UWB_NET_LEASE_SF        75
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

/* BLINK mode's substitute for the schedule-gap check above. A blink-mode
 * tag is never `in_map` (Task 4B design section 1.3: sched[] is reserved/
 * zero in a BLINK cell), so it has no data-driven way to notice its seat
 * was reclaimed -- KEEPALIVE has no ack on the wire ("the contract has no
 * such frame", uwb_gateway.c) in EITHER mode, and TWR's detector works only
 * because presence in the beacon's rotation was an indirect confirmation
 * that a live seat still exists. Without a wire-protocol ack to replace
 * that, a blink-mode tag cannot tell "my last N keepalives silently failed
 * and the gateway reclaimed my seat" from "everything is fine" -- so this
 * bounds the exposure instead of trying to detect it: after this many
 * consecutive OPTIMISTIC keepalive renewals (uwb_net.c resets
 * lease_remaining to full on every SEND, not on confirmation, so it can
 * cycle forever without ever really reaching zero) with no intervening
 * JOIN, force a rejoin anyway. Caught on the bench 2026-08-31: a tag whose
 * seat had been silently reclaimed kept blinking on its stale seat_id
 * forever with zero warning, and gw_core_join() hands that same seat_id to
 * the next fresh joiner -- two tags transmitting in the identical
 * blink_sched slot, indefinitely, with nothing to end it.
 *
 * 3 cycles * (UWB_NET_LEASE_SF / 2) is the same order of magnitude as
 * UWB_NET_SCHED_GAP_MAX (75 SF ~= 15 s) that TWR relies on, so a blink tag
 * loses at most ~22 s of position reporting during the forced rejoin
 * instead of colliding forever. This is a stopgap, not a fix: the real fix
 * is a wire-level KEEPALIVE ack so lease renewal has ground truth, which is
 * out of scope for Task 4B and left for later. */
#define UWB_NET_BLINK_KA_CYCLES_MAX  3u
/* Must equal UWB_PROTO_VER in uwb_frame_802_15_4z.h -- the gateway stamps beacon
 * byte 10 with that one and the tag drops any beacon that does not match this
 * one. Bumping only the frame module leaves the tag deaf in SCAN with no
 * diagnostic. Pinned by tests/uwb_net/test_proto_ver_matches_frame_module.
 *
 * Bumped 3 -> 4 for Task 4B, the BLINK slotted MAC (see the anchor repo's
 * docs/superpowers/specs/2026-08-30-blink-slotted-mac-design.md section 1.3
 * and CLAUDE.md entry on UWB_PROTO_VER). The beacon's wire format does not
 * change; a v4 gateway running BLINK mode transmits sched[] reserved/zero and
 * assigns airtime by seat_id identity instead, and a v3 tag would otherwise
 * have no way to notice that reinterpretation. Same flag-day consequence as
 * the 2 -> 3 bump: this firmware is deaf in SCAN against a v3 (pre-Task-4B)
 * gateway from this point on, and both firmwares must be reflashed together. */
#define UWB_NET_PROTO_VER       4

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

/* ---- listen_skip, the tier period, and the lease are ONE parameter ------
 *
 * The cap stays, and the reason it stays is worth stating because the scale
 * design's task list predicted the opposite. Separating a tag's SEAT from the
 * SLOT it transmits in (contract v3) did not remove this constraint: the
 * gateway still ages every lease once per superframe and reclaims at zero, so a
 * tag that sleeps past its renewal deadline still loses its seat, exactly as
 * before. Verified against gw_core.c rather than assumed.
 *
 * What contract v3 DID add is a second, different constraint that did not exist
 * before, and it binds tighter than the lease:
 *
 *   The gateway now RESERVES airtime according to the granted tier and
 *   schedules the tag once per tier period. A tag that sleeps longer than its
 *   tier period sleeps through slots reserved for it -- wasting capacity that
 *   now genuinely costs other tags. Under the old code a tag owned its slot
 *   permanently, so oversleeping cost nothing and the two cadences could drift
 *   apart unnoticed.
 *
 * So the design's listen_skip values (300 / 75 / 1) are wrong twice over: too
 * long for the lease, AND far longer than the tier periods (25 / 5 / 1) the
 * gateway schedules against. The three numbers are one number:
 *
 *     participate every N superframes
 *       => tier period  = N
 *       => listen_skip  = N
 *       => lease        > 2N + margin
 *
 * uwb_net_tier_listen_skip() derives the skip from the granted tier so they
 * cannot disagree, and tests/uwb_net pins the lease inequality for every tier.
 * Deep skipping (N in the hundreds) needs a lease sized from a declared skip
 * factor AND a tier period to match -- a gateway change, not a tag one, and out
 * of scope for contract v3.
 *
 * The cap equals the slowest tier's period, which is what makes it a
 * consequence rather than an arbitrary clamp. Applied on read, not on write, so
 * changing it needs no stored value rewritten. */
#define UWB_LISTEN_SKIP_CAP  25u

/* Superframes between participations for each tier. MUST match the gateway's
 * tier_period[] in gw_core.c -- it is the cadence the beacon's slot map is
 * built from, so a disagreement means the tag sleeps through slots reserved for
 * it or wakes for slots that were never coming. */
#define UWB_NET_PERIOD_FAST     1u
#define UWB_NET_PERIOD_SLOW     5u
#define UWB_NET_PERIOD_IDLE     25u

/* Margin between the wake cadence and the renewal deadline; mirrors the
 * gateway's GW_LEASE_MARGIN_SF. */
#define UWB_NET_LEASE_MARGIN_SF 4u

/* Superframes between participations for `tier`. Unknown tiers read as IDLE --
 * the slowest, never the fastest, so a corrupt grant cannot make a tag
 * transmit more often than it was allowed. Pure. */
uint16_t uwb_net_tier_period(uwb_tier_t tier);

/* The listen-skip a tag at `tier` should actually use: its tier period, capped
 * by UWB_LISTEN_SKIP_CAP and never zero. Derived rather than configured, so the
 * wake cadence cannot drift away from the cadence the gateway schedules. Pure.
 *
 * `pwr tier` still sets listen_skip directly for experiments; this is what the
 * runner should use in normal operation. */
uint16_t uwb_net_tier_listen_skip(uwb_tier_t tier);

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
/* TDoA BLINK (0xF0). Emitted INSTEAD OF UWB_ACT_RUN_SWEEP, in the exact same
 * cadence slot, when ctx.blink_mode is set -- never alongside it. The default
 * is false, so a tag that is not told otherwise behaves bit for bit as before
 * (TWR sweep + 0xEA POS).
 *
 * Deliberately outside UWB_ACT_RANGING_MASK: a BLINK is a one-way transmission
 * the tag does no arithmetic on. The tag's own TX antenna delay shifts every
 * anchor's reception by the same amount and therefore cancels in the range
 * DIFFERENCES the gateway solves, so an uncalibrated tag's BLINK is still
 * usable -- unlike its TWR sweep, whose absolute range is meaningless. */
#define UWB_ACT_SEND_BLINK      (1u << 7)

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
    /* TDoA transmit mode. When true the RANGING cadence emits
     * UWB_ACT_SEND_BLINK where it would otherwise emit UWB_ACT_RUN_SWEEP;
     * nothing else about the FSM changes and the seat protocol is untouched.
     * Set by the caller (the runner, from the persisted blink_cfg) before
     * each uwb_net_handle() call. Defaults to false via the zero-initialised
     * context, which is today's TWR behaviour. */
    bool      blink_mode;
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
    /* BLINK mode only: consecutive optimistic KEEPALIVE renewals since the
     * last JOIN, for UWB_NET_BLINK_KA_CYCLES_MAX. Reset on every fresh
     * GRANT; TWR mode never touches this (it has last_sched_fc instead). */
    uint8_t   blink_ka_cycles;
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
