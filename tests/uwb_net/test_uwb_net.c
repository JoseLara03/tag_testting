#include "uwb_net.h"
#include "uwb_frame_802_15_4z.h"
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c); g_fail++; } } while(0)

static const uint8_t EUI[8] = {0xDE,0xAD,0xBE,0xEF,0,0,0,1};

static void test_init(void)
{
    struct uwb_net_ctx c;
    uwb_net_init(&c, EUI);
    CHECK(c.state == UWB_ST_SCAN);
    CHECK(c.req_tier == UWB_TIER_FAST);
    CHECK(c.filt_tier == UWB_TIER_FAST);
    CHECK(c.part_count == 0);
    CHECK(memcmp(c.eui, EUI, 8) == 0);
}

/* listen_skip is clamped on *read* so that lifting UWB_LISTEN_SKIP_CAP when the
 * gateway lease contract lands does not require rewriting any stored value. */
static void test_tier_params(void)
{
    struct uwb_tier_params p;

    uwb_net_reset_tier_params();

    /* Defaults, seen through the cap. FAST is 1, deliberately not the design's
     * 25: at 25 the tag only maintained its seat and never ranged. */
    uwb_net_get_tier_params(UWB_TIER_FAST, &p);
    CHECK(p.listen_skip == 1u && p.range_every == 1u);
    uwb_net_get_tier_params(UWB_TIER_SLOW, &p);
    CHECK(p.listen_skip == UWB_LISTEN_SKIP_CAP);   /* stored 75, capped to 25 */
    CHECK(p.range_every == 1u);
    uwb_net_get_tier_params(UWB_TIER_IDLE, &p);
    CHECK(p.listen_skip == UWB_LISTEN_SKIP_CAP);   /* stored 300, capped to 25 */

    /* The write is not clamped -- the stored value survives the cap. */
    struct uwb_tier_params set = { 300u, 4u };
    uwb_net_set_tier_params(UWB_TIER_IDLE, &set);
    uwb_net_get_tier_params(UWB_TIER_IDLE, &p);
    CHECK(p.listen_skip == UWB_LISTEN_SKIP_CAP);
    CHECK(p.range_every == 4u);

    /* Zeros are never handed out: they would mean "never wake" / "never range". */
    struct uwb_tier_params zero = { 0u, 0u };
    uwb_net_set_tier_params(UWB_TIER_SLOW, &zero);
    uwb_net_get_tier_params(UWB_TIER_SLOW, &p);
    CHECK(p.listen_skip == 1u && p.range_every == 1u);

    /* An out-of-range tier reads back FAST rather than indexing off the end. */
    uwb_net_get_tier_params((uwb_tier_t)99, &p);
    struct uwb_tier_params f;
    uwb_net_get_tier_params(UWB_TIER_FAST, &f);
    CHECK(p.listen_skip == f.listen_skip && p.range_every == f.range_every);

    uwb_net_reset_tier_params();
    uwb_net_get_tier_params(UWB_TIER_IDLE, &p);
    CHECK(p.range_every == 1u);
}

/* Design §6.2: promotion is immediate, both demotions are held. */
static void test_tier_filter(void)
{
    struct uwb_net_ctx c;
    uwb_net_init(&c, EUI);

    /* Activity edge -> FAST at once. */
    CHECK(uwb_net_tier_filter(&c, true, 1000u) == UWB_TIER_FAST);

    /* Still, but inside the FAST hold: stays FAST right up to the boundary. */
    CHECK(uwb_net_tier_filter(&c, false, 1000u + UWB_TIER_HOLD_FAST_MS - 1u)
          == UWB_TIER_FAST);
    /* Exactly at the boundary the demotion fires. */
    CHECK(uwb_net_tier_filter(&c, false, 1000u + UWB_TIER_HOLD_FAST_MS)
          == UWB_TIER_SLOW);

    uint32_t t_slow = 1000u + UWB_TIER_HOLD_FAST_MS;

    /* SLOW -> IDLE only after a further hold of continued stillness. */
    CHECK(uwb_net_tier_filter(&c, false, t_slow + UWB_TIER_HOLD_SLOW_MS - 1u)
          == UWB_TIER_SLOW);
    CHECK(uwb_net_tier_filter(&c, false, t_slow + UWB_TIER_HOLD_SLOW_MS)
          == UWB_TIER_IDLE);

    /* Any activity edge, from any rung, goes straight back to FAST. */
    CHECK(uwb_net_tier_filter(&c, true, t_slow + 1000000u) == UWB_TIER_FAST);

    /* Flapping: a person shifting in a chair must not bounce the tier. Each
     * activity edge re-arms the FAST hold, so a still sample between two
     * edges never demotes. */
    struct uwb_net_ctx cf;
    uwb_net_init(&cf, EUI);
    uint32_t t = 500u;
    for (int i = 0; i < 20; i++) {
        CHECK(uwb_net_tier_filter(&cf, true, t) == UWB_TIER_FAST);
        t += UWB_TIER_HOLD_FAST_MS - 1u;
        CHECK(uwb_net_tier_filter(&cf, false, t) == UWB_TIER_FAST);
        t += 1u;
    }

    /* Wrap-safe: the same sequence across the uint32 ms wrap. */
    struct uwb_net_ctx cw;
    uwb_net_init(&cw, EUI);
    uint32_t base = 0xFFFFFF00u;
    CHECK(uwb_net_tier_filter(&cw, true, base) == UWB_TIER_FAST);
    CHECK(uwb_net_tier_filter(&cw, false, base + UWB_TIER_HOLD_FAST_MS - 1u)
          == UWB_TIER_FAST);
    CHECK(uwb_net_tier_filter(&cw, false, base + UWB_TIER_HOLD_FAST_MS)
          == UWB_TIER_SLOW);
}

static struct uwb_net_event ev_beacon(uint32_t fc, bool in_map, uint8_t slot)
{
    struct uwb_net_event e; memset(&e, 0, sizeof(e));
    e.kind = UWB_EV_BEACON; e.proto_ver = UWB_NET_PROTO_VER;
    e.frame_counter = fc; e.in_map = in_map; e.map_slot = slot;
    return e;
}

static struct uwb_net_event ev_grant(uint16_t sa, uint8_t slot, uint8_t tier, uint16_t lease)
{
    struct uwb_net_event e; memset(&e, 0, sizeof(e));
    e.kind = UWB_EV_GRANT; e.g_short_addr = sa; e.g_slot = slot;
    e.g_tier = tier; e.g_lease = lease;
    /* All phases granted, unless a test overrides it -- the pre-Phase-3
     * behaviour of "every superframe is a participation", so tests not
     * specifically exercising phase gating are unaffected by its addition. */
    e.g_phase_mask = 0xFFFFu;
    return e;
}

static void test_scan_join(void)
{
    struct uwb_net_ctx c; uwb_net_init(&c, EUI);

    /* Wrong proto version: ignored, stays SCAN -- and sleeps rather than
     * holding the receiver open for the whole superframe. */
    struct uwb_net_event bad = ev_beacon(1, false, 0); bad.proto_ver = 99;
    CHECK(uwb_net_handle(&c, &bad) == UWB_ACT_SLEEP);
    CHECK(c.state == UWB_ST_SCAN);

    /* Valid beacon: -> JOINING, emit join. */
    struct uwb_net_event b = ev_beacon(5, false, 0);
    CHECK(uwb_net_handle(&c, &b) == UWB_ACT_SEND_JOIN);
    CHECK(c.state == UWB_ST_JOINING);

    /* GRANT_MISS retries up to N, then back to SCAN. */
    struct uwb_net_event miss; memset(&miss, 0, sizeof(miss)); miss.kind = UWB_EV_GRANT_MISS;
    for (int i = 1; i < UWB_NET_JOIN_RETRY_MAX; i++)
        CHECK(uwb_net_handle(&c, &miss) == UWB_ACT_SEND_JOIN);
    CHECK(uwb_net_handle(&c, &miss) == UWB_ACT_NONE);   /* Nth failure */
    CHECK(c.state == UWB_ST_SCAN);
}

static void test_grant_discover(void)
{
    struct uwb_net_ctx c; uwb_net_init(&c, EUI);
    struct uwb_net_event b = ev_beacon(0, false, 0);
    uwb_net_handle(&c, &b);                         /* -> JOINING */

    struct uwb_net_event g = ev_grant(0x0007, 3, UWB_TIER_FAST, 50);
    CHECK(uwb_net_handle(&c, &g) == UWB_ACT_RUN_DISCOVER);
    CHECK(c.state == UWB_ST_DISCOVER);
    CHECK(c.short_addr == 0x0007 && c.slot_index == 3);
    CHECK(c.tier == UWB_TIER_FAST && c.lease_remaining == 50);

    /* Discovery finds too few anchors -> retry. */
    struct uwb_net_event d; memset(&d, 0, sizeof(d));
    d.kind = UWB_EV_DISCOVERED; d.n_anchors = 2;
    CHECK(uwb_net_handle(&c, &d) == UWB_ACT_RUN_DISCOVER);
    CHECK(c.state == UWB_ST_DISCOVER);

    /* Enough anchors -> RANGING. */
    d.n_anchors = 4;
    CHECK(uwb_net_handle(&c, &d) == UWB_ACT_NONE);
    CHECK(c.state == UWB_ST_RANGING && c.n_anchors == 4);

    /* Lease reclaimed mid-DISCOVER (addr absent) -> SCAN. */
    struct uwb_net_ctx c2; uwb_net_init(&c2, EUI);
    uwb_net_handle(&c2, &b); uwb_net_handle(&c2, &g);   /* DISCOVER */
    struct uwb_net_event lost = ev_beacon(1, false, 0); /* in_map = false */
    CHECK(uwb_net_handle(&c2, &lost) == UWB_ACT_TO_SCAN);
    CHECK(c2.state == UWB_ST_SCAN);
}

/* Regression: a tag stuck in DISCOVER must keep renewing its lease, otherwise
 * the gateway reclaims its seat after UWB_NET_LEASE_SF superframes and the tag
 * is bounced back to SCAN (the ~10 s re-join cycle observed on hardware). */
static void test_discover_keeps_lease(void)
{
    struct uwb_net_ctx c; uwb_net_init(&c, EUI);
    struct uwb_net_event b = ev_beacon(0, false, 0);
    uwb_net_handle(&c, &b);                              /* -> JOINING */
    struct uwb_net_event g = ev_grant(0x0007, 3, UWB_TIER_FAST, UWB_NET_LEASE_SF);
    uwb_net_handle(&c, &g);                              /* -> DISCOVER, lease=50 */
    CHECK(c.state == UWB_ST_DISCOVER);

    /* Drive far more beacons than the lease while discovery never reaches
     * MIN_ANCHORS. The tag must stay in DISCOVER and must emit a keepalive
     * before the lease could have expired. */
    bool saw_keepalive = false;
    for (int i = 0; i < UWB_NET_LEASE_SF * 3; i++) {
        struct uwb_net_event bi = ev_beacon((uint32_t)(i + 1), true, 3);
        uint32_t a = uwb_net_handle(&c, &bi);
        CHECK(a & UWB_ACT_RUN_DISCOVER);
        CHECK(c.state == UWB_ST_DISCOVER);   /* never bounced to SCAN */
        if (a & UWB_ACT_SEND_KEEPALIVE) saw_keepalive = true;
        /* Local lease must never reach 0 (would mean gateway reclaim). */
        CHECK(c.lease_remaining > 0);
    }
    CHECK(saw_keepalive);
    CHECK(c.slot_index == 3);
}

/* Helper: drive a fresh ctx into RANGING with the given tier. */
static void to_ranging(struct uwb_net_ctx *c, uwb_tier_t tier)
{
    uwb_net_init(c, EUI);
    struct uwb_net_event b = ev_beacon(0, false, 0); uwb_net_handle(c, &b);
    struct uwb_net_event g = ev_grant(0x0007, 3, tier, UWB_NET_LEASE_SF); uwb_net_handle(c, &g);
    struct uwb_net_event d; memset(&d, 0, sizeof(d)); d.kind = UWB_EV_DISCOVERED; d.n_anchors = 4;
    uwb_net_handle(c, &d);   /* -> RANGING */

    /* Existing keepalive-on-low-lease tests predate part_since_pos gating
     * (Task 12) and assert on the lease threshold alone; preset the count
     * past UWB_NET_KEEPALIVE_AFTER_N so they see the same behaviour as
     * before. Tests of the suppression itself reset this explicitly. */
    c->part_since_pos = UWB_NET_KEEPALIVE_AFTER_N;
}

/* Helpers for phase-mask (Phase 3, Task 11) tests. */
static struct uwb_net_event ev_grant_phase(uint16_t sa, uint8_t slot, uint8_t tier,
                                           uint16_t lease, uint16_t phase_mask)
{
    struct uwb_net_event e = ev_grant(sa, slot, tier, lease);
    e.g_phase_mask = phase_mask;
    return e;
}

static void to_ranging_phase(struct uwb_net_ctx *c, uwb_tier_t tier, uint16_t phase_mask)
{
    uwb_net_init(c, EUI);
    struct uwb_net_event b = ev_beacon(0, false, 0); uwb_net_handle(c, &b);
    struct uwb_net_event g = ev_grant_phase(0x0007, 3, tier, UWB_NET_LEASE_SF, phase_mask);
    uwb_net_handle(c, &g);
    struct uwb_net_event d; memset(&d, 0, sizeof(d)); d.kind = UWB_EV_DISCOVERED; d.n_anchors = 4;
    uwb_net_handle(c, &d);   /* -> RANGING */
    c->part_since_pos = UWB_NET_KEEPALIVE_AFTER_N;   /* see to_ranging()'s comment */
}

/* A single granted phase participates exactly once per UWB_NET_CYCLE_C (16)
 * superframes -- fed only the beacons that phase would actually wake the tag
 * for (the runner's job, Task 12, is to never generate the others), the FSM
 * must never bounce to SCAN and must age the lease by the real elapsed gap
 * each time. */
static void test_single_phase_participates_once_per_cycle(void)
{
    const uint16_t mask = (uint16_t)(1u << 5);   /* phase 5 only */
    struct uwb_net_ctx c;
    to_ranging_phase(&c, UWB_TIER_FAST, mask);

    uint16_t lease_before = c.lease_remaining;
    for (int cycle = 0; cycle < 5; cycle++) {
        uint32_t fc = (uint32_t)cycle * UWB_NET_CYCLE_C + 5u;
        struct uwb_net_event b = ev_beacon(fc, true, 3);
        uint32_t a = uwb_net_handle(&c, &b);

        CHECK(!(a & UWB_ACT_TO_SCAN));
        CHECK(c.state == UWB_ST_RANGING);
        if (cycle > 0) {
            /* Elapsed is exactly one cycle between consecutive participations. */
            CHECK((uint16_t)(lease_before - c.lease_remaining) == UWB_NET_CYCLE_C ||
                 (a & UWB_ACT_SEND_KEEPALIVE));   /* unless a renewal landed */
        }
        lease_before = c.lease_remaining;
    }
}

/* A mover granted 4 phases participates 4 times per 16-superframe cycle. */
static void test_multi_phase_mover_participates_n_times_per_cycle(void)
{
    const uint16_t mask = (uint16_t)((1u << 0) | (1u << 4) | (1u << 8) | (1u << 12));
    struct uwb_net_ctx c;
    to_ranging_phase(&c, UWB_TIER_FAST, mask);

    static const uint32_t fcs[] = { 4, 8, 12, 16, 20, 24, 28, 32 };  /* two cycles */
    int participations = 0;

    for (unsigned i = 0; i < sizeof(fcs) / sizeof(fcs[0]); i++) {
        struct uwb_net_event b = ev_beacon(fcs[i], true, 3);
        uint32_t a = uwb_net_handle(&c, &b);

        CHECK(!(a & UWB_ACT_TO_SCAN));
        CHECK(c.state == UWB_ST_RANGING);
        participations++;
    }
    CHECK(participations == 8);   /* 4 phases x 2 cycles */
}

/* The mask is what the gateway granted; the map is what it is publishing
 * right now. A beacon landing on a superframe this grant never authorized --
 * even though the tag is still (per this stale/buggy beacon) IN the slot
 * map -- is a disagreement, and must force re-JOIN rather than being
 * silently accepted or silently ignored. */
static void test_phase_mask_disagreement_forces_rejoin(void)
{
    const uint16_t mask = (uint16_t)(1u << 5);   /* phase 5 only */
    struct uwb_net_ctx c;
    to_ranging_phase(&c, UWB_TIER_FAST, mask);

    struct uwb_net_event b = ev_beacon(6u, true, 3);   /* phase 6: not granted */
    uint32_t a = uwb_net_handle(&c, &b);

    CHECK(a & UWB_ACT_TO_SCAN);
    CHECK(c.state == UWB_ST_SCAN);
}

/* A beacon for a phase this tag was NOT granted, in which the tag is correctly
 * ABSENT from the slot map, must be ignored -- not read as a lost lease.
 *
 * This is the case the suite was missing. The other phase tests feed the FSM
 * only its own phases, on the stated assumption that the runner never delivers
 * the others. The runner cannot honour that during acquisition: its
 * phase-derived sleep only engages once beacon tracking has locked
 * (bt_narrow && beacon_sched_have_ref), and until then every superframe's
 * beacon is received. Under the old `!in_map || !phase_active` test the first
 * foreign-phase beacon bounced the tag to SCAN, so it could never stay seated
 * long enough for tracking to lock -- unbootstrappable by construction, and
 * seen on hardware as the gateway re-GRANTing the same address every single
 * superframe, forever.
 *
 * Complements test_phase_mask_disagreement_forces_rejoin above, which covers a
 * foreign phase WITH in_map set: that is a genuine contradiction and must still
 * re-JOIN. Absence on a foreign phase is normal; presence is the anomaly. */
static void test_foreign_phase_absent_from_map_is_ignored(void)
{
    const uint16_t mask = (uint16_t)(1u << 5);   /* phase 5 only */
    struct uwb_net_ctx c;
    to_ranging_phase(&c, UWB_TIER_FAST, mask);

    /* Every superframe of a full cycle, as the radio actually hears them. */
    for (uint32_t fc = 0; fc < UWB_NET_CYCLE_C; fc++) {
        if (fc == 5u) {
            continue;                   /* our own phase, covered elsewhere */
        }
        struct uwb_net_event b = ev_beacon(fc, false, 0);   /* absent, as normal */
        uint32_t a = uwb_net_handle(&c, &b);

        CHECK(!(a & UWB_ACT_TO_SCAN));
        CHECK(c.state == UWB_ST_RANGING);
    }
}

/* KEEPALIVE fires only after UWB_NET_KEEPALIVE_AFTER_N participations with no
 * POS frame sent, once the lease is low -- a real POS transmission (stronger
 * liveness proof, Task 12) resets the count and suppresses it again. */
static void test_keepalive_suppressed_until_n_participations_without_pos(void)
{
    struct uwb_net_ctx c;
    to_ranging(&c, UWB_TIER_FAST);
    c.part_since_pos = 0;               /* undo to_ranging()'s test-fixture preset */
    c.lease_remaining = UWB_NET_LEASE_SF / 2;   /* already at the renewal threshold */

    uint32_t fc = c.frame_counter;
    for (uint16_t i = 1; i < UWB_NET_KEEPALIVE_AFTER_N; i++) {
        struct uwb_net_event b = ev_beacon(++fc, true, 3);
        uint32_t a = uwb_net_handle(&c, &b);
        CHECK(!(a & UWB_ACT_SEND_KEEPALIVE));   /* suppressed: too few participations */
    }

    /* The Nth participation with still no POS: keepalive fires. */
    struct uwb_net_event bn = ev_beacon(++fc, true, 3);
    CHECK(uwb_net_handle(&c, &bn) & UWB_ACT_SEND_KEEPALIVE);

    /* A real POS transmission resets the count, so the very next low-lease
     * participation is suppressed again. */
    c.lease_remaining = UWB_NET_LEASE_SF / 2;
    struct uwb_net_event sw; memset(&sw, 0, sizeof(sw));
    sw.kind = UWB_EV_SWEPT; sw.n_anchors = 4; sw.pos_sent = true;
    uwb_net_handle(&c, &sw);

    struct uwb_net_event b2 = ev_beacon(++fc, true, 3);
    CHECK(!(uwb_net_handle(&c, &b2) & UWB_ACT_SEND_KEEPALIVE));
}

/* uwb_net_phase_skip_to_next() drives the runner's wake planning (Task 12):
 * it must land exactly on the next granted phase, wrapping the cycle when
 * needed, and never loop forever on an empty mask. */
static void test_phase_skip_to_next(void)
{
    /* Single phase: from just before it, just after it (wraps a full cycle),
     * and from exactly on it (must return the FULL cycle, not 0 -- the next
     * occurrence, not this one). */
    const uint16_t one = (uint16_t)(1u << 5);
    CHECK(uwb_net_phase_skip_to_next(one, 0u) == 5u);
    CHECK(uwb_net_phase_skip_to_next(one, 5u) == UWB_NET_CYCLE_C);
    CHECK(uwb_net_phase_skip_to_next(one, 6u) == 15u);   /* wraps to phase 5 of next cycle */

    /* Multiple phases: the nearest one wins. */
    const uint16_t four = (uint16_t)((1u << 0) | (1u << 4) | (1u << 8) | (1u << 12));
    CHECK(uwb_net_phase_skip_to_next(four, 1u) == 3u);    /* -> phase 4 */
    CHECK(uwb_net_phase_skip_to_next(four, 12u) == 4u);   /* -> phase 0 of the next cycle, 4 away */

    /* Empty mask: bounded, returns the full cycle rather than looping forever. */
    CHECK(uwb_net_phase_skip_to_next(0u, 0u) == UWB_NET_CYCLE_C);

    /* Wrap-safe: works the same right at the frame_counter wrap boundary. */
    CHECK(uwb_net_phase_skip_to_next(one, 0xFFFFFFFFu) == 6u);   /* fc%16==15 -> next is 5 (of the next cycle) */
}

/* UWB_NET_CYCLE_C (16) is a power of two dividing 2^32 evenly, so
 * frame_counter % UWB_NET_CYCLE_C must be continuous across the uint32_t
 * wrap -- no phase skipped or repeated at the boundary. */
static void test_phase_active_wrap_does_not_skip_phase(void)
{
    const uint16_t mask_hi = (uint16_t)(1u << 15);
    const uint16_t mask_lo = (uint16_t)(1u << 0);

    uint32_t fc = 0xFFFFFFFFu;   /* fc % 16 == 15 */
    CHECK(uwb_net_phase_active(mask_hi, fc));
    CHECK(!uwb_net_phase_active(mask_lo, fc));

    fc++;   /* wraps to 0; fc % 16 == 0, the very next phase after 15 */
    CHECK(fc == 0u);
    CHECK(uwb_net_phase_active(mask_lo, fc));
    CHECK(!uwb_net_phase_active(mask_hi, fc));
}

static void test_ranging(void)
{
    struct uwb_net_ctx c; to_ranging(&c, UWB_TIER_FAST);

    /* FAST beacon, lease healthy, in map, due -> sweep + sleep, no keepalive. */
    struct uwb_net_event b = ev_beacon(2, true, 3);
    uint32_t a = uwb_net_handle(&c, &b);
    CHECK(a & UWB_ACT_RUN_SWEEP);
    CHECK(!(a & UWB_ACT_SEND_KEEPALIVE));
    CHECK(c.slot_index == 3);

    /* range_every counts *participations*, not frame counters: once whole
     * superframes are skipped the frame counter advances while the tag is
     * asleep, so a modulo test would fire on whichever superframe the tag
     * happened to wake in. Drive SLOW with range_every = 3 and a frame counter
     * that jumps arbitrarily -- the sweep must land on every third
     * participation regardless. */
    uwb_net_reset_tier_params();
    struct uwb_tier_params every3 = { 75u, 3u };
    uwb_net_set_tier_params(UWB_TIER_SLOW, &every3);

    struct uwb_net_ctx cs; to_ranging(&cs, UWB_TIER_SLOW);
    static const uint32_t fcs[] = { 7, 400, 401, 9000, 9025, 9050, 12345 };
    for (unsigned i = 0; i < sizeof(fcs) / sizeof(fcs[0]); i++) {
        struct uwb_net_event bi = ev_beacon(fcs[i], true, 3);
        uint32_t ai = uwb_net_handle(&cs, &bi);
        bool want_sweep = ((i + 1) % 3u) == 0u;
        CHECK(((ai & UWB_ACT_RUN_SWEEP) != 0) == want_sweep);
        CHECK(((ai & UWB_ACT_SLEEP) != 0) == !want_sweep);
    }
    uwb_net_reset_tier_params();

    /* Default range_every = 1: every participation sweeps, in every tier. */
    struct uwb_net_ctx cs1; to_ranging(&cs1, UWB_TIER_SLOW);
    struct uwb_net_event b1 = ev_beacon(1, true, 3);
    CHECK(uwb_net_handle(&cs1, &b1) & UWB_ACT_RUN_SWEEP);
    struct uwb_net_event b5 = ev_beacon(5, true, 3);
    CHECK(uwb_net_handle(&cs1, &b5) & UWB_ACT_RUN_SWEEP);

    /* Lease decays to half -> keepalive flag set, lease renewed. */
    struct uwb_net_ctx ck; to_ranging(&ck, UWB_TIER_FAST);
    ck.lease_remaining = UWB_NET_LEASE_SF / 2;     /* at threshold */
    struct uwb_net_event bk = ev_beacon(2, true, 3);
    CHECK(uwb_net_handle(&ck, &bk) & UWB_ACT_SEND_KEEPALIVE);
    CHECK(ck.lease_remaining == UWB_NET_LEASE_SF);  /* renewed optimistically */

    /* Beacon miss x M -> SCAN, never a TX action. */
    struct uwb_net_ctx cm; to_ranging(&cm, UWB_TIER_FAST);
    struct uwb_net_event miss; memset(&miss, 0, sizeof(miss)); miss.kind = UWB_EV_BEACON_MISS;
    CHECK(uwb_net_handle(&cm, &miss) == UWB_ACT_NONE);
    CHECK(uwb_net_handle(&cm, &miss) == UWB_ACT_NONE);
    CHECK(uwb_net_handle(&cm, &miss) == UWB_ACT_TO_SCAN);
    CHECK(cm.state == UWB_ST_SCAN);

    /* Addr absent from map -> SCAN immediately. */
    struct uwb_net_ctx cr; to_ranging(&cr, UWB_TIER_FAST);
    struct uwb_net_event gone = ev_beacon(2, false, 0);
    CHECK(uwb_net_handle(&cr, &gone) == UWB_ACT_TO_SCAN);
    CHECK(cr.state == UWB_ST_SCAN);

    /* Sweep returns too few anchors -> re-discover. */
    struct uwb_net_ctx cd; to_ranging(&cd, UWB_TIER_FAST);
    struct uwb_net_event sw; memset(&sw, 0, sizeof(sw)); sw.kind = UWB_EV_SWEPT; sw.n_anchors = 2;
    CHECK(uwb_net_handle(&cd, &sw) == UWB_ACT_RUN_DISCOVER);
    CHECK(cd.state == UWB_ST_DISCOVER);
}

/* Open Work item 3: UWB_ST_SCAN must emit UWB_ACT_SLEEP. An unjoined tag used
 * to hold RX open across ~100% of every superframe -- the tag's worst power
 * state, and the one every failed `cal` run leaves it in. */
static void test_scan_sleeps(void)
{
    struct uwb_net_ctx c; uwb_net_init(&c, EUI);

    struct uwb_net_event miss; memset(&miss, 0, sizeof(miss));
    miss.kind = UWB_EV_BEACON_MISS;
    CHECK(uwb_net_handle(&c, &miss) == UWB_ACT_SLEEP);
    CHECK(c.state == UWB_ST_SCAN);

    /* Repeatedly, and without ever accumulating toward a state change. */
    for (int i = 0; i < 10; i++) {
        CHECK(uwb_net_handle(&c, &miss) == UWB_ACT_SLEEP);
    }
    CHECK(c.state == UWB_ST_SCAN);

    /* A usable beacon still joins instead of sleeping. */
    struct uwb_net_event b = ev_beacon(5, false, 0);
    CHECK(uwb_net_handle(&c, &b) == UWB_ACT_SEND_JOIN);

    /* And the sleep must not have cost the SCAN alert rule: a HELP raised out
     * of coverage still goes out blind on both BEACON and BEACON_MISS. */
    struct uwb_net_ctx ca; uwb_net_init(&ca, EUI);
    struct uwb_net_event ma = miss; ma.alert_pending = true;
    CHECK(uwb_net_handle(&ca, &ma) == (UWB_ACT_SLEEP | UWB_ACT_SEND_ALERT));
}

/* Regression, introduced by superframe skipping: the gateway ages every lease
 * once per superframe whether or not the tag listened. A tag that re-syncs
 * every 25 superframes must renew on every re-sync -- decrementing the local
 * lease by one per *received* beacon would renew 25x too late and the seat
 * would be reclaimed on the first skip (RESCAN seat, and a full JOIN + GRANT +
 * re-discovery to get it back, which costs more than the skip saves). */
static void test_lease_ages_by_elapsed(void)
{
    struct uwb_net_ctx c; to_ranging(&c, UWB_TIER_FAST);

    /* A single beacon 25 superframes later must age the lease by 25, which is
     * exactly the renewal threshold. */
    uint32_t fc = c.frame_counter + 25u;
    struct uwb_net_event b = ev_beacon(fc, true, 3);
    uint32_t a = uwb_net_handle(&c, &b);
    CHECK(a & UWB_ACT_SEND_KEEPALIVE);
    CHECK(c.lease_remaining == UWB_NET_LEASE_SF);   /* renewed */

    /* Sustained: 200 re-syncs at skip 25 and the local lease must never run
     * out between renewals. */
    for (int i = 0; i < 200; i++) {
        fc += 25u;
        struct uwb_net_event bi = ev_beacon(fc, true, 3);
        uwb_net_handle(&c, &bi);
        CHECK(c.state == UWB_ST_RANGING);
        CHECK(c.lease_remaining > 0);
    }

    /* Phase 3: still holds when the tag is awake only 1 superframe in 16 (a
     * single granted phase), not just at the pre-v3 skip-25 cadence above. */
    struct uwb_net_ctx cp;
    to_ranging_phase(&cp, UWB_TIER_FAST, (uint16_t)(1u << 0));
    uint32_t fcp = 16u;
    for (int i = 0; i < 20; i++) {
        struct uwb_net_event bi = ev_beacon(fcp, true, 3);
        uint32_t ai = uwb_net_handle(&cp, &bi);
        CHECK(!(ai & UWB_ACT_TO_SCAN));
        CHECK(cp.state == UWB_ST_RANGING);
        CHECK(cp.lease_remaining > 0);
        fcp += UWB_NET_CYCLE_C;
    }

    /* Every superframe: still exactly one decrement per beacon, so nothing
     * about the un-skipped case changed. */
    struct uwb_net_ctx c1; to_ranging(&c1, UWB_TIER_FAST);
    uint16_t before = c1.lease_remaining;
    struct uwb_net_event b1 = ev_beacon(c1.frame_counter + 1u, true, 3);
    uwb_net_handle(&c1, &b1);
    CHECK(c1.lease_remaining == before - 1u);

    /* Wrap-safe across the gateway's uint32 frame counter. */
    struct uwb_net_ctx cw; to_ranging(&cw, UWB_TIER_FAST);
    cw.frame_counter = 0xFFFFFFF0u;
    cw.lease_remaining = UWB_NET_LEASE_SF;
    struct uwb_net_event bw = ev_beacon(0x00000004u, true, 3);   /* +20 */
    uwb_net_handle(&cw, &bw);
    CHECK(cw.lease_remaining == UWB_NET_LEASE_SF - 20u);

    /* A gateway restart (counter jumps backwards) zeroes the lease, which
     * forces a keepalive -- the right answer for a gateway that just rebooted. */
    struct uwb_net_ctx cr; to_ranging(&cr, UWB_TIER_FAST);
    cr.frame_counter = 900000u;
    cr.lease_remaining = UWB_NET_LEASE_SF;
    struct uwb_net_event br = ev_beacon(3u, true, 3);
    CHECK(uwb_net_handle(&cr, &br) & UWB_ACT_SEND_KEEPALIVE);
    CHECK(cr.lease_remaining == UWB_NET_LEASE_SF);   /* renewed after zeroing */
}

static void test_gate_actions(void)
{
    /* Only the sweep does TWR, so only the sweep depends on the antenna delay. */
    const uint32_t ranging = UWB_ACT_RUN_SWEEP;
    const uint32_t housekeeping = UWB_ACT_SEND_JOIN | UWB_ACT_SEND_KEEPALIVE
                                | UWB_ACT_RUN_DISCOVER
                                | UWB_ACT_SLEEP | UWB_ACT_TO_SCAN;

    /* Calibrated: every action passes through untouched. */
    CHECK(uwb_net_gate_actions(ranging | housekeeping, true)
          == (ranging | housekeeping));

    /* Uncalibrated: the sweep is cleared. */
    CHECK((uwb_net_gate_actions(ranging, false) & UWB_ACT_RUN_SWEEP) == 0);

    /* Uncalibrated: DISCOVER survives. It does no TWR, and gating it would
     * strand the tag in UWB_ST_DISCOVER — which is also the only way it ever
     * reaches UWB_ST_RANGING, the sole source of UWB_ACT_SLEEP. Clearing it
     * would therefore cost the radio's deep sleep as well as the state. */
    CHECK((uwb_net_gate_actions(UWB_ACT_RUN_DISCOVER, false)
           & UWB_ACT_RUN_DISCOVER) != 0);

    /* Uncalibrated: everything that keeps the seat survives. This is the
     * property that matters -- a tag that stops ranging must not also stop
     * renewing its lease, or it silently drops off the network. */
    CHECK(uwb_net_gate_actions(ranging | housekeeping, false) == housekeeping);

    /* Nothing in, nothing out, either way. */
    CHECK(uwb_net_gate_actions(UWB_ACT_NONE, true) == UWB_ACT_NONE);
    CHECK(uwb_net_gate_actions(UWB_ACT_NONE, false) == UWB_ACT_NONE);
}

/* The net layer rejects any beacon whose byte-10 proto_ver != UWB_NET_PROTO_VER,
 * and the gateway stamps that byte with the frame module's UWB_PROTO_VER. The two
 * constants live in different headers, so bumping one alone silently strands the
 * tag in SCAN: it hears every beacon and answers none. That is exactly what
 * happened when the frame module went to v2 and this header stayed at v1. */
static void test_proto_ver_matches_frame_module(void)
{
    CHECK(UWB_NET_PROTO_VER == UWB_PROTO_VER);
}

/* UWB_ACT_SEND_ALERT: JOINING / DISCOVER / RANGING fire only on an actual
 * beacon; SCAN fires on both BEACON and BEACON_MISS. In every case the rest
 * of the action word / state transition must be identical to the same event
 * with alert_pending == false -- an alert must never suppress or alter
 * anything else. */
static void test_send_alert(void)
{
    /* -- SCAN -- */
    {
        struct uwb_net_ctx c0; uwb_net_init(&c0, EUI);
        struct uwb_net_ctx c1; uwb_net_init(&c1, EUI);
        struct uwb_net_event b0 = ev_beacon(5, false, 0);
        struct uwb_net_event b1 = b0; b1.alert_pending = true;

        uint32_t a0 = uwb_net_handle(&c0, &b0);
        uint32_t a1 = uwb_net_handle(&c1, &b1);
        CHECK(a1 == (a0 | UWB_ACT_SEND_ALERT));
        CHECK(c0.state == c1.state);

        struct uwb_net_ctx c2; uwb_net_init(&c2, EUI);
        struct uwb_net_ctx c3; uwb_net_init(&c3, EUI);
        struct uwb_net_event m0; memset(&m0, 0, sizeof(m0)); m0.kind = UWB_EV_BEACON_MISS;
        struct uwb_net_event m1 = m0; m1.alert_pending = true;

        uint32_t am0 = uwb_net_handle(&c2, &m0);
        uint32_t am1 = uwb_net_handle(&c3, &m1);
        CHECK(am1 == (am0 | UWB_ACT_SEND_ALERT));
        CHECK(c2.state == c3.state);
    }

    /* -- JOINING -- */
    {
        struct uwb_net_ctx c0; uwb_net_init(&c0, EUI);
        struct uwb_net_ctx c1; uwb_net_init(&c1, EUI);
        struct uwb_net_event b = ev_beacon(0, false, 0);
        uwb_net_handle(&c0, &b); uwb_net_handle(&c1, &b);   /* both -> JOINING */
        CHECK(c0.state == UWB_ST_JOINING && c1.state == UWB_ST_JOINING);

        struct uwb_net_event j0 = ev_beacon(1, false, 0);   /* retry-join beacon */
        struct uwb_net_event j1 = j0; j1.alert_pending = true;
        uint32_t a0 = uwb_net_handle(&c0, &j0);
        uint32_t a1 = uwb_net_handle(&c1, &j1);
        CHECK(a1 == (a0 | UWB_ACT_SEND_ALERT));
        CHECK(c0.state == c1.state);

        /* BEACON_MISS is not a JOINING event kind in this FSM's contract via
         * beacons only, so drive GRANT_MISS instead to confirm alert_pending
         * has no effect on a non-BEACON event in a synced state. */
        struct uwb_net_event gm; memset(&gm, 0, sizeof(gm)); gm.kind = UWB_EV_GRANT_MISS;
        struct uwb_net_event gm1 = gm; gm1.alert_pending = true;
        uint32_t g0 = uwb_net_handle(&c0, &gm);
        uint32_t g1 = uwb_net_handle(&c1, &gm1);
        CHECK(g0 == g1);   /* no alert bit: not a BEACON event */
    }

    /* -- DISCOVER -- */
    {
        struct uwb_net_ctx c0; uwb_net_init(&c0, EUI);
        struct uwb_net_ctx c1; uwb_net_init(&c1, EUI);
        struct uwb_net_event b = ev_beacon(0, false, 0);
        struct uwb_net_event g = ev_grant(0x0007, 3, UWB_TIER_FAST, 50);
        uwb_net_handle(&c0, &b); uwb_net_handle(&c0, &g);
        uwb_net_handle(&c1, &b); uwb_net_handle(&c1, &g);
        CHECK(c0.state == UWB_ST_DISCOVER && c1.state == UWB_ST_DISCOVER);

        struct uwb_net_event db0 = ev_beacon(1, true, 3);
        struct uwb_net_event db1 = db0; db1.alert_pending = true;
        uint32_t a0 = uwb_net_handle(&c0, &db0);
        uint32_t a1 = uwb_net_handle(&c1, &db1);
        CHECK(a1 == (a0 | UWB_ACT_SEND_ALERT));
        CHECK(c0.state == c1.state);

        struct uwb_net_event dm; memset(&dm, 0, sizeof(dm)); dm.kind = UWB_EV_BEACON_MISS;
        struct uwb_net_event dm1 = dm; dm1.alert_pending = true;
        uint32_t m0 = uwb_net_handle(&c0, &dm);
        uint32_t m1 = uwb_net_handle(&c1, &dm1);
        CHECK(m0 == m1);   /* miss never fires the alert outside SCAN */
    }

    /* -- RANGING -- */
    {
        struct uwb_net_ctx c0; to_ranging(&c0, UWB_TIER_FAST);
        struct uwb_net_ctx c1; to_ranging(&c1, UWB_TIER_FAST);

        struct uwb_net_event rb0 = ev_beacon(2, true, 3);
        struct uwb_net_event rb1 = rb0; rb1.alert_pending = true;
        uint32_t a0 = uwb_net_handle(&c0, &rb0);
        uint32_t a1 = uwb_net_handle(&c1, &rb1);
        CHECK(a1 == (a0 | UWB_ACT_SEND_ALERT));
        CHECK(a0 & UWB_ACT_RUN_SWEEP);   /* sanity: a real action was in play */
        CHECK(c0.state == c1.state);

        struct uwb_net_event rm; memset(&rm, 0, sizeof(rm)); rm.kind = UWB_EV_BEACON_MISS;
        struct uwb_net_event rm1 = rm; rm1.alert_pending = true;
        uint32_t m0 = uwb_net_handle(&c0, &rm);
        uint32_t m1 = uwb_net_handle(&c1, &rm1);
        CHECK(m0 == m1);   /* "never TX on a missed beacon" rule stands */
    }

    /* Gate: UWB_ACT_SEND_ALERT survives uwb_net_gate_actions() uncalibrated. */
    CHECK((uwb_net_gate_actions(UWB_ACT_SEND_ALERT, false) & UWB_ACT_SEND_ALERT) != 0);
}

/* The regression the sweep gate exists for.
 *
 * One sweep that comes back short must not latch the tag into re-discovering
 * forever. The failure this reproduces was observed on hardware: after the
 * anchors were power-cycled, three anchors answered every DISCOVERY with
 * verdict "sent" and the tag still never emitted a single WAVE poll again
 * until it was itself rebooted. */
static void test_sweep_gate_recovers_after_short_sweep(void)
{
    struct uwb_sweep_gate g;
    uint32_t now = 1000u;

    uwb_sweep_gate_init(&g);

    /* Nothing discovered yet: the first participation must discover. */
    CHECK(uwb_sweep_gate_rediscover_due(&g, now));

    /* Discovery finds enough anchors, so the next participation sweeps. */
    uwb_sweep_gate_discovered(&g, now, 0, 1, UWB_NET_MIN_ANCHORS);
    CHECK(!uwb_sweep_gate_rediscover_due(&g, now));

    /* The anchors are power-cycled and the sweep ranges nobody. */
    uwb_sweep_gate_swept(&g, 0u);
    CHECK(uwb_sweep_gate_rediscover_due(&g, now));

    /* The anchors are back and discovery sees all of them again. The very next
     * participation must sweep. This is the assertion the latch failed: the
     * rediscover branch refreshed neither the count it was gated on nor any
     * path back to it. */
    now += 200u;
    uwb_sweep_gate_discovered(&g, now, 0, 1, UWB_NET_MIN_ANCHORS);
    CHECK(!uwb_sweep_gate_rediscover_due(&g, now));

    /* ...and a round that still finds too few must keep re-discovering, so the
     * fix is not simply "always sweep after a discovery". */
    uwb_sweep_gate_swept(&g, 0u);
    now += 200u;
    uwb_sweep_gate_discovered(&g, now, 0, 1, UWB_NET_MIN_ANCHORS - 1u);
    CHECK(uwb_sweep_gate_rediscover_due(&g, now));
}

/* Grouped discovery (Task 6): a round that finds only 2 of ITS group's
 * anchors is not a short sweep once other groups already have known-good
 * anchors -- last_sweep_n must be the sum across the whole cycle. This is
 * exactly the latching shape test_sweep_gate_recovers_after_short_sweep()
 * guards against, reintroduced at the per-group level if the sum were
 * dropped in favour of the single most recent round's count. */
static void test_sweep_gate_group_cycle_sums_across_groups(void)
{
    struct uwb_sweep_gate g;
    uint32_t now = 1000u;

    uwb_sweep_gate_init(&g);

    /* 32 anchors over 8 groups of 4: each round finds all 4 of its group. A
     * full cycle must reach UWB_NET_MIN_ANCHORS well before the last group,
     * and never falsely latch mid-cycle just because any ONE round's own
     * count is below the threshold on its own (it isn't here, but the point
     * is the gate must not require every group answer before sweeping). */
    for (uint8_t grp = 0; grp < 8; grp++) {
        now += 200u;
        uwb_sweep_gate_discovered(&g, now, grp, 8, 4u);
        if (grp == 0) {
            /* First group alone already clears MIN_ANCHORS (4 >= 3). */
            CHECK(!uwb_sweep_gate_rediscover_due(&g, now));
        }
    }
    /* A full cycle: last_sweep_n must be 32 (4 anchors x 8 groups), not 4
     * (just the most recent round). */
    CHECK(g.last_sweep_n == 32u);

    /* Now one specific group's anchors go dark (power-cycled), reporting 0.
     * The other 7 groups' anchors are still known-good from their last visit,
     * so the tag must still consider the sweep healthy -- it must NOT
     * rediscover forever just because ITS most recent round was short. */
    now += 200u;
    uwb_sweep_gate_discovered(&g, now, 3, 8, 0u);
    CHECK(g.last_sweep_n == 28u);              /* 32 - the 4 lost anchors */
    CHECK(!uwb_sweep_gate_rediscover_due(&g, now));

    /* If EVERY group goes dark, the cycle sum eventually drops below
     * MIN_ANCHORS and the gate correctly asks to keep rediscovering. */
    for (uint8_t grp = 0; grp < 8; grp++) {
        now += 200u;
        uwb_sweep_gate_discovered(&g, now, grp, 8, 0u);
    }
    CHECK(g.last_sweep_n == 0u);
    CHECK(uwb_sweep_gate_rediscover_due(&g, now));

    /* A cycle-size change (e.g. anchor count re-provisioned) resets the
     * per-group accounting rather than mixing counts from the old cycle
     * shape into the new one. */
    uwb_sweep_gate_init(&g);
    uwb_sweep_gate_discovered(&g, now, 0, 8, 4u);
    CHECK(g.last_sweep_n == 4u);
    uwb_sweep_gate_discovered(&g, now, 0, 4, 4u);   /* n_groups shrank */
    CHECK(g.last_sweep_n == 4u);   /* old 8-group counts discarded, not summed in */

    /* Malformed group/n_groups falls back to a single-group round instead of
     * indexing off the end or corrupting the accounting. */
    uwb_sweep_gate_init(&g);
    uwb_sweep_gate_discovered(&g, now, 5, 3, 7u);   /* group >= n_groups */
    CHECK(g.last_sweep_n == 7u);
    uwb_sweep_gate_discovered(&g, now, 0, 0, 9u);   /* n_groups == 0 */
    CHECK(g.last_sweep_n == 9u);
    uwb_sweep_gate_discovered(&g, now, 0,
                              UWB_SWEEP_GATE_N_GROUPS_MAX + 1, 5u);  /* n_groups too big */
    CHECK(g.last_sweep_n == 5u);
}

/* The periodic refresh, and that its arithmetic survives the ms-clock wrap. */
static void test_sweep_gate_interval(void)
{
    struct uwb_sweep_gate g;

    uwb_sweep_gate_init(&g);
    uwb_sweep_gate_discovered(&g, 0u, 0, 1, UWB_NET_MIN_ANCHORS);
    uwb_sweep_gate_swept(&g, UWB_NET_MIN_ANCHORS);
    CHECK(!uwb_sweep_gate_rediscover_due(&g, UWB_NET_REDISCOVER_INTERVAL_MS - 1u));
    CHECK(uwb_sweep_gate_rediscover_due(&g, UWB_NET_REDISCOVER_INTERVAL_MS));

    /* Signed difference: 0xFFFFFF00 + 512 wraps to 0x100. */
    uwb_sweep_gate_init(&g);
    uwb_sweep_gate_discovered(&g, 0xFFFFFF00u, 0, 1, UWB_NET_MIN_ANCHORS);
    uwb_sweep_gate_swept(&g, UWB_NET_MIN_ANCHORS);
    CHECK(!uwb_sweep_gate_rediscover_due(&g, 0x00000100u));
    CHECK(uwb_sweep_gate_rediscover_due(&g,
              0x00000100u + UWB_NET_REDISCOVER_INTERVAL_MS));
}

int main(void)
{
    test_proto_ver_matches_frame_module();
    test_init();
    test_tier_params();
    test_tier_filter();
    test_scan_sleeps();
    test_scan_join();
    test_grant_discover();
    test_discover_keeps_lease();
    test_ranging();
    test_single_phase_participates_once_per_cycle();
    test_multi_phase_mover_participates_n_times_per_cycle();
    test_phase_mask_disagreement_forces_rejoin();
    test_foreign_phase_absent_from_map_is_ignored();
    test_phase_skip_to_next();
    test_keepalive_suppressed_until_n_participations_without_pos();
    test_phase_active_wrap_does_not_skip_phase();
    test_lease_ages_by_elapsed();
    test_gate_actions();
    test_send_alert();
    test_sweep_gate_recovers_after_short_sweep();
    test_sweep_gate_group_cycle_sums_across_groups();
    test_sweep_gate_interval();
    printf(g_fail ? "FAILED %d\n" : "OK\n", g_fail);
    return g_fail ? 1 : 0;
}
