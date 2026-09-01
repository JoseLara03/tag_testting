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

    /* Defaults are now each tier's PERIOD, so no default needs the cap at all
     * -- see test_defaults_are_the_tier_periods below for why that matters. */
    uwb_net_get_tier_params(UWB_TIER_FAST, &p);
    CHECK(p.listen_skip == UWB_NET_PERIOD_FAST && p.range_every == 1u);
    uwb_net_get_tier_params(UWB_TIER_SLOW, &p);
    CHECK(p.listen_skip == UWB_NET_PERIOD_SLOW);   /* 5, not the capped 25 */
    CHECK(p.range_every == 1u);
    uwb_net_get_tier_params(UWB_TIER_IDLE, &p);
    CHECK(p.listen_skip == UWB_NET_PERIOD_IDLE);   /* 25 == the cap, but by
                                                    * derivation, not by clamp */

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
    CHECK(c.short_addr == 0x0007 && c.seat_id == 3);
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

    /* One absence mid-DISCOVER is NOT a reclaim under contract v3 -- the slot
     * map is a schedule, so being left out of one is the ordinary case. This
     * assertion is deliberately inverted from what it was under v2, where a
     * single absence dropped the tag to SCAN and thereby made gateway-side slot
     * multiplexing impossible. Loss is now detected by
     * UWB_NET_SCHED_GAP_MAX; see test_absence_beyond_the_gap_rejoins. */
    struct uwb_net_ctx c2; uwb_net_init(&c2, EUI);
    uwb_net_handle(&c2, &b); uwb_net_handle(&c2, &g);   /* DISCOVER */
    struct uwb_net_event lost = ev_beacon(1, false, 0); /* in_map = false */
    uint32_t la = uwb_net_handle(&c2, &lost);
    CHECK(!(la & UWB_ACT_TO_SCAN));
    CHECK(la & UWB_ACT_SLEEP);
    CHECK(c2.state == UWB_ST_DISCOVER);
}

/* TDoA has no tag<->anchor binding at all (design spec §7): a blinking tag
 * must reach RANGING without ever needing UWB_NET_MIN_ANCHORS discovered,
 * since it neither polls nor solves anything. Before this fix the tag was
 * stuck in DISCOVER forever whenever discovery could not reach 3 anchors,
 * regardless of blink_mode -- exactly the binding dependency TDoA exists to
 * remove. */
static void test_blink_skips_discover(void)
{
    struct uwb_net_ctx c; uwb_net_init(&c, EUI);
    c.blink_mode = true;
    struct uwb_net_event b = ev_beacon(0, false, 0);
    uwb_net_handle(&c, &b);                             /* -> JOINING */

    struct uwb_net_event g = ev_grant(0x0007, 3, UWB_TIER_FAST, 50);
    CHECK(uwb_net_handle(&c, &g) == UWB_ACT_RUN_DISCOVER);
    CHECK(c.state == UWB_ST_DISCOVER);

    /* No UWB_EV_DISCOVERED at all -- an ordinary BEACON is enough to escape
     * straight to RANGING, with zero anchors ever reported. */
    struct uwb_net_event b2 = ev_beacon(1, true, 3);
    CHECK(uwb_net_handle(&c, &b2) == UWB_ACT_NONE);
    CHECK(c.state == UWB_ST_RANGING);

    /* And once there, it actually blinks rather than sweeping -- FAST's
     * default range_every is 1, so the very next participation qualifies. */
    struct uwb_net_event b3 = ev_beacon(2, true, 3);
    uint32_t a3 = uwb_net_handle(&c, &b3);
    CHECK(a3 & UWB_ACT_SEND_BLINK);
    CHECK(!(a3 & UWB_ACT_RUN_SWEEP));

    /* Non-blink tags are unaffected: same setup with blink_mode false still
     * needs real discovery. */
    struct uwb_net_ctx c2; uwb_net_init(&c2, EUI);
    uwb_net_handle(&c2, &b);
    uwb_net_handle(&c2, &g);
    CHECK(c2.state == UWB_ST_DISCOVER);
    CHECK(uwb_net_handle(&c2, &b2) == UWB_ACT_RUN_DISCOVER);
    CHECK(c2.state == UWB_ST_DISCOVER);   /* still stuck without a real DISCOVERED */
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
    CHECK(c.seat_id == 3);
    CHECK(c.tx_slot == 3);
}

/* Helper: drive a fresh ctx into RANGING with the given tier. */
static void to_ranging(struct uwb_net_ctx *c, uwb_tier_t tier)
{
    uwb_net_init(c, EUI);
    struct uwb_net_event b = ev_beacon(0, false, 0); uwb_net_handle(c, &b);
    struct uwb_net_event g = ev_grant(0x0007, 3, tier, UWB_NET_LEASE_SF); uwb_net_handle(c, &g);
    struct uwb_net_event d; memset(&d, 0, sizeof(d)); d.kind = UWB_EV_DISCOVERED; d.n_anchors = 4;
    uwb_net_handle(c, &d);   /* -> RANGING */
}

static void test_ranging(void)
{
    struct uwb_net_ctx c; to_ranging(&c, UWB_TIER_FAST);

    /* FAST beacon, lease healthy, in map, due -> sweep + sleep, no keepalive. */
    struct uwb_net_event b = ev_beacon(2, true, 3);
    uint32_t a = uwb_net_handle(&c, &b);
    CHECK(a & UWB_ACT_RUN_SWEEP);
    CHECK(!(a & UWB_ACT_SEND_KEEPALIVE));
    /* The scheduled slot, which is what the runner uses for TDMA timing. */
    CHECK(c.tx_slot == 3);

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

    /* Absent from the map -> sleep, keep the seat. Inverted from v2 for the
     * same reason as the DISCOVER case above. */
    struct uwb_net_ctx cr; to_ranging(&cr, UWB_TIER_FAST);
    struct uwb_net_event gone = ev_beacon(2, false, 0);
    uint32_t ga = uwb_net_handle(&cr, &gone);
    CHECK(!(ga & UWB_ACT_TO_SCAN));
    CHECK(ga & UWB_ACT_SLEEP);
    CHECK(!(ga & UWB_ACT_RUN_SWEEP));
    CHECK(cr.state == UWB_ST_RANGING);

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

    /* A single beacon 25 superframes later must age the lease by 25 -- that is
     * what this test is really about, and it still holds.
     *
     * What changed: the renewal threshold is UWB_NET_LEASE_SF / 2, so at a
     * lease of 50 a 25-superframe skip landed EXACTLY on it and the keepalive
     * fired on every single wake with no margin at all. This test asserted that
     * as correct behaviour; it was in fact the zero-margin condition the lease
     * was raised to 75 to remove. Now 75 - 25 = 50 is still comfortably above
     * the threshold of 37, so no keepalive is due yet, and that gap IS the fix.
     *
     * Whether zero margin was the whole of the bench failure recorded in
     * uwb_net.h ("keepalives on the air and no position fixes") is not something
     * these tests can settle -- margin can only help, and the claim here is no
     * stronger than that. */
    uint32_t fc = c.frame_counter + 25u;
    struct uwb_net_event b = ev_beacon(fc, true, 3);
    uint32_t a = uwb_net_handle(&c, &b);
    CHECK(c.lease_remaining == UWB_NET_LEASE_SF - 25u);
    CHECK(c.lease_remaining > (UWB_NET_LEASE_SF / 2));
    CHECK(!(a & UWB_ACT_SEND_KEEPALIVE));           /* not due: margin exists */

    /* One more skip and it IS due, still well before expiry. */
    fc += 25u;
    struct uwb_net_event b2 = ev_beacon(fc, true, 3);
    uint32_t a2 = uwb_net_handle(&c, &b2);
    CHECK(a2 & UWB_ACT_SEND_KEEPALIVE);
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
    uwb_sweep_gate_discovered(&g, now, UWB_NET_MIN_ANCHORS);
    CHECK(!uwb_sweep_gate_rediscover_due(&g, now));

    /* The anchors are power-cycled and the sweep ranges nobody. */
    uwb_sweep_gate_swept(&g, 0u);
    CHECK(uwb_sweep_gate_rediscover_due(&g, now));

    /* The anchors are back and discovery sees all of them again. The very next
     * participation must sweep. This is the assertion the latch failed: the
     * rediscover branch refreshed neither the count it was gated on nor any
     * path back to it. */
    now += 200u;
    uwb_sweep_gate_discovered(&g, now, UWB_NET_MIN_ANCHORS);
    CHECK(!uwb_sweep_gate_rediscover_due(&g, now));

    /* ...and a round that still finds too few must keep re-discovering, so the
     * fix is not simply "always sweep after a discovery". */
    uwb_sweep_gate_swept(&g, 0u);
    now += 200u;
    uwb_sweep_gate_discovered(&g, now, UWB_NET_MIN_ANCHORS - 1u);
    CHECK(uwb_sweep_gate_rediscover_due(&g, now));
}

/* The periodic refresh, and that its arithmetic survives the ms-clock wrap. */
static void test_sweep_gate_interval(void)
{
    struct uwb_sweep_gate g;

    uwb_sweep_gate_init(&g);
    uwb_sweep_gate_discovered(&g, 0u, UWB_NET_MIN_ANCHORS);
    uwb_sweep_gate_swept(&g, UWB_NET_MIN_ANCHORS);
    CHECK(!uwb_sweep_gate_rediscover_due(&g, UWB_NET_REDISCOVER_INTERVAL_MS - 1u));
    CHECK(uwb_sweep_gate_rediscover_due(&g, UWB_NET_REDISCOVER_INTERVAL_MS));

    /* Signed difference: 0xFFFFFF00 + 512 wraps to 0x100. */
    uwb_sweep_gate_init(&g);
    uwb_sweep_gate_discovered(&g, 0xFFFFFF00u, UWB_NET_MIN_ANCHORS);
    uwb_sweep_gate_swept(&g, UWB_NET_MIN_ANCHORS);
    CHECK(!uwb_sweep_gate_rediscover_due(&g, 0x00000100u));
    CHECK(uwb_sweep_gate_rediscover_due(&g,
              0x00000100u + UWB_NET_REDISCOVER_INTERVAL_MS));
}


/* ---- Contract v3: the slot map is a SCHEDULE, not an ownership table ----
 *
 * Under v2 a single absence from the map meant "seat reclaimed" and dropped the
 * tag to SCAN. That is what made the gateway unable to time-multiplex slots: any
 * superframe it left a seated tag out of the map, that tag tore down and
 * rejoined. These tests pin the new behaviour.
 */

/* Absence is the ORDINARY case for every tier but FAST, and it must cost the
 * tag nothing but a sleep. */
static void test_absent_from_map_sleeps_and_keeps_seat(void)
{
    struct uwb_net_ctx c;
    to_ranging(&c, UWB_TIER_IDLE);

    uint16_t addr = c.short_addr;
    uint8_t  seat = c.seat_id;

    /* Scheduled once, then absent for a full IDLE period. */
    for (uint32_t fc = 1; fc <= 24; fc++) {
        struct uwb_net_event b = ev_beacon(fc, false, 0);
        uint32_t a = uwb_net_handle(&c, &b);

        CHECK(c.state == UWB_ST_RANGING);        /* NOT bounced to SCAN */
        CHECK(!(a & UWB_ACT_TO_SCAN));
        CHECK(a & UWB_ACT_SLEEP);                /* just sleep */
        CHECK(!(a & UWB_ACT_RUN_SWEEP));         /* and do not transmit */
        CHECK(c.short_addr == addr);
        CHECK(c.seat_id == seat);
    }
}

/* But absence cannot be tolerated forever, or a tag whose seat really was
 * reclaimed would sleep and self-renew against a gateway that forgot it. */
static void test_absence_beyond_the_gap_rejoins(void)
{
    struct uwb_net_ctx c;
    to_ranging(&c, UWB_TIER_IDLE);

    bool went_to_scan = false;

    for (uint32_t fc = 1; fc <= UWB_NET_SCHED_GAP_MAX + 2u; fc++) {
        struct uwb_net_event b = ev_beacon(fc, false, 0);
        uint32_t a = uwb_net_handle(&c, &b);

        if (a & UWB_ACT_TO_SCAN) {
            went_to_scan = true;
            CHECK(c.state == UWB_ST_SCAN);
            /* Not one superframe early: the threshold is a real bound. */
            CHECK(fc > UWB_NET_SCHED_GAP_MAX);
            break;
        }
    }
    CHECK(went_to_scan);
}

/* Being scheduled resets the clock, so a tag served at its cadence never
 * accumulates toward the gap however long it runs. */
static void test_scheduled_at_cadence_never_times_out(void)
{
    struct uwb_net_ctx c;
    to_ranging(&c, UWB_TIER_IDLE);

    /* Served every 25th superframe, as the gateway's IDLE cadence does. */
    for (uint32_t fc = 1; fc <= UWB_NET_SCHED_GAP_MAX * 6u; fc++) {
        bool sched = (fc % 25u == 1u);
        struct uwb_net_event b = ev_beacon(fc, sched, 4);

        uwb_net_handle(&c, &b);
        CHECK(c.state == UWB_ST_RANGING);
    }
}

/* KEEPALIVE is CAP traffic and owes nothing to the CFP schedule. If it stopped
 * while the tag was unscheduled, a slow-tier tag would stop renewing exactly
 * while it waited to be scheduled, and the gateway would reclaim the seat it
 * was waiting on. */
static void test_keepalive_fires_while_unscheduled(void)
{
    struct uwb_net_ctx c;
    to_ranging(&c, UWB_TIER_IDLE);

    bool saw = false;

    for (uint32_t fc = 1; fc <= UWB_NET_SCHED_GAP_MAX; fc++) {
        struct uwb_net_event b = ev_beacon(fc, false, 0);
        uint32_t a = uwb_net_handle(&c, &b);

        if (a & UWB_ACT_SEND_KEEPALIVE) { saw = true; break; }
    }
    CHECK(saw);
}

/* seat_id and tx_slot are different things, and conflating them is the trap
 * this split exists to remove: the runner uses tx_slot for TDMA timing, and a
 * seat id can reach GW_MAX_SEATS (128), which as a slot offset would place the
 * poll far outside the superframe. */
static void test_seat_id_and_tx_slot_are_independent(void)
{
    struct uwb_net_ctx c;
    uwb_net_init(&c, EUI);

    struct uwb_net_event b = ev_beacon(0, false, 0);
    uwb_net_handle(&c, &b);

    /* A seat id well beyond N_CFP, as the gateway will now hand out. */
    struct uwb_net_event g = ev_grant(0x0123, 77, UWB_TIER_FAST, UWB_NET_LEASE_SF);
    uwb_net_handle(&c, &g);
    CHECK(c.seat_id == 77);

    struct uwb_net_event d;
    memset(&d, 0, sizeof(d));
    d.kind = UWB_EV_DISCOVERED;
    d.n_anchors = 4;
    uwb_net_handle(&c, &d);

    /* The beacon schedules it in slot 2. seat_id must NOT follow. */
    struct uwb_net_event b2 = ev_beacon(1, true, 2);
    uwb_net_handle(&c, &b2);
    CHECK(c.tx_slot == 2);
    CHECK(c.seat_id == 77);

    /* A different slot next time; the seat is still the same seat. */
    struct uwb_net_event b3 = ev_beacon(2, true, 9);
    uwb_net_handle(&c, &b3);
    CHECK(c.tx_slot == 9);
    CHECK(c.seat_id == 77);
}

/* A fresh GRANT must start the unscheduled-gap clock. Left at 0 it would look
 * like a 4-billion-superframe gap against any live gateway counter and drop the
 * tag straight back to SCAN on its first beacon. */
static void test_grant_seeds_the_gap_clock(void)
{
    struct uwb_net_ctx c;
    uwb_net_init(&c, EUI);

    /* A gateway that has been up a long time. */
    struct uwb_net_event b = ev_beacon(4000000000u, false, 0);
    uwb_net_handle(&c, &b);

    struct uwb_net_event g = ev_grant(0x0009, 5, UWB_TIER_IDLE, UWB_NET_LEASE_SF);
    uwb_net_handle(&c, &g);
    CHECK(c.state == UWB_ST_DISCOVER);

    struct uwb_net_event b2 = ev_beacon(4000000001u, false, 0);
    uwb_net_handle(&c, &b2);
    CHECK(c.state == UWB_ST_DISCOVER);      /* not thrown back to SCAN */
}

/* The gateway's frame counter wraps at 2^32. The gap is an unsigned difference,
 * so a tag straddling the wrap must not read it as an enormous absence. */
static void test_gap_survives_frame_counter_wrap(void)
{
    struct uwb_net_ctx c;
    uwb_net_init(&c, EUI);

    struct uwb_net_event b = ev_beacon(0xFFFFFFF0u, false, 0);
    uwb_net_handle(&c, &b);
    struct uwb_net_event g = ev_grant(0x000A, 1, UWB_TIER_FAST, UWB_NET_LEASE_SF);
    uwb_net_handle(&c, &g);
    struct uwb_net_event d;
    memset(&d, 0, sizeof(d));
    d.kind = UWB_EV_DISCOVERED;
    d.n_anchors = 4;
    uwb_net_handle(&c, &d);

    /* Straight across the wrap, scheduled throughout. */
    uint32_t fcs[] = { 0xFFFFFFF1u, 0xFFFFFFF8u, 0xFFFFFFFFu, 0u, 1u, 8u };

    for (unsigned int i = 0; i < sizeof(fcs) / sizeof(fcs[0]); i++) {
        struct uwb_net_event bi = ev_beacon(fcs[i], true, 0);

        uwb_net_handle(&c, &bi);
        CHECK(c.state == UWB_ST_RANGING);
    }
}

/* DISCOVER has the same rule: unscheduled means sleep, not tear down. A tag
 * that has a seat but not yet enough anchors must keep the seat. */
static void test_discover_absent_from_map_keeps_seat(void)
{
    struct uwb_net_ctx c;
    uwb_net_init(&c, EUI);

    struct uwb_net_event b = ev_beacon(0, false, 0);
    uwb_net_handle(&c, &b);
    struct uwb_net_event g = ev_grant(0x0007, 3, UWB_TIER_SLOW, UWB_NET_LEASE_SF);
    uwb_net_handle(&c, &g);

    for (uint32_t fc = 1; fc <= 4; fc++) {
        struct uwb_net_event bi = ev_beacon(fc, false, 0);
        uint32_t a = uwb_net_handle(&c, &bi);

        CHECK(c.state == UWB_ST_DISCOVER);
        CHECK(!(a & UWB_ACT_RUN_DISCOVER));   /* discovery needs a slot */
        CHECK(a & UWB_ACT_SLEEP);
    }

    /* Scheduled: now it may discover. */
    struct uwb_net_event bs = ev_beacon(5, true, 1);
    CHECK(uwb_net_handle(&c, &bs) & UWB_ACT_RUN_DISCOVER);
}

/* ---- T5: listen_skip, the tier period and the lease are one parameter --- */

/* The inequality that makes a wake cadence sustainable. If this fails, a tag
 * at that tier sleeps past its own renewal deadline and loses its seat every
 * cycle -- which is the bench failure uwb_net.h records, and the reason the
 * lease moved from 50 to 75. */
static void test_every_tier_period_fits_the_lease(void)
{
    const uwb_tier_t tiers[] = { UWB_TIER_IDLE, UWB_TIER_SLOW,
        		     UWB_TIER_FAST };

    for (unsigned int i = 0; i < 3; i++) {
        uint16_t p = uwb_net_tier_period(tiers[i]);

        /* Renewal happens at half the lease, so the wake cadence plus a
         * margin for missed beacons must fit inside that half. */
        CHECK(p + UWB_NET_LEASE_MARGIN_SF < (UWB_NET_LEASE_SF / 2));
    }

    /* And at the OLD lease of 50 the slowest tier did NOT fit -- 25 + 4 is
     * not less than 25. Pinned so the reason for 75 cannot be forgotten and
     * quietly reverted. */
    CHECK(!(UWB_NET_PERIOD_IDLE + UWB_NET_LEASE_MARGIN_SF < (50 / 2)));
}

/* The skip is DERIVED from the granted tier, not configured beside it. Under
 * contract v3 the gateway reserves airtime per tier and schedules the tag once
 * per tier period, so a skip longer than the period sleeps through slots
 * reserved for it -- wasting capacity that now costs other tags. The design
 * values (300 / 75 / 1) are wrong in exactly that way. */
static void test_listen_skip_matches_the_tier_period(void)
{
    CHECK(uwb_net_tier_period(UWB_TIER_FAST) == 1u);
    CHECK(uwb_net_tier_period(UWB_TIER_SLOW) == 5u);
    CHECK(uwb_net_tier_period(UWB_TIER_IDLE) == 25u);

    for (unsigned int t = 0; t < UWB_TIER_COUNT; t++) {
        uwb_tier_t tier = (uwb_tier_t)t;
        uint16_t skip = uwb_net_tier_listen_skip(tier);

        CHECK(skip >= 1u);                       /* never zero */
        CHECK(skip <= UWB_LISTEN_SKIP_CAP);
        /* Never oversleeps its reserved slots. */
        CHECK(skip <= uwb_net_tier_period(tier));
    }

    /* The cap equals the slowest tier period, which is what makes it a
     * consequence of the tier table rather than an arbitrary clamp. */
    CHECK(UWB_LISTEN_SKIP_CAP == UWB_NET_PERIOD_IDLE);

    /* An out-of-range tier reads as the SLOWEST, never the fastest, so a
     * corrupt grant cannot make a tag transmit more often than allowed. */
    CHECK(uwb_net_tier_period((uwb_tier_t)99) == UWB_NET_PERIOD_IDLE);
    CHECK(uwb_net_tier_listen_skip((uwb_tier_t)99) <=
          UWB_NET_PERIOD_IDLE);
}

/* A tag sleeping its derived skip must survive indefinitely. This is the whole
 * point of the exercise: the power saving the tag never actually collected,
 * because every configuration that saved anything also lost the seat. */
static void test_idle_tag_survives_sleeping_its_own_cadence(void)
{
    struct uwb_net_ctx c;
    to_ranging(&c, UWB_TIER_IDLE);

    uint16_t skip = uwb_net_tier_listen_skip(UWB_TIER_IDLE);
    uint32_t fc = c.frame_counter;

    for (int i = 0; i < 400; i++) {
        fc += skip;
        /* Scheduled on the wakes the gateway reserved for it. */
        struct uwb_net_event b = ev_beacon(fc, true, 2);

        uwb_net_handle(&c, &b);
        CHECK(c.state == UWB_ST_RANGING);
        CHECK(c.lease_remaining > 0);         /* never expires */
    }
}

/* The default listen_skip of every tier must BE that tier's period.
 *
 * This is the test that would have caught the live defect the Phase 1 review
 * found: T5 derived uwb_net_tier_listen_skip() from the tier period and
 * host-tested it, but nothing called it -- the runner kept reading
 * tier_params[].listen_skip, whose SLOW default was the design's 75. The
 * read-clamp to UWB_LISTEN_SKIP_CAP hid half the problem and made the other
 * half look correct: IDLE's 300 clamped to 25, which happens to equal the IDLE
 * period, so IDLE passed by accident. SLOW's 75 clamped to 25 as well -- against
 * a SLOW period of 5, i.e. the tag awake for one in every five slots the
 * gateway had already reserved and charged against GW_SCHED_CAPACITY for it.
 *
 * A clamp is not agreement. Pin the derivation. */
static void test_defaults_are_the_tier_periods(void)
{
    const uwb_tier_t tiers[] = { UWB_TIER_IDLE, UWB_TIER_SLOW, UWB_TIER_FAST };

    uwb_net_reset_tier_params();

    for (unsigned int i = 0; i < 3; i++) {
        struct uwb_tier_params p;

        uwb_net_get_tier_params(tiers[i], &p);
        CHECK(p.listen_skip == uwb_net_tier_listen_skip(tiers[i]));
        CHECK(p.listen_skip == uwb_net_tier_period(tiers[i]));
    }

    /* And the specific value that was wrong, named so a revert is loud: SLOW
     * must not read back as the cap. */
    struct uwb_tier_params slow;

    uwb_net_get_tier_params(UWB_TIER_SLOW, &slow);
    CHECK(slow.listen_skip == 5u);
    CHECK(slow.listen_skip != UWB_LISTEN_SKIP_CAP);
}

/* TDoA blink mode: same cadence slot, different emission, and never both.
 * Also pins the default -- a zero-initialised context must still sweep. */
static void test_blink_mode(void)
{
    /* Default (blink_mode false) is unchanged TWR behaviour. */
    struct uwb_net_ctx c; to_ranging(&c, UWB_TIER_FAST);
    CHECK(c.blink_mode == false);
    struct uwb_net_event b = ev_beacon(2, true, 3);
    uint32_t a = uwb_net_handle(&c, &b);
    CHECK(a & UWB_ACT_RUN_SWEEP);
    CHECK(!(a & UWB_ACT_SEND_BLINK));

    /* Blink mode: BLINK instead of SWEEP, on exactly the same superframes. */
    struct uwb_net_ctx cb; to_ranging(&cb, UWB_TIER_FAST);
    cb.blink_mode = true;
    struct uwb_net_event bb = ev_beacon(2, true, 3);
    uint32_t ab = uwb_net_handle(&cb, &bb);
    CHECK(ab & UWB_ACT_SEND_BLINK);
    CHECK(!(ab & UWB_ACT_RUN_SWEEP));
    /* tx_slot is NOT latched in blink mode -- Task 4B (blink-slotted MAC)
     * moved the tag's transmit slot from the beacon's per-superframe
     * rotation (tx_slot) to its own seat_id (blink_slot_for_seat() in
     * uwb_net_runner.c). Asserting tx_slot == 3 here was this test's own
     * pre-Task-4B assumption that a blinking tag still occupies a TWR
     * rotation slot, which is now stale. */
    CHECK(cb.tx_slot == 0);

    /* The cadence itself is unchanged: drive range_every = 3 in both modes
     * and require the two action words to differ ONLY in which of the two
     * bits is set. */
    uwb_net_reset_tier_params();
    struct uwb_tier_params every3 = { 75u, 3u };
    uwb_net_set_tier_params(UWB_TIER_SLOW, &every3);

    struct uwb_net_ctx cs;  to_ranging(&cs,  UWB_TIER_SLOW);
    struct uwb_net_ctx csb; to_ranging(&csb, UWB_TIER_SLOW);
    csb.blink_mode = true;
    /* Small sequential deltas, deliberately NOT the big skips this array
     * used before Task 4B's forced-rejoin safety net
     * (UWB_NET_BLINK_KA_CYCLES_MAX): those jumps aged lease_remaining past
     * the KEEPALIVE threshold on nearly every iteration, which now forces
     * blink mode into TO_SCAN partway through -- correctly, since that is
     * exactly the mechanism test_blink_mode_forces_rejoin_after_stale_ka()
     * below tests on purpose. This section's own job is cadence-bit parity
     * between TWR and blink, which does not need a lease renewal at all. */
    static const uint32_t fcs2[] = { 1, 2, 3, 4, 5, 6, 7 };
    unsigned n_blinks = 0;
    for (unsigned i = 0; i < sizeof(fcs2) / sizeof(fcs2[0]); i++) {
        struct uwb_net_event e1 = ev_beacon(fcs2[i], true, 3);
        struct uwb_net_event e2 = ev_beacon(fcs2[i], true, 3);
        uint32_t a1 = uwb_net_handle(&cs,  &e1);
        uint32_t a2 = uwb_net_handle(&csb, &e2);
        bool due = ((i + 1) % 3u) == 0u;
        CHECK(((a1 & UWB_ACT_RUN_SWEEP)  != 0) == due);
        CHECK(((a2 & UWB_ACT_SEND_BLINK) != 0) == due);
        CHECK(!(a2 & UWB_ACT_RUN_SWEEP));
        /* Everything else -- keepalive, sleep, to-scan -- identical. */
        CHECK((a1 & ~(UWB_ACT_RUN_SWEEP | UWB_ACT_SEND_BLINK)) ==
              (a2 & ~(UWB_ACT_RUN_SWEEP | UWB_ACT_SEND_BLINK)));
        if (due) { n_blinks++; }
    }
    /* Negative-control guard: the loop must actually have exercised the due
     * branch, so a future change that skips every iteration cannot pass. */
    CHECK(n_blinks == 2u);
    uwb_net_reset_tier_params();

    /* BLINK is not gated on calibration -- see UWB_ACT_SEND_BLINK's comment:
     * the tag's TX antenna delay is common-mode and cancels in the range
     * differences the gateway solves. */
    CHECK(uwb_net_gate_actions(UWB_ACT_SEND_BLINK, false) & UWB_ACT_SEND_BLINK);
}

/* Regression for the bug this project actually hit on the bench (2026-08-31):
 * a Task 4B gateway in BLINK mode transmits sched[] reserved/zero (design
 * section 1.3), so `in_map` is never true for ANY blinking tag, ever. Before
 * this fix, uwb_net_handle() treated that exactly like a TWR tag that lost
 * its rotation turn: SLEEP every superframe, then TO_SCAN once
 * UWB_NET_SCHED_GAP_MAX superframes elapsed with no in_map beacon -- forcing
 * every blinking tag into an infinite SCAN -> JOIN -> DISCOVER -> RANGING ->
 * (silently sleep for 15 s) -> TO_SCAN loop that never once emits
 * UWB_ACT_SEND_BLINK. Observed on hardware as E2/E4 discovery traffic
 * recurring every ~15 s with no BLINK on the air. */
static void test_blink_mode_ignores_in_map(void)
{
    struct uwb_net_ctx c;

    to_ranging(&c, UWB_TIER_FAST);
    c.blink_mode = true;

    /* More superframes than UWB_NET_SCHED_GAP_MAX, all with in_map == false
     * -- exactly what a real BLINK-mode gateway's beacon looks like to
     * every tag it has admitted. Every single one must still emit
     * UWB_ACT_SEND_BLINK (FAST tier: every participation) and NEVER
     * UWB_ACT_TO_SCAN or UWB_ACT_SLEEP. */
    for (uint32_t fc = 1; fc <= (uint32_t)UWB_NET_SCHED_GAP_MAX + 20u; fc++) {
        struct uwb_net_event ev = ev_beacon(fc, false, 0);
        uint32_t act = uwb_net_handle(&c, &ev);

        CHECK(act & UWB_ACT_SEND_BLINK);
        CHECK(!(act & UWB_ACT_RUN_SWEEP));
        CHECK(!(act & UWB_ACT_TO_SCAN));
        CHECK(!(act & UWB_ACT_SLEEP));
        CHECK(c.state == UWB_ST_RANGING);
    }
}

/* Regression for the bug found on the bench 2026-08-31 running 5 tags: a
 * blink-mode tag whose seat was silently reclaimed (KEEPALIVE has no ack,
 * "the contract has no such frame") kept blinking on its stale seat_id
 * forever, with zero warning -- and gw_core_join() hands that exact seat_id
 * to the next fresh joiner, so two tags end up transmitting in the same
 * blink_sched slot indefinitely. See UWB_NET_BLINK_KA_CYCLES_MAX's comment
 * in uwb_net.h for the full mechanism. This proves the stopgap: after
 * UWB_NET_BLINK_KA_CYCLES_MAX optimistic renewals with no intervening JOIN,
 * the tag forces itself back to SCAN instead of renewing forever. */
static void test_blink_mode_forces_rejoin_after_stale_ka(void)
{
    struct uwb_net_ctx c;

    to_ranging(&c, UWB_TIER_FAST);
    c.blink_mode = true;

    uint32_t fc = 0;
    bool     saw_to_scan = false;

    /* Each step ages the lease past its half-life so every beacon triggers
     * an "optimistic" KEEPALIVE renewal -- exactly the silently-failing-
     * forever case this stopgap exists for, since there is no ack to tell
     * the tag any of them actually landed. */
    for (unsigned i = 0; i < UWB_NET_BLINK_KA_CYCLES_MAX + 2u; i++) {
        fc += (UWB_NET_LEASE_SF / 2u) + 1u;
        struct uwb_net_event ev = ev_beacon(fc, false, 0);
        uint32_t act = uwb_net_handle(&c, &ev);

        if (act & UWB_ACT_TO_SCAN) {
            saw_to_scan = true;
            CHECK(c.state == UWB_ST_SCAN);
            break;
        }
    }

    CHECK(saw_to_scan);
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
    test_blink_skips_discover();
    test_blink_mode_ignores_in_map();
    test_blink_mode_forces_rejoin_after_stale_ka();
    test_discover_keeps_lease();
    test_ranging();
    test_lease_ages_by_elapsed();
    test_every_tier_period_fits_the_lease();
    test_listen_skip_matches_the_tier_period();
    test_idle_tag_survives_sleeping_its_own_cadence();
    test_defaults_are_the_tier_periods();
    test_gate_actions();
    test_send_alert();
    test_blink_mode();
    test_sweep_gate_recovers_after_short_sweep();
    test_sweep_gate_interval();
    test_absent_from_map_sleeps_and_keeps_seat();
    test_absence_beyond_the_gap_rejoins();
    test_scheduled_at_cadence_never_times_out();
    test_keepalive_fires_while_unscheduled();
    test_seat_id_and_tx_slot_are_independent();
    test_grant_seeds_the_gap_clock();
    test_gap_survives_frame_counter_wrap();
    test_discover_absent_from_map_keeps_seat();
    printf(g_fail ? "FAILED %d\n" : "OK\n", g_fail);
    return g_fail ? 1 : 0;
}
