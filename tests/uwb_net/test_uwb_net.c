#include "uwb_net.h"
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c); g_fail++; } } while(0)

static const uint8_t EUI[8] = {0xDE,0xAD,0xBE,0xEF,0,0,0,1};

static void test_init_and_cadence(void)
{
    struct uwb_net_ctx c;
    uwb_net_init(&c, EUI);
    CHECK(c.state == UWB_ST_SCAN);
    CHECK(c.req_tier == UWB_TIER_FAST);
    CHECK(memcmp(c.eui, EUI, 8) == 0);

    /* FAST: every superframe. */
    CHECK(uwb_tier_due(UWB_TIER_FAST, 0));
    CHECK(uwb_tier_due(UWB_TIER_FAST, 7));
    /* SLOW: every 5. */
    CHECK(uwb_tier_due(UWB_TIER_SLOW, 10));
    CHECK(!uwb_tier_due(UWB_TIER_SLOW, 11));
    /* IDLE: every 25. */
    CHECK(uwb_tier_due(UWB_TIER_IDLE, 50));
    CHECK(!uwb_tier_due(UWB_TIER_IDLE, 51));
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

    /* Wrong proto version: ignored, stays SCAN. */
    struct uwb_net_event bad = ev_beacon(1, false, 0); bad.proto_ver = 99;
    CHECK(uwb_net_handle(&c, &bad) == UWB_ACT_NONE);
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

    /* SLOW tier, frame_counter not a multiple of 5 -> sleep, no sweep. */
    struct uwb_net_ctx cs; to_ranging(&cs, UWB_TIER_SLOW);
    struct uwb_net_event b1 = ev_beacon(1, true, 3);
    CHECK(uwb_net_handle(&cs, &b1) == UWB_ACT_SLEEP);
    struct uwb_net_event b5 = ev_beacon(5, true, 3);
    CHECK(uwb_net_handle(&cs, &b5) & UWB_ACT_RUN_SWEEP);

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

static void test_gate_actions(void)
{
    const uint32_t ranging = UWB_ACT_RUN_DISCOVER | UWB_ACT_RUN_SWEEP;
    const uint32_t housekeeping = UWB_ACT_SEND_JOIN | UWB_ACT_SEND_KEEPALIVE
                                | UWB_ACT_SLEEP | UWB_ACT_TO_SCAN;

    /* Calibrated: every action passes through untouched. */
    CHECK(uwb_net_gate_actions(ranging | housekeeping, true)
          == (ranging | housekeeping));

    /* Uncalibrated: both ranging actions are cleared. */
    CHECK((uwb_net_gate_actions(ranging, false) & UWB_ACT_RUN_DISCOVER) == 0);
    CHECK((uwb_net_gate_actions(ranging, false) & UWB_ACT_RUN_SWEEP) == 0);

    /* Uncalibrated: everything that keeps the seat survives. This is the
     * property that matters -- a tag that stops ranging must not also stop
     * renewing its lease, or it silently drops off the network. */
    CHECK(uwb_net_gate_actions(ranging | housekeeping, false) == housekeeping);

    /* Nothing in, nothing out, either way. */
    CHECK(uwb_net_gate_actions(UWB_ACT_NONE, true) == UWB_ACT_NONE);
    CHECK(uwb_net_gate_actions(UWB_ACT_NONE, false) == UWB_ACT_NONE);
}

int main(void)
{
    test_init_and_cadence();
    test_scan_join();
    test_grant_discover();
    test_discover_keeps_lease();
    test_ranging();
    test_gate_actions();
    printf(g_fail ? "FAILED %d\n" : "OK\n", g_fail);
    return g_fail ? 1 : 0;
}
