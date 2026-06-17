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

int main(void)
{
    test_init_and_cadence();
    test_scan_join();
    printf(g_fail ? "FAILED %d\n" : "OK\n", g_fail);
    return g_fail ? 1 : 0;
}
