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

int main(void)
{
    test_init_and_cadence();
    printf(g_fail ? "FAILED %d\n" : "OK\n", g_fail);
    return g_fail ? 1 : 0;
}
