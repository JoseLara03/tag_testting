#include "../../src/rx_stats_core.h"
#include <stdio.h>

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

#define HZ      1000000u   /* 1 cycle == 1 microsecond */
#define SF_CYC   200000u   /* 200 ms superframe in cycles */

int main(void)
{
    struct rx_stats_core c;
    int on_mean = -1, on_max = -1, off_min = -1, off_max = -1;
    uint32_t n = 0, miss = 0;

    /* Empty: get returns 0. */
    rx_stats_core_reset(&c, SF_CYC, HZ);
    CHECK(rx_stats_core_get(&c, &on_mean, &on_max, &off_min, &off_max, &n, &miss) == 0);

    /* First beacon: on-duration recorded, NO offset yet.
     * arm=1000, beacon=151000 -> on = 150000 us -> 150 ms. */
    rx_stats_core_beacon(&c, 1000u, 151000u);
    CHECK(rx_stats_core_get(&c, &on_mean, &on_max, &off_min, &off_max, &n, &miss) == 1);
    CHECK(on_mean == 150);
    CHECK(n == 1);
    CHECK(off_min == 0 && off_max == 0);   /* no offset sample yet */

    /* Second beacon LATE: prev=151000, predicted=351000, actual=351500 -> +500 us.
     * on = 351500-200000 = 151500 us -> 151 ms. */
    rx_stats_core_beacon(&c, 200000u, 351500u);
    rx_stats_core_get(&c, &on_mean, &on_max, &off_min, &off_max, &n, &miss);
    CHECK(off_min == 500 && off_max == 500);
    CHECK(on_max == 151);
    CHECK(n == 2);

    /* Third beacon EARLY: prev=351500, predicted=551500, actual=551200 -> -300 us. */
    rx_stats_core_beacon(&c, 400000u, 551200u);
    rx_stats_core_get(&c, &on_mean, &on_max, &off_min, &off_max, &n, &miss);
    CHECK(off_min == -300 && off_max == 500);
    CHECK(n == 3);

    /* Miss: invalidates phase + counts. The next beacon must NOT record an
     * offset across the gap. */
    rx_stats_core_miss(&c);
    rx_stats_core_beacon(&c, 800000u, 951000u);   /* fresh reference, no offset */
    rx_stats_core_get(&c, &on_mean, &on_max, &off_min, &off_max, &n, &miss);
    CHECK(miss == 1);
    CHECK(n == 4);
    CHECK(off_min == -300 && off_max == 500);     /* unchanged: no new offset */

    /* A following beacon DOES record an offset from the post-miss reference.
     * prev=951000, predicted=1151000, actual=1151100 -> +100 us (within spread). */
    rx_stats_core_beacon(&c, 1000000u, 1151100u);
    rx_stats_core_get(&c, &on_mean, &on_max, &off_min, &off_max, &n, &miss);
    CHECK(off_min == -300 && off_max == 500);
    CHECK(n == 5);

    /* Skipped-but-not-missed beacon (steady-state re-discovery): the gateway
     * beacon is periodic, so a ~2-superframe gap with NO recorded miss must FOLD
     * to the true sub-superframe phase, not report a spurious ~+1 superframe.
     * fresh: beacon at 100000 (no offset); next at 100000 + 2*SF - 10 = 499990.
     * delta = 399990, n = round(399990/200000) = 2, off = 399990 - 400000 = -10. */
    rx_stats_core_reset(&c, SF_CYC, HZ);
    rx_stats_core_beacon(&c, 0u, 100000u);          /* first, no offset */
    rx_stats_core_beacon(&c, 300000u, 499990u);     /* +2 SF gap, -10 us phase */
    rx_stats_core_get(&c, &on_mean, &on_max, &off_min, &off_max, &n, &miss);
    CHECK(off_min == -10 && off_max == -10);        /* folded, not ~+200000 */
    CHECK(n == 2);

    /* Reset clears everything. */
    rx_stats_core_reset(&c, SF_CYC, HZ);
    CHECK(rx_stats_core_get(&c, NULL, NULL, NULL, NULL, NULL, NULL) == 0);

    printf("rx_stats_core: %d failure(s)\n", fails);
    return fails == 0 ? 0 : 1;
}
