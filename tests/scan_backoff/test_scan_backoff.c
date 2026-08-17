#include "scan_backoff_core.h"
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c); g_fail++; } } while(0)

static void test_climb_and_saturate(void)
{
    struct scan_backoff b;

    memset(&b, 0, sizeof(b));
    scan_backoff_reset(&b);
    CHECK(scan_backoff_rung(&b) == 0);
    CHECK(scan_backoff_next_ms(&b) == 10u * 1000u);

    for (uint8_t i = 1; i < SCAN_BACKOFF_RUNGS; i++) {
        scan_backoff_fail(&b);
        CHECK(scan_backoff_rung(&b) == i);
    }

    /* Terminal rung: further failures must not climb past the table. */
    uint32_t top = scan_backoff_next_ms(&b);
    for (int i = 0; i < 50; i++) {
        scan_backoff_fail(&b);
    }
    CHECK(scan_backoff_rung(&b) == SCAN_BACKOFF_RUNGS - 1u);
    CHECK(scan_backoff_next_ms(&b) == top);
    CHECK(top == 30u * 60u * 1000u);
}

/* Strictly increasing, so climbing a rung always costs less current. */
static void test_intervals_monotonic(void)
{
    struct scan_backoff b;
    uint32_t prev = 0;

    memset(&b, 0, sizeof(b));
    for (uint8_t r = 0; r < SCAN_BACKOFF_RUNGS; r++) {
        b.rung = r;
        uint32_t ms = scan_backoff_next_ms(&b);
        CHECK(ms > prev);
        prev = ms;
    }

    /* A rung index past the table (corrupt state) must still return a legal
     * interval rather than read off the end. */
    b.rung = 200;
    CHECK(scan_backoff_next_ms(&b) == prev);
}

static void test_motion_resets_from_every_rung(void)
{
    for (uint8_t r = 0; r < SCAN_BACKOFF_RUNGS; r++) {
        struct scan_backoff b;

        memset(&b, 0, sizeof(b));
        b.rung = r;
        scan_backoff_motion(&b);
        CHECK(scan_backoff_rung(&b) == 0);
        CHECK(scan_backoff_next_ms(&b) == 10u * 1000u);
    }
}

static void test_alert_pin(void)
{
    struct scan_backoff b;

    memset(&b, 0, sizeof(b));
    scan_backoff_reset(&b);
    scan_backoff_fail(&b);
    scan_backoff_fail(&b);
    scan_backoff_fail(&b);
    CHECK(scan_backoff_rung(&b) == 3);

    /* Raising a HELP drops straight to rung 0 and holds it there. */
    scan_backoff_alert(&b, true);
    CHECK(scan_backoff_rung(&b) == 0);
    for (int i = 0; i < 20; i++) {
        scan_backoff_fail(&b);
        CHECK(scan_backoff_rung(&b) == 0);
    }

    /* Releasing the pin leaves the rung where it is -- it must not jump. */
    scan_backoff_alert(&b, false);
    CHECK(scan_backoff_rung(&b) == 0);

    /* ...and the ladder resumes climbing normally from there. */
    scan_backoff_fail(&b);
    CHECK(scan_backoff_rung(&b) == 1);

    /* Release from a mid-ladder rung: same rule, no jump either way. */
    struct scan_backoff c;
    memset(&c, 0, sizeof(c));
    c.rung = 4;
    c.alert_pinned = true;
    scan_backoff_alert(&c, false);
    CHECK(scan_backoff_rung(&c) == 4);

    /* A coverage reset must not clear a standing alert pin: the alert state
     * owns that flag, not the beacon. */
    struct scan_backoff d;
    memset(&d, 0, sizeof(d));
    scan_backoff_alert(&d, true);
    scan_backoff_reset(&d);
    scan_backoff_fail(&d);
    CHECK(scan_backoff_rung(&d) == 0);
}

int main(void)
{
    test_climb_and_saturate();
    test_intervals_monotonic();
    test_motion_resets_from_every_rung();
    test_alert_pin();
    printf("scan_backoff_core: %d failure(s)\n", g_fail);
    return g_fail ? 1 : 0;
}
