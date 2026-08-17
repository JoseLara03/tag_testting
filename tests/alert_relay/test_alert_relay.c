#include "alert_relay.h"
#include "uwb_frame_802_15_4z.h"
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c); g_fail++; } } while(0)

static const uint8_t EUI_A[8] = {0xAA,0,0,0,0,0,0,1};
static const uint8_t EUI_B[8] = {0xBB,0,0,0,0,0,0,2};

static struct uwb_alert mk(const uint8_t eui[8], uint8_t state, uint8_t epoch,
                           uint8_t repeat_seq, uint8_t sender_hop, uint8_t ttl)
{
    struct uwb_alert a;
    memset(&a, 0, sizeof(a));
    memcpy(a.orig_eui, eui, 8);
    a.state       = state;
    a.epoch       = epoch;
    a.repeat_seq  = repeat_seq;
    a.sender_hop  = sender_hop;
    a.ttl         = ttl;
    a.orig_addr   = 0x1234;
    a.batt_soc    = 50;
    a.last_x      = 1.0f;
    a.last_y      = 2.0f;
    return a;
}

/* ---- latch tests -------------------------------------------------------- */

static void test_latch_stale_help_after_cancel(void)
{
    struct alert_latch l; alert_latch_reset(&l);

    struct uwb_alert h5 = mk(EUI_A, UWB_ALERT_STATE_HELP, 5, 0, 0xFF, 6);
    CHECK(alert_latch_apply(&l, &h5, 1000) == ALERT_LATCH_RAISED);

    struct uwb_alert c5 = mk(EUI_A, UWB_ALERT_STATE_CANCEL, 5, 0, 0xFF, 6);
    CHECK(alert_latch_apply(&l, &c5, 2000) == ALERT_LATCH_CLEARED);

    /* A relayed copy of the original HELP(5) arriving late (different
     * repeat_seq downstream is irrelevant to the latch -- it keys on eui). */
    struct uwb_alert h5late = mk(EUI_A, UWB_ALERT_STATE_HELP, 5, 3, 1, 4);
    CHECK(alert_latch_apply(&l, &h5late, 3000) == ALERT_LATCH_IGNORED);
}

static void test_latch_new_emergency_after_cancel(void)
{
    struct alert_latch l; alert_latch_reset(&l);

    struct uwb_alert h5 = mk(EUI_A, UWB_ALERT_STATE_HELP, 5, 0, 0xFF, 6);
    CHECK(alert_latch_apply(&l, &h5, 1000) == ALERT_LATCH_RAISED);
    struct uwb_alert c5 = mk(EUI_A, UWB_ALERT_STATE_CANCEL, 5, 0, 0xFF, 6);
    CHECK(alert_latch_apply(&l, &c5, 2000) == ALERT_LATCH_CLEARED);

    struct uwb_alert h6 = mk(EUI_A, UWB_ALERT_STATE_HELP, 6, 0, 0xFF, 6);
    CHECK(alert_latch_apply(&l, &h6, 3000) == ALERT_LATCH_RAISED);
}

static void test_latch_epoch_wrap(void)
{
    struct alert_latch l; alert_latch_reset(&l);

    struct uwb_alert c250 = mk(EUI_A, UWB_ALERT_STATE_CANCEL, 250, 0, 0xFF, 6);
    CHECK(alert_latch_apply(&l, &c250, 1000) == ALERT_LATCH_CLEARED);

    struct uwb_alert h2 = mk(EUI_A, UWB_ALERT_STATE_HELP, 2, 0, 0xFF, 6);
    CHECK(alert_latch_apply(&l, &h2, 2000) == ALERT_LATCH_RAISED);

    struct uwb_alert h250 = mk(EUI_A, UWB_ALERT_STATE_HELP, 250, 0, 0xFF, 6);
    CHECK(alert_latch_apply(&l, &h250, 3000) == ALERT_LATCH_IGNORED);
}

static void test_latch_cancel_no_prior_help(void)
{
    struct alert_latch l; alert_latch_reset(&l);

    struct uwb_alert c7 = mk(EUI_A, UWB_ALERT_STATE_CANCEL, 7, 0, 0xFF, 6);
    CHECK(alert_latch_apply(&l, &c7, 1000) == ALERT_LATCH_CLEARED);

    /* Recorded: a later HELP with the very same epoch is now stale. */
    struct uwb_alert h7 = mk(EUI_A, UWB_ALERT_STATE_HELP, 7, 0, 0xFF, 6);
    CHECK(alert_latch_apply(&l, &h7, 2000) == ALERT_LATCH_IGNORED);
}

static void test_latch_refresh_and_independent_euis(void)
{
    struct alert_latch l; alert_latch_reset(&l);

    struct uwb_alert h1 = mk(EUI_A, UWB_ALERT_STATE_HELP, 1, 0, 0xFF, 6);
    CHECK(alert_latch_apply(&l, &h1, 1000) == ALERT_LATCH_RAISED);
    /* A repeat of the same active epoch just refreshes. */
    struct uwb_alert h1rep = mk(EUI_A, UWB_ALERT_STATE_HELP, 1, 1, 0xFF, 6);
    CHECK(alert_latch_apply(&l, &h1rep, 1500) == ALERT_LATCH_REFRESHED);

    /* A different originator EUI has entirely independent state. */
    struct uwb_alert hB = mk(EUI_B, UWB_ALERT_STATE_HELP, 1, 0, 0xFF, 6);
    CHECK(alert_latch_apply(&l, &hB, 1600) == ALERT_LATCH_RAISED);
}

/* ---- dedup tests ---------------------------------------------------------- */

static void test_dedup_basic(void)
{
    struct alert_dedup d; alert_dedup_reset(&d);
    struct uwb_alert a = mk(EUI_A, UWB_ALERT_STATE_HELP, 5, 0, 0xFF, 6);

    CHECK(alert_dedup_admit(&d, &a, 0) == true);      /* first time */
    CHECK(alert_dedup_admit(&d, &a, 10) == false);    /* exact repeat: duplicate */

    /* repeat_seq + 1 is a different key -- floods again. */
    struct uwb_alert a2 = a; a2.repeat_seq = 1;
    CHECK(alert_dedup_admit(&d, &a2, 20) == true);
    CHECK(alert_dedup_admit(&d, &a2, 30) == false);

    /* Same key, but the earlier entry aged out -- admitted again. */
    CHECK(alert_dedup_admit(&d, &a, ALERT_DEDUP_AGE_MS + 100) == true);
}

static void test_dedup_eviction_order(void)
{
    struct alert_dedup d; alert_dedup_reset(&d);

    /* Fill the cache with ALERT_DEDUP_N distinct keys at increasing seen_ms,
     * so entry 0 is the oldest and entry N-1 the newest. */
    for (int i = 0; i < ALERT_DEDUP_N; i++) {
        struct uwb_alert a = mk(EUI_A, UWB_ALERT_STATE_HELP, (uint8_t)i, 0, 0xFF, 6);
        CHECK(alert_dedup_admit(&d, &a, (uint32_t)(i * 100)) == true);
    }

    /* Cache is full and none are aged out. One more distinct key must evict
     * the oldest (entry for epoch 0, seen at t=0), not the newest. */
    struct uwb_alert fresh = mk(EUI_A, UWB_ALERT_STATE_HELP, 200, 0, 0xFF, 6);
    uint32_t now = (uint32_t)((ALERT_DEDUP_N - 1) * 100 + 1);
    CHECK(alert_dedup_admit(&d, &fresh, now) == true);

    /* The evicted (oldest) key is now free again -- admitted as new. */
    struct uwb_alert evicted = mk(EUI_A, UWB_ALERT_STATE_HELP, 0, 0, 0xFF, 6);
    CHECK(alert_dedup_admit(&d, &evicted, now + 1) == true);

    /* The newest original key must still be recognised as a duplicate --
     * proof the eviction did not touch it. */
    struct uwb_alert newest = mk(EUI_A, UWB_ALERT_STATE_HELP,
                                 (uint8_t)(ALERT_DEDUP_N - 1), 0, 0xFF, 6);
    CHECK(alert_dedup_admit(&d, &newest, now + 2) == false);
}

/* ---- relay decision tests ------------------------------------------------- */

static void test_relay_decision(void)
{
    struct uwb_alert a = mk(EUI_A, UWB_ALERT_STATE_HELP, 1, 0, 3, 0);
    CHECK(alert_should_relay(&a, 1, false) == false);   /* ttl == 0 */

    a.ttl = 6;
    CHECK(alert_should_relay(&a, 1, true) == false);    /* gateway never relays */

    a.sender_hop = 1;
    CHECK(alert_should_relay(&a, 1, false) == false);   /* sender_hop <= my_hop */
    CHECK(alert_should_relay(&a, 2, false) == false);

    a.sender_hop = 3;
    CHECK(alert_should_relay(&a, 1, false) == true);    /* sender_hop > my_hop */

    a.sender_hop = UWB_ALERT_HOP_UNKNOWN;
    CHECK(alert_should_relay(&a, 5, false) == true);    /* no gradient yet */
}

static void test_relay_prepare(void)
{
    struct uwb_alert a = mk(EUI_A, UWB_ALERT_STATE_HELP, 9, 2, UWB_ALERT_HOP_UNKNOWN, 6);
    a.orig_addr = 0x0102;
    a.batt_soc  = 77;
    a.last_x    = 3.5f;
    a.last_y    = -4.25f;

    struct uwb_alert before = a;
    alert_prepare_relay(&a, 4);

    CHECK(a.sender_hop == 4);
    CHECK(a.ttl == before.ttl - 1);
    /* Everything else byte-for-byte untouched. */
    CHECK(memcmp(a.orig_eui, before.orig_eui, sizeof(a.orig_eui)) == 0);
    CHECK(a.epoch == before.epoch);
    CHECK(a.state == before.state);
    CHECK(a.repeat_seq == before.repeat_seq);
    CHECK(a.orig_addr == before.orig_addr);
    CHECK(a.batt_soc == before.batt_soc);
    CHECK(memcmp(&a.last_x, &before.last_x, sizeof(a.last_x)) == 0);
    CHECK(memcmp(&a.last_y, &before.last_y, sizeof(a.last_y)) == 0);
}

int main(void)
{
    test_latch_stale_help_after_cancel();
    test_latch_new_emergency_after_cancel();
    test_latch_epoch_wrap();
    test_latch_cancel_no_prior_help();
    test_latch_refresh_and_independent_euis();
    test_dedup_basic();
    test_dedup_eviction_order();
    test_relay_decision();
    test_relay_prepare();
    printf(g_fail ? "FAILED %d\n" : "OK\n", g_fail);
    return g_fail ? 1 : 0;
}
