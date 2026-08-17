#include "tag_alert_core.h"
#include <stdio.h>

static int g_fail = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c); g_fail++; } } while(0)

static void test_raise_advances_epoch_once(void)
{
    struct tag_alert_core c;
    tag_alert_core_init(&c, 0);

    CHECK(tag_alert_core_raise(&c, 1000) == true);
    CHECK(c.state == TAG_ALERT_HELP);
    CHECK(c.epoch == 1);
    CHECK(c.repeat_seq == 0);

    uint8_t st, ep, rep;
    CHECK(tag_alert_core_due(&c, 1000, &st, &ep, &rep) == true);
    CHECK(st == UWB_ALERT_STATE_HELP && ep == 1 && rep == 0);
}

static void test_double_raise_is_noop(void)
{
    struct tag_alert_core c;
    tag_alert_core_init(&c, 5);

    CHECK(tag_alert_core_raise(&c, 0) == true);
    CHECK(c.epoch == 6);

    /* A second double-press while already HELP must not burn another epoch
     * or restart the repeat sequence. */
    tag_alert_core_sent(&c, 0);   /* repeat_seq now 1 */
    CHECK(tag_alert_core_raise(&c, 100) == false);
    CHECK(c.epoch == 6);
    CHECK(c.repeat_seq == 1);
    CHECK(c.state == TAG_ALERT_HELP);
}

static void test_help_repeats_forever(void)
{
    struct tag_alert_core c;
    tag_alert_core_init(&c, 0);
    tag_alert_core_raise(&c, 0);

    uint32_t now = 0;
    for (int i = 0; i < 100; i++) {
        uint8_t st, ep, rep;
        CHECK(tag_alert_core_due(&c, now, &st, &ep, &rep) == true);
        CHECK(st == UWB_ALERT_STATE_HELP);
        tag_alert_core_sent(&c, now);
        CHECK(c.state == TAG_ALERT_HELP);   /* never stops on its own */
        now += ALERT_REPEAT_MS;
    }
    CHECK(c.repeat_seq == 100);
}

static void test_cancel_keeps_epoch_and_stops_after_n(void)
{
    struct tag_alert_core c;
    tag_alert_core_init(&c, 0);
    tag_alert_core_raise(&c, 0);
    CHECK(c.epoch == 1);

    tag_alert_core_cancel(&c, 500);
    CHECK(c.state == TAG_ALERT_CANCELLING);
    CHECK(c.epoch == 1);            /* unchanged: names the epoch cancelled */
    CHECK(c.repeat_seq == 0);
    CHECK(c.cancels_left == ALERT_CANCEL_REPEATS);

    uint32_t now = 500;
    unsigned sends = 0;
    while (c.state == TAG_ALERT_CANCELLING) {
        uint8_t st, ep, rep;
        CHECK(tag_alert_core_due(&c, now, &st, &ep, &rep) == true);
        CHECK(st == UWB_ALERT_STATE_CANCEL);
        CHECK(ep == 1);
        tag_alert_core_sent(&c, now);
        sends++;
        now += ALERT_REPEAT_MS;
        CHECK(sends <= ALERT_CANCEL_REPEATS);   /* guard against a runaway loop */
    }
    CHECK(sends == ALERT_CANCEL_REPEATS);
    CHECK(c.state == TAG_ALERT_OFF);

    /* Off for good: no further TX due. */
    uint8_t st, ep, rep;
    CHECK(tag_alert_core_due(&c, now, &st, &ep, &rep) == false);
}

static void test_cancel_from_off_is_noop(void)
{
    struct tag_alert_core c;
    tag_alert_core_init(&c, 3);

    tag_alert_core_cancel(&c, 0);
    CHECK(c.state == TAG_ALERT_OFF);
    CHECK(c.epoch == 3);

    uint8_t st, ep, rep;
    CHECK(tag_alert_core_due(&c, 0, &st, &ep, &rep) == false);
}

static void test_raise_during_cancelling_starts_new_epoch(void)
{
    struct tag_alert_core c;
    tag_alert_core_init(&c, 0);
    tag_alert_core_raise(&c, 0);       /* epoch 1, HELP */
    tag_alert_core_cancel(&c, 100);    /* CANCELLING, epoch still 1 */
    CHECK(c.state == TAG_ALERT_CANCELLING);

    CHECK(tag_alert_core_raise(&c, 200) == true);
    CHECK(c.state == TAG_ALERT_HELP);
    CHECK(c.epoch == 2);               /* a genuinely new emergency */
    CHECK(c.repeat_seq == 0);

    uint8_t st, ep, rep;
    CHECK(tag_alert_core_due(&c, 200, &st, &ep, &rep) == true);
    CHECK(st == UWB_ALERT_STATE_HELP && ep == 2);
}

static void test_now_ms_wrap(void)
{
    struct tag_alert_core c;
    tag_alert_core_init(&c, 0);

    /* Raise right before the 32-bit wrap, then let the schedule cross it. */
    uint32_t near_wrap = 0xFFFFFFFFu - 100;
    CHECK(tag_alert_core_raise(&c, near_wrap) == true);

    uint8_t st, ep, rep;
    CHECK(tag_alert_core_due(&c, near_wrap, &st, &ep, &rep) == true);
    tag_alert_core_sent(&c, near_wrap);   /* next_tx_ms = near_wrap + 10000, wraps */

    /* Not due yet, just after the raise (signed-difference comparison must
     * not be fooled by the wrap). */
    CHECK(tag_alert_core_due(&c, near_wrap + 1, &st, &ep, &rep) == false);

    /* Once the wrapped deadline is reached, it still fires. */
    uint32_t deadline = near_wrap + ALERT_REPEAT_MS;   /* wraps past 0xFFFFFFFF */
    CHECK(tag_alert_core_due(&c, deadline, &st, &ep, &rep) == true);
}

int main(void)
{
    test_raise_advances_epoch_once();
    test_double_raise_is_noop();
    test_help_repeats_forever();
    test_cancel_keeps_epoch_and_stops_after_n();
    test_cancel_from_off_is_noop();
    test_raise_during_cancelling_starts_new_epoch();
    test_now_ms_wrap();
    printf(g_fail ? "FAILED %d\n" : "OK\n", g_fail);
    return g_fail ? 1 : 0;
}
