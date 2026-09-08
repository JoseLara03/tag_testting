#include "anchor_pool_core.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

static void test_insert_and_select(void)
{
    struct anchor_pool p;
    anchor_pool_init(&p);

    anchor_pool_observe_score(&p, 1, 10.0f, 1000);
    anchor_pool_observe_score(&p, 2, 30.0f, 1000);
    anchor_pool_observe_score(&p, 3, 20.0f, 1000);

    CHECK(anchor_pool_live_count(&p) == 3);

    uint8_t sel[ANCHOR_SELECT_MAX];
    uint8_t n = anchor_pool_select(&p, sel, ANCHOR_SELECT_MAX);
    CHECK(n == 3);
    CHECK(sel[0] == 2);   /* highest score first */
    CHECK(sel[1] == 3);
    CHECK(sel[2] == 1);
}

static void test_never_reports_more_than_live(void)
{
    struct anchor_pool p;
    anchor_pool_init(&p);

    /* Empty pool: select must not fabricate entries. */
    uint8_t sel[ANCHOR_SELECT_MAX];
    CHECK(anchor_pool_select(&p, sel, ANCHOR_SELECT_MAX) == 0);

    /* Two live anchors: never report 4 just because max_out allows it. */
    anchor_pool_observe_score(&p, 5, 1.0f, 0);
    anchor_pool_observe_score(&p, 6, 2.0f, 0);
    CHECK(anchor_pool_select(&p, sel, ANCHOR_SELECT_MAX) == 2);

    /* Fill past ANCHOR_SELECT_MAX: selection caps at max_out, not at live count. */
    for (uint8_t id = 10; id < 10 + ANCHOR_POOL_MAX; id++) {
        anchor_pool_observe_score(&p, id, (float)id, 0);
    }
    CHECK(anchor_pool_live_count(&p) == ANCHOR_POOL_MAX);
    CHECK(anchor_pool_select(&p, sel, ANCHOR_SELECT_MAX) == ANCHOR_SELECT_MAX);
}

static void test_ema_update(void)
{
    struct anchor_pool p;
    anchor_pool_init(&p);

    anchor_pool_observe_score(&p, 1, 10.0f, 0);
    /* Second observation blends toward the new score (EMA_ALPHA=0.3), not a
     * plain overwrite. */
    anchor_pool_observe_score(&p, 1, 20.0f, 100);
    float expect = 0.3f * 20.0f + 0.7f * 10.0f;   /* 13.0 */

    uint8_t sel[1];
    /* Insert a lower-scored competitor so ordering exposes the blended value. */
    anchor_pool_observe_score(&p, 2, expect - 1.0f, 100);
    CHECK(anchor_pool_select(&p, sel, 1) == 1);
    CHECK(sel[0] == 1);

    anchor_pool_observe_score(&p, 3, expect + 1.0f, 100);
    CHECK(anchor_pool_select(&p, sel, 1) == 1);
    CHECK(sel[0] == 3);   /* now outranks anchor 1 */
}

static void test_decay_missed(void)
{
    struct anchor_pool p;
    anchor_pool_init(&p);

    anchor_pool_observe_score(&p, 1, 10.0f, 0);
    anchor_pool_observe_score(&p, 2, 10.0f, 0);

    /* New round: anchor 1 answers again, anchor 2 does not. */
    anchor_pool_begin_round(&p);
    anchor_pool_observe_score(&p, 1, 10.0f, 100);
    anchor_pool_decay_missed(&p);

    uint8_t sel[2];
    CHECK(anchor_pool_select(&p, sel, 2) == 2);
    CHECK(sel[0] == 1);   /* decayed anchor 2 now ranks below anchor 1 */
    CHECK(sel[1] == 2);
}

static void test_staleness_expiry(void)
{
    struct anchor_pool p;
    anchor_pool_init(&p);

    anchor_pool_observe_score(&p, 1, 10.0f, 1000);
    CHECK(anchor_pool_live_count(&p) == 1);

    /* Not yet stale. */
    anchor_pool_expire_stale(&p, 1000 + ANCHOR_ENTRY_STALE_MS - 1);
    CHECK(anchor_pool_live_count(&p) == 1);

    /* Past the threshold: dropped. */
    anchor_pool_expire_stale(&p, 1000 + ANCHOR_ENTRY_STALE_MS + 1);
    CHECK(anchor_pool_live_count(&p) == 0);

    uint8_t sel[ANCHOR_SELECT_MAX];
    CHECK(anchor_pool_select(&p, sel, ANCHOR_SELECT_MAX) == 0);
}

static void test_staleness_wraps_uint32(void)
{
    struct anchor_pool p;
    anchor_pool_init(&p);

    /* last_seen_ms close to the uint32_t max; now_ms has wrapped around 0.
     * Elapsed is still small and the entry must survive. */
    uint32_t last = 0xFFFFFFFFu - 100u;
    uint32_t now  = 50u;   /* wrapped: elapsed = 150 ms */
    anchor_pool_observe_score(&p, 1, 10.0f, last);
    anchor_pool_expire_stale(&p, now);
    CHECK(anchor_pool_live_count(&p) == 1);
}

static void test_observe_pos_seeds_without_evicting(void)
{
    struct anchor_pool p;
    anchor_pool_init(&p);

    anchor_pool_observe_score(&p, 1, 10.0f, 0);
    CHECK(anchor_pool_live_count(&p) == 1);

    /* ANNOUNCE from a new anchor seeds a geometry-only entry. */
    anchor_pool_observe_pos(&p, 2, 1.0f, 2.0f, 3.0f, 100);
    CHECK(anchor_pool_live_count(&p) == 2);

    /* ANNOUNCE refreshes an existing entry's position without touching its score. */
    anchor_pool_observe_pos(&p, 1, 4.0f, 5.0f, NAN, 200);
    /* still ranks by its CIR score, unaffected by the position update */
    uint8_t sel[2];
    CHECK(anchor_pool_select(&p, sel, 2) == 2);

    /* Fill the pool solid with CIR-ranked anchors, then confirm an ANNOUNCE
     * for a brand-new id does NOT evict a ranked entry to make room. */
    struct anchor_pool full;
    anchor_pool_init(&full);
    for (uint8_t id = 0; id < ANCHOR_POOL_MAX; id++) {
        anchor_pool_observe_score(&full, id, (float)id, 0);
    }
    anchor_pool_observe_pos(&full, 200, 1.0f, 1.0f, 1.0f, 0);
    CHECK(anchor_pool_live_count(&full) == ANCHOR_POOL_MAX);   /* unchanged */
    uint8_t sel_full[ANCHOR_SELECT_MAX];
    CHECK(anchor_pool_select(&full, sel_full, ANCHOR_SELECT_MAX) == ANCHOR_SELECT_MAX);
    for (uint8_t i = 0; i < ANCHOR_SELECT_MAX; i++) {
        CHECK(sel_full[i] != 200);   /* the unseated announce never got in */
    }
}

int main(void)
{
    test_insert_and_select();
    test_never_reports_more_than_live();
    test_ema_update();
    test_decay_missed();
    test_staleness_expiry();
    test_staleness_wraps_uint32();
    test_observe_pos_seeds_without_evicting();
    if (g_fail) { printf("%d CHECK(s) FAILED\n", g_fail); return 1; }
    printf("ALL TESTS PASSED\n");
    return 0;
}
