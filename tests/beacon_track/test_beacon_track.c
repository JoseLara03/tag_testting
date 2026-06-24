#include "../../src/beacon_track_core.h"
#include <stdio.h>

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

int main(void)
{
    struct beacon_track c;
    bool narrow;
    uint32_t arm = 0, win = 0;

    /* --- Test A: lock transition + plan, period unchanged (gaps == seed) --- */
    /* seed 200ms, guard 5ms, warmup_n 3, ema_shift 3 */
    beacon_track_reset(&c, 200u, 5u, 3u, 3u);

    /* No beacon yet -> ACQUIRING (full window). */
    beacon_track_plan(&c, &narrow, &arm, &win);
    CHECK(narrow == false);

    beacon_track_beacon(&c, 1000u);                 /* warmup 1, no gap yet */
    beacon_track_plan(&c, &narrow, &arm, &win);
    CHECK(narrow == false);                         /* 1 < 3 */

    beacon_track_beacon(&c, 1200u);                 /* gap 200 -> err 0; warmup 2 */
    beacon_track_plan(&c, &narrow, &arm, &win);
    CHECK(narrow == false);                         /* 2 < 3 */

    beacon_track_beacon(&c, 1400u);                 /* gap 200; warmup 3 -> TRACKING */
    beacon_track_plan(&c, &narrow, &arm, &win);
    CHECK(narrow == true);
    CHECK(arm == 1400u + 200u - 5u);                /* last + period_est - guard = 1595 */
    CHECK(win == 10u);                              /* 2 * guard */
    CHECK(beacon_track_period_ms(&c) == 200u);      /* unchanged: all gaps == seed */

    /* --- Test B: EMA convergence on a positive error --- */
    beacon_track_reset(&c, 200u, 5u, 3u, 3u);
    beacon_track_beacon(&c, 0u);                    /* first, no gap */
    CHECK(beacon_track_period_ms(&c) == 200u);
    beacon_track_beacon(&c, 208u);                  /* gap 208, err +8, +8>>3 = +1 */
    CHECK(beacon_track_period_ms(&c) == 201u);
    beacon_track_beacon(&c, 416u);                  /* gap 208, err +7, 7/8=0 -> nudge +1 */
    CHECK(beacon_track_period_ms(&c) == 202u);      /* no dead band: nonzero err always moves */

    /* --- Test C: miss -> ACQUIRING, warmup reset, period_est retained, re-lock --- */
    beacon_track_reset(&c, 200u, 5u, 3u, 3u);
    beacon_track_beacon(&c, 1000u);
    beacon_track_beacon(&c, 1200u);
    beacon_track_beacon(&c, 1400u);                 /* TRACKING, period 200 */
    beacon_track_plan(&c, &narrow, &arm, &win);
    CHECK(narrow == true);

    beacon_track_miss(&c);                          /* drop to ACQUIRING */
    beacon_track_plan(&c, &narrow, &arm, &win);
    CHECK(narrow == false);
    CHECK(beacon_track_period_ms(&c) == 200u);      /* period retained */

    /* Next beacon re-establishes the reference WITHOUT an EMA update (have_last
     * was cleared by the miss, so the post-gap sample cannot poison the EMA). */
    beacon_track_beacon(&c, 1700u);                 /* warmup 1; no gap applied */
    CHECK(beacon_track_period_ms(&c) == 200u);
    beacon_track_beacon(&c, 1900u);                 /* warmup 2 */
    beacon_track_beacon(&c, 2100u);                 /* warmup 3 -> TRACKING */
    beacon_track_plan(&c, &narrow, &arm, &win);
    CHECK(narrow == true);
    CHECK(arm == 2100u + 200u - 5u);                /* 2295 */

    /* --- Test D: negative convergence, no dead-band stranding ---
     * Real case: seed 200ms, true gateway period 195ms.  Each gap gives
     * err = -5; the EMA must still drive period_est down to 195, otherwise the
     * window is mis-centred 5ms late and ~half the (early-jitter) beacons fall
     * off the front edge and miss.  (A round-toward-zero shift strands it at
     * 200.) */
    beacon_track_reset(&c, 200u, 5u, 2u, 3u);
    {
        uint32_t t = 0u;
        beacon_track_beacon(&c, t);                 /* first, no gap */
        for (int i = 0; i < 20; i++) {
            t += 195u;
            beacon_track_beacon(&c, t);
        }
    }
    CHECK(beacon_track_period_ms(&c) == 195u);      /* converged, not stuck at 200 */

    printf("beacon_track_core: %d failure(s)\n", fails);
    return fails == 0 ? 0 : 1;
}
