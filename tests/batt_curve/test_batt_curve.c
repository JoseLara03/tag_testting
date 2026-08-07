#include "../../src/batt_curve.h"
#include <stdio.h>

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

int main(void)
{
    /* Saturation at both ends, and beyond them. */
    CHECK(lipo_mv_to_pct(4200) == 100);
    CHECK(lipo_mv_to_pct(4500) == 100);   /* above full: clamp, not extrapolate */
    CHECK(lipo_mv_to_pct(3500) == 0);
    CHECK(lipo_mv_to_pct(3000) == 0);     /* below empty: clamp, not negative */

    /* Exact table points return their exact percentage. */
    CHECK(lipo_mv_to_pct(4150) == 95);
    CHECK(lipo_mv_to_pct(4020) == 80);
    CHECK(lipo_mv_to_pct(3840) == 50);
    CHECK(lipo_mv_to_pct(3770) == 30);
    CHECK(lipo_mv_to_pct(3610) == 5);

    /* Interpolation between points, at fractions that land on whole percents.
     * 4180 is 60% of the way from 4150 (95%) to 4200 (100%) -> 98. */
    CHECK(lipo_mv_to_pct(4180) == 98);
    /* 3722 is 60% of the way from 3710 (15%) to 3730 (20%) -> 18. */
    CHECK(lipo_mv_to_pct(3722) == 18);
    /* 3860 is halfway between 3850 (55%) and 3870 (60%) -> 57 or 58. */
    CHECK(lipo_mv_to_pct(3860) >= 57 && lipo_mv_to_pct(3860) <= 58);

    /* An interpolated value always lies strictly between its neighbours. */
    CHECK(lipo_mv_to_pct(3795) > 35 && lipo_mv_to_pct(3795) < 40);

    /* The plateau really is a plateau: 3700-3850 mV must span a large part of
     * the range. This is the accuracy caveat, asserted so it cannot silently
     * regress into a curve that pretends to be more linear than the cell is. */
    CHECK((lipo_mv_to_pct(3850) - lipo_mv_to_pct(3700)) >= 35);

    /* Monotonic and in range across the whole sweep. */
    int prev = -1;
    for (int mv = 3400; mv <= 4300; mv++) {
        int pct = lipo_mv_to_pct(mv);
        if (pct < 0 || pct > 100) {
            printf("FAIL: %d mV -> %d%% out of range\n", mv, pct);
            fails++;
            break;
        }
        if (pct < prev) {
            printf("FAIL: %d mV -> %d%% dropped below previous %d%%\n", mv, pct, prev);
            fails++;
            break;
        }
        prev = pct;
    }

    printf("batt_curve: %d failure(s)\n", fails);
    return fails == 0 ? 0 : 1;
}
