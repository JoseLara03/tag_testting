#include "../../src/batt_window.h"
#include <stdio.h>

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

int main(void)
{
    struct batt_window w;

    /* Empty window: get returns 0, nothing written. */
    batt_window_reset(&w);
    CHECK(batt_window_get(&w, NULL, NULL, NULL, NULL) == 0);

    /* Three samples: min/mean/max/count. */
    batt_window_add(&w, 30);
    batt_window_add(&w, 50);
    batt_window_add(&w, 40);
    int mn = -1, me = -1, mx = -1; uint32_t n = 0;
    CHECK(batt_window_get(&w, &mn, &me, &mx, &n) == 1);
    CHECK(mn == 30);
    CHECK(mx == 50);
    CHECK(me == 40);
    CHECK(n == 3);

    /* Single sample: min == mean == max. */
    batt_window_reset(&w);
    batt_window_add(&w, 7);
    CHECK(batt_window_get(&w, &mn, &me, &mx, &n) == 1);
    CHECK(mn == 7 && me == 7 && mx == 7 && n == 1);

    /* Reset clears. */
    batt_window_reset(&w);
    CHECK(batt_window_get(&w, NULL, NULL, NULL, NULL) == 0);

    printf("batt_window: %d failure(s)\n", fails);
    return fails == 0 ? 0 : 1;
}
