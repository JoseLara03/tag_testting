#include "../../src/tag_ui_color.h"
#include <stdio.h>

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

/* The four SoC levels, as representative percentages. */
#define SOC_GREEN   100
#define SOC_YELLOW   60
#define SOC_ORANGE   30
#define SOC_RED      10

int main(void)
{
    uint8_t r, g, b;

    /* Each level by shape, not by exact value. */
    soc_to_rgb(SOC_GREEN, &r, &g, &b);
    CHECK(r == 0 && g > 0 && b == 0);            /* green: green channel only */

    soc_to_rgb(SOC_YELLOW, &r, &g, &b);
    CHECK(r > 0 && r == g && b == 0);            /* yellow: red == green */

    soc_to_rgb(SOC_ORANGE, &r, &g, &b);
    CHECK(r > g && g > 0 && b == 0);             /* orange: red dominant, green present */

    soc_to_rgb(SOC_RED, &r, &g, &b);
    CHECK(r > 0 && g == 0 && b == 0);            /* red: red channel only */

    /* Threshold boundaries are inclusive on the upper level. */
    uint8_t r2, g2, b2;
    soc_to_rgb(75, &r, &g, &b);
    soc_to_rgb(SOC_GREEN, &r2, &g2, &b2);
    CHECK(r == r2 && g == g2 && b == b2);        /* 75 is still green */

    soc_to_rgb(74, &r, &g, &b);
    soc_to_rgb(SOC_YELLOW, &r2, &g2, &b2);
    CHECK(r == r2 && g == g2 && b == b2);        /* 74 is yellow */

    soc_to_rgb(25, &r, &g, &b);
    soc_to_rgb(SOC_ORANGE, &r2, &g2, &b2);
    CHECK(r == r2 && g == g2 && b == b2);        /* 25 is still orange */

    soc_to_rgb(24, &r, &g, &b);
    soc_to_rgb(SOC_RED, &r2, &g2, &b2);
    CHECK(r == r2 && g == g2 && b == b2);        /* 24 is red */

    /* The four level colours must stay pairwise distinct: two levels rendering
     * as the same triplet would be indistinguishable on the strip. */
    const int levels[4] = { SOC_GREEN, SOC_YELLOW, SOC_ORANGE, SOC_RED };
    uint8_t rgb[4][3];

    for (int i = 0; i < 4; i++) {
        soc_to_rgb(levels[i], &rgb[i][0], &rgb[i][1], &rgb[i][2]);
    }
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
            int same = rgb[i][0] == rgb[j][0] &&
                       rgb[i][1] == rgb[j][1] &&
                       rgb[i][2] == rgb[j][2];
            if (same) {
                printf("FAIL: SoC %d and %d both render as (%u,%u,%u)\n",
                       levels[i], levels[j], rgb[i][0], rgb[i][1], rgb[i][2]);
                fails++;
            }
        }
    }

    /* No percentage renders as fully dark: dark means "not showing", and the
     * readout must never be mistaken for the strip being off. */
    for (int soc = 0; soc <= 100; soc++) {
        soc_to_rgb(soc, &r, &g, &b);
        if (r == 0 && g == 0 && b == 0) {
            printf("FAIL: SoC %d renders as fully dark\n", soc);
            fails++;
            break;
        }
    }

    printf("tag_ui_color: %d failure(s)\n", fails);
    return fails == 0 ? 0 : 1;
}
