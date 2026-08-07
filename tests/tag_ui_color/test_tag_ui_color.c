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

    /* Full brightness: green, yellow, orange, red by shape, not exact value. */
    soc_to_rgb(SOC_GREEN, 0, &r, &g, &b);
    CHECK(r == 0 && g > 0 && b == 0);            /* green: green channel only */

    soc_to_rgb(SOC_YELLOW, 0, &r, &g, &b);
    CHECK(r > 0 && r == g && b == 0);            /* yellow: red == green */

    soc_to_rgb(SOC_ORANGE, 0, &r, &g, &b);
    CHECK(r > g && g > 0 && b == 0);             /* orange: red dominant, green present */

    soc_to_rgb(SOC_RED, 0, &r, &g, &b);
    CHECK(r > 0 && g == 0 && b == 0);            /* red: red channel only */

    /* Threshold boundaries are inclusive on the upper level. */
    uint8_t r2, g2, b2;
    soc_to_rgb(75, 0, &r, &g, &b);
    soc_to_rgb(SOC_GREEN, 0, &r2, &g2, &b2);
    CHECK(r == r2 && g == g2 && b == b2);        /* 75 is still green */

    soc_to_rgb(74, 0, &r, &g, &b);
    soc_to_rgb(SOC_YELLOW, 0, &r2, &g2, &b2);
    CHECK(r == r2 && g == g2 && b == b2);        /* 74 is yellow */

    soc_to_rgb(25, 0, &r, &g, &b);
    soc_to_rgb(SOC_ORANGE, 0, &r2, &g2, &b2);
    CHECK(r == r2 && g == g2 && b == b2);        /* 25 is still orange */

    soc_to_rgb(24, 0, &r, &g, &b);
    soc_to_rgb(SOC_RED, 0, &r2, &g2, &b2);
    CHECK(r == r2 && g == g2 && b == b2);        /* 24 is red */

    /* Dimming actually dims. */
    soc_to_rgb(SOC_GREEN, 0, &r, &g, &b);
    soc_to_rgb(SOC_GREEN, 1, &r2, &g2, &b2);
    CHECK(g2 < g && g2 > 0);

    /* The property that matters: at every dim level we actually use, the four
     * level colours stay pairwise distinct. Shifting quantises the channels,
     * and orange (r>g>0) is the one that collapses onto red first. */
    for (int shift = 0; shift <= 1; shift++) {
        const int levels[4] = { SOC_GREEN, SOC_YELLOW, SOC_ORANGE, SOC_RED };
        uint8_t rgb[4][3];

        for (int i = 0; i < 4; i++) {
            soc_to_rgb(levels[i], shift, &rgb[i][0], &rgb[i][1], &rgb[i][2]);
        }

        for (int i = 0; i < 4; i++) {
            for (int j = i + 1; j < 4; j++) {
                int same = rgb[i][0] == rgb[j][0] &&
                           rgb[i][1] == rgb[j][1] &&
                           rgb[i][2] == rgb[j][2];
                if (same) {
                    printf("FAIL: shift %d makes SoC %d and %d both (%u,%u,%u)\n",
                           shift, levels[i], levels[j],
                           rgb[i][0], rgb[i][1], rgb[i][2]);
                    fails++;
                }
            }
        }
    }

    /* Never lit at all is not a valid level colour: something must be visible. */
    for (int soc = 0; soc <= 100; soc++) {
        soc_to_rgb(soc, 1, &r, &g, &b);
        if (r == 0 && g == 0 && b == 0) {
            printf("FAIL: SoC %d renders as fully dark at shift 1\n", soc);
            fails++;
            break;
        }
    }

    printf("tag_ui_color: %d failure(s)\n", fails);
    return fails == 0 ? 0 : 1;
}
