#include "tag_ui_color.h"

/* Base brightness of each level colour, at full (unshifted) intensity. The
 * strip is viewed up close, so these are deliberately low. */
static const struct {
    int min_soc;
    uint8_t r, g, b;
} levels[] = {
    { 75, 0, 10, 0 },   /* green  */
    { 50, 10, 10, 0 },  /* yellow */
    { 25, 10,  4, 0 },  /* orange */
    {  0, 10,  0, 0 },  /* red    */
};

#define LEVELS_N ((int)(sizeof(levels) / sizeof(levels[0])))

void soc_to_rgb(int soc, int dim_shift, uint8_t *r, uint8_t *g, uint8_t *b)
{
    int i;

    for (i = 0; i < LEVELS_N - 1; i++) {
        if (soc >= levels[i].min_soc) {
            break;
        }
    }

    *r = levels[i].r >> dim_shift;
    *g = levels[i].g >> dim_shift;
    *b = levels[i].b >> dim_shift;
}
