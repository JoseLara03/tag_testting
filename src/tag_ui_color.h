#ifndef TAG_UI_COLOR_H_
#define TAG_UI_COLOR_H_

#include <stdint.h>

/* Map a state-of-charge percentage to the RGB triplet shown on the strip:
 * green >=75, yellow >=50, orange >=25, red below that.
 *
 * dim_shift right-shifts every channel, so the resting display can be darker
 * than the button-press readout without changing the thresholds. Careful when
 * raising it: shifting quantises the channels, and past a point orange and red
 * collapse onto the same triplet. tests/tag_ui_color/ pins that down.
 *
 * Pure function, no device access. */
void soc_to_rgb(int soc, int dim_shift, uint8_t *r, uint8_t *g, uint8_t *b);

#endif /* TAG_UI_COLOR_H_ */
