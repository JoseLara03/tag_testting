#ifndef TAG_UI_COLOR_H_
#define TAG_UI_COLOR_H_

#include <stdint.h>

/* Map a state-of-charge percentage to the RGB triplet shown on the strip:
 * green >=75, yellow >=50, orange >=25, red below that. Shown only while the
 * button readout is active; the strip is dark the rest of the time.
 *
 * Pure function, no device access — see tests/tag_ui_color/. */
void soc_to_rgb(int soc, uint8_t *r, uint8_t *g, uint8_t *b);

#endif /* TAG_UI_COLOR_H_ */
