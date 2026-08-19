#ifndef CAL_DIAG_H_
#define CAL_DIAG_H_

#include <stdint.h>

/*
 * Cal-image-only RX-path diagnostics: `cal listen` and `cal probe`.
 * See spec/2026-08-19-tag-calibration-image-design.md §6.
 * Compiled only under CONFIG_TAG_CAL_MODE.
 */

/* Start the diagnostics thread. Call once from main() bring-up, after
 * uwb_radio_owner_set_unmanaged(). */
void cal_diag_start(void);

/* Parse a `cal listen`/`cal probe` NUS command, enqueue it, and return
 * immediately -- called from the BT RX thread, which must never block.
 * Any "cal ..." text this module does not recognise is silently ignored;
 * the caller (tag_cmd.c) is responsible for forwarding everything else to
 * cal_on_rx(). */
void cal_diag_on_rx(const uint8_t *data, uint16_t len);

#endif /* CAL_DIAG_H_ */
