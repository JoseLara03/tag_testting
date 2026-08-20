#ifndef CAL_RUN_H
#define CAL_RUN_H

#include <stdint.h>

/* Persist a solved result and activate it. 0 on success. Cal-image-only. */
int cal_store(uint16_t tx, uint16_t rx, uint32_t ref_mm, uint16_t residual_mm);

/* Erase the stored record and mark calibration invalid. 0 on success. */
int cal_clear(void);

/* ---- last-run verdict (survives a dropped BLE link, not a reset) ----
 *
 * A calibration run owns the radio for seconds and ends with an NVS write,
 * and the BLE link does not reliably survive that -- ble_log_send() drops
 * silently with no connection, so a verdict that is only pushed is
 * regularly lost. Latch it here and read it back with `cal last` after
 * reconnecting. RAM-only on purpose: if the tag reset during the run,
 * `cal last` reports "CAL none" rather than a stale verdict, distinguishing
 * a crash from a dropped link. */
void        cal_set_last_result(const char *s);
const char *cal_get_last_result(void);

/* NUS command parser for `cal clear`/`cal last`/`cal selftest`/`cal <mm>`.
 * Cal-image-only; tag_cmd.c routes every `cal ...` command except
 * `cal status` here ahead of cal_on_rx() (src/cal.h). */
void cal_run_on_rx(const uint8_t *data, uint16_t len);

/* Start the calibration-run thread. Call once from main.c under
 * CONFIG_TAG_CAL_MODE, after cal_init(). */
void cal_run_start(void);

#endif /* CAL_RUN_H */
