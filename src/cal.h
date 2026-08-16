#ifndef CAL_H
#define CAL_H

#include <stdint.h>
#include <stdbool.h>

/* Mount NVS, load the stored calibration for the current PHY, and register the
 * BLE `cal` command handler. Returns true if a valid record was loaded. */
bool cal_init(void);

/* Active antenna delays. Meaningful only when cal_is_valid() is true. */
void cal_get_ant_dly(uint16_t *tx, uint16_t *rx);

/* True if a valid calibration is currently loaded/active. */
bool cal_is_valid(void);

/* Erase the stored record and mark calibration invalid. 0 on success. */
int cal_clear(void);

/* ---- ranging-thread side ---- */

/* If a `cal <mm>` command was queued, copy the reference into out_ref_mm,
 * clear the request, and return true. */
bool cal_take_request(uint32_t *out_ref_mm);

/* Block until a calibration request arrives (used while ranging is gated). */
void cal_wait_request(void);

/* Persist a solved result and activate it. 0 on success. */
int cal_store(uint16_t tx, uint16_t rx, uint32_t ref_mm, uint16_t residual_mm);

/* ---- last-run verdict (survives a dropped BLE link, not a reset) ----
 *
 * A calibration run owns the radio for seconds and ends with an NVS write, and
 * the BLE link does not reliably survive that. ble_log_send() drops silently
 * when there is no connection, so a verdict that is only pushed is regularly
 * lost -- which is why a run could complete with the operator never learning
 * whether it worked. Latch it here and read it back with `cal last` after
 * reconnecting.
 *
 * Deliberately RAM-only: if the tag reset during the run, `cal last` reports
 * "CAL none" rather than a stale verdict, which distinguishes a crash from a
 * dropped link. */
void        cal_set_last_result(const char *s);
const char *cal_get_last_result(void);

/* NUS command parser for `cal ...` commands. Invoked by the tag_cmd dispatcher. */
void cal_on_rx(const uint8_t *data, uint16_t len);

#endif /* CAL_H */
