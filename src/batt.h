#ifndef BATT_H_
#define BATT_H_

/* Read battery state of charge as a percentage (0-100), estimated from the
 * cell voltage via the LiPo discharge curve in batt_curve.h.
 *
 * Returns 0 on success; -EBUSY while the charger is connected (the terminal
 * voltage then says nothing about the charge, so no percentage is produced);
 * -ENODEV / -EIO / -EINVAL otherwise.
 *
 * This is an estimate, not a fuel gauge: in the 3700-3850 mV plateau, where
 * roughly half the capacity lives, expect around +/-10 percentage points. A
 * true reading would need the nRF Fuel Gauge library plus a battery model
 * profiled from this exact cell. */
int batt_read_soc(int *soc);

/* Last state of charge sampled by the periodic monitor: 0-100, or 0xFF when no
 * reading is available (charger connected, or gauge error). Non-blocking — safe
 * to call from timing-critical paths, unlike batt_read_soc(). Returns 0xFF
 * until batt_monitor_start() has run at least once. */
uint8_t batt_soc_cached(void);

/* Read average battery current in milliamps (discharge as a positive
 * magnitude). Returns 0 on success; negative errno on failure. */
int batt_read_current(int *ma);

#include "ble_log.h"

/* Start the periodic current-sampling timer (call once after BLE is up). */
void batt_monitor_start(void);

/* Feed BLE connection state: while disconnected the current is sampled into
 * the idle window; on disconnect the window is reset to start a fresh one.
 * Call from the BLE state callback. */
void batt_on_ble_state(ble_state_t state);

/* Read the disconnected-window current stats (mA): mean/min/max over the
 * samples collected while BLE was disconnected. Returns 1 if at least one
 * sample exists (out params written), else 0. Read on demand (e.g. the
 * `pwr idle` command) AFTER reconnecting and subscribing to notifications. */
int batt_get_idle_window(int *mean_ma, int *min_ma, int *max_ma, uint32_t *count);

#endif /* BATT_H_ */
