#ifndef BATT_H_
#define BATT_H_

/* Read battery state of charge as a percentage (0-100).
 * Returns 0 on success and writes *soc; negative errno on failure
 * (e.g. -ENODEV when the gauge is not ready / no battery attached). */
int batt_read_soc(int *soc);

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
