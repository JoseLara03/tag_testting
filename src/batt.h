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

/* Feed BLE connection state: resets the window on disconnect, arms the
 * idle-window report on connect. Call from the BLE state callback. */
void batt_on_ble_state(ble_state_t state);

#endif /* BATT_H_ */
