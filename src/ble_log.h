#ifndef BLE_LOG_H
#define BLE_LOG_H

#include <stdint.h>
#include <stdbool.h>

typedef void (*ble_rx_handler_t)(const uint8_t *data, uint16_t len);

typedef enum {
    BLE_STATE_ADVERTISING,
    BLE_STATE_CONNECTED,
} ble_state_t;

typedef void (*ble_state_cb_t)(ble_state_t state);

int  ble_log_init(void);
void ble_log_wait_ready(void);
void ble_log_send(const char *msg);

/* Register a handler invoked from the NUS RX callback (nrfxlib BT RX thread)
 * for every write the central sends. Pass NULL to unregister. */
void ble_log_set_rx_handler(ble_rx_handler_t handler);

/* Open a bounded (60 s) advertising window. The tag does not advertise at boot:
 * continuous connectable advertising costs ~150 uA of a 250 uA budget for a
 * diagnostics-only channel. Called from the button single-press gesture (the
 * same one that shows the battery colour) and from the NFC field/write
 * callback. Idempotent -- a second call restarts the 60 s timer. Safe from any
 * thread including the BT RX thread: the work is deferred to the system
 * workqueue. */
void ble_log_adv_window(void);

/* Force continuous advertising for a debug session (`pwr adv on|off`). RAM
 * only -- never persisted, so a forgotten override cannot outlive a reboot.
 * Advertising is also continuous while uwb_radio_sleep_enabled() is false. */
void ble_log_adv_force(bool on);
bool ble_log_adv_forced(void);

/* Register a handler called on connect (BLE_STATE_CONNECTED) and after
 * advertising restarts post-disconnect (BLE_STATE_ADVERTISING).
 * The ADVERTISING callback runs from the system workqueue. */
void ble_log_set_state_cb(ble_state_cb_t cb);

#endif /* BLE_LOG_H */
