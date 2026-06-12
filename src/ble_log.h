#ifndef BLE_LOG_H
#define BLE_LOG_H

#include <stdint.h>

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

/* Register a handler called on connect (BLE_STATE_CONNECTED) and after
 * advertising restarts post-disconnect (BLE_STATE_ADVERTISING).
 * The ADVERTISING callback runs from the system workqueue. */
void ble_log_set_state_cb(ble_state_cb_t cb);

#endif /* BLE_LOG_H */
