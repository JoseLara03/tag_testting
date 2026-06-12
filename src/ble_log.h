#ifndef BLE_LOG_H
#define BLE_LOG_H

#include <stdint.h>

typedef void (*ble_rx_handler_t)(const uint8_t *data, uint16_t len);

int  ble_log_init(void);
void ble_log_wait_ready(void);
void ble_log_send(const char *msg);

/* Register a handler invoked from the NUS RX callback (nrfxlib BT RX thread)
 * for every write the central sends. Pass NULL to unregister. */
void ble_log_set_rx_handler(ble_rx_handler_t handler);

#endif /* BLE_LOG_H */
