#ifndef BLE_LOG_H
#define BLE_LOG_H

int  ble_log_init(void);
void ble_log_wait_connected(void);
void ble_log_send(const char *msg);

#endif /* BLE_LOG_H */
