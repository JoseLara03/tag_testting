#ifndef UWB_RADIO_OPS_H
#define UWB_RADIO_OPS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "pos_solver.h"   /* struct pos_meas */

/* Blocking RX of one beacon; returns frame length or <0 on timeout. */
int  uwb_radio_rx_beacon(uint8_t *buf, size_t buf_len, uint32_t timeout_ms);
/* TX one frame in a CAP mini-slot (caller applies Aloha backoff via slot arg). */
int  uwb_radio_tx_cap(const uint8_t *buf, size_t len, uint8_t minislot);
/* Run discovery; fill anchor coords/ids; return anchor count. */
int  uwb_radio_discover(struct pos_meas *out, size_t max);
/* Range the previously discovered anchors; fill ranges; return count. */
int  uwb_radio_sweep(struct pos_meas *out, size_t max);
/* Sleep the DW3000 until `wake_ms` (monotonic). */
void uwb_radio_sleep_until(uint32_t wake_ms);
uint32_t uwb_radio_now_ms(void);

#endif /* UWB_RADIO_OPS_H */
