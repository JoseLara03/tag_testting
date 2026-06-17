#include "uwb_simple_rx.h"
#include "ble_log.h"
#include "deca_device_api.h"
#include <string.h>
#include <stdio.h>
#include <zephyr/kernel.h>

static uint8_t rx_buffer[FRAME_LEN_MAX];

void simple_rx(void)
{
    uint32_t status;
    uint16_t frame_len;

    while (1) {
        memset(rx_buffer, 0, sizeof(rx_buffer));

        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        do {
            status = dwt_readsysstatuslo();
        } while (!(status & (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)));

        if (status & DWT_INT_RXFCG_BIT_MASK) {
            frame_len = dwt_getframelength();
            if (frame_len <= FRAME_LEN_MAX) {
                dwt_readrxdata(rx_buffer, frame_len - FCS_LEN, 0);
            }
            dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
            ble_log_send("Frame Received\n");
        } else {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
            char errmsg[20];
            snprintf(errmsg, sizeof(errmsg), "ERR:0x%08X\n", status);
            ble_log_send(errmsg);
            k_msleep(100);
        }
    }
}
