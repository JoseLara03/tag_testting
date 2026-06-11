#include "uwb_simple_tx.h"
#include "zephyr/kernel.h"
#include "port.h"
#include "ble_log.h"
#include "deca_device_api.h"
#include "port.h"
#include "deca_spi.h"

static uint8_t tx_msg[] = { 0xC5, 0, 'I', 'N', 'N', 'O', 'V', 'A', '0', '1' };
#define FRAME_LENGTH (sizeof(tx_msg) + FCS_LEN) // The real length that is going to be transmitted

void simple_tx(void){
    while (1)
    {
        /* Write frame data to DW IC and prepare transmission. See NOTE 3 below.*/
        dwt_writetxdata(FRAME_LENGTH - FCS_LEN, tx_msg, 0); /* Zero offset in TX buffer. */

        /* In this example since the length of the transmitted frame does not change,
         * nor the other parameters of the dwt_writetxfctrl function, the
         * dwt_writetxfctrl call could be outside the main while(1) loop.
         */
        dwt_writetxfctrl(FRAME_LENGTH, 0, 0); /* Zero offset in TX buffer, no ranging. */

        /* Start transmission. */
        dwt_starttx(DWT_START_TX_IMMEDIATE);
        /* Poll DW IC until TX frame sent event set. See NOTE 4 below.
         * STATUS register is 4 bytes long but, as the event we are looking
         * at is in the first byte of the register, we can use this simplest
         * API function to access it.*/
        while (1){
            uint32_t status = dwt_readsysstatuslo();
            if (status & DWT_INT_TXFRS_BIT_MASK){
                dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
                break;
            }
        }

        /* Clear TX frame sent event. */
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

        ble_log_send("TX Frame Sent\n");

        /* Execute a delay between transmissions. */
        Sleep(TX_DELAY_MS);

        /* Increment the blink frame sequence number (modulo 256). */
        tx_msg[BLINK_FRAME_SN_IDX]++;
    }
}