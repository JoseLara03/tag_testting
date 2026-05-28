#include "uwb.h"
#include "port.h"
#include "deca_spi.h"
#include "deca_probe_interface.h"
#include "ble_log.h"
#include <deca_device_api.h>
#include <zephyr/kernel.h>
#include <stdio.h>

static uint32_t dev_id = 0xDEADBEEFU;

int uwb_init(int max_retries)
{
    char msg[64];

    for (int i = 0; i < max_retries; i++) {
        gpio_init();
        dw_irq_init();
        dwm3001c_spi_init();
        reset_DWIC();

        if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS) {
            snprintf(msg, sizeof(msg), "probe fail %d/%d\n", i + 1, max_retries);
            ble_log_send(msg);
            k_msleep(100);
            continue;
        }

        if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS) {
            snprintf(msg, sizeof(msg), "init fail %d/%d\n", i + 1, max_retries);
            ble_log_send(msg);
            k_msleep(100);
            continue;
        }

        dev_id = dwt_readdevid();
        snprintf(msg, sizeof(msg), "OK ID=0x%08X\n", dev_id);
        ble_log_send(msg);
        return 0;
    }

    snprintf(msg, sizeof(msg), "all %d fail\n", max_retries);
    ble_log_send(msg);
    return -1;
}

uint32_t uwb_get_dev_id(void)
{
    return dev_id;
}
