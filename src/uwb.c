#include "uwb.h"
#include "port.h"
#include "deca_spi.h"
#include "deca_probe_interface.h"
#include "ble_log.h"
#include <deca_device_api.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include "phy_config.h"
#include "uwb_simple_tx.h"
#include "uwb_simple_rx.h"

static uint32_t dev_id = 0xDEADBEEFU;

extern dwt_txconfig_t txconfig_options;
extern dwt_config_t config_options;

static int ret = 0;

int uwb_init(int max_retries)
{
    char msg[64];

    for (int i = 0; i < max_retries; i++) {
        gpio_init();
        dw_irq_init();
        dwm3001c_spi_init();

        // desabilitar irq para que este no encienda en cuando se reinicia
        port_DisableEXT_IRQ();
        reset_DWIC();
        k_msleep(2);

        if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS) {
            snprintf(msg, sizeof(msg), "probe fail %d/%d\n", i + 1, max_retries);
            ble_log_send(msg);
            k_msleep(100);
            continue;
        }

        if (dwt_initialise(DWT_READ_OTP_BAT | DWT_READ_OTP_LID | DWT_READ_OTP_PID | DWT_READ_OTP_TMP) != DWT_SUCCESS) {
            snprintf(msg, sizeof(msg), "init fail %d/%d\n", i + 1, max_retries);
            ble_log_send(msg);
            k_msleep(100);
            continue;
        }

        dev_id = dwt_readdevid();
        snprintf(msg, sizeof(msg), "OK ID=0x%08X\n", dev_id);
        ble_log_send(msg);
        port_set_dw_ic_spi_fastrate();
        k_msleep(2);
        
        ret = dwt_configure(&config_options);
        
        if (ret != DWT_SUCCESS){
            if (ret == DWT_ERROR){
                snprintf(msg, sizeof(msg), "ERR PHY:%d\n", ret);
                ble_log_send(msg);
            } else if (ret == DWT_ERR_RX_CAL_PGF){
                snprintf(msg, sizeof(msg), "ERR PGF:%d\n", ret);
                ble_log_send(msg);
            } else {
                snprintf(msg, sizeof(msg), "ERR UWB:%d\n", ret);
                ble_log_send(msg);
            }
            continue;
        }

        dwt_configuretxrf(&txconfig_options);

        dwt_setlnapamode(DWT_LNA_ENABLE /* | DWT_PA_ENABLE */);
        // dwt_setlnapamode(DWT_LNA_PA_DISABLE);
        
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

void uwb_simple_tx(void)
{
    simple_tx();
}

void uwb_simple_rx(void)
{
    simple_rx();
}
