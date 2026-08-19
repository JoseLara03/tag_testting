#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/sys/byteorder.h>
#include <hal/nrf_ficr.h>
#include "ble_log.h"
#include "batt.h"
#include "uwb.h"
#include "uwb_ss_initiator.h"
#include "uwb_net_runner.h"
#include "motion.h"
#include "tag_ui.h"
#include "cal.h"
#include "tag_alert.h"
#include "tag_cmd.h"
#include "rx_stats.h"
#include "wdt.h"
#include "storage.h"
#include "nfc_tag.h"
#ifdef CONFIG_TAG_CAL_MODE
#include "uwb_radio_owner.h"
#include "cal_diag.h"
#include "cal_led.h"
#endif

static const struct device *strip = DEVICE_DT_GET(DT_ALIAS(led_strip));

int main(void)
{
    if (!device_is_ready(strip)) {
        k_sleep(K_FOREVER);
        return 0;
    }

    if (ble_log_init() != 0) {
        k_sleep(K_FOREVER);
        return 0;
    }

    ble_log_set_state_cb(batt_on_ble_state);
    if (storage_init() != 0) {
        ble_log_send("STOR fail\n");
    }

    /* Arm the boot-guard watchdog: if DW3000 bring-up hangs or fails, the
     * watchdog is never fed and the SoC resets in ~10 s, retrying the boot. */
    tag_wdt_start_boot_guard();

    if (uwb_init(3) == 0) {
        ble_log_send("Config OK\n");

#ifndef CONFIG_TAG_CAL_MODE
        uint8_t eui[8];
        sys_put_le32(NRF_FICR->DEVICEID[0], &eui[0]);
        sys_put_le32(NRF_FICR->DEVICEID[1], &eui[4]);
#endif

        if (cal_init()) {
            ble_log_send("CAL loaded\n");
        } else {
            ble_log_send("CAL REQUIRED\n");
        }
#ifndef CONFIG_TAG_CAL_MODE
        if (nfc_tag_init() != 0) {
            ble_log_send("NFC fail\n");
        }
#endif
        tag_cmd_init();
        rx_stats_reset();
#ifdef CONFIG_TAG_CAL_MODE
        uwb_radio_owner_set_unmanaged();
#else
        tag_alert_init();
#endif
        uwb_ss_initiator_start();
#ifdef CONFIG_TAG_CAL_MODE
        cal_diag_start();
        cal_led_init(strip);
#else
        uwb_net_runner_start(eui);
        if (motion_init() != 0) {
            ble_log_send("motion init fail\n");
        }
        tag_ui_init();
        batt_monitor_start();
#endif
        tag_wdt_run_feeder();
    } else {
        /* Red: fatal DW3000 init failure — the only signal with no BLE. */
        struct led_rgb pixel = (struct led_rgb){.r = 10, .g = 0, .b = 0};
        led_strip_update_rgb(strip, &pixel, 1);
        ble_log_send("DW3000: init failed\n");
    }

    k_sleep(K_FOREVER);
    return 0;
}
