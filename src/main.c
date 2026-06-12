#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include "ble_log.h"
#include "uwb.h"
#include "uwb_ss_initiator.h"
#include "motion.h"
#include "tag_ui.h"
#include "cal.h"

static const struct device *strip = DEVICE_DT_GET(DT_ALIAS(led_strip));

static void on_ble_state(ble_state_t state)
{
    struct led_rgb px;

    if (state == BLE_STATE_CONNECTED) {
        px = (struct led_rgb){.r = 0, .g = 10, .b = 0};   /* green  */
    } else {
        px = (struct led_rgb){.r = 0, .g = 0, .b = 10};   /* blue = advertising */
    }
    led_strip_update_rgb(strip, &px, 1);
}

int main(void)
{
    struct led_rgb pixel;

    if (!device_is_ready(strip)) {
        k_sleep(K_FOREVER);
        return 0;
    }

    if (ble_log_init() != 0) {
        k_sleep(K_FOREVER);
        return 0;
    }
    ble_log_set_state_cb(on_ble_state);

    /* Cyan: initializing DW3000 on SPI1 */
    pixel = (struct led_rgb){.r = 0, .g = 10, .b = 10};
    led_strip_update_rgb(strip, &pixel, 1);

    if (uwb_init(3) == 0) {
        /* Green: DW3000 SPI1 OK */
        pixel = (struct led_rgb){.r = 0, .g = 10, .b = 0};
        led_strip_update_rgb(strip, &pixel, 1);
        ble_log_send("Config OK\n");
        if (cal_init()) {
            ble_log_send("CAL loaded\n");
        } else {
            ble_log_send("CAL REQUIRED\n");
        }
        uwb_ss_initiator_start();
        if (motion_init() != 0) {
            ble_log_send("motion init fail\n");
        }
        tag_ui_init();
    } else {
        /* Red: DW3000 SPI1 init failed */
        pixel = (struct led_rgb){.r = 10, .g = 0, .b = 0};
        led_strip_update_rgb(strip, &pixel, 1);
        ble_log_send("DW3000: init failed\n");
    }

    k_sleep(K_FOREVER);
    return 0;
}
