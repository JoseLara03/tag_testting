#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include "lis2hh12_if.h"
#include "lis2hh12_reg.h"

static const struct device *strip = DEVICE_DT_GET(DT_ALIAS(led_strip));

int main(void)
{
    stmdev_ctx_t dev_ctx = {0};
    uint8_t who_am_i = 0;
    struct led_rgb pixel;

    if (!device_is_ready(strip)) {
        k_sleep(K_FOREVER);
        return 0;
    }

    if (lis2hh12_if_init(&dev_ctx) != 0) {
        /* I2C bus not ready — blue */
        pixel = (struct led_rgb){.r = 0, .g = 0, .b = 10};
        led_strip_update_rgb(strip, &pixel, 1);
        k_sleep(K_FOREVER);
        return 0;
    }

    if (lis2hh12_dev_id_get(&dev_ctx, &who_am_i) != 0 || who_am_i != LIS2HH12_ID) {
        /* I2C error or wrong ID — red */
        pixel = (struct led_rgb){.r = 10, .g = 0, .b = 0};
    } else {
        /* WHO_AM_I = 0x41 — green */
        pixel = (struct led_rgb){.r = 0, .g = 10, .b = 0};
    }

    led_strip_update_rgb(strip, &pixel, 1);
    k_sleep(K_FOREVER);
    return 0;
}
