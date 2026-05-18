#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/gpio.h>
#include "lis2hh12_if.h"
#include "lis2hh12_reg.h"
#include "ble_log.h"

static const struct device *strip = DEVICE_DT_GET(DT_ALIAS(led_strip));
static uint8_t who_am_i_value = 0;

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    (void)dev;
    (void)cb;
    (void)pins;

    /* Send WHO_AM_I value over BLE on button short-press */
    char msg[32];
    snprintf(msg, sizeof(msg), "WHO_AM_I=0x%02x\n", who_am_i_value);
    ble_log_send(msg);
}

static struct gpio_callback button_cb_data;

int main(void)
{
    stmdev_ctx_t dev_ctx = {0};
    struct led_rgb pixel;

    if (!device_is_ready(strip)) {
        k_sleep(K_FOREVER);
        return 0;
    }

    const struct device *button = DEVICE_DT_GET(DT_ALIAS(button0));
    if (!device_is_ready(button)) {
        k_sleep(K_FOREVER);
        return 0;
    }

    int ret = gpio_pin_configure(button, DT_GPIO_PIN(DT_ALIAS(button0), gpios),
                                 GPIO_INPUT | DT_GPIO_FLAGS(DT_ALIAS(button0), gpios));
    if (ret != 0) {
        k_sleep(K_FOREVER);
        return 0;
    }

    uint32_t button_pin = DT_GPIO_PIN(DT_ALIAS(button0), gpios);
    gpio_init_callback(&button_cb_data, button_pressed, BIT(button_pin));
    gpio_add_callback(button, &button_cb_data);
    gpio_pin_interrupt_configure(button, button_pin, GPIO_INT_EDGE_RISING | GPIO_INT_PRIORITY_LEVEL(7));

    if (lis2hh12_if_init(&dev_ctx) != 0) {
        /* I2C bus not ready — blue */
        pixel = (struct led_rgb){.r = 0, .g = 0, .b = 10};
        led_strip_update_rgb(strip, &pixel, 1);
        k_sleep(K_FOREVER);
        return 0;
    }

    if (lis2hh12_dev_id_get(&dev_ctx, &who_am_i_value) != 0 || who_am_i_value != LIS2HH12_ID) {
        /* I2C error or wrong ID — red */
        pixel = (struct led_rgb){.r = 10, .g = 0, .b = 0};
    } else {
        /* WHO_AM_I = 0x41 — green */
        pixel = (struct led_rgb){.r = 0, .g = 10, .b = 0};
    }

    led_strip_update_rgb(strip, &pixel, 1);

    if (ble_log_init() != 0) {
        k_sleep(K_FOREVER);
        return 0;
    }

    ble_log_wait_ready();

    k_sleep(K_FOREVER);
    return 0;
}
