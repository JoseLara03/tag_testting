#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/gpio.h>
#include "lis2hh12_if.h"
#include "lis2hh12_reg.h"
#include "ble_log.h"

static const struct device *strip = DEVICE_DT_GET(DT_ALIAS(led_strip));

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    (void)dev;
    (void)cb;
    (void)pins;

    /* Send button press notification over BT */
    ble_log_send("BUTTON_PRESSED\n");
}

static struct gpio_callback button_cb_data;

int main(void)
{
    stmdev_ctx_t dev_ctx = {0};
    uint8_t who_am_i = 0;
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

    uint8_t button_pin = DT_GPIO_PIN(DT_ALIAS(button0), gpios);
    int ret = gpio_pin_configure(button, button_pin,
                                  GPIO_INPUT | DT_GPIO_FLAGS(DT_ALIAS(button0), gpios));
    if (ret != 0) {
        k_sleep(K_FOREVER);
        return 0;
    }

    gpio_init_callback(&button_cb_data, button_pressed, BIT(button_pin));
    gpio_add_callback(button, &button_cb_data);
    gpio_pin_interrupt_configure(button, button_pin, GPIO_INT_EDGE_RISING);

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
