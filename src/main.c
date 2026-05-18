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

/* Button press duration detection */
#define LONG_PRESS_MS 3000

/* Button state machine */
enum button_state {
    BUTTON_IDLE,
    BUTTON_PRESSING,
    BUTTON_LONG_PRESS_DETECTED
};

static enum button_state button_state = BUTTON_IDLE;
static struct k_timer long_press_timer;
static bool orange_led_active = false;
static struct gpio_callback button_cb_rising;
static struct gpio_callback button_cb_falling;

/* Timer callback: called when 3s button hold detected */
static void long_press_timer_expired(struct k_timer *timer)
{
    (void)timer;
    button_state = BUTTON_LONG_PRESS_DETECTED;
}

/* Rising edge: button pressed */
static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    (void)dev;
    (void)cb;
    (void)pins;

    if (button_state != BUTTON_IDLE) {
        return;  /* Debounce: ignore if not idle */
    }

    button_state = BUTTON_PRESSING;
    k_timer_start(&long_press_timer, K_MSEC(LONG_PRESS_MS), K_NO_WAIT);
}

/* Falling edge: button released */
static void button_released(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    (void)dev;
    (void)cb;
    (void)pins;

    struct led_rgb pixel;

    if (button_state == BUTTON_IDLE) {
        return;  /* Debounce: ignore if idle */
    }

    k_timer_stop(&long_press_timer);

    if (button_state == BUTTON_PRESSING) {
        /* Short press: less than 3s */
        char msg[32];
        snprintf(msg, sizeof(msg), "WHO_AM_I=0x%02x\n", who_am_i_value);
        ble_log_send(msg);
    } else if (button_state == BUTTON_LONG_PRESS_DETECTED) {
        /* Long press: toggle orange LED */
        orange_led_active = !orange_led_active;
        if (orange_led_active) {
            /* Orange: (255, 165, 0) scaled to 0-10 range ≈ (8, 5, 0) */
            pixel = (struct led_rgb){.r = 8, .g = 5, .b = 0};
        } else {
            /* Return to green (WHO_AM_I detected) */
            pixel = (struct led_rgb){.r = 0, .g = 10, .b = 0};
        }
        led_strip_update_rgb(strip, &pixel, 1);
    }

    button_state = BUTTON_IDLE;
}

int main(void)
{
    stmdev_ctx_t dev_ctx = {0};
    struct led_rgb pixel;

    if (!device_is_ready(strip)) {
        k_sleep(K_FOREVER);
        return 0;
    }

    const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio0)) {
        k_sleep(K_FOREVER);
        return 0;
    }

    uint8_t button_pin = 17;
    int ret = gpio_pin_configure(gpio0, button_pin, GPIO_INPUT | GPIO_ACTIVE_HIGH);
    if (ret != 0) {
        k_sleep(K_FOREVER);
        return 0;
    }

    /* Initialize long press timer */
    k_timer_init(&long_press_timer, long_press_timer_expired, NULL);

    /* Setup rising edge (button press) callback */
    gpio_init_callback(&button_cb_rising, button_pressed, BIT(button_pin));
    gpio_add_callback(gpio0, &button_cb_rising);

    /* Setup falling edge (button release) callback */
    gpio_init_callback(&button_cb_falling, button_released, BIT(button_pin));
    gpio_add_callback(gpio0, &button_cb_falling);

    /* Enable both rising and falling edge interrupts */
    gpio_pin_interrupt_configure(gpio0, button_pin,
                                 GPIO_INT_EDGE_RISING | GPIO_INT_EDGE_FALLING);

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
