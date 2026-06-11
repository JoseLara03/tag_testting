#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "motion.h"
#include "lis2hh12_if.h"
#include "uwb_ss_initiator.h"

/* INT1 pin: P0.28, declared as accel_int in the board DTS. */
static const struct gpio_dt_spec accel_int =
    GPIO_DT_SPEC_GET(DT_NODELABEL(accel_int), gpios);

/* Logical pin level that means "moving". The LIS2HH12 INT1_INACT polarity
 * is verified on hardware in Step 6; flip this (0/1) if cadence is inverted. */
#define ACCEL_INT_MOVING_LEVEL  1

/* Activity tuning (LIS2HH12, FS=2g, ODR=50 Hz):
 *  ACT_THS LSB = FS/128 = 2000 mg / 128 ≈ 15.6 mg.  8 -> ~125 mg.
 *  ACT_DUR LSB = 8 / ODR = 8 / 50 = 0.16 s.  31 -> ~5.0 s no-motion window. */
#define ACT_THRESHOLD   8U
#define ACT_DURATION    31U

static stmdev_ctx_t dev_ctx;
static struct gpio_callback int_cb;

static void int_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    int level = gpio_pin_get_dt(&accel_int);   /* logical level (debounced by hardware engine) */

    uwb_set_moving(level == ACCEL_INT_MOVING_LEVEL);
}

int motion_init(void)
{
    int err;

    err = lis2hh12_if_init(&dev_ctx);
    if (err) {
        return err;
    }

    /* Sensor: 2g full scale, 50 Hz ODR. */
    if (lis2hh12_xl_full_scale_set(&dev_ctx, LIS2HH12_2g) != 0) {
        return -EIO;
    }
    if (lis2hh12_xl_data_rate_set(&dev_ctx, LIS2HH12_XL_ODR_50Hz) != 0) {
        return -EIO;
    }

    /* Activity/inactivity engine. */
    if (lis2hh12_act_threshold_set(&dev_ctx, ACT_THRESHOLD) != 0) {
        return -EIO;
    }
    if (lis2hh12_act_duration_set(&dev_ctx, ACT_DURATION) != 0) {
        return -EIO;
    }

    /* INT1 electrical config: active high, not latched (level follows state). */
    if (lis2hh12_pin_polarity_set(&dev_ctx, LIS2HH12_ACTIVE_HIGH) != 0) {
        return -EIO;
    }
    if (lis2hh12_pin_notification_set(&dev_ctx, LIS2HH12_INT_PULSED) != 0) {
        return -EIO;
    }

    /* Route inactivity/activity signal to INT1. */
    lis2hh12_pin_int1_route_t route = {0};
    route.int1_inact = 1;
    if (lis2hh12_pin_int1_route_set(&dev_ctx, route) != 0) {
        return -EIO;
    }

    /* GPIO interrupt on INT1 (P0.28), both edges. */
    if (!gpio_is_ready_dt(&accel_int)) {
        return -ENODEV;
    }
    err = gpio_pin_configure_dt(&accel_int, GPIO_INPUT);
    if (err) {
        return err;
    }
    err = gpio_pin_interrupt_configure_dt(&accel_int, GPIO_INT_EDGE_BOTH);
    if (err) {
        return err;
    }
    gpio_init_callback(&int_cb, int_handler, BIT(accel_int.pin));
    gpio_add_callback(accel_int.port, &int_cb);

    /* Seed current state from the pin so we do not wait for the first edge. */
    uwb_set_moving(gpio_pin_get_dt(&accel_int) == ACCEL_INT_MOVING_LEVEL);

    return 0;
}
