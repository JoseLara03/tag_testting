#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "motion.h"
#include "lis2hh12_if.h"
#include "uwb_ss_initiator.h"

/* INT1 pin: P0.28, declared as accel_int in the board DTS. */
static const struct gpio_dt_spec accel_int =
    GPIO_DT_SPEC_GET(DT_NODELABEL(accel_int), gpios);

/* Logical pin level that means "moving". INT1_INACT is an inactivity status:
 * with active-high polarity the pin is HIGH while still, LOW while moving
 * (datasheet/AN4662). Verified on hardware; flip (0/1) if cadence is inverted. */
#define ACCEL_INT_MOVING_LEVEL  0

/* Activity tuning (LIS2HH12, FS=2g, ODR=10 Hz):
 *  ACT_THS LSB = FS/128 = 2000 mg / 128 ≈ 15.6 mg.  8 -> ~125 mg.
 *  ACT_DUR LSB = 8 / ODR = 8 / 10 = 0.80 s.  6 -> ~4.8 s no-motion window.
 *
 * ACT_DURATION is tied to the ODR and must move with it: at the previous
 * 50 Hz the LSB was 0.16 s and 31 gave the same ~5 s window. Dropping to
 * 10 Hz (~180 uA -> ~50 uA) while leaving ACT_DURATION at 31 would silently
 * stretch the inactivity window to ~24.8 s, so the tag would keep ranging at
 * the FAST cadence for 25 s after it stopped moving -- the opposite of the
 * intent. ACT_THS is unaffected: its LSB is FS/128, independent of ODR. */
#define ACT_THRESHOLD   8U
#define ACT_DURATION    6U

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

    /* Sensor: 2g full scale, 10 Hz ODR. */
    if (lis2hh12_xl_full_scale_set(&dev_ctx, LIS2HH12_2g) != 0) {
        return -EIO;
    }
    if (lis2hh12_xl_data_rate_set(&dev_ctx, LIS2HH12_XL_ODR_10Hz) != 0) {
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
