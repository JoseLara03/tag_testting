#include "cal_led.h"
#include "cal.h"

#include <zephyr/drivers/led_strip.h>
#include <string.h>

static const struct device *led_strip_dev;

static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    struct led_rgb px = { .r = r, .g = g, .b = b };

    if (led_strip_dev != NULL) {
        led_strip_update_rgb(led_strip_dev, &px, 1);
    }
}

void cal_led_init(const struct device *strip)
{
    led_strip_dev = strip;

    if (cal_is_valid()) {
        led_set(0, 3, 0);   /* dim green: calibration stored */
    } else {
        led_set(3, 2, 0);   /* dim amber: no calibration */
    }
}

void cal_led_on_result(const char *verdict)
{
    if (strncmp(verdict, "CAL running", 11) == 0) {
        led_set(0, 0, 10);  /* blue: run in progress */
    } else if (strncmp(verdict, "CAL OK", 6) == 0) {
        led_set(0, 10, 0);  /* green: CAL OK */
    } else if (strncmp(verdict, "CAL FAIL", 8) == 0) {
        led_set(10, 0, 0);  /* red: CAL FAIL ... */
    }
    /* Anything else latched here ("CAL none", a `cal status` echo, etc.) is
     * not a verdict from a run just finished or in progress; leave the LED
     * showing whatever it last showed. */
}
