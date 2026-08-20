#ifndef CAL_LED_H_
#define CAL_LED_H_

#include <zephyr/device.h>

/*
 * Cal-image status LED: reflects the last calibration verdict via
 * cal_set_last_result()'s funnel in cal_run.c. See spec §7.
 * Compiled only under CONFIG_TAG_CAL_MODE.
 */

/* Set the initial LED state (dim green if a calibration is already stored,
 * dim amber otherwise) and remember the strip device for later updates. Call
 * once from main() bring-up, after cal_init(). */
void cal_led_init(const struct device *strip);

/* Called from cal_set_last_result() with the verdict string just latched.
 * Recognises the fixed set of prefixes cal.c/uwb_ss_initiator.c produce
 * ("CAL running", "CAL OK...", "CAL FAIL...") and drives the LED
 * accordingly; any other latched string (e.g. "CAL none", a `cal status`
 * echo) leaves the LED as it was. */
void cal_led_on_result(const char *verdict);

#endif /* CAL_LED_H_ */
