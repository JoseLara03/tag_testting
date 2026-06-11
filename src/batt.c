#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include "batt.h"

static const struct device *fg = DEVICE_DT_GET(DT_NODELABEL(fuel_gauge));

int batt_read_soc(int *soc)
{
    struct sensor_value val;

    if (soc == NULL) {
        return -EINVAL;
    }
    if (!device_is_ready(fg)) {
        return -ENODEV;
    }
    if (sensor_sample_fetch(fg) < 0) {
        return -EIO;
    }
    if (sensor_channel_get(fg, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE, &val) < 0) {
        return -EIO;
    }

    *soc = val.val1;   /* val1 = percent */
    return 0;
}
