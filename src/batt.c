#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include "batt.h"
#include "ble_log.h"

static const struct device *fg = DEVICE_DT_GET(DT_NODELABEL(fuel_gauge));
static struct k_work  batt_work;
static struct k_timer batt_timer;

static void batt_work_handler(struct k_work *w)
{
    ARG_UNUSED(w);

    struct sensor_value voltage, soc;

    if (sensor_sample_fetch(fg) < 0) {
        ble_log_send("BATT: ERR\n");
        return;
    }

    if (sensor_channel_get(fg, SENSOR_CHAN_GAUGE_VOLTAGE, &voltage) < 0 ||
        sensor_channel_get(fg, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE, &soc) < 0) {
        ble_log_send("BATT: ERR\n");
        return;
    }

    /* val1 = V, val2 = µV fractional → mV */
    int mv = voltage.val1 * 1000 + voltage.val2 / 1000;

    char msg[32];
    snprintf(msg, sizeof(msg), "BATT: %dmV %d%%\n", mv, soc.val1);
    ble_log_send(msg);
}

static void batt_timer_cb(struct k_timer *t)
{
    ARG_UNUSED(t);
    k_work_submit(&batt_work);
}

int batt_init(void)
{
    if (!device_is_ready(fg)) {
        return -ENODEV;
    }

    k_work_init(&batt_work, batt_work_handler);
    k_timer_init(&batt_timer, batt_timer_cb, NULL);
    k_timer_start(&batt_timer, K_SECONDS(10), K_SECONDS(10));

    return 0;
}
