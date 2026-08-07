#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include "batt.h"
#include "batt_curve.h"
#include "batt_window.h"

static const struct device *fg = DEVICE_DT_GET(DT_NODELABEL(npm1304_charger));

/* True while the charger is plugged in. This attribute does its own register
 * read, so it needs no preceding sensor_sample_fetch(). If the query fails,
 * report "not charging": a possibly-inflated percentage beats none at all. */
static bool vbus_present(void)
{
    struct sensor_value val;

    if (sensor_attr_get(fg, (enum sensor_channel)SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS,
                        (enum sensor_attribute)SENSOR_ATTR_NPM13XX_CHARGER_VBUS_PRESENT,
                        &val) < 0) {
        return false;
    }

    return val.val1 != 0;
}

static int batt_read_millivolts(int *mv)
{
    struct sensor_value val;

    if (!device_is_ready(fg)) {
        return -ENODEV;
    }
    if (sensor_sample_fetch(fg) < 0) {
        return -EIO;
    }
    if (sensor_channel_get(fg, SENSOR_CHAN_GAUGE_VOLTAGE, &val) < 0) {
        return -EIO;
    }

    *mv = (val.val1 * 1000) + (val.val2 / 1000);
    return 0;
}

int batt_read_soc(int *soc)
{
    int mv;
    int err;

    if (soc == NULL) {
        return -EINVAL;
    }
    if (!device_is_ready(fg)) {
        return -ENODEV;
    }

    /* While charging, the terminal voltage is driven by the charger, not by
     * the cell's state — mapping it through the curve would report near-full
     * for a half-empty battery. Say "charging" instead of lying. */
    if (vbus_present()) {
        return -EBUSY;
    }

    err = batt_read_millivolts(&mv);
    if (err != 0) {
        return err;
    }

    *soc = lipo_mv_to_pct(mv);
    return 0;
}

int batt_read_current(int *ma)
{
    struct sensor_value val;

    if (ma == NULL) {
        return -EINVAL;
    }
    if (!device_is_ready(fg)) {
        return -ENODEV;
    }
    if (sensor_sample_fetch(fg) < 0) {
        return -EIO;
    }
    if (sensor_channel_get(fg, SENSOR_CHAN_GAUGE_AVG_CURRENT, &val) < 0) {
        return -EIO;
    }

    /* Zephyr current units: val1 = whole amps, val2 = microamp fraction. */
    long ua  = (long)val.val1 * 1000000L + val.val2;
    long mma = ua / 1000L;
    *ma = (int)(mma < 0 ? -mma : mma);
    return 0;
}

#define BATT_SAMPLE_MS 5000

static struct batt_window idle_win;
static volatile bool      batt_connected;

static void batt_work_fn(struct k_work *work)
{
    ARG_UNUSED(work);

    int ma;
    if (batt_read_current(&ma) != 0) {
        return;   /* no gauge / no battery: skip this tick */
    }

    /* Only accumulate while BLE is off: a live connection adds the BLE radio's
     * current and would skew the field-current estimate. The window is read on
     * demand via batt_get_idle_window() (the `pwr idle` command). */
    if (!batt_connected) {
        batt_window_add(&idle_win, ma);
    }
}

K_WORK_DEFINE(batt_work, batt_work_fn);

static void batt_timer_fn(struct k_timer *t)
{
    ARG_UNUSED(t);
    k_work_submit(&batt_work);   /* gauge I2C reads must not run in timer ISR */
}

K_TIMER_DEFINE(batt_timer, batt_timer_fn, NULL);

void batt_monitor_start(void)
{
    batt_window_reset(&idle_win);
    k_timer_start(&batt_timer, K_MSEC(BATT_SAMPLE_MS), K_MSEC(BATT_SAMPLE_MS));
}

void batt_on_ble_state(ble_state_t state)
{
    if (state == BLE_STATE_CONNECTED) {
        batt_connected = true;   /* stop accumulating; window holds the just-
                                  * finished disconnected period for `pwr idle` */
    } else {
        batt_connected = false;
        batt_window_reset(&idle_win);   /* start a fresh disconnected window */
    }
}

int batt_get_idle_window(int *mean_ma, int *min_ma, int *max_ma, uint32_t *count)
{
    /* batt_window_get order is (min, mean, max, count). */
    return batt_window_get(&idle_win, min_ma, mean_ma, max_ma, count);
}
