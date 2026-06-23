#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include "batt.h"
#include "batt_window.h"

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
static volatile bool      report_pending;

static void batt_work_fn(struct k_work *work)
{
    ARG_UNUSED(work);

    int ma;
    if (batt_read_current(&ma) != 0) {
        return;   /* no gauge / no battery: skip this tick */
    }

    if (batt_connected) {
        if (report_pending) {
            int mn = 0, me = 0, mx = 0; uint32_t n = 0;
            if (batt_window_get(&idle_win, &mn, &me, &mx, &n)) {
                char msg[20];
                snprintf(msg, sizeof(msg), "Idle:%d %d/%d n%u\n", me, mn, mx, n);
                ble_log_send(msg);
            }
            report_pending = false;
        }
    } else {
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
        batt_connected = true;
        report_pending = true;   /* dump the idle window on the next tick */
    } else {
        batt_connected = false;
        batt_window_reset(&idle_win);   /* start a fresh disconnected window */
    }
}
