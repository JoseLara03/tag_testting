#include <zephyr/kernel.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <stdio.h>
#include "batt.h"
#include "ble_log.h"

static const struct device *fg = DEVICE_DT_GET(DT_NODELABEL(fuel_gauge));
static struct k_work  batt_work;
static struct k_timer batt_timer;

static void batt_work_handler(struct k_work *w)
{
    ARG_UNUSED(w);

    fuel_gauge_prop_t props[] = {
        FUEL_GAUGE_VOLTAGE,
        FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE,
    };
    union fuel_gauge_prop_val vals[ARRAY_SIZE(props)];

    int err = fuel_gauge_get_props(fg, props, vals, ARRAY_SIZE(props));
    if (err) {
        ble_log_send("BATT: ERR\n");
        return;
    }

    char msg[32];
    snprintf(msg, sizeof(msg), "BATT: %dmV %d%%\n",
             vals[0].voltage / 1000,
             (int)vals[1].relative_state_of_charge);
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
