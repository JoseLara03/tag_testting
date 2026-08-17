#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <bluetooth/services/nus.h>
#include "ble_log.h"
#include "uwb_net_runner.h"   /* uwb_radio_sleep_enabled() -- bench-debug escape */

static volatile struct bt_conn *current_conn;
static K_SEM_DEFINE(notif_sem, 0, 1);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static ble_state_cb_t state_cb;

void ble_log_set_state_cb(ble_state_cb_t cb)
{
    state_cb = cb;
}

/* Slow connectable advertising, 1.0-1.2 s. BT_LE_ADV_CONN_FAST_2 (100-150 ms,
 * running forever) costs ~150 uA -- roughly 60% of the whole 250 uA power
 * budget -- for a channel nothing operational depends on: positions reach the
 * gateway over the 0xEA POS frame, and NUS is diagnostics/config only. */
#define ADV_PARAM_SLOW \
    BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN, BT_GAP_ADV_SLOW_INT_MIN, \
                    BT_GAP_ADV_SLOW_INT_MAX, NULL)

/* How long a ble_log_adv_window() stays open. */
#define ADV_WINDOW_MS  60000U

/* `pwr adv on` -- RAM only, deliberately not persisted: a debug session that
 * outlives a reboot would silently cost the budget it was meant to measure. */
static bool adv_forced;

/* True while advertising must run continuously rather than in a 60 s window.
 * `pwr sleep off` means the radio duty-cycling is off for bench work, so the
 * BLE policy follows it -- debugging is unaffected by this change. */
static bool adv_continuous(void)
{
    return adv_forced || !uwb_radio_sleep_enabled();
}

/* Deferred advertising restart — must NOT be called from the BT RX thread
 * because bt_le_adv_start() sends an HCI command and waits for the completion
 * event, which is also processed by the BT RX thread → deadlock. Everything
 * that opens a window goes through this work item for that reason, including
 * `pwr adv on`, which arrives on the BT RX thread. */
static struct k_work adv_work;
static struct k_work_delayable adv_stop_work;

static void adv_work_fn(struct k_work *work)
{
    ARG_UNUSED(work);
    int err = bt_le_adv_start(ADV_PARAM_SLOW, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

    /* -EALREADY is the normal case for a second window request: keep going and
     * re-arm the timer, which is what makes ble_log_adv_window() idempotent. */
    if (err != 0 && err != -EALREADY) {
        return;
    }
    if (state_cb) {
        state_cb(BLE_STATE_ADVERTISING);
    }
    if (adv_continuous()) {
        (void)k_work_cancel_delayable(&adv_stop_work);
    } else {
        (void)k_work_reschedule(&adv_stop_work, K_MSEC(ADV_WINDOW_MS));
    }
}

static void adv_stop_work_fn(struct k_work *work)
{
    ARG_UNUSED(work);

    /* A live connection already suppressed advertising, and the disconnect
     * handler opens a fresh window -- so never tear it down from under one. */
    if (current_conn != NULL || adv_continuous()) {
        return;
    }
    (void)bt_le_adv_stop();
}

void ble_log_adv_window(void)
{
    k_work_submit(&adv_work);
}

void ble_log_adv_force(bool on)
{
    adv_forced = on;
    if (on) {
        k_work_submit(&adv_work);
    } else {
        /* Do not stop here: leave a normal 60 s window behind so an operator
         * who typed `pwr adv off` over NUS is not disconnected mid-session. */
        (void)k_work_reschedule(&adv_stop_work, K_MSEC(ADV_WINDOW_MS));
    }
}

bool ble_log_adv_forced(void)
{
    return adv_forced;
}

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (!err) {
        current_conn = bt_conn_ref(conn);
        if (state_cb) {
            state_cb(BLE_STATE_CONNECTED);
        }
    }
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    if (current_conn) {
        bt_conn_unref((struct bt_conn *)current_conn);
        current_conn = NULL;
    }
    k_work_submit(&adv_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = on_connected,
    .disconnected = on_disconnected,
};

static ble_rx_handler_t rx_handler;

void ble_log_set_rx_handler(ble_rx_handler_t handler)
{
    rx_handler = handler;
}

static void nus_received(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    ARG_UNUSED(conn);
    if (rx_handler) {
        rx_handler(data, len);
    }
}

static void nus_send_enabled(enum bt_nus_send_status status)
{
    if (status == BT_NUS_SEND_STATUS_ENABLED) {
        k_sem_give(&notif_sem);
    }
}

static struct bt_nus_cb nus_cb = {
    .received     = nus_received,
    .send_enabled = nus_send_enabled,
};

int ble_log_init(void)
{
    int err;

    k_work_init(&adv_work, adv_work_fn);
    k_work_init_delayable(&adv_stop_work, adv_stop_work_fn);

    err = bt_enable(NULL);
    if (err) {
        return err;
    }

    err = bt_nus_init(&nus_cb);
    if (err) {
        return err;
    }

    /* No advertising at boot. The tag becomes discoverable only on a button
     * press or an NFC field (ble_log_adv_window()), or continuously under
     * `pwr adv on` / `pwr sleep off`. Anything that reads state over NUS --
     * `cal status`, `pwr idle`, `pwr rx` -- now needs a button press first. */
    return 0;
}

void ble_log_wait_ready(void)
{
    /* Blocks until the central writes the NUS TX CCCD (enables notifications) */
    k_sem_take(&notif_sem, K_FOREVER);
}

void ble_log_send(const char *msg)
{
    struct bt_conn *conn = (struct bt_conn *)current_conn;

    if (!conn) {
        return;
    }

    /* Retry on -ENOMEM: BLE TX buffer pool can be momentarily exhausted
     * when several sends are issued back-to-back. Block until drained. */
    for (int i = 0; i < 50; i++) {
        int err = bt_nus_send(conn, (const uint8_t *)msg, strlen(msg));
        if (err != -ENOMEM) {
            return;
        }
        k_msleep(10);
    }
}
