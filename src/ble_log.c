#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <bluetooth/services/nus.h>
#include "ble_log.h"

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

/* Deferred advertising restart — must NOT be called from the BT RX thread
 * because bt_le_adv_start() sends an HCI command and waits for the completion
 * event, which is also processed by the BT RX thread → deadlock. */
static struct k_work adv_work;

static void adv_work_fn(struct k_work *work)
{
    ARG_UNUSED(work);
    bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (state_cb) {
        state_cb(BLE_STATE_ADVERTISING);
    }
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

    err = bt_enable(NULL);
    if (err) {
        return err;
    }

    err = bt_nus_init(&nus_cb);
    if (err) {
        return err;
    }

    return bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
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
