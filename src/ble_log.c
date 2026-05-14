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

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (!err) {
        current_conn = bt_conn_ref(conn);
    }
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    if (current_conn) {
        bt_conn_unref((struct bt_conn *)current_conn);
        current_conn = NULL;
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = on_connected,
    .disconnected = on_disconnected,
};

static void nus_received(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    /* TX-only — ignore incoming data */
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
    bt_nus_send(conn, (const uint8_t *)msg, strlen(msg));
}
