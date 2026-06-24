#include "cal.h"
#include "cal_math.h"
#include "ble_log.h"
#include "phy_config.h"   /* CONFIG_OPTION */
#include "storage.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define CAL_NVS_ID          1

static struct cal_record active;
static bool             active_valid;

static K_SEM_DEFINE(cal_req_sem, 0, 1);
static volatile uint32_t cal_req_ref_mm;
static volatile bool     cal_req_pending;

/* ---- command parser (runs in BT RX thread) -------------------------------- */
void cal_on_rx(const uint8_t *data, uint16_t len)
{
    char buf[24];
    uint16_t n = (len < sizeof(buf) - 1) ? len : (sizeof(buf) - 1);

    memcpy(buf, data, n);
    buf[n] = '\0';
    /* strip trailing CR/LF */
    while (n > 0 && (buf[n - 1] == '\r' || buf[n - 1] == '\n')) {
        buf[--n] = '\0';
    }

    if (strcmp(buf, "cal clear") == 0) {
        ble_log_send(cal_clear() == 0 ? "CAL cleared\n" : "CAL FAIL nvs\n");
        return;
    }
    if (strcmp(buf, "cal status") == 0) {
        if (active_valid) {
            char msg[20];
            (void)snprintf(msg, sizeof(msg), "CAL %u/%u\n",
                           active.tx_ant_dly, active.rx_ant_dly);
            ble_log_send(msg);
        } else {
            ble_log_send("CAL REQUIRED\n");
        }
        return;
    }
    if (strcmp(buf, "cal selftest") == 0) {
        char msg[20];
        (void)snprintf(msg, sizeof(msg), "SELFTEST %d\n", cal_math_selftest());
        ble_log_send(msg);
        return;
    }
    if (strncmp(buf, "cal ", 4) == 0) {
        char *end;
        long mm = strtol(buf + 4, &end, 10);
        if (end != buf + 4 && mm > 0) {
            cal_req_ref_mm = (uint32_t)mm;
            cal_req_pending = true;
            k_sem_give(&cal_req_sem);
            ble_log_send("CAL start\n");
            return;
        }
    }
    ble_log_send("CAL ERR usage: cal <mm>\n");
}

/* ---- public API ------------------------------------------------------------ */
bool cal_init(void)
{
    struct cal_record r;
    int got = storage_read(CAL_NVS_ID, &r, sizeof(r));

    if (got == (int)sizeof(r) && cal_record_valid(&r, (uint8_t)CONFIG_OPTION)) {
        active = r;
        active_valid = true;
        return true;
    }
    active_valid = false;
    return false;
}

void cal_get_ant_dly(uint16_t *tx, uint16_t *rx)
{
    *tx = active.tx_ant_dly;
    *rx = active.rx_ant_dly;
}

bool cal_is_valid(void)
{
    return active_valid;
}

int cal_clear(void)
{
    int rc = storage_delete(CAL_NVS_ID);

    if (rc == 0) {
        active_valid = false;
    }
    return rc;
}

bool cal_take_request(uint32_t *out_ref_mm)
{
    if (!cal_req_pending) {
        return false;
    }
    *out_ref_mm = cal_req_ref_mm;
    cal_req_pending = false;
    return true;
}

void cal_wait_request(void)
{
    k_sem_take(&cal_req_sem, K_FOREVER);
}

int cal_store(uint16_t tx, uint16_t rx, uint32_t ref_mm, uint16_t residual_mm)
{
    struct cal_record r = {0};

    r.phy_option  = (uint8_t)CONFIG_OPTION;
    r.tx_ant_dly  = tx;
    r.rx_ant_dly  = rx;
    r.ref_mm      = ref_mm;
    r.residual_mm = residual_mm;
    cal_record_finalize(&r);

    int rc = storage_write(CAL_NVS_ID, &r, sizeof(r));

    if (rc < 0) {
        return rc;
    }
    active = r;
    active_valid = true;
    return 0;
}
