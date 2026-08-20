#include "cal.h"
#include "cal_math.h"
#include "ble_log.h"
#include "phy_config.h"   /* CONFIG_OPTION, TX_ANT_DLY, RX_ANT_DLY */
#include "storage.h"

#include <string.h>
#include <stdio.h>

static struct cal_record active;
static bool               active_valid;

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
    if (!active_valid) {
        /* No stored record: hand back the factory reference. Callers that
         * must not range uncalibrated are gated by uwb_net_gate_actions();
         * this is the backstop for the rest. */
        *tx = TX_ANT_DLY;
        *rx = RX_ANT_DLY;
        return;
    }

    *tx = active.tx_ant_dly;
    *rx = active.rx_ant_dly;
}

bool cal_is_valid(void)
{
    return active_valid;
}

void cal_internal_activate(const struct cal_record *r)
{
    active = *r;
    active_valid = true;
}

void cal_internal_invalidate(void)
{
    active_valid = false;
}

void cal_on_rx(const uint8_t *data, uint16_t len)
{
    char buf[24];
    uint16_t n = (len < sizeof(buf) - 1) ? len : (sizeof(buf) - 1);

    memcpy(buf, data, n);
    buf[n] = '\0';
    while (n > 0 && (buf[n - 1] == '\r' || buf[n - 1] == '\n')) {
        buf[--n] = '\0';
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

    /* Every other `cal ...` command writes or drives the radio, and a
     * production build never does either -- see
     * docs/superpowers/specs/2026-08-20-cal-image-rewrite-design.md. */
    ble_log_send("CAL ERR unavailable\n");
}
