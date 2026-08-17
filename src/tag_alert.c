#include "tag_alert.h"
#include "tag_alert_core.h"
#include "storage.h"
#include "cal_math.h"          /* cal_crc32 -- shared, host-tested CRC32 */
#include "batt.h"
#include "uwb_ss_initiator.h"  /* pos_last_get */
#include "uwb_net_runner.h"    /* uwb_net_runner_wake */

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <hal/nrf_ficr.h>
#include <math.h>
#include <string.h>
#include <stddef.h>

#define ALERT_NVS_ID  3

/* { epoch; active; crc } -- written only on a state change (raise, or the
 * completion of the bounded CANCEL sequence), never per 10 s repeat. See
 * design §3 / plan Task 4. "active" means a HELP is the tag's current
 * condition -- NOT "the CANCELLING countdown is still running". That way a
 * reset mid-CANCEL resumes the HELP rather than silently going quiet before
 * the operator has actually seen the cancel take effect on the gateway. */
struct alert_nvs_record {
    uint8_t  epoch;
    uint8_t  active;
    uint16_t crc;
};

static struct tag_alert_core core;
static uint8_t                tag_eui[UWB_FRAME_EUI_LEN];
K_MUTEX_DEFINE(alert_mtx);

static uint16_t alert_record_crc(const struct alert_nvs_record *r)
{
    return (uint16_t)cal_crc32(r, offsetof(struct alert_nvs_record, crc));
}

/* Caller must hold alert_mtx. */
static void alert_persist_locked(void)
{
    struct alert_nvs_record r;

    r.epoch  = core.epoch;
    r.active = (core.state == TAG_ALERT_HELP) ? 1u : 0u;
    r.crc    = alert_record_crc(&r);
    storage_write(ALERT_NVS_ID, &r, sizeof(r));
}

void tag_alert_init(void)
{
    struct alert_nvs_record r;
    uint8_t epoch  = 0;
    bool    resume = false;

    int got = storage_read(ALERT_NVS_ID, &r, sizeof(r));
    if (got == (int)sizeof(r) && alert_record_crc(&r) == r.crc) {
        epoch  = r.epoch;
        resume = (r.active != 0);
    }

    /* Same derivation as main.c's tag EUI (NRF_FICR->DEVICEID), computed here
     * too rather than threading it through every alert API call. */
    sys_put_le32(NRF_FICR->DEVICEID[0], &tag_eui[0]);
    sys_put_le32(NRF_FICR->DEVICEID[1], &tag_eui[4]);

    k_mutex_lock(&alert_mtx, K_FOREVER);
    tag_alert_core_init(&core, epoch);
    if (resume) {
        /* Resume the HELP at the persisted epoch -- do NOT call
         * tag_alert_core_raise(), which would burn a new epoch. A tag that
         * browns out mid-emergency must not come back silent. */
        core.state      = TAG_ALERT_HELP;
        core.repeat_seq = 0;
        core.next_tx_ms = 0;   /* due as soon as the runner asks */
        core.armed      = true;
    }
    k_mutex_unlock(&alert_mtx);
}

void tag_alert_raise(void)
{
    k_mutex_lock(&alert_mtx, K_FOREVER);
    if (tag_alert_core_raise(&core, k_uptime_get_32())) {
        alert_persist_locked();
    }
    k_mutex_unlock(&alert_mtx);
    /* The runner can be parked in a multi-superframe skip, and the alert is
     * only transmitted from inside its loop. Wake it so a HELP press reaches
     * the air in ~one superframe instead of waiting out the skip. Outside the
     * mutex: the runner takes the same mutex from tag_alert_frame_due(). */
    uwb_net_runner_wake();
}

void tag_alert_cancel(void)
{
    k_mutex_lock(&alert_mtx, K_FOREVER);
    /* No persist here: the state change that matters for NVS is the
     * *completion* of the cancel sequence (see tag_alert_sent()), so a reset
     * during the bounded CANCEL repeats still resumes as an active HELP. */
    tag_alert_core_cancel(&core, k_uptime_get_32());
    k_mutex_unlock(&alert_mtx);
    uwb_net_runner_wake();   /* same reasoning as tag_alert_raise() */
}

bool tag_alert_active(void)
{
    bool active;

    k_mutex_lock(&alert_mtx, K_FOREVER);
    active = (core.state == TAG_ALERT_HELP);
    k_mutex_unlock(&alert_mtx);
    return active;
}

uint8_t tag_alert_epoch(void)
{
    uint8_t epoch;

    k_mutex_lock(&alert_mtx, K_FOREVER);
    epoch = core.epoch;
    k_mutex_unlock(&alert_mtx);
    return epoch;
}

bool tag_alert_frame_due(struct uwb_alert *a, uint16_t short_addr, uint32_t now_ms)
{
    uint8_t state = 0, epoch = 0, rep = 0;

    k_mutex_lock(&alert_mtx, K_FOREVER);
    bool due = tag_alert_core_due(&core, now_ms, &state, &epoch, &rep);
    k_mutex_unlock(&alert_mtx);

    if (!due) {
        return false;
    }

    memset(a, 0, sizeof(*a));
    a->state      = state;
    a->epoch      = epoch;
    a->repeat_seq = rep;
    a->sender_hop = UWB_ALERT_HOP_UNKNOWN;   /* originator: any anchor picks it up */
    a->ttl        = ALERT_TTL_INIT;
    memcpy(a->orig_eui, tag_eui, UWB_FRAME_EUI_LEN);
    a->orig_addr  = short_addr;
    a->batt_soc   = batt_soc_cached();

    float x, y;
    if (pos_last_get(&x, &y)) {
        a->last_x = x;
        a->last_y = y;
    } else {
        a->last_x = NAN;
        a->last_y = NAN;
    }
    return true;
}

void tag_alert_sent(uint32_t now_ms)
{
    k_mutex_lock(&alert_mtx, K_FOREVER);
    tag_alert_st_t prev = core.state;

    tag_alert_core_sent(&core, now_ms);

    if (prev == TAG_ALERT_CANCELLING && core.state == TAG_ALERT_OFF) {
        alert_persist_locked();   /* cancel-complete: active = 0 */
    }
    k_mutex_unlock(&alert_mtx);
}
