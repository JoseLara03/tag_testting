#include "blink_cfg.h"
#include "storage.h"
#include "cal_math.h"      /* cal_crc32 -- same reuse pos_cfg.c documents */
#include "ble_log.h"

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define BLINK_CFG_NVS_ID  5   /* 1 = cal, 2 = NFC name, 3 = alert, 4 = pos_cfg */

struct blink_cfg_record {
    uint32_t magic;
    uint8_t  version;
    uint8_t  enabled;      /* 0 or 1; any other value is rejected */
    uint16_t _pad;         /* explicit padding for stable layout */
    uint32_t crc32;        /* integrity, computed last */
};

#define BLINK_CFG_MAGIC    0x424C4E4Bu  /* "BLNK" */
#define BLINK_CFG_VERSION  1u
#define BLINK_CFG_CRC_LEN  (offsetof(struct blink_cfg_record, crc32))

/* No lock: a single bool, written only from the BT RX thread and read from
 * the runner thread. There is no cross-field invariant a torn read could
 * violate, and a bool cannot tear on this core. */
static bool active;   /* false = TWR sweep, today's behaviour */

int blink_cfg_init(void)
{
    struct blink_cfg_record r;
    int got = storage_read(BLINK_CFG_NVS_ID, &r, sizeof(r));

    if (got == (int)sizeof(r) && r.magic == BLINK_CFG_MAGIC &&
        r.version == BLINK_CFG_VERSION &&
        r.crc32 == cal_crc32(&r, BLINK_CFG_CRC_LEN) && r.enabled <= 1u) {
        active = (r.enabled != 0u);
        return 0;
    }

    /* No valid record: the safe default, explicitly, so the result does not
     * depend on this never having been called before. */
    active = false;
    if (got < 0 && got != -ENOENT) {
        return got;
    }
    return -ENOENT;
}

bool blink_cfg_enabled(void)
{
    return active;
}

int blink_cfg_set(bool enabled)
{
    struct blink_cfg_record r = {0};

    r.magic   = BLINK_CFG_MAGIC;
    r.version = BLINK_CFG_VERSION;
    r.enabled = enabled ? 1u : 0u;
    r._pad    = 0;
    r.crc32   = cal_crc32(&r, BLINK_CFG_CRC_LEN);

    /* Persist before applying, same order as cal_store()/pos_cfg_set(). */
    int rc = storage_write(BLINK_CFG_NVS_ID, &r, sizeof(r));
    if (rc < 0) {
        return rc;
    }
    active = enabled;
    return 0;
}

bool blink_cfg_on_cmd(const char *cmd)
{
    /* Claim the whole `blink` keyword, the way pos_cfg_on_cmd() claims `pos`.
     * The word-boundary check rejects a string that merely starts with it. */
    if (strncmp(cmd, "blink", 5) != 0 || (cmd[5] != '\0' && cmd[5] != ' ')) {
        return false;
    }

    const char *rest = cmd + 5;

    while (*rest == ' ') {
        rest++;
    }

    if (*rest == '\0') {
        ble_log_send(blink_cfg_enabled() ? "BLINK on\n" : "BLINK off\n");
        return true;
    }
    if (strcmp(rest, "on") == 0 || strcmp(rest, "off") == 0) {
        bool en = (strcmp(rest, "on") == 0);
        int  rc = blink_cfg_set(en);

        if (rc < 0) {
            ble_log_send("BLINK ERR nvs\n");
            return true;
        }
        /* Echo what is actually active, not what was typed. Takes effect on
         * the next cadence slot -- no reboot needed. */
        ble_log_send(blink_cfg_enabled() ? "BLINK SET on\n" : "BLINK SET off\n");
        return true;
    }

    ble_log_send("BLINK ERR usage\n");
    return true;
}
