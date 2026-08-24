#include "pos_cfg.h"
#include "storage.h"
#include "cal_math.h"          /* cal_crc32 -- shared, host-tested CRC32; see
                                 * its use in tag_alert.c for precedent on
                                 * reusing it outside cal.c itself. */
#include "ble_log.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>

#define POS_CFG_NVS_ID  4   /* 1 = cal, 2 = NFC name, 3 = alert state */

/* Same shape as struct cal_record (src/cal_math.h): magic + version guard the
 * id/schema, explicit padding keeps the layout independent of compiler
 * packing choices, crc32 covers everything before itself and is computed
 * last. Kept deliberately identical to that pattern rather than the lighter
 * {fields; crc16} form in tag_alert.c -- this is the one CLAUDE.md points at
 * for "the persistence pattern", and there is no reason for a third shape in
 * a codebase that already has two. */
struct pos_cfg_record {
    uint32_t magic;
    uint8_t  version;
    uint8_t  _pad;         /* explicit padding for stable layout */
    int16_t  anchor_h_cm;
    int16_t  tag_h_cm;
    uint16_t _pad2;        /* explicit padding for stable layout */
    uint32_t crc32;        /* integrity, computed last */
};

#define POS_CFG_MAGIC    0x504F5343u  /* "POSC" */
#define POS_CFG_VERSION  1u
#define POS_CFG_CRC_LEN  (offsetof(struct pos_cfg_record, crc32))

/* Current config, applied at compile time to the documented defaults so a
 * missed or not-yet-run pos_cfg_init() still leaves the range model with a
 * plausible dz instead of an uninitialized one.
 *
 * No lock. This is written only from the BT RX thread (pos_cfg_on_cmd() ->
 * pos_cfg_set()) and read from wherever the range model runs; unlike
 * tag_alert.c's {epoch, state, ...} tuple there is no cross-field invariant
 * a torn read could violate -- anchor_h_cm and tag_h_cm are independently
 * range-validated before they ever reach `active`, so the worst a race can
 * produce is one old field paired with one new field, both still individually
 * valid heights. */
static struct pos_cfg active = {
    .anchor_h_cm = POS_CFG_DEF_ANCHOR_H_CM,
    .tag_h_cm    = POS_CFG_DEF_TAG_H_CM,
};

static bool heights_valid(int32_t anchor_h_cm, int32_t tag_h_cm)
{
    return anchor_h_cm >= POS_CFG_H_MIN_CM && anchor_h_cm <= POS_CFG_H_MAX_CM &&
           tag_h_cm    >= POS_CFG_H_MIN_CM && tag_h_cm    <= POS_CFG_H_MAX_CM;
}

static void record_finalize(struct pos_cfg_record *r)
{
    r->magic   = POS_CFG_MAGIC;
    r->version = POS_CFG_VERSION;
    r->_pad    = 0;
    r->_pad2   = 0;
    r->crc32   = cal_crc32(r, POS_CFG_CRC_LEN);
}

static bool record_valid(const struct pos_cfg_record *r)
{
    if (r->magic != POS_CFG_MAGIC || r->version != POS_CFG_VERSION) {
        return false;
    }
    return r->crc32 == cal_crc32(r, POS_CFG_CRC_LEN);
}

/* ---- public API ------------------------------------------------------------ */

int pos_cfg_init(void)
{
    struct pos_cfg_record r;
    int got = storage_read(POS_CFG_NVS_ID, &r, sizeof(r));

    /* heights_valid() here is a second, independent check on top of the CRC:
     * the CRC only proves the bytes were not corrupted in flash, not that
     * they were ever a value this version's setter would have accepted. A
     * stale/foreign-but-intact record must not hand a bogus dz to the range
     * model. */
    if (got == (int)sizeof(r) && record_valid(&r) &&
        heights_valid(r.anchor_h_cm, r.tag_h_cm)) {
        active.anchor_h_cm = r.anchor_h_cm;
        active.tag_h_cm    = r.tag_h_cm;
        return 0;
    }

    /* No valid record: fall back to the documented defaults. `active` is
     * already initialized to them, but set explicitly so this function's
     * result does not depend on never having been called before. */
    active.anchor_h_cm = POS_CFG_DEF_ANCHOR_H_CM;
    active.tag_h_cm    = POS_CFG_DEF_TAG_H_CM;

    /* A genuine storage error (bad device, not mounted) is reported as-is.
     * "no entry" and "entry present but corrupt/out-of-range" both collapse
     * to -ENOENT: from the caller's point of view both just mean the
     * defaults are now in effect, and a corrupt payload is not a storage
     * failure -- the read itself succeeded. */
    if (got < 0 && got != -ENOENT) {
        return got;
    }
    return -ENOENT;
}

void pos_cfg_get(struct pos_cfg *out)
{
    *out = active;
}

int pos_cfg_set(int16_t anchor_h_cm, int16_t tag_h_cm)
{
    if (!heights_valid(anchor_h_cm, tag_h_cm)) {
        return -EINVAL;
    }

    struct pos_cfg_record r = {0};

    r.anchor_h_cm = anchor_h_cm;
    r.tag_h_cm    = tag_h_cm;
    record_finalize(&r);

    /* Persist before applying, same order as cal_store(): on a failed NVS
     * write `active` must stay exactly what it was, not a value that is live
     * in RAM but was never actually saved. */
    int rc = storage_write(POS_CFG_NVS_ID, &r, sizeof(r));
    if (rc < 0) {
        return rc;
    }

    active.anchor_h_cm = anchor_h_cm;
    active.tag_h_cm    = tag_h_cm;
    return 0;
}

float pos_cfg_dz_m(void)
{
    int32_t diff_cm = (int32_t)active.anchor_h_cm - (int32_t)active.tag_h_cm;

    return (float)diff_cm / 100.0f;
}

/* ---- command parser (runs in BT RX thread) -------------------------------- */

/* Parse a decimal non-negative integer from *p, advancing past it and any
 * surrounding spaces. Same shape as tag_cmd.c's parse_u32(), plus a hard cap
 * on the accumulator: valid heights top out at POS_CFG_H_MAX_CM (1500), so
 * more than a few digits is already invalid input, and capping early avoids
 * two failure modes at once -- an int32 overflow on a pathological digit run,
 * and a later (int16_t) cast in pos_cfg_on_cmd() wrapping a huge value back
 * into the valid 0..1500 window (e.g. 66036 truncates to 500 and would
 * otherwise be silently accepted as if it had been typed that way). Returns
 * false, leaving *out untouched, if there was no digit at all or the cap
 * was hit. */
static bool parse_cm(const char **p, int32_t *out)
{
    const char *s = *p;
    int32_t     v = 0;
    bool        any = false;

    while (*s == ' ') {
        s++;
    }
    while (*s >= '0' && *s <= '9') {
        v = v * 10 + (*s - '0');
        if (v > 100000) {
            return false;
        }
        s++;
        any = true;
    }
    while (*s == ' ') {
        s++;
    }
    if (!any) {
        return false;
    }
    *p = s;
    *out = v;
    return true;
}

bool pos_cfg_on_cmd(const char *cmd)
{
    /* Claim the whole "pos" keyword, the way the "pwr"/"cal" branches in
     * tag_cmd.c own their whole prefix: an unrecognized "pos ..." gets a
     * usage reply from here rather than falling through to cal_on_rx, which
     * would answer a mistyped position command with a "cal <mm>" usage line
     * that has nothing to do with what was typed. The word-boundary check
     * (cmd[3] must be NUL or a space) rejects a string like "posz" that
     * happens to start with "pos" but is not this command at all. */
    if (strncmp(cmd, "pos", 3) != 0 || (cmd[3] != '\0' && cmd[3] != ' ')) {
        return false;
    }

    const char *rest = cmd + 3;

    while (*rest == ' ') {
        rest++;
    }

    if (strcmp(rest, "z") == 0) {
        struct pos_cfg c;

        pos_cfg_get(&c);
        /* "Z a1500 t1500\n" = 14 bytes worst case, well under the 20-byte
         * NUS payload limit. */
        char msg[20];
        /* Clamped into a range the compiler can see before formatting.
         * Both fields are validated to [POS_CFG_H_MIN_CM,
         * POS_CFG_H_MAX_CM] on every path that reaches `active`, but that
         * is invisible here: gcc sizes the worst case from int16_t's full
         * range and warns the reply can want up to 23 bytes of a 20-byte
         * buffer (-Wformat-truncation). Clamping locally makes the bound
         * provable AND stops a regression in the validation from silently
         * producing an over-20-byte payload, which NUS drops with no
         * error at all. */
        unsigned a = (c.anchor_h_cm < 0) ? 0u : (unsigned)c.anchor_h_cm;
        unsigned t = (c.tag_h_cm    < 0) ? 0u : (unsigned)c.tag_h_cm;

        if (a > 9999u) { a = 9999u; }
        if (t > 9999u) { t = 9999u; }

        snprintf(msg, sizeof(msg), "Z a%u t%u\n", a, t);
        ble_log_send(msg);
        return true;
    }

    if (strncmp(rest, "z ", 2) == 0) {
        const char *p = rest + 2;
        int32_t     a_cm, t_cm;

        if (!parse_cm(&p, &a_cm) || !parse_cm(&p, &t_cm) || *p != '\0') {
            /* Missing arg, non-numeric input, or trailing garbage after the
             * second number -- reject rather than silently taking a partial
             * parse. "Z ERR usage\n" = 12 bytes. */
            ble_log_send("Z ERR usage\n");
            return true;
        }
        if (!heights_valid(a_cm, t_cm)) {
            /* Checked here, on the wide int32 values straight out of
             * parse_cm(), before either value is cast down to int16_t --
             * see parse_cm()'s comment on why the cast alone cannot be
             * trusted to catch an out-of-range input. "Z ERR range\n" =
             * 12 bytes. */
            ble_log_send("Z ERR range\n");
            return true;
        }

        int rc = pos_cfg_set((int16_t)a_cm, (int16_t)t_cm);
        if (rc < 0) {
            /* "Z ERR nvs\n" = 10 bytes. */
            ble_log_send("Z ERR nvs\n");
            return true;
        }

        struct pos_cfg c;

        pos_cfg_get(&c);
        /* Echo what is actually active, not what was typed -- there is no
         * clamping between the two here, but this matches the same
         * echo-after-set idiom tag_cmd.c's cmd_tier() uses.
         * "Z SET a1500 t1500\n" = 18 bytes worst case. */
        char msg[20];
        /* Clamped into a range the compiler can see before formatting.
         * Both fields are validated to [POS_CFG_H_MIN_CM,
         * POS_CFG_H_MAX_CM] on every path that reaches `active`, but that
         * is invisible here: gcc sizes the worst case from int16_t's full
         * range and warns the reply can want up to 23 bytes of a 20-byte
         * buffer (-Wformat-truncation). Clamping locally makes the bound
         * provable AND stops a regression in the validation from silently
         * producing an over-20-byte payload, which NUS drops with no
         * error at all. */
        unsigned a = (c.anchor_h_cm < 0) ? 0u : (unsigned)c.anchor_h_cm;
        unsigned t = (c.tag_h_cm    < 0) ? 0u : (unsigned)c.tag_h_cm;

        if (a > 9999u) { a = 9999u; }
        if (t > 9999u) { t = 9999u; }

        snprintf(msg, sizeof(msg), "Z SET a%u t%u\n", a, t);
        ble_log_send(msg);
        return true;
    }

    /* "pos" alone, or an unrecognized "pos <something>". */
    ble_log_send("Z ERR usage\n");
    return true;
}
