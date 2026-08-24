#include "pos_dbg.h"
#include "pos_solver.h"   /* POS_MAX_ANCHORS */
#include "ble_log.h"
#include <math.h>
#include <string.h>

/*
 * TEMPORARY raw-range debug log -- see pos_dbg.h and
 * spec/2026-08-22-position-filtering-design.md ("Temporary instrumentation:
 * raw-range log over BLE") for the record layouts and the removal checklist.
 * Everything the feature needs lives in this one file on purpose, so deleting
 * it later is one file removal plus the small integration edits listed there.
 *
 * Part 1 (below the "pure encoders" marker) is host-tested in
 * tests/pos_dbg/test_pos_dbg.c and must not reach into Part 2's state.
 * Part 2 is the Zephyr-facing glue: enable flags, the emit path and the `dbg`
 * command family.
 */

/* SWEEP record layout hardcodes 4 anchor slots: quality bytes at offsets
 * 14-17, res at 18-19, a 4-bit mask, and pos_dbg_sweep() passes
 * &session.set_aid[2] for the second SET record. If POS_MAX_ANCHORS were
 * 3 those reads run past the end; if 5+ the quality loop overwrites res. */
#ifdef __ZEPHYR__
#include <zephyr/toolchain.h>
BUILD_ASSERT(POS_MAX_ANCHORS == 4,
             "SWEEP layout hardcodes 4 slots: q at 14-17, res at 18-19, a "
             "4-bit mask, and the base-2 SET record");
#else
/* Host build: the same check, using a C99-portable construct. */
typedef char pos_dbg_slots_check[(POS_MAX_ANCHORS == 4) ? 1 : -1];
#endif

/* twr_log()/twr_log_raw() are declared in uwb_ss_initiator.h, but that header
 * pulls in <zephyr/kernel.h> (k_timeout_t, K_MSGQ_DEFINE, ...), which the host
 * gcc used for tests/pos_dbg/ does not have on its include path. Forward-
 * declaring the two functions we actually call -- with signatures that are
 * plain C and must stay identical to the real declarations -- avoids that
 * dependency entirely, so this whole file (encoders *and* glue) builds and
 * links as one host-tested unit against stub definitions, rather than only
 * the encoders. The real firmware link is unaffected: a matching extern
 * declaration in another translation unit is ordinary, valid C. */
void twr_log(const char *fmt, ...);
void twr_log_raw(const uint8_t *buf, size_t len);

/* ============================================================================
 * Part 1 -- pure encoders. No Zephyr, no globals below this point except the
 * ones declared inside each function. Byte layouts are pos_dbg.h's contract;
 * every multi-byte field is written byte-by-byte so this does not depend on
 * host/target endianness or on struct packing.
 * ==========================================================================*/

size_t pos_dbg_enc_sweep(uint8_t *buf, size_t cap,
                         uint8_t seq, uint16_t dt_ms, uint8_t epoch,
                         uint8_t mask, uint8_t tier, bool moving, bool solved,
                         const int16_t *r_cm, const uint8_t *q,
                         uint16_t res_cm)
{
    uint8_t flags;
    int     i;

    if (buf == NULL || cap < POS_DBG_SWEEP_LEN) {
        return 0;
    }

    flags = (uint8_t)(mask & 0x0Fu);
    flags = (uint8_t)(flags | ((uint8_t)((tier & 0x03u) << 4)));
    flags = (uint8_t)(flags | (moving ? (1u << 6) : 0u));
    flags = (uint8_t)(flags | (solved ? (1u << 7) : 0u));

    buf[0] = POS_DBG_MAGIC_SWEEP;
    buf[1] = seq;
    buf[2] = (uint8_t)dt_ms;
    buf[3] = (uint8_t)(dt_ms >> 8);
    buf[4] = epoch;
    buf[5] = flags;

    /* Slots whose mask bit is clear are written 0, regardless of whatever the
     * caller left in r_cm/q there -- pos_dbg.h documents this as the wire
     * contract, and enforcing it here (rather than trusting every caller to
     * have zeroed unused slots first) is what makes it actually hold. */
    for (i = 0; i < POS_MAX_ANCHORS; i++) {
        int16_t  v   = 0;
        uint16_t off = (uint16_t)(6 + 2 * i);

        if ((mask & (1u << i)) != 0 && r_cm != NULL) {
            v = r_cm[i];
        }
        buf[off]     = (uint8_t)((uint16_t)v);
        buf[off + 1] = (uint8_t)(((uint16_t)v) >> 8);
    }

    /* q == NULL encodes all-zero quality (the "dbg q off" default). */
    for (i = 0; i < POS_MAX_ANCHORS; i++) {
        uint8_t qv = 0;

        if (q != NULL && (mask & (1u << i)) != 0) {
            qv = q[i];
        }
        buf[14 + i] = qv;
    }

    buf[18] = (uint8_t)res_cm;
    buf[19] = (uint8_t)(res_cm >> 8);

    return POS_DBG_SWEEP_LEN;
}

size_t pos_dbg_enc_set(uint8_t *buf, size_t cap, uint8_t epoch, uint8_t base,
                       const uint8_t *aid, const int16_t *x_cm,
                       const int16_t *y_cm)
{
    int i;

    if (buf == NULL || cap < POS_DBG_SET_LEN) {
        return 0;
    }
    if (aid == NULL || x_cm == NULL || y_cm == NULL) {
        return 0;
    }
    if (base != 0 && base != 2) {
        return 0;
    }

    buf[0] = POS_DBG_MAGIC_SET;
    buf[1] = epoch;
    buf[2] = base;

    for (i = 0; i < (int)POS_DBG_SET_SLOTS; i++) {
        uint16_t off = (uint16_t)(3 + i * 5);   /* 1 aid + 2 x + 2 y per slot */
        uint16_t xv  = (uint16_t)x_cm[i];
        uint16_t yv  = (uint16_t)y_cm[i];

        buf[off]     = aid[i];
        buf[off + 1] = (uint8_t)xv;
        buf[off + 2] = (uint8_t)(xv >> 8);
        buf[off + 3] = (uint8_t)yv;
        buf[off + 4] = (uint8_t)(yv >> 8);
    }

    return POS_DBG_SET_LEN;
}

size_t pos_dbg_enc_mark(uint8_t *buf, size_t cap, uint8_t seq, uint8_t mark_id)
{
    if (buf == NULL || cap < POS_DBG_MARK_LEN) {
        return 0;
    }

    buf[0] = POS_DBG_MAGIC_MARK;
    buf[1] = seq;
    buf[2] = mark_id;
    buf[3] = 0;   /* reserved */

    return POS_DBG_MARK_LEN;
}

int16_t pos_dbg_m_to_cm(float m)
{
    float cm;
    float rounded;

    /* Ranges/coordinates arrive over the air; NaN/Inf must never reach the
     * cast below (undefined behaviour), same reasoning as fmt_coord() in
     * uwb_ss_initiator.c. Unlike fmt_coord(), a finite out-of-range value is
     * saturated rather than zeroed -- pos_dbg.h calls this out explicitly
     * because a saturated close-in or far-out reading is still useful signal
     * for the offline tuning this log exists for, whereas silently reporting
     * 0 would look like a real zero range. */
    if (isnan(m) || isinf(m)) {
        return 0;
    }

    cm = m * 100.0f;
    rounded = (cm >= 0.0f) ? (cm + 0.5f) : (cm - 0.5f);

    if (rounded >= (float)INT16_MAX) {
        return INT16_MAX;
    }
    if (rounded <= (float)INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)rounded;
}

/* ============================================================================
 * Part 2 -- Zephyr glue. RAM-only state, off at boot, never persisted: a
 * debug flag surviving a reset would silently cost the power budget this
 * whole design exists to measure (same reasoning CLAUDE.md gives for
 * `pwr adv on` in ble_log.c).
 * ==========================================================================*/

static bool enabled;
static bool quality_enabled;

/* Per-session bookkeeping. Reset on every "dbg on" (a fresh log start) so a
 * stale prev_ms/epoch/anchor-set from an earlier session never leaks into the
 * new one -- see reset_session(). */
static struct {
    bool     have_prev_ms;
    uint32_t prev_ms;
    uint8_t  seq;
    uint8_t  last_seq;      /* most recent SWEEP seq, for `dbg mark` */
    bool     have_set;
    uint8_t  epoch;
    uint8_t  set_aid[POS_MAX_ANCHORS];
    int16_t  set_ax[POS_MAX_ANCHORS];
    int16_t  set_ay[POS_MAX_ANCHORS];
    uint8_t  mark_id;
} session;

static void reset_session(void)
{
    memset(&session, 0, sizeof(session));
}

bool pos_dbg_enabled(void)
{
    return enabled;
}

bool pos_dbg_quality_enabled(void)
{
    return quality_enabled;
}

bool pos_dbg_on_cmd(const char *cmd)
{
    if (strncmp(cmd, "dbg", 3) != 0) {
        return false;   /* not ours; let the caller try the next handler */
    }

    if (strcmp(cmd, "dbg on") == 0) {
        reset_session();
        enabled = true;
        ble_log_send("DBG on\n");
        return true;
    }
    if (strcmp(cmd, "dbg off") == 0) {
        enabled = false;
        ble_log_send("DBG off\n");
        return true;
    }
    if (strcmp(cmd, "dbg q on") == 0) {
        quality_enabled = true;
        ble_log_send("DBG q on\n");
        return true;
    }
    if (strcmp(cmd, "dbg q off") == 0) {
        quality_enabled = false;
        ble_log_send("DBG q off\n");
        return true;
    }
    if (strcmp(cmd, "dbg mark") == 0) {
        uint8_t buf[POS_DBG_MARK_LEN];
        size_t  n;

        session.mark_id++;
        n = pos_dbg_enc_mark(buf, sizeof(buf), session.last_seq, session.mark_id);
        if (n > 0) {
            twr_log_raw(buf, n);
        }
        /* Keep on twr_log() to stay ordered behind the binary MARK record;
         * ble_log_send() would let the text overtake the record. */
        twr_log("MARK %u\n", (unsigned)session.mark_id);
        return true;
    }

    /* Starts with "dbg" but not a form above -- still ours, so reply rather
     * than silently forwarding an operator typo to the cal parser. */
    ble_log_send("DBG ?\n");
    return true;
}

void pos_dbg_sweep(uint32_t now_ms, const uint8_t *aid, const int16_t *ax_cm,
                   const int16_t *ay_cm, const int16_t *r_cm, const uint8_t *q,
                   uint8_t mask, uint8_t tier, bool moving, bool solved,
                   uint16_t res_cm)
{
    uint8_t  buf[POS_DBG_REC_MAX];
    size_t   n;
    uint16_t dt_ms;
    uint8_t  seq;
    bool     changed;
    int      i;

    if (!enabled) {
        return;   /* never blocks, never even builds a record when off */
    }

    /* dt_ms is generated here and is authoritative -- see pos_dbg.h. The
     * first record of a session has no baseline to diff against. */
    if (!session.have_prev_ms) {
        dt_ms = POS_DBG_DT_OVF;
    } else {
        uint32_t diff = now_ms - session.prev_ms;   /* unsigned: wrap-safe */

        dt_ms = (diff >= 0xFFFFu) ? POS_DBG_DT_OVF : (uint16_t)diff;
    }
    session.prev_ms      = now_ms;
    session.have_prev_ms = true;

    seq = session.seq++;
    session.last_seq = seq;

    /* Selected-set change detection against the last SET emitted. A missing
     * aid array reads as every slot empty, so losing the array entirely is
     * treated the same as the anchor set shrinking to nothing. */
    changed = !session.have_set;
    for (i = 0; i < POS_MAX_ANCHORS; i++) {
        uint8_t a = aid   ? aid[i]   : POS_DBG_AID_NONE;
        int16_t x = ax_cm ? ax_cm[i] : 0;
        int16_t y = ay_cm ? ay_cm[i] : 0;

        if (a != session.set_aid[i] || x != session.set_ax[i] ||
            y != session.set_ay[i]) {
            changed = true;
        }
    }

    if (changed) {
        if (session.have_set) {
            session.epoch++;   /* first emission of a session stays at 0 */
        }
        for (i = 0; i < POS_MAX_ANCHORS; i++) {
            session.set_aid[i] = aid   ? aid[i]   : POS_DBG_AID_NONE;
            session.set_ax[i]  = ax_cm ? ax_cm[i] : 0;
            session.set_ay[i]  = ay_cm ? ay_cm[i] : 0;
        }
        session.have_set = true;

        n = pos_dbg_enc_set(buf, sizeof(buf), session.epoch, 0,
                            &session.set_aid[0], &session.set_ax[0],
                            &session.set_ay[0]);
        if (n > 0) {
            twr_log_raw(buf, n);
        }
        n = pos_dbg_enc_set(buf, sizeof(buf), session.epoch, 2,
                            &session.set_aid[2], &session.set_ax[2],
                            &session.set_ay[2]);
        if (n > 0) {
            twr_log_raw(buf, n);
        }
    }

    n = pos_dbg_enc_sweep(buf, sizeof(buf), seq, dt_ms, session.epoch,
                         mask, tier, moving, solved, r_cm,
                         quality_enabled ? q : NULL, res_cm);
    if (n > 0) {
        twr_log_raw(buf, n);
    }
}
