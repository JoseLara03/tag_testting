#include "pos_dbg.h"
#include "pos_solver.h"   /* POS_MAX_ANCHORS */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

/* ============================================================================
 * Stubs for twr_log()/twr_log_raw() (real declarations: uwb_ss_initiator.h,
 * which pos_dbg.c deliberately does not include -- see the comment there).
 * These let the WHOLE module link on the host, glue included, and let the
 * tests below assert on what the glue would actually have transmitted.
 * ==========================================================================*/

#define HIST_MAX     32
#define HIST_ROW_LEN 20   /* POS_DBG_REC_MAX */

static uint8_t hist_buf[HIST_MAX][HIST_ROW_LEN];
static size_t  hist_len[HIST_MAX];
static int     hist_n;

static char last_text[64];
static int  text_calls;

void twr_log_raw(const uint8_t *buf, size_t len)
{
    if (hist_n < HIST_MAX) {
        size_t n = (len > HIST_ROW_LEN) ? HIST_ROW_LEN : len;

        memcpy(hist_buf[hist_n], buf, n);
        hist_len[hist_n] = n;
        hist_n++;
    }
}

void twr_log(const char *fmt, ...)
{
    va_list ap;

    text_calls++;
    va_start(ap, fmt);
    vsnprintf(last_text, sizeof(last_text), fmt, ap);
    va_end(ap);
}

/* Command replies go straight out via ble_log_send() rather than through the
 * queue, because twr_log() drops silently when the queue is busy with sweep
 * traffic -- which is exactly when a capture session is running. Counted
 * separately from twr_log so a test can tell the two paths apart; `dbg mark`
 * deliberately still uses twr_log to stay ordered behind its binary record. */
void ble_log_send(const char *msg)
{
    text_calls++;
    snprintf(last_text, sizeof(last_text), "%s", msg);
}

static void reset_stub(void)
{
    hist_n = 0;
    text_calls = 0;
    last_text[0] = '\0';
}

/* ============================================================================
 * Part 1 -- pure encoders
 * ==========================================================================*/

static void test_sweep_full_record_exact_bytes(void)
{
    /* Every field populated, including both int16 extremes, to nail down the
     * byte layout and little-endianness in one shot. */
    uint8_t  buf[POS_DBG_SWEEP_LEN];
    int16_t  r[POS_MAX_ANCHORS] = { 100, -50, 32767, -32768 };
    uint8_t  q[POS_MAX_ANCHORS] = { 10, 20, 30, 40 };
    uint8_t  expected[POS_DBG_SWEEP_LEN] = {
        0xA5, 0x07, 0xD2, 0x04, 0x03, 0xEF,
        0x64, 0x00, 0xCE, 0xFF, 0xFF, 0x7F, 0x00, 0x80,
        0x0A, 0x14, 0x1E, 0x28,
        0x2B, 0x02,
    };
    size_t n;

    n = pos_dbg_enc_sweep(buf, sizeof(buf), 7, 1234, 3, 0x0F, 2, true, true,
                          r, q, 555);

    CHECK(n == POS_DBG_SWEEP_LEN);
    CHECK(memcmp(buf, expected, POS_DBG_SWEEP_LEN) == 0);
}

static void test_sweep_masked_slots_forced_zero(void)
{
    /* Only anchors 0 and 2 responded (mask 0b0101). Slots 1 and 3 carry
     * garbage in both r_cm and q on purpose -- the encoder must still write
     * zero for them, per pos_dbg.h's "should be written zero" contract.
     * Also exercises a negative range and POS_DBG_RES_NONE (unsolved). */
    uint8_t  buf[POS_DBG_SWEEP_LEN];
    int16_t  r[POS_MAX_ANCHORS] = { 11, 999, -7, 555 };   /* slots 1,3 = garbage */
    uint8_t  q[POS_MAX_ANCHORS] = { 1, 88, 2, 77 };       /* slots 1,3 = garbage */
    uint8_t  expected[POS_DBG_SWEEP_LEN] = {
        0xA5, 0x00, 0x00, 0x00, 0x00, 0x05,
        0x0B, 0x00, 0x00, 0x00, 0xF9, 0xFF, 0x00, 0x00,
        0x01, 0x00, 0x02, 0x00,
        0xFF, 0xFF,
    };
    size_t n;

    n = pos_dbg_enc_sweep(buf, sizeof(buf), 0, 0, 0, 0x05, 0, false, false,
                          r, q, POS_DBG_RES_NONE);

    CHECK(n == POS_DBG_SWEEP_LEN);
    CHECK(memcmp(buf, expected, POS_DBG_SWEEP_LEN) == 0);
}

static void test_sweep_null_q_is_all_zero_quality(void)
{
    uint8_t buf[POS_DBG_SWEEP_LEN];
    int16_t r[POS_MAX_ANCHORS] = { 1, 2, 3, 4 };
    size_t  n;

    n = pos_dbg_enc_sweep(buf, sizeof(buf), 1, 1, 1, 0x0F, 0, false, false,
                          r, NULL, 0);

    CHECK(n == POS_DBG_SWEEP_LEN);
    CHECK(buf[14] == 0 && buf[15] == 0 && buf[16] == 0 && buf[17] == 0);
}

static void test_sweep_null_r_is_all_zero_ranges(void)
{
    /* Not documented as a supported NULL, but the encoder must not crash on
     * it, and treating "no ranges" as all-zero is the only sane behaviour. */
    uint8_t buf[POS_DBG_SWEEP_LEN];
    uint8_t q[POS_MAX_ANCHORS] = { 1, 2, 3, 4 };
    size_t  n;

    n = pos_dbg_enc_sweep(buf, sizeof(buf), 1, 1, 1, 0x0F, 0, false, false,
                          NULL, q, 0);

    CHECK(n == POS_DBG_SWEEP_LEN);
    for (int i = 0; i < POS_MAX_ANCHORS; i++) {
        CHECK(buf[6 + 2 * i] == 0 && buf[6 + 2 * i + 1] == 0);
    }
}

static void test_sweep_flags_bit_packing_round_trips(void)
{
    /* Isolate the mask nibble: with tier/moving/solved all at their zero
     * value, flags must equal the mask exactly. */
    static const uint8_t masks[] = { 0x0, 0x1, 0x2, 0x4, 0x8, 0x3, 0x5, 0x9, 0xF };

    for (size_t i = 0; i < sizeof(masks) / sizeof(masks[0]); i++) {
        uint8_t buf[POS_DBG_SWEEP_LEN];
        int16_t r[POS_MAX_ANCHORS] = { 0, 0, 0, 0 };

        pos_dbg_enc_sweep(buf, sizeof(buf), 0, 0, 0, masks[i], 0, false, false,
                          r, NULL, 0);
        CHECK(buf[5] == masks[i]);
    }

    /* Now isolate tier (2 bits) x moving x solved, mask fixed at all-set so
     * the low nibble is a known constant (0x0F) throughout. */
    for (unsigned tier = 0; tier <= 3; tier++) {
        for (int moving = 0; moving <= 1; moving++) {
            for (int solved = 0; solved <= 1; solved++) {
                uint8_t buf[POS_DBG_SWEEP_LEN];
                int16_t r[POS_MAX_ANCHORS] = { 0, 0, 0, 0 };
                uint8_t expected = (uint8_t)(0x0Fu | ((tier & 0x03u) << 4) |
                                             (moving ? (1u << 6) : 0u) |
                                             (solved ? (1u << 7) : 0u));

                pos_dbg_enc_sweep(buf, sizeof(buf), 0, 0, 0, 0x0F,
                                  (uint8_t)tier, moving != 0, solved != 0,
                                  r, NULL, 0);
                CHECK(buf[5] == expected);
            }
        }
    }
}

static void test_sweep_null_buf_and_undersized_cap(void)
{
    uint8_t sentinel[POS_DBG_SWEEP_LEN];
    int16_t r[POS_MAX_ANCHORS] = { 1, 2, 3, 4 };
    size_t  n;

    memset(sentinel, 0xCC, sizeof(sentinel));

    n = pos_dbg_enc_sweep(NULL, POS_DBG_SWEEP_LEN, 0, 0, 0, 0x0F, 0, false,
                          false, r, NULL, 0);
    CHECK(n == 0);

    n = pos_dbg_enc_sweep(sentinel, POS_DBG_SWEEP_LEN - 1, 0, 0, 0, 0x0F, 0,
                          false, false, r, NULL, 0);
    CHECK(n == 0);
    for (size_t i = 0; i < sizeof(sentinel); i++) {
        CHECK(sentinel[i] == 0xCC);   /* undersized cap must write nothing */
    }
}

static void test_set_exact_bytes_and_aid_none(void)
{
    uint8_t buf[POS_DBG_SET_LEN];
    uint8_t aid[POS_DBG_SET_SLOTS] = { 3, POS_DBG_AID_NONE };
    int16_t x[POS_DBG_SET_SLOTS]   = { 150, 0 };
    int16_t y[POS_DBG_SET_SLOTS]   = { -200, 0 };
    uint8_t expected[POS_DBG_SET_LEN] = {
        0xA6, 0x05, 0x00,
        0x03, 0x96, 0x00, 0x38, 0xFF,
        0xFF, 0x00, 0x00, 0x00, 0x00,
    };
    size_t n;

    n = pos_dbg_enc_set(buf, sizeof(buf), 5, 0, aid, x, y);

    CHECK(n == POS_DBG_SET_LEN);
    CHECK(memcmp(buf, expected, POS_DBG_SET_LEN) == 0);
    CHECK(buf[8] == POS_DBG_AID_NONE);
}

static void test_set_base2_offset_in_wire_not_reindexed(void)
{
    /* base=2 changes only the `base` byte written to the wire -- the caller's
     * aid/x/y arrays are read from index 0, not offset by base again. */
    uint8_t buf[POS_DBG_SET_LEN];
    uint8_t aid[POS_DBG_SET_SLOTS] = { 7, 8 };
    int16_t x[POS_DBG_SET_SLOTS]   = { 1, 2 };
    int16_t y[POS_DBG_SET_SLOTS]   = { 3, 4 };
    size_t  n;

    n = pos_dbg_enc_set(buf, sizeof(buf), 9, 2, aid, x, y);

    CHECK(n == POS_DBG_SET_LEN);
    CHECK(buf[1] == 9);      /* epoch */
    CHECK(buf[2] == 2);      /* base */
    CHECK(buf[3] == 7);      /* aid[0], not aid[2] -- there is no aid[2] here */
    CHECK(buf[8] == 8);      /* aid[1] */
}

static void test_set_bad_base_rejected(void)
{
    uint8_t buf[POS_DBG_SET_LEN];
    uint8_t aid[POS_DBG_SET_SLOTS] = { 1, 2 };
    int16_t x[POS_DBG_SET_SLOTS]   = { 0, 0 };
    int16_t y[POS_DBG_SET_SLOTS]   = { 0, 0 };

    CHECK(pos_dbg_enc_set(buf, sizeof(buf), 0, 1, aid, x, y) == 0);
    CHECK(pos_dbg_enc_set(buf, sizeof(buf), 0, 3, aid, x, y) == 0);
}

static void test_set_null_buf_and_undersized_cap(void)
{
    uint8_t sentinel[POS_DBG_SET_LEN];
    uint8_t aid[POS_DBG_SET_SLOTS] = { 1, 2 };
    int16_t x[POS_DBG_SET_SLOTS]   = { 0, 0 };
    int16_t y[POS_DBG_SET_SLOTS]   = { 0, 0 };
    size_t  n;

    memset(sentinel, 0xCC, sizeof(sentinel));

    n = pos_dbg_enc_set(NULL, POS_DBG_SET_LEN, 0, 0, aid, x, y);
    CHECK(n == 0);

    n = pos_dbg_enc_set(sentinel, POS_DBG_SET_LEN - 1, 0, 0, aid, x, y);
    CHECK(n == 0);
    for (size_t i = 0; i < sizeof(sentinel); i++) {
        CHECK(sentinel[i] == 0xCC);
    }
}

static void test_mark_exact_bytes(void)
{
    uint8_t buf[POS_DBG_MARK_LEN];
    uint8_t expected[POS_DBG_MARK_LEN] = { 0xA7, 42, 9, 0 };
    size_t  n;

    n = pos_dbg_enc_mark(buf, sizeof(buf), 42, 9);

    CHECK(n == POS_DBG_MARK_LEN);
    CHECK(memcmp(buf, expected, POS_DBG_MARK_LEN) == 0);
}

static void test_mark_null_buf_and_undersized_cap(void)
{
    uint8_t sentinel[POS_DBG_MARK_LEN];
    size_t  n;

    memset(sentinel, 0xCC, sizeof(sentinel));

    n = pos_dbg_enc_mark(NULL, POS_DBG_MARK_LEN, 1, 1);
    CHECK(n == 0);

    n = pos_dbg_enc_mark(sentinel, POS_DBG_MARK_LEN - 1, 1, 1);
    CHECK(n == 0);
    for (size_t i = 0; i < sizeof(sentinel); i++) {
        CHECK(sentinel[i] == 0xCC);
    }
}

static void test_m_to_cm_normal_rounding_and_sign(void)
{
    CHECK(pos_dbg_m_to_cm(0.0f) == 0);
    CHECK(pos_dbg_m_to_cm(1.236f) == 124);     /* rounds, not truncates */
    CHECK(pos_dbg_m_to_cm(-0.07f) == -7);      /* negative survives as negative */
    CHECK(pos_dbg_m_to_cm(3.0f) == 300);
}

static void test_m_to_cm_saturates_at_int16_range(void)
{
    CHECK(pos_dbg_m_to_cm(400.0f) == INT16_MAX);
    CHECK(pos_dbg_m_to_cm(-400.0f) == INT16_MIN);
    CHECK(pos_dbg_m_to_cm(1.0e30f) == INT16_MAX);
    CHECK(pos_dbg_m_to_cm(-1.0e30f) == INT16_MIN);
}

static void test_m_to_cm_rejects_nan_and_inf(void)
{
    CHECK(pos_dbg_m_to_cm((float)NAN) == 0);
    CHECK(pos_dbg_m_to_cm((float)INFINITY) == 0);
    CHECK(pos_dbg_m_to_cm((float)-INFINITY) == 0);
}

/* ============================================================================
 * Part 2 -- glue, exercised through the twr_log/twr_log_raw stubs above.
 * ==========================================================================*/

static void test_cmd_ignores_non_dbg_commands(void)
{
    reset_stub();
    CHECK(pos_dbg_on_cmd("cal status") == false);
    CHECK(pos_dbg_on_cmd("pwr rx") == false);
    CHECK(text_calls == 0);   /* not consumed => no reply of its own */
}

static void test_cmd_on_off_and_quality_toggles(void)
{
    reset_stub();
    CHECK(pos_dbg_on_cmd("dbg on") == true);
    CHECK(pos_dbg_enabled() == true);
    CHECK(strcmp(last_text, "DBG on\n") == 0);

    reset_stub();
    CHECK(pos_dbg_on_cmd("dbg q on") == true);
    CHECK(pos_dbg_quality_enabled() == true);
    CHECK(strcmp(last_text, "DBG q on\n") == 0);

    reset_stub();
    CHECK(pos_dbg_on_cmd("dbg nonsense") == true);   /* still ours, unknown form */
    CHECK(strcmp(last_text, "DBG ?\n") == 0);

    reset_stub();
    CHECK(pos_dbg_on_cmd("dbg q off") == true);
    CHECK(pos_dbg_quality_enabled() == false);

    reset_stub();
    CHECK(pos_dbg_on_cmd("dbg off") == true);
    CHECK(pos_dbg_enabled() == false);
}

static void test_sweep_is_noop_when_disabled(void)
{
    uint8_t aid[POS_MAX_ANCHORS] = { 1, 2, 3, 4 };
    int16_t ax[POS_MAX_ANCHORS]  = { 0, 0, 0, 0 };
    int16_t ay[POS_MAX_ANCHORS]  = { 0, 0, 0, 0 };
    int16_t r[POS_MAX_ANCHORS]   = { 0, 0, 0, 0 };
    uint8_t q[POS_MAX_ANCHORS]   = { 0, 0, 0, 0 };

    pos_dbg_on_cmd("dbg off");
    reset_stub();

    pos_dbg_sweep(100, aid, ax, ay, r, q, 0x0F, 0, false, false, 0);

    CHECK(hist_n == 0);
}

/* Full session flow: log start (SET x2 + SWEEP), an unchanged sweep (SWEEP
 * only), a coordinate change (epoch bump, SET x2 + SWEEP again), the quality
 * gate, and `dbg mark`. Each step's expected bytes are hand-computed. */
static void test_sweep_session_flow(void)
{
    uint8_t aid[POS_MAX_ANCHORS] = { 1, 2, 3, 4 };
    int16_t ax[POS_MAX_ANCHORS]  = { 100, 200, 300, 400 };
    int16_t ay[POS_MAX_ANCHORS]  = { 10, 20, 30, 40 };
    int16_t r[POS_MAX_ANCHORS]   = { 150, 250, 350, 450 };
    uint8_t q[POS_MAX_ANCHORS]   = { 1, 2, 3, 4 };
    uint8_t expected_sweep_common[14] = {
        /* r0..r3 for the fixed r[] above, mask 0x0F, flags for
         * tier=2, moving=true, solved=true -> 0x0F|0x20|0x40|0x80 = 0xEF */
        0x96, 0x00, 0xFA, 0x00, 0x5E, 0x01, 0xC2, 0x01,   /* r0..r3 */
        0, 0, 0, 0,                                        /* q, off by default */
        0xE7, 0x03,                                        /* res_cm = 999 */
    };

    CHECK(pos_dbg_on_cmd("dbg q off") == true);   /* known baseline */
    CHECK(pos_dbg_on_cmd("dbg on") == true);      /* fresh session: seq/epoch/set reset */

    /* --- first sweep: log start, forces SET x2 then SWEEP --- */
    reset_stub();
    pos_dbg_sweep(1000, aid, ax, ay, r, q, 0x0F, 2, true, true, 999);

    CHECK(hist_n == 3);
    CHECK(hist_len[0] == POS_DBG_SET_LEN && hist_buf[0][0] == POS_DBG_MAGIC_SET);
    CHECK(hist_buf[0][1] == 0 && hist_buf[0][2] == 0);            /* epoch 0, base 0 */
    CHECK(hist_buf[0][3] == 1 && hist_buf[0][4] == 0x64 && hist_buf[0][5] == 0x00);
    CHECK(hist_buf[0][6] == 0x0A && hist_buf[0][7] == 0x00);
    CHECK(hist_buf[0][8] == 2 && hist_buf[0][9] == 0xC8 && hist_buf[0][10] == 0x00);
    CHECK(hist_buf[0][11] == 0x14 && hist_buf[0][12] == 0x00);

    CHECK(hist_len[1] == POS_DBG_SET_LEN && hist_buf[1][0] == POS_DBG_MAGIC_SET);
    CHECK(hist_buf[1][1] == 0 && hist_buf[1][2] == 2);            /* epoch 0, base 2 */
    CHECK(hist_buf[1][3] == 3 && hist_buf[1][4] == 0x2C && hist_buf[1][5] == 0x01);
    CHECK(hist_buf[1][6] == 0x1E && hist_buf[1][7] == 0x00);
    CHECK(hist_buf[1][8] == 4 && hist_buf[1][9] == 0x90 && hist_buf[1][10] == 0x01);
    CHECK(hist_buf[1][11] == 0x28 && hist_buf[1][12] == 0x00);

    CHECK(hist_len[2] == POS_DBG_SWEEP_LEN && hist_buf[2][0] == POS_DBG_MAGIC_SWEEP);
    CHECK(hist_buf[2][1] == 0);                        /* seq starts at 0 */
    CHECK(hist_buf[2][2] == 0xFF && hist_buf[2][3] == 0xFF);   /* no baseline yet */
    CHECK(hist_buf[2][4] == 0);                        /* epoch 0 */
    CHECK(hist_buf[2][5] == 0xEF);
    CHECK(memcmp(&hist_buf[2][6], expected_sweep_common, sizeof(expected_sweep_common)) == 0);

    /* --- second sweep, same anchor set: SWEEP only, dt_ms real, epoch held --- */
    reset_stub();
    pos_dbg_sweep(1050, aid, ax, ay, r, q, 0x0F, 2, true, true, 999);

    CHECK(hist_n == 1);
    CHECK(hist_buf[0][0] == POS_DBG_MAGIC_SWEEP);
    CHECK(hist_buf[0][1] == 1);                        /* seq advances */
    CHECK(hist_buf[0][2] == 50 && hist_buf[0][3] == 0);/* dt_ms = 1050-1000 */
    CHECK(hist_buf[0][4] == 0);                        /* epoch unchanged */

    /* --- third sweep, one coordinate changes: epoch bumps, SET x2 + SWEEP --- */
    ax[0] = 999;
    reset_stub();
    pos_dbg_sweep(1100, aid, ax, ay, r, q, 0x0F, 2, true, true, 999);

    CHECK(hist_n == 3);
    CHECK(hist_buf[0][1] == 1 && hist_buf[0][2] == 0);   /* SET base0, new epoch 1 */
    CHECK(hist_buf[0][4] == 0xE7 && hist_buf[0][5] == 0x03);   /* x[0] = 999 */
    CHECK(hist_buf[1][1] == 1 && hist_buf[1][2] == 2);   /* SET base2, new epoch 1 */
    CHECK(hist_buf[2][1] == 2);                          /* seq advances again */
    CHECK(hist_buf[2][4] == 1);                          /* SWEEP carries new epoch */

    /* --- quality gate: off by default (checked above via all-zero q bytes);
     * turning it on must make the actual q[] values appear --- */
    CHECK(pos_dbg_on_cmd("dbg q on") == true);
    reset_stub();
    pos_dbg_sweep(1150, aid, ax, ay, r, q, 0x0F, 2, true, true, 999);

    CHECK(hist_n == 1);   /* anchor set unchanged again */
    CHECK(hist_buf[0][14] == 1 && hist_buf[0][15] == 2 &&
          hist_buf[0][16] == 3 && hist_buf[0][17] == 4);

    /* --- dbg mark: names the most recent SWEEP seq, mark_id increments --- */
    reset_stub();
    CHECK(pos_dbg_on_cmd("dbg mark") == true);
    CHECK(hist_n == 1);
    CHECK(hist_buf[0][0] == POS_DBG_MAGIC_MARK);
    CHECK(hist_buf[0][1] == 3);      /* seq of the sweep just above */
    CHECK(hist_buf[0][2] == 1);      /* first mark since "dbg on" reset the session */
    CHECK(hist_buf[0][3] == 0);
    CHECK(strcmp(last_text, "MARK 1\n") == 0);

    reset_stub();
    pos_dbg_on_cmd("dbg mark");
    CHECK(hist_buf[0][2] == 2);      /* mark_id increments */

    pos_dbg_on_cmd("dbg off");
    pos_dbg_on_cmd("dbg q off");
}

int main(void)
{
    test_sweep_full_record_exact_bytes();
    test_sweep_masked_slots_forced_zero();
    test_sweep_null_q_is_all_zero_quality();
    test_sweep_null_r_is_all_zero_ranges();
    test_sweep_flags_bit_packing_round_trips();
    test_sweep_null_buf_and_undersized_cap();

    test_set_exact_bytes_and_aid_none();
    test_set_base2_offset_in_wire_not_reindexed();
    test_set_bad_base_rejected();
    test_set_null_buf_and_undersized_cap();

    test_mark_exact_bytes();
    test_mark_null_buf_and_undersized_cap();

    test_m_to_cm_normal_rounding_and_sign();
    test_m_to_cm_saturates_at_int16_range();
    test_m_to_cm_rejects_nan_and_inf();

    test_cmd_ignores_non_dbg_commands();
    test_cmd_on_off_and_quality_toggles();
    test_sweep_is_noop_when_disabled();
    test_sweep_session_flow();

    if (g_fail) { printf("%d FAILURES\n", g_fail); return 1; }
    printf("ALL TESTS PASSED\n");
    return 0;
}
