#include "cal_run.h"
#include "cal.h"
#include "cal_math.h"
#include "phy_config.h"   /* CONFIG_OPTION */
#include "storage.h"
#include "cal_led.h"

#include <string.h>
#include "uwb_wave_frame.h"
#include "deca_device_api.h"

#include <zephyr/kernel.h>

#include "cal_run_math.h"
#include "uwb_radio_owner.h"
#include "uwb_ss_initiator.h"   /* twr_log() */
#include "ble_log.h"

#include <stdio.h>
#include <stdarg.h>

/* Written by the "calrun" thread (cal_run_verdict()/cal_set_last_result())
 * as it produces verdicts, read by the BT RX thread (cal_get_last_result(),
 * from cal_run_on_rx() servicing `cal last`). No lock: the worst a race can
 * produce is a torn diagnostic line, which is not worth a lock for a
 * human-read status string. */
#define CAL_LAST_LEN 20
static char cal_last[CAL_LAST_LEN] = "CAL none\n";

void cal_set_last_result(const char *s)
{
    strncpy(cal_last, s, sizeof(cal_last) - 1);
    cal_last[sizeof(cal_last) - 1] = '\0';
    cal_led_on_result(cal_last);
}

const char *cal_get_last_result(void)
{
    return cal_last;
}

int cal_clear(void)
{
    int rc = storage_delete(CAL_NVS_ID);

    if (rc == 0) {
        cal_internal_invalidate();
    }
    return rc;
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
    cal_internal_activate(&r);
    return 0;
}

/* ---- polled SS-TWR exchange (no interrupts) --------------------------------
 *
 * Extends cal_diag.c's do_probe() pattern instead of reusing
 * uwb_ss_initiator.c's interrupt-driven do_one_range()/wait_event(): this
 * calibration engine never enables the DW3000 IRQ, so dwt_isr() never runs
 * while it owns the radio -- see
 * docs/superpowers/specs/2026-08-20-cal-image-rewrite-design.md for why.
 */

#define RX_BUF_LEN       32
#define SPEED_OF_LIGHT   299702547.0   /* m/s, matching uwb_ss_initiator.c */
#define CAL_RANGE_POLL_TIMEOUT_MS  20U

static uint8_t cal_run_frame_seq_nb;
static uint8_t cal_run_rx_buf[RX_BUF_LEN];

static uint32_t cal_run_get_ts_4b(const uint8_t *b)
{
    uint32_t ts = 0;

    for (int i = 0; i < 4; i++) {
        ts |= (uint32_t)b[i] << (i * 8);
    }
    return ts;
}

/* Bounded wait for ANY bit in mask, polling SYS_STATUS_LO -- mirrors
 * cal_diag.c's wait_any_sysstatus_lo(), duplicated rather than shared
 * because cal_diag.c is a sibling cal-image-only file with no shared header
 * for this helper; both are small enough that this is not worth a new
 * shared module. */
static uint32_t cal_run_wait_any_sysstatus_lo(uint32_t mask, uint32_t timeout_ms)
{
    uint32_t t_end = k_uptime_get_32() + timeout_ms;

    for (;;) {
        uint32_t status = dwt_readsysstatuslo();

        if (status & mask) {
            return status & mask;
        }
        if ((int32_t)(t_end - k_uptime_get_32()) <= 0) {
            return 0;
        }
    }
}

/* One polled SS-TWR exchange using the non-addressed calibration WAVE frame
 * (see uwb_wave_frame.h) -- wire-compatible with a stock Qorvo
 * ss_twr_responder. Writes the measured distance in millimetres to *out_mm
 * and returns true on a valid response. Returns false on timeout, RX error,
 * an oversized/undersized frame, or a header mismatch. Caller must already
 * hold the radio with interrupts disabled for the DW3000 (see
 * cal_run_claim_radio() in Task 4) -- this function never touches
 * interrupt state itself. */
static bool cal_range_poll(int32_t *out_mm)
{
    uint8_t poll[]     = UWB_WAVE_POLL_INIT;
    uint8_t resp_ref[] = UWB_WAVE_RESP_INIT;

    poll[ALL_MSG_SN_IDX] = cal_run_frame_seq_nb;
    dwt_writetxdata(sizeof(poll), poll, 0);
    dwt_writetxfctrl(sizeof(poll) + FCS_LEN, 0, 1);
    cal_run_frame_seq_nb++;

    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        return false;
    }

    uint32_t got = cal_run_wait_any_sysstatus_lo(
        DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO,
        CAL_RANGE_POLL_TIMEOUT_MS);

    if (got == 0) {
        return false;   /* software timeout -- no hardware status bit latched */
    }
    if (!(got & DWT_INT_RXFCG_BIT_MASK)) {
        /* RX error or the hardware RX timeout armed by cal_run_claim_radio()
         * (dwt_setrxtimeout()/dwt_setpreambledetecttimeout()) -- either can
         * latch a status bit, so clear both rather than assuming which one
         * fired. */
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
        return false;   /* RX error or RX timeout */
    }
    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

    uint16_t flen = dwt_getframelength();

    if (flen < ALL_MSG_COMMON_LEN + FCS_LEN) {
        /* Too short to be a real response: reading it would pair stale
         * bytes past the received length with a header match that only
         * looks valid -- the same stale-buffer hazard the oversized-frame
         * check below guards against, mirroring cal_diag.c's do_probe(). */
        return false;
    }
    if (flen > RX_BUF_LEN) {
        /* Do not read: cal_run_rx_buf still holds the previous exchange, and
         * reading a mismatched length here would pair stale timestamps with
         * a fresh header match -- the exact stale-buffer hazard
         * do_one_range() was fixed for. */
        return false;
    }
    dwt_readrxdata(cal_run_rx_buf, flen, 0);
    cal_run_rx_buf[ALL_MSG_SN_IDX] = 0;

    if (memcmp(cal_run_rx_buf, resp_ref, ALL_MSG_COMMON_LEN) != 0) {
        return false;
    }

    uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
    uint32_t resp_rx_ts = dwt_readrxtimestamplo32();
    double   clock_offset_ratio =
        ((double)dwt_readclockoffset()) / (uint32_t)(1 << 26);
    uint32_t poll_rx_ts = cal_run_get_ts_4b(&cal_run_rx_buf[UWB_WAVE_RESP_POLL_RX_TS_IDX]);
    uint32_t resp_tx_ts = cal_run_get_ts_4b(&cal_run_rx_buf[UWB_WAVE_RESP_RESP_TX_TS_IDX]);

    int32_t rtd_init = (int32_t)(resp_rx_ts - poll_tx_ts);
    int32_t rtd_resp = (int32_t)(resp_tx_ts - poll_rx_ts);

    double tof = ((rtd_init - rtd_resp * (1 - clock_offset_ratio)) / 2.0)
                 * DWT_TIME_UNITS;
    *out_mm = (int32_t)(tof * SPEED_OF_LIGHT * 1000.0);
    return true;
}

/* ---- radio ownership + PHY setup for one calibration run ------------------- */

#define POLL_TX_TO_RESP_RX_DLY_UUS  1000U
#define RESP_RX_TIMEOUT_UUS         2000U
#define PRE_TIMEOUT                  128U
#define CAL_RUN_RADIO_WAIT          K_SECONDS(2)

/* Wall-clock backstop for one sampling iteration, independent of
 * cal_range_poll()'s own per-exchange timeout -- cheap insurance against an
 * unexpected stall, same reasoning uwb_ss_initiator.c's old
 * CAL_ITER_BUDGET_MS used. */
#define CAL_ITER_BUDGET_MS  5000U

static bool cal_run_claim_radio(void)
{
    if (!uwb_radio_request(CAL_RUN_RADIO_WAIT)) {
        return false;
    }
    dwt_forcetrxoff();
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    dwt_setinterrupt(0xFFFFFFFFU, 0xFFFFFFFFU, DWT_DISABLE_INT);
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);
    return true;
}

/* Restores all five PHY-state items per the uwb_radio_owner.h handover
 * contract before releasing, mirroring cal_diag.c's release_radio(). */
static void cal_run_release_radio(void)
{
    uint16_t tx, rx;

    dwt_forcetrxoff();
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);
    if (cal_is_valid()) {
        cal_get_ant_dly(&tx, &rx);
    } else {
        tx = TX_ANT_DLY;
        rx = RX_ANT_DLY;
    }
    dwt_settxantennadelay(tx);
    dwt_setrxantennadelay(rx);

    uwb_radio_release();
}

static void cal_run_apply_total_dly(uint16_t total, uint16_t *tx, uint16_t *rx)
{
    cal_split_dly(total, tx, rx);
    dwt_settxantennadelay(*tx);
    dwt_setrxantennadelay(*rx);
}

static uint16_t cal_run_active_total_seed(void)
{
    if (cal_is_valid()) {
        uint16_t tx, rx;

        cal_get_ant_dly(&tx, &rx);
        return (uint16_t)(tx + rx);
    }
    return (uint16_t)(TX_ANT_DLY + RX_ANT_DLY);
}

/* Emit a calibration verdict: enqueue it for BLE and latch it for `cal last`
 * (see cal_run.h's comment on cal_set_last_result() for why both). */
static void cal_run_verdict(const char *fmt, ...)
{
    char    line[24];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    cal_set_last_result(line);
    twr_log("%s", line);
}

/* samples[] is CAL_MAX_SAMPLES long and one iteration fills at most one
 * entry per pass, so the per-iteration count must never exceed it. */
BUILD_ASSERT(CAL_SAMPLES_PER_ITER <= CAL_MAX_SAMPLES,
             "cal sample budget exceeds the samples[] array");

/* Iterative auto-solve: collect CAL_SAMPLES_PER_ITER polled ranges per
 * iteration, reject outliers, correct the combined antenna delay toward
 * ref_mm, repeat until the residual is within CAL_ACCEPT_MM or CAL_MAX_ITERS
 * is exhausted. Stores to NVS on success. Same algorithm as the old
 * run_calibration_locked() -- only the exchange primitive (polling, not
 * interrupt-driven) and the decision logic's location (cal_run_math.c,
 * host-tested) changed. */
static void cal_run_execute(uint32_t ref_mm)
{
    static int32_t samples[CAL_MAX_SAMPLES];

    uint16_t tx, rx;
    uint16_t total = cal_run_active_total_seed();

    cal_run_apply_total_dly(total, &tx, &rx);

    for (uint32_t it = 0; it < CAL_MAX_ITERS; it++) {
        size_t   got = 0;
        uint32_t t_end = k_uptime_get_32() + CAL_ITER_BUDGET_MS;

        for (uint32_t i = 0; i < CAL_SAMPLES_PER_ITER; i++) {
            if ((int32_t)(t_end - k_uptime_get_32()) <= 0) {
                break;
            }
            int32_t mm;

            if (cal_range_poll(&mm) && got < CAL_MAX_SAMPLES) {
                samples[got++] = mm;
            }
            k_sleep(K_MSEC(5));
        }

        int32_t  err;
        size_t   kept;
        uint16_t new_total;
        enum cal_run_verdict v = cal_run_iteration_result(
            samples, got, CAL_SAMPLES_PER_ITER, (int32_t)ref_mm, total,
            &err, &kept, &new_total);
        /* kept is captured by the shared decision function but not
         * currently logged -- the per-iteration IT%u D=... K=... line that
         * used to consume it was deliberately removed as a debugging-only
         * diagnostic earlier in this rewrite. */
        (void)kept;

        if (v == CAL_RUN_NO_RESP) {
            cal_run_verdict("CAL FAIL no-resp\n");
            return;
        }

        twr_log("CAL it%u e=%dmm\n", it + 1, err);

        if (v == CAL_RUN_CONVERGED) {
            /* Latch OK before the NVS write -- the write is the single
             * most likely point for the BLE link to drop. */
            cal_run_verdict("CAL OK %u/%u\n", tx, rx);
            if (cal_store(tx, rx, ref_mm, (uint16_t)((err < 0) ? -err : err)) != 0) {
                cal_run_verdict("CAL FAIL nvs\n");
            }
            return;
        }

        total = new_total;
        cal_run_apply_total_dly(total, &tx, &rx);
    }
    cal_run_verdict("CAL FAIL res\n");
}

/* Set before cal_run_run() starts work and cleared when it returns. cal_run_q
 * has depth 1 and the thread dequeues almost instantly, so a second `cal
 * <mm>` arriving while a run is in progress usually finds an empty queue --
 * this flag is what actually rejects it, checked in cal_run_on_rx() before
 * enqueueing. */
static volatile bool cal_run_busy;

static void cal_run_run(uint32_t ref_mm)
{
    if (!cal_run_claim_radio()) {
        cal_run_verdict("CAL FAIL busy\n");
        return;
    }

    cal_run_execute(ref_mm);

    cal_run_release_radio();
}

/* ---- dedicated thread + command parsing ------------------------------------ */

struct cal_run_req {
    uint32_t ref_mm;
};

K_MSGQ_DEFINE(cal_run_q, sizeof(struct cal_run_req), 1, 4);

#define CAL_RUN_PRIO   6
#define CAL_RUN_STACK  4096   /* cal_filtered_mean() alone holds ~1 KB of
                                * locals (two int32_t[128] arrays) -- the
                                * deepest path in the cal image. */

K_THREAD_STACK_DEFINE(cal_run_stack, CAL_RUN_STACK);
static struct k_thread cal_run_tid;

static void cal_run_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    struct cal_run_req req;

    while (1) {
        k_msgq_get(&cal_run_q, &req, K_FOREVER);
        cal_run_busy = true;
        cal_run_run(req.ref_mm);
        cal_run_busy = false;
    }
}

/* Minimal unsigned-decimal parser, mirroring cal_diag.c's parse_u32(). */
static bool cal_run_parse_u32(const char *s, uint32_t *out)
{
    if (*s < '0' || *s > '9') {
        return false;
    }
    uint32_t v = 0;

    while (*s >= '0' && *s <= '9') {
        v = v * 10u + (uint32_t)(*s - '0');
        s++;
    }
    *out = v;
    return true;
}

void cal_run_on_rx(const uint8_t *data, uint16_t len)
{
    char buf[24];
    uint16_t n = (len < sizeof(buf) - 1) ? len : (uint16_t)(sizeof(buf) - 1);

    memcpy(buf, data, n);
    buf[n] = '\0';
    while (n > 0 && (buf[n - 1] == '\r' || buf[n - 1] == '\n')) {
        buf[--n] = '\0';
    }

    if (strcmp(buf, "cal clear") == 0) {
        ble_log_send(cal_clear() == 0 ? "CAL cleared\n" : "CAL FAIL nvs\n");
        return;
    }
    if (strcmp(buf, "cal last") == 0) {
        ble_log_send(cal_get_last_result());
        return;
    }
    if (strcmp(buf, "cal selftest") == 0) {
        char msg[20];

        (void)snprintf(msg, sizeof(msg), "SELFTEST %d\n",
                       cal_math_selftest() + cal_run_math_selftest());
        ble_log_send(msg);
        return;
    }
    if (strncmp(buf, "cal ", 4) == 0) {
        uint32_t mm;

        if (cal_run_parse_u32(buf + 4, &mm) && mm > 0) {
            struct cal_run_req req = { .ref_mm = mm };

            if (cal_run_busy) {
                ble_log_send("CAL FAIL busy\n");
                return;
            }
            if (k_msgq_put(&cal_run_q, &req, K_NO_WAIT) == 0) {
                cal_set_last_result("CAL running\n");
                ble_log_send("CAL start\n");
            } else {
                ble_log_send("CAL FAIL busy\n");
            }
            return;
        }
    }
    ble_log_send("CAL ERR cal <mm>\n");
}

void cal_run_start(void)
{
    k_thread_create(&cal_run_tid, cal_run_stack,
                    K_THREAD_STACK_SIZEOF(cal_run_stack),
                    cal_run_fn, NULL, NULL, NULL,
                    CAL_RUN_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&cal_run_tid, "calrun");
}
