/*
 * SS-TWR Initiator — Zephyr RTOS adaptation for nRF52833 + DW3000
 *
 * Interrupt-driven (no polling): DW3000 IRQ fires -> callback gives semaphore ->
 * high-priority ranging thread wakes up and processes the event. Computes
 * distance with clock-offset correction and enqueues "D:x.xxm" results in
 * twr_msgq, drained by a dedicated BLE sender thread so the timing-critical
 * path never blocks on BLE TX.
 *
 * PHY (dwt_configure / dwt_configuretxrf / dwt_setlnapamode) was already
 * applied by uwb_init() and persists in the DW3000.
 */

#include "uwb_ss_initiator.h"   /* also pulls in uwb_net_runner.h for uwb_net_set_tier */
#include "port.h"
#include "deca_device_api.h"
#include "ble_log.h"
#include "phy_config.h"
#include "cal.h"
#include "cal_math.h"
#include "pos_solver.h"
#include "uwb_radio_owner.h"
#include "uwb_frame_802_15_4z.h"
#include "batt.h"

#include <zephyr/kernel.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* Antenna delay (TX_ANT_DLY / RX_ANT_DLY) comes from phy_config.h — the
 * calibration knob. Edit it there and rebuild to re-calibrate. */

/* ---- Timing (seeded from the working DS initiator for CONFIG_OPTION_07) ----- */
#define UUS_TO_DWT_TIME             65536UL
#define POLL_TX_TO_RESP_RX_DLY_UUS  1000U   /* RX turns on this long after poll TX */
#define RESP_RX_TIMEOUT_UUS         2000U   /* covers full response frame air time */
#define PRE_TIMEOUT                  128U
/* RNG_FAST_MS / RNG_SLOW_MS removed — cadence now owned by uwb_net_runner. */

/* Settle time between anchors within one cycle (radio turnaround margin). */
#define INTER_ANCHOR_DELAY_MS  10U

/* Calibration procedure parameters. */
#define CAL_SAMPLES_PER_ITER  100U   /* ranges averaged per iteration */
#define CAL_MAX_ITERS         4U     /* give up after this many corrections */
#define CAL_ACCEPT_MM         15     /* residual error considered converged */

/* SPEED_OF_LIGHT is a Qorvo shared_defines macro not present in this project's
 * driver headers; define it locally (m/s, as used by the Qorvo examples). */
#define SPEED_OF_LIGHT  299702547.0

/* ---- Frames (tag convention: no FCS placeholder; +FCS_LEN in writetxfctrl) -- */
#define ALL_MSG_COMMON_LEN       10
#define ALL_MSG_SN_IDX            2
#define RESP_MSG_POLL_RX_TS_IDX  10
#define RESP_MSG_RESP_TX_TS_IDX  14
#define RX_BUF_LEN               32

static uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0 };
static uint8_t rx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1 };

/* ---- Positioning frames (addressed; anchor self-reports its (x,y)) --------
 * Poll : [hdr 0..9][anchor_id @10]
 * Resp : [hdr 0..9][anchor_id @10][poll_rx_ts @11..14][resp_tx_ts @15..18]
 *        [x f32 @19..22][y f32 @23..26]
 * Distinct from the non-addressed calibration frames above. */
#define POS_ANCHOR_ID_IDX        10
#define POS_POLL_RX_TS_IDX       11
#define POS_RESP_TX_TS_IDX       15
#define POS_ANCHOR_X_IDX         19
#define POS_ANCHOR_Y_IDX         23
#define POS_RESP_LEN_MIN  (POS_ANCHOR_Y_IDX + (int)sizeof(float) + FCS_LEN)

static uint8_t pos_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0 };
static uint8_t pos_resp_ref[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1 };

static uint8_t  frame_seq_nb;
static uint8_t  rx_buf[RX_BUF_LEN];

/* TEMPORARY (diagnostics for the ~74 m calibration readings). Counts why polls
 * were discarded, so an iteration's sample count can be accounted for rather
 * than inferred. Remove once the root cause is found. */
static uint32_t cal_n_big;    /* frame longer than rx_buf -- stale-buffer path */
static uint32_t cal_n_hdr;    /* header did not match the expected response   */

/* ---- BLE result message queue ---------------------------------------------- */
#define TWR_MSG_LEN  20

struct twr_msg {
    char text[TWR_MSG_LEN];
};

/* Unique symbol names: names kept distinct from the example in
 * examples/uwb_ds_initiator.c, which is not currently compiled. */
K_MSGQ_DEFINE(ss_twr_msgq, sizeof(struct twr_msg), 8, 4);

void twr_log(const char *fmt, ...)
{
    struct twr_msg m;
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(m.text, TWR_MSG_LEN, fmt, ap);
    va_end(ap);
    k_msgq_put(&ss_twr_msgq, &m, K_NO_WAIT); /* drop if full */
}

/* ---- BLE sender thread ----------------------------------------------------- */
#define BLE_TX_PRIO   6
#define BLE_TX_STACK  512

K_THREAD_STACK_DEFINE(ss_ble_tx_stack, BLE_TX_STACK);
static struct k_thread ss_ble_tx_tid;

static void ble_tx_fn(void *p1, void *p2, void *p3)
{
    struct twr_msg m;

    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    while (1) {
        if (k_msgq_get(&ss_twr_msgq, &m, K_FOREVER) == 0) {
            ble_log_send(m.text);
        }
    }
}

/* ---- DW3000 IRQ event ------------------------------------------------------ */
/* irq_evt_t is now declared in uwb_ss_initiator.h and shared with the runner. */

static volatile irq_evt_t last_evt;
static K_SEM_DEFINE(irq_sem, 0, 1);

static void cb_txdone(const dwt_cb_data_t *d) { ARG_UNUSED(d); last_evt = EVT_TXFRS; k_sem_give(&irq_sem); }
static void cb_rxok  (const dwt_cb_data_t *d) { ARG_UNUSED(d); last_evt = EVT_RXFCG; k_sem_give(&irq_sem); }
static void cb_rxto  (const dwt_cb_data_t *d) { ARG_UNUSED(d); last_evt = EVT_RXTO;  k_sem_give(&irq_sem); }
static void cb_rxerr (const dwt_cb_data_t *d) { ARG_UNUSED(d); last_evt = EVT_RXERR; k_sem_give(&irq_sem); }

/*
 * Enable DW3000 IRQ, wait for an event semaphore, then disable IRQ.
 * Returns EVT_RXTO when the kernel timeout expires (no event received).
 * IRQ is always DISABLED on return so the caller can safely do SPI work.
 */
irq_evt_t wait_event(k_timeout_t timeout)
{
    k_sem_reset(&irq_sem);
    last_evt = EVT_NONE;

    port_EnableEXT_IRQ();

    /* Handle an edge that fired while IRQ was disabled. */
    if (port_CheckEXT_IRQ()) {
        process_deca_irq();
    }

    if (k_sem_take(&irq_sem, timeout) != 0) {
        port_DisableEXT_IRQ();
        return EVT_RXTO;
    }

    port_DisableEXT_IRQ();
    return last_evt;
}

/* ---- Little-endian 4-byte timestamp field reader --------------------------- */
static uint32_t get_ts_4b(const uint8_t *b)
{
    uint32_t ts = 0;

    for (int i = 0; i < 4; i++) {
        ts |= (uint32_t)b[i] << (i * 8);
    }
    return ts;
}

/* ---- SS-TWR ranging thread ------------------------------------------------- */
#define SS_TWR_PRIO   2
#define SS_TWR_STACK  2048

K_THREAD_STACK_DEFINE(ss_twr_stack, SS_TWR_STACK);
static struct k_thread ss_twr_tid;

#define INT_RX_PHASE  (DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK | \
                       DWT_INT_RXPTO_BIT_MASK  | SYS_STATUS_ALL_RX_ERR)

/*
 * Run a single SS-TWR exchange. On a valid response, writes the measured
 * distance in millimetres to *out_mm and returns true. Returns false on
 * timeout, RX error, or an unexpected frame. Assumes interrupts/antenna delay
 * are already configured by the caller.
 */
static bool do_one_range(int32_t *out_mm)
{
    dwt_setinterrupt(INT_RX_PHASE, 0, DWT_ENABLE_INT_ONLY);

    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg) + FCS_LEN, 0, 1);
    /* The positioning path checks this too. A poll rejected before it reaches
     * the air must not look like a poll that got no answer. */
    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        frame_seq_nb++;
        return false;
    }
    frame_seq_nb++;

    irq_evt_t evt = wait_event(K_MSEC(20));

    if (evt != EVT_RXFCG) {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return false;
    }

    uint16_t flen = dwt_getframelength();
    if (flen > RX_BUF_LEN) {
        /* The frame does not fit, so rx_buf still holds the PREVIOUS exchange.
         * Falling through would pair stale anchor timestamps with fresh local
         * ones and yield a plausible-looking but meaningless distance --
         * silently, because the stale header still passes the memcmp below.
         * do_one_range_anchor() already rejects oversized frames; this path
         * did not. */
        cal_n_big++;   /* TEMPORARY (diagnostics) */
        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
        return false;
    }
    dwt_readrxdata(rx_buf, flen, 0);
    rx_buf[ALL_MSG_SN_IDX] = 0;

    if (memcmp(rx_buf, rx_resp_msg, ALL_MSG_COMMON_LEN) != 0) {
        cal_n_hdr++;   /* TEMPORARY (diagnostics) */
        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
        return false;
    }

    uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
    uint32_t resp_rx_ts = dwt_readrxtimestamplo32();
    double clock_offset_ratio =
        ((double)dwt_readclockoffset()) / (uint32_t)(1 << 26);
    uint32_t poll_rx_ts = get_ts_4b(&rx_buf[RESP_MSG_POLL_RX_TS_IDX]);
    uint32_t resp_tx_ts = get_ts_4b(&rx_buf[RESP_MSG_RESP_TX_TS_IDX]);

    int32_t rtd_init = (int32_t)(resp_rx_ts - poll_tx_ts);
    int32_t rtd_resp = (int32_t)(resp_tx_ts - poll_rx_ts);

    double tof = ((rtd_init - rtd_resp * (1 - clock_offset_ratio)) / 2.0)
                 * DWT_TIME_UNITS;
    *out_mm = (int32_t)(tof * SPEED_OF_LIGHT * 1000.0);
    return true;
}

/*
 * Run a single addressed SS-TWR exchange against anchor `aid`. On a valid,
 * id-matched response, writes the range in metres to *range_m and the anchor's
 * self-reported coordinates to *ax and *ay, then returns true. Returns false on
 * timeout, RX error, wrong magic, or an anchor_id mismatch. Relies on the
 * rx-after-tx delay / timeout / antenna delay configured by ss_twr_fn.
 */
bool do_one_range_anchor(uint8_t aid, float *range_m, float *ax, float *ay)
{
    dwt_setinterrupt(INT_RX_PHASE, 0, DWT_ENABLE_INT_ONLY);

    pos_poll_msg[ALL_MSG_SN_IDX]    = frame_seq_nb;
    pos_poll_msg[POS_ANCHOR_ID_IDX] = aid;
    dwt_writetxdata(sizeof(pos_poll_msg), pos_poll_msg, 0);
    dwt_writetxfctrl(sizeof(pos_poll_msg) + FCS_LEN, 0, 1);
    int tx_rc = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    frame_seq_nb++;

    if (tx_rc != DWT_SUCCESS) {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return false;
    }

    irq_evt_t evt = wait_event(K_MSEC(20));

    if (evt != EVT_RXFCG) {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return false;
    }

    uint16_t flen = dwt_getframelength();
    if (flen < POS_RESP_LEN_MIN || flen > RX_BUF_LEN) {
        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
        return false;
    }
    dwt_readrxdata(rx_buf, flen, 0);
    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

    rx_buf[ALL_MSG_SN_IDX] = 0;
    if (memcmp(rx_buf, pos_resp_ref, ALL_MSG_COMMON_LEN) != 0) {
        return false;
    }
    if (rx_buf[POS_ANCHOR_ID_IDX] != aid) {
        return false;   /* response from a different anchor */
    }

    uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
    uint32_t resp_rx_ts = dwt_readrxtimestamplo32();
    double clock_offset_ratio =
        ((double)dwt_readclockoffset()) / (uint32_t)(1 << 26);
    uint32_t poll_rx_ts = get_ts_4b(&rx_buf[POS_POLL_RX_TS_IDX]);
    uint32_t resp_tx_ts = get_ts_4b(&rx_buf[POS_RESP_TX_TS_IDX]);

    int32_t rtd_init = (int32_t)(resp_rx_ts - poll_tx_ts);
    int32_t rtd_resp = (int32_t)(resp_tx_ts - poll_rx_ts);

    double tof = ((rtd_init - rtd_resp * (1 - clock_offset_ratio)) / 2.0)
                 * DWT_TIME_UNITS;
    *range_m = (float)(tof * SPEED_OF_LIGHT);

    memcpy(ax, &rx_buf[POS_ANCHOR_X_IDX], sizeof(float));
    memcpy(ay, &rx_buf[POS_ANCHOR_Y_IDX], sizeof(float));
    return true;
}

/* Apply a combined antenna delay to the DW3000 (split equally TX/RX). */
static void apply_total_dly(uint16_t total, uint16_t *tx, uint16_t *rx)
{
    cal_split_dly(total, tx, rx);
    dwt_settxantennadelay(*tx);
    dwt_setrxantennadelay(*rx);
}

/* Combined antenna-delay seed for a fresh calibration run. */
static uint16_t active_total_seed(void)
{
    if (cal_is_valid()) {
        uint16_t tx, rx;
        cal_get_ant_dly(&tx, &rx);
        return (uint16_t)(tx + rx);
    }
    return (uint16_t)(TX_ANT_DLY + RX_ANT_DLY);  /* factory reference fallback */
}

/*
 * Iterative auto-solve: collect CAL_SAMPLES_PER_ITER ranges, reject outliers,
 * correct the combined antenna delay toward ref_mm, repeat until the residual
 * is within CAL_ACCEPT_MM or CAL_MAX_ITERS is exhausted. Stores to NVS on
 * success. Reports progress over BLE.
 */
static void run_calibration_locked(uint32_t ref_mm)
{
    static int32_t samples[CAL_MAX_SAMPLES];

    uint16_t tx, rx;
    /* Seed from the active value if valid, else the factory reference. */
    uint16_t total = active_total_seed();
    apply_total_dly(total, &tx, &rx);

    for (uint32_t it = 0; it < CAL_MAX_ITERS; it++) {
        size_t got = 0;
        cal_n_big = 0;   /* TEMPORARY (diagnostics) */
        cal_n_hdr = 0;   /* TEMPORARY (diagnostics) */
        for (uint32_t i = 0; i < CAL_SAMPLES_PER_ITER; i++) {
            int32_t mm;
            if (do_one_range(&mm)) {
                samples[got++] = mm;
            }
            k_sleep(K_MSEC(5));
        }

        int32_t mean;
        size_t kept;
        if (got < CAL_SAMPLES_PER_ITER / 4 ||
            !cal_filtered_mean(samples, got, &mean, &kept)) {
            twr_log("CAL FAIL no-resp\n");
            return;
        }

        /* TEMPORARY (diagnostics). The mean alone cannot distinguish a tight
         * cluster from a bimodal set: cal_filtered_mean() rejects outliers at
         * 6*MAD, and when half the samples are far away the MAD is itself huge,
         * so nothing is rejected and the mean lands between the two groups.
         * min/max and the far-sample count make that visible. */
        int32_t smin = samples[0], smax = samples[0];
        uint32_t n_far = 0;
        for (size_t k = 0; k < got; k++) {
            if (samples[k] < smin) { smin = samples[k]; }
            if (samples[k] > smax) { smax = samples[k]; }
            if (samples[k] > 10000 || samples[k] < -10000) { n_far++; }
        }
        twr_log("CALd g=%u k=%u far=%u min=%d max=%d big=%u hdr=%u\n",
                (unsigned)got, (unsigned)kept, n_far, smin, smax,
                cal_n_big, cal_n_hdr);

        int32_t err = mean - (int32_t)ref_mm;
        int32_t abserr = (err < 0) ? -err : err;
        twr_log("CAL it%u e=%dmm\n", it + 1, err);

        if (abserr <= CAL_ACCEPT_MM) {
            if (cal_store(tx, rx, ref_mm, (uint16_t)abserr) == 0) {
                twr_log("CAL OK %u/%u\n", tx, rx);
            } else {
                twr_log("CAL FAIL nvs\n");
            }
            return;
        }

        total = cal_solve_step(mean, (int32_t)ref_mm, total);
        apply_total_dly(total, &tx, &rx);
    }
    twr_log("CAL FAIL res\n");
}

#define CAL_RADIO_WAIT  K_SECONDS(2)

/* Calibration owns the radio for its whole run: consistent conditions across
 * all samples matter more than keeping beacon sync, and this is a bench
 * operation. The runner reacquires and re-locks afterwards.
 *
 * The wrapper exists so that every exit path of run_calibration_locked() --
 * two early returns plus the fall-through off the end -- releases the radio.
 * A missed release blocks the runner forever. */
static void run_calibration(uint32_t ref_mm)
{
    if (!uwb_radio_request(CAL_RADIO_WAIT)) {
        twr_log("CAL FAIL busy\n");
        return;
    }

    dwt_forcetrxoff();
    /* Clear whatever the abort asserted; a stale RX-error bit would otherwise
     * surface as a spurious EVT_RXERR in the thread taking over the radio. */
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    run_calibration_locked(ref_mm);

    dwt_forcetrxoff();
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    uwb_radio_release();
}

/* Format a metre value as a signed "x.xx" string (centimetre resolution),
 * without relying on %f. */
static void fmt_coord(char *buf, size_t len, float v)
{
    /* Reject NaN/Inf/out-of-range (untrusted radio data) before the int cast,
     * which would otherwise be undefined behaviour. The negated range test is
     * also false for NaN, so NaN is caught too. The +/-9999.99 m bound also
     * caps each coordinate at 7 chars so "P:x.xx,y.yy\n" always fits TWR_MSG_LEN. */
    if (!(v > -10000.0f && v < 10000.0f)) {
        v = 0.0f;
    }

    int cm = (int)(v * 100.0f);     /* truncates toward zero */
    const char *sign = (cm < 0) ? "-" : "";

    if (cm < 0) {
        cm = -cm;
    }
    snprintf(buf, len, "%s%d.%02d", sign, cm / 100, cm % 100);
}

/*
 * Single output seam for a solved position. Today: format "P:x.xx,y.yy\n"
 * (<=20 bytes for the NUS limit) and enqueue to the BLE sender. A future
 * UWB-to-master sender replaces only this function body.
 */
void position_publish(const struct pos_result *pos, uint8_t n_anchors,
                      uint16_t src_addr)
{
    char xs[16], ys[16];

    /* The BLE console line stays alongside the UWB frame. It is the only
     * independent check that the tag solved what the gateway received, and
     * during bring-up that is worth one twr_log() call. */
    fmt_coord(xs, sizeof(xs), pos->x);
    fmt_coord(ys, sizeof(ys), pos->y);
    twr_log("P:%s,%s\n", xs, ys);

    uint8_t buf[UWB_FRAME_LEN_POS];
    int len = uwb_frame_pos_build(buf, sizeof(buf), src_addr,
                                  pos->x, pos->y, pos->residual_m,
                                  n_anchors, batt_soc_cached());
    if (len < 0) {
        return;
    }

    /* Force IDLE before this TX: the last anchor exchange of the sweep can
     * time out at the kernel level with no DW3000 event, leaving the PHY in
     * an unknown state, and an immediate-TX command is not honoured from
     * non-IDLE. Nothing has been transmitted or received yet at this point,
     * so there is no new abort here for dwt_writesysstatuslo() to clear. */
    dwt_forcetrxoff();

    uwb_frame_set_seq_num(buf, frame_seq_nb++);

    dwt_writetxdata((uint16_t)len, buf, 0);
    dwt_writetxfctrl((uint16_t)(len + FCS_LEN), 0, 0);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
        dwt_forcetrxoff();
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return;
    }

    /* Bounded poll for TXFRS rather than wait_event(): that helper opens with
     * k_sem_reset() and closes with a global port_DisableEXT_IRQ(), which is
     * destructive to the runner's own RX arming (see src/uwb_radio_owner.h).
     * Polling the status register touches no shared IRQ state. The frame is
     * ~1.3 ms of airtime; 30 iterations at 100 us each is ~3 ms, leaving
     * headroom against T_SLOT_MS=24 while still catching a stuck radio well
     * short of the next tag's slot. */
    for (int i = 0; i < 30; i++) {
        if (dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK) {
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
            return;
        }
        k_busy_wait(100);
    }
    dwt_forcetrxoff();
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
}

static void ss_twr_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    dwt_setcallbacks(cb_txdone, cb_rxok, cb_rxto, cb_rxerr, NULL, NULL, NULL);
    port_set_dwic_isr(dwt_isr);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    bool ranging = cal_is_valid();
    if (ranging) {
        uint16_t tx, rx;
        cal_get_ant_dly(&tx, &rx);
        dwt_settxantennadelay(tx);
        dwt_setrxantennadelay(rx);
        twr_log("SS-TWR start\n");
    } else {
        twr_log("CAL REQUIRED\n");
    }

    /* Section (b) — the free-running ranging sweep — has been removed.
     * The runner thread (uwb_net_runner.c) now owns the ranging cadence.
     * This loop only services calibration requests. */
    while (1) {
        uint32_t ref_mm;
        if (cal_take_request(&ref_mm)) {
            run_calibration(ref_mm);
            /* The antenna delays are applied by the runner's reacquire path,
             * which is inside the handover. Writing them here would be an
             * unsynchronized SPI access against a runner that is already back
             * on the radio. */
            ranging = cal_is_valid();
        } else if (!ranging) {
            cal_wait_request();   /* block until a cal command arrives */
        } else {
            k_sleep(K_MSEC(100)); /* yield while awaiting optional cal request */
        }
    }
}

/* Compatibility shim: motion.c calls this; route to the runner's tier API.
 * Static -> SLOW (1 s cadence); moving -> FAST (200 ms cadence). */
void uwb_set_moving(bool moving)
{
    uwb_net_set_tier(moving ? UWB_TIER_FAST : UWB_TIER_SLOW);
}

void uwb_ss_initiator_start(void)
{
    k_thread_create(&ss_ble_tx_tid, ss_ble_tx_stack,
                    K_THREAD_STACK_SIZEOF(ss_ble_tx_stack),
                    ble_tx_fn,  NULL, NULL, NULL,
                    BLE_TX_PRIO, 0, K_NO_WAIT);

    k_thread_create(&ss_twr_tid, ss_twr_stack,
                    K_THREAD_STACK_SIZEOF(ss_twr_stack),
                    ss_twr_fn, NULL, NULL, NULL,
                    SS_TWR_PRIO, 0, K_NO_WAIT);
}
