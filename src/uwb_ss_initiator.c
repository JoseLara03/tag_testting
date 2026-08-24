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
#include "uwb_wave_frame.h"
#include "port.h"
#include "deca_device_api.h"
#include "ble_log.h"
#include "phy_config.h"
#include "cal.h"
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

/* SPEED_OF_LIGHT is a Qorvo shared_defines macro not present in this project's
 * driver headers; define it locally (m/s, as used by the Qorvo examples). */
#define SPEED_OF_LIGHT  299702547.0

/* ---- Frames (tag convention: no FCS placeholder; +FCS_LEN in writetxfctrl) -- */
#define RX_BUF_LEN               32

/* ---- Positioning frames (addressed; anchor self-reports its (x,y)) --------
 * Poll : [hdr 0..9][anchor_id @10]
 * Resp : [hdr 0..9][anchor_id @10][poll_rx_ts @11..14][resp_tx_ts @15..18]
 *        [x f32 @19..22][y f32 @23..26]
 * Distinct from the non-addressed calibration frames above. */
#define POS_RESP_LEN_MIN  (UWB_WAVE_POS_ANCHOR_Y_IDX + (int)sizeof(float) + FCS_LEN)

static uint8_t pos_poll_msg[] = UWB_WAVE_POS_POLL_INIT;
static uint8_t pos_resp_ref[] = UWB_WAVE_POS_RESP_INIT;

static uint8_t  frame_seq_nb;
static uint8_t  rx_buf[RX_BUF_LEN];

/* ---- BLE result message queue ---------------------------------------------- */
#define TWR_MSG_LEN  20

/* `len` carries the payload length explicitly so the queue can hold binary
 * records as well as text. The raw-range debug records contain NUL bytes, and
 * the old strlen()-based send truncated at the first one.
 * TEMPORARY, with the debug log -- see the removal checklist in
 * spec/2026-08-22-position-filtering-design.md. */
struct twr_msg {
    uint8_t len;
    char    text[TWR_MSG_LEN];
};

/* Unique symbol names: names kept distinct from the example in
 * examples/uwb_ds_initiator.c, which is not currently compiled.
 *
 * Depth 16, not 8, for the duration of the raw-range capture campaign: the
 * debug log adds one record per sweep on top of the existing traffic and
 * k_msgq_put() drops silently when full. Revert to 8 with the log. */
K_MSGQ_DEFINE(ss_twr_msgq, sizeof(struct twr_msg), 16, 4);

void twr_log(const char *fmt, ...)
{
    struct twr_msg m;
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(m.text, TWR_MSG_LEN, fmt, ap);
    va_end(ap);
    m.len = (uint8_t)strlen(m.text);
    k_msgq_put(&ss_twr_msgq, &m, K_NO_WAIT); /* drop if full */
}

void twr_log_raw(const uint8_t *buf, size_t len)
{
    struct twr_msg m;

    if (buf == NULL || len == 0 || len > TWR_MSG_LEN) {
        return;
    }
    memcpy(m.text, buf, len);
    m.len = (uint8_t)len;
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
            ble_log_send_raw((const uint8_t *)m.text, m.len);
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
 * Enable DW3000 IRQ, wait for an event, then disable IRQ.
 * Returns EVT_RXTO when the kernel timeout expires (no event received).
 * IRQ is always DISABLED on return so the caller can safely do SPI work.
 *
 * dwt_isr() is dispatched from the GPIO ISR (via process_deca_irq), where it
 * both decodes the event into last_evt and gives irq_sem -- so k_sem_take()
 * returning means the event is already decoded.
 *
 * DO NOT RESTRUCTURE THIS WITHOUT HARDWARE EVIDENCE. Two rewrites were tried
 * while chasing a calibration crash and both had to be reverted, each dropping
 * accepted ranges to 1 in 100 (`CALp 100 1`, zero size/header rejects, i.e. 99
 * pure RX timeouts):
 *
 *   1. Deferring dwt_isr() into this thread via an ISR notifier + semaphore.
 *   2. Draining with the IRQ masked *before* arming it, and returning early
 *      when that drain produced an event.
 *
 * Both look more correct than what is here -- (2) genuinely closes a window
 * where this thread runs SPI with the interrupt armed -- but the receive path
 * depends on the order below in ways that are not captured by reading it. The
 * crash that motivated the rewrites was an array overrun in
 * run_calibration_locked(), not anything in this function.
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
/* 4096, not 2048: the calibration path is by far the deepest in this firmware.
 * cal_filtered_mean() alone declares two int32_t[CAL_MAX_SAMPLES] arrays --
 * 1 KB of locals in a single frame -- on top of run_calibration_locked(),
 * run_calibration(), the double-precision ToF math, and vsnprintf() in
 * twr_log(). At 2048 that ran with little margin, and the failure mode is an
 * MPU fault (CONFIG_HW_STACK_PROTECTION=y) which, with no console, is a silent
 * reset: exactly "the tag disconnects and never says whether cal worked".
 * The extra 2 KB is nothing against 128 KB of RAM. */
#define SS_TWR_STACK  4096

K_THREAD_STACK_DEFINE(ss_twr_stack, SS_TWR_STACK);
static struct k_thread ss_twr_tid;

#define INT_RX_PHASE  (DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK | \
                       DWT_INT_RXPTO_BIT_MASK  | SYS_STATUS_ALL_RX_ERR)

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
    pos_poll_msg[UWB_WAVE_POS_ANCHOR_ID_IDX] = aid;
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
    if (rx_buf[UWB_WAVE_POS_ANCHOR_ID_IDX] != aid) {
        return false;   /* response from a different anchor */
    }

    uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
    uint32_t resp_rx_ts = dwt_readrxtimestamplo32();
    double clock_offset_ratio =
        ((double)dwt_readclockoffset()) / (uint32_t)(1 << 26);
    uint32_t poll_rx_ts = get_ts_4b(&rx_buf[UWB_WAVE_POS_POLL_RX_TS_IDX]);
    uint32_t resp_tx_ts = get_ts_4b(&rx_buf[UWB_WAVE_POS_RESP_TX_TS_IDX]);

    int32_t rtd_init = (int32_t)(resp_rx_ts - poll_tx_ts);
    int32_t rtd_resp = (int32_t)(resp_tx_ts - poll_rx_ts);

    double tof = ((rtd_init - rtd_resp * (1 - clock_offset_ratio)) / 2.0)
                 * DWT_TIME_UNITS;
    *range_m = (float)(tof * SPEED_OF_LIGHT);

    memcpy(ax, &rx_buf[UWB_WAVE_POS_ANCHOR_X_IDX], sizeof(float));
    memcpy(ay, &rx_buf[UWB_WAVE_POS_ANCHOR_Y_IDX], sizeof(float));
    return true;
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
static float    last_pos_x, last_pos_y;
static volatile bool last_pos_valid;

bool pos_last_get(float *x, float *y)
{
    if (!last_pos_valid) {
        return false;
    }
    *x = last_pos_x;
    *y = last_pos_y;
    return true;
}

void position_publish(const struct pos_result *pos, uint8_t n_anchors,
                      uint16_t src_addr)
{
    char xs[16], ys[16];

    last_pos_x     = pos->x;
    last_pos_y     = pos->y;
    last_pos_valid = true;

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

    if (cal_is_valid()) {
        uint16_t tx, rx;

        cal_get_ant_dly(&tx, &rx);
        dwt_settxantennadelay(tx);
        dwt_setrxantennadelay(rx);
        twr_log("SS-TWR start\n");
    } else {
        twr_log("CAL REQUIRED\n");
    }

    /* One-time DW3000 setup for production ranging (do_one_range_anchor(),
     * driven by uwb_net_runner.c's own thread) and, in the calibration
     * image, for cal_run.c's polling-based exchanges -- neither depends on
     * this thread running any further. See
     * docs/superpowers/specs/2026-08-20-cal-image-rewrite-design.md. */
    k_sleep(K_FOREVER);
}

/* Compatibility shim: motion.c calls this; route to the runner's motion API,
 * which applies the tier hysteresis and the coverage-ladder reset. */
void uwb_set_moving(bool moving)
{
    /* Raw state, not a tier: uwb_net_tier_filter() in the runner owns the
     * hysteresis now (design §6.2), and it needs the edge rather than a
     * conclusion already drawn from it. */
    uwb_net_set_moving(moving);
    /* The tier is only read at the top of the runner's loop, and the loop can
     * be parked in a multi-second skip. Wake it so the new cadence starts now
     * rather than at the end of the skip -- the tag would otherwise be well
     * into motion before it began ranging at the moving rate. Called from the
     * accelerometer's GPIO ISR; uwb_net_runner_wake() is ISR-safe. */
    uwb_net_runner_wake();
}

size_t uwb_ss_stack_unused(void)
{
    size_t unused = 0;

    if (k_thread_stack_space_get(&ss_twr_tid, &unused) != 0) {
        return 0;   /* CONFIG_INIT_STACKS off, or thread not started */
    }
    return unused;
}

size_t uwb_ss_ble_stack_unused(void)
{
    size_t unused = 0;

    if (k_thread_stack_space_get(&ss_ble_tx_tid, &unused) != 0) {
        return 0;
    }
    return unused;
}

void uwb_ss_initiator_start(void)
{
    k_thread_create(&ss_ble_tx_tid, ss_ble_tx_stack,
                    K_THREAD_STACK_SIZEOF(ss_ble_tx_stack),
                    ble_tx_fn,  NULL, NULL, NULL,
                    BLE_TX_PRIO, 0, K_NO_WAIT);
    /* Named so a fatal error can say which thread died -- see `fault`. */
    k_thread_name_set(&ss_ble_tx_tid, "bletx");

    k_thread_create(&ss_twr_tid, ss_twr_stack,
                    K_THREAD_STACK_SIZEOF(ss_twr_stack),
                    ss_twr_fn, NULL, NULL, NULL,
                    SS_TWR_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&ss_twr_tid, "sstwr");
}
