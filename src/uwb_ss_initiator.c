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

#include "uwb_ss_initiator.h"
#include "port.h"
#include "deca_device_api.h"
#include "ble_log.h"
#include "phy_config.h"
#include "cal.h"
#include "cal_math.h"

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
#define RNG_FAST_MS                 200U   /* cadence while moving */
#define RNG_SLOW_MS                 1000U   /* cadence after ~5 s of no motion */

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
#define RX_BUF_LEN               20

static uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0 };
static uint8_t rx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1 };

static uint8_t  frame_seq_nb;
static uint8_t  rx_buf[RX_BUF_LEN];

/* ---- BLE result message queue ---------------------------------------------- */
#define TWR_MSG_LEN  20

struct twr_msg {
    char text[TWR_MSG_LEN];
};

/* Unique symbol names: names kept distinct from the example in
 * examples/uwb_ds_initiator.c, which is not currently compiled. */
K_MSGQ_DEFINE(ss_twr_msgq, sizeof(struct twr_msg), 8, 4);

static void twr_log(const char *fmt, ...)
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
typedef enum {
    EVT_NONE = 0,
    EVT_TXFRS,
    EVT_RXFCG,
    EVT_RXTO,
    EVT_RXERR,
} irq_evt_t;

static volatile irq_evt_t last_evt;
static K_SEM_DEFINE(irq_sem, 0, 1);

static volatile bool ss_moving = true;     /* start fast until motion module reports otherwise */
static K_SEM_DEFINE(range_tick, 0, 1);

static void cb_txdone(const dwt_cb_data_t *d) { ARG_UNUSED(d); last_evt = EVT_TXFRS; k_sem_give(&irq_sem); }
static void cb_rxok  (const dwt_cb_data_t *d) { ARG_UNUSED(d); last_evt = EVT_RXFCG; k_sem_give(&irq_sem); }
static void cb_rxto  (const dwt_cb_data_t *d) { ARG_UNUSED(d); last_evt = EVT_RXTO;  k_sem_give(&irq_sem); }
static void cb_rxerr (const dwt_cb_data_t *d) { ARG_UNUSED(d); last_evt = EVT_RXERR; k_sem_give(&irq_sem); }

/*
 * Enable DW3000 IRQ, wait for an event semaphore, then disable IRQ.
 * Returns EVT_RXTO when the kernel timeout expires (no event received).
 * IRQ is always DISABLED on return so the caller can safely do SPI work.
 */
static irq_evt_t wait_event(k_timeout_t timeout)
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
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    frame_seq_nb++;

    irq_evt_t evt = wait_event(K_MSEC(20));

    if (evt != EVT_RXFCG) {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return false;
    }

    uint16_t flen = dwt_getframelength();
    if (flen <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buf, flen, 0);
    }
    rx_buf[ALL_MSG_SN_IDX] = 0;

    if (memcmp(rx_buf, rx_resp_msg, ALL_MSG_COMMON_LEN) != 0) {
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
static void run_calibration(uint32_t ref_mm)
{
    static int32_t samples[CAL_MAX_SAMPLES];

    uint16_t tx, rx;
    /* Seed from the active value if valid, else the factory reference. */
    uint16_t total = active_total_seed();
    apply_total_dly(total, &tx, &rx);

    for (uint32_t it = 0; it < CAL_MAX_ITERS; it++) {
        size_t got = 0;
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

    while (1) {
        uint32_t ref_mm;
        if (cal_take_request(&ref_mm)) {
            run_calibration(ref_mm);
            ranging = cal_is_valid();
            if (ranging) {
                uint16_t tx, rx;
                cal_get_ant_dly(&tx, &rx);
                dwt_settxantennadelay(tx);
                dwt_setrxantennadelay(rx);
            }
        }

        if (!ranging) {
            cal_wait_request();   /* block until a cal command arrives */
            continue;
        }

        int32_t mm;
        if (do_one_range(&mm)) {
            int32_t v = mm;
            const char *sign = (v < 0) ? "-" : "";
            if (v < 0) {
                v = -v;
            }
            twr_log("D:%s%d.%02dm\n", sign, v / 1000, (v % 1000) / 10);
        }

        uint32_t wait_ms = ss_moving ? RNG_FAST_MS : RNG_SLOW_MS;
        k_sem_take(&range_tick, K_MSEC(wait_ms));
    }
}

/* ---- Public API ------------------------------------------------------------ */

void uwb_set_moving(bool moving)
{
    bool was_moving = ss_moving;

    ss_moving = moving;
    if (moving && !was_moving) {
        k_sem_give(&range_tick);   /* cut a slow wait short */
    }
}

void uwb_ss_initiator_start(void)
{
    k_thread_create(&ss_ble_tx_tid, ss_ble_tx_stack,
                    K_THREAD_STACK_SIZEOF(ss_ble_tx_stack),
                    ble_tx_fn, NULL, NULL, NULL,
                    BLE_TX_PRIO, 0, K_NO_WAIT);

    k_thread_create(&ss_twr_tid, ss_twr_stack,
                    K_THREAD_STACK_SIZEOF(ss_twr_stack),
                    ss_twr_fn, NULL, NULL, NULL,
                    SS_TWR_PRIO, 0, K_NO_WAIT);
}
