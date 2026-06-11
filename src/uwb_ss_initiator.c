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

#include <zephyr/kernel.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ---- Calibration knob: antenna delay (edit + rebuild to iterate) ----------- */
#define TX_ANT_DLY  16385U
#define RX_ANT_DLY  16385U

/* ---- Timing (seeded from the working DS initiator for CONFIG_OPTION_07) ----- */
#define UUS_TO_DWT_TIME             65536UL
#define POLL_TX_TO_RESP_RX_DLY_UUS  2000U   /* RX turns on this long after poll TX */
#define RESP_RX_TIMEOUT_UUS         4000U   /* covers full response frame air time */
#define PRE_TIMEOUT                  128U
#define RNG_DELAY_MS                1000U

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

/* Unique symbol names: uwb_ds_initiator.c defines same-named globals and is
 * also compiled into the image, so these must not collide. */
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

static void ss_twr_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    dwt_setcallbacks(cb_txdone, cb_rxok, cb_rxto, cb_rxerr, NULL, NULL, NULL);
    port_set_dwic_isr(dwt_isr);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    twr_log("SS-TWR start\n");

    while (1) {
        /* Only RX-class events may wake us; the poll TXFRS is masked out. */
        dwt_setinterrupt(INT_RX_PHASE, 0, DWT_ENABLE_INT_ONLY);

        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg) + FCS_LEN, 0, 1);
        /* RESPONSE_EXPECTED: RX is enabled automatically after poll TX. */
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
        frame_seq_nb++;

        /* 20 ms kernel fallback: comfortably covers the ~3 ms exchange at this PHY. */
        irq_evt_t evt = wait_event(K_MSEC(20));

        if (evt == EVT_RXFCG) {
            uint16_t flen = dwt_getframelength();

            if (flen <= RX_BUF_LEN) {
                dwt_readrxdata(rx_buf, flen, 0);
            }
            rx_buf[ALL_MSG_SN_IDX] = 0;

            if (memcmp(rx_buf, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {
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
                double distance = tof * SPEED_OF_LIGHT;

                /* Format "D:x.xxm" with integer math to avoid depending on
                 * floating-point printf support (not enabled in this build). */
                int cm = (int)(distance * 100.0);
                const char *sign = (cm < 0) ? "-" : "";

                if (cm < 0) {
                    cm = -cm;
                }
                twr_log("D:%s%d.%02dm\n", sign, cm / 100, cm % 100);

            } else {
                dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
                /* unexpected frame: stay quiet, just retry next cycle */
            }

        } else {
            /* Timeout or RX error — clear residual status and retry. */
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }

        k_msleep(RNG_DELAY_MS);
    }
}

/* ---- Public API ------------------------------------------------------------ */

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
