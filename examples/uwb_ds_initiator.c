/*
 * DS TWR Initiator — Zephyr RTOS adaptation for nRF52833 + DW3000
 *
 * Interrupt-driven (no polling): DW3000 IRQ fires → callback gives semaphore →
 * high-priority ranging thread wakes up and processes the event.
 *
 * Results are enqueued in twr_msgq and consumed by a dedicated BLE sender
 * thread, so the timing-critical path never blocks on BLE TX.
 */

#include "uwb_ds_initiator.h"
#include "port.h"
#include "deca_device_api.h"
#include "ble_log.h"

#include <zephyr/kernel.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ---- Timing constants ------------------------------------------------------ */
#define TX_ANT_DLY  16385U
#define RX_ANT_DLY  16385U

/*
 * CPU_PROCESSING_TIME: extra margin (µs) for thread wake-up latency and SPI
 * operations in the interrupt-driven approach. Tune down if the responder's
 * POLL_RX_TO_RESP_TX delay is also adjusted.
 */
#define CPU_PROCESSING_TIME            400U
#define POLL_TX_TO_RESP_RX_DLY_UUS   (1000U + CPU_PROCESSING_TIME)
#define RESP_RX_TO_FINAL_TX_DLY_UUS  (2000U + CPU_PROCESSING_TIME)
#define RESP_RX_TIMEOUT_UUS           2000U
#define PRE_TIMEOUT                    128U

/* 1 UWB µs = 512 × 128 device time units (499.2 MHz × 128 internal clock). */
#define UUS_TO_DWT_TIME  65536UL

#define RNG_DELAY_MS  1000U

/* ---- Frame definitions ----------------------------------------------------- */
#define ALL_MSG_COMMON_LEN         10
#define ALL_MSG_SN_IDX              2
#define FINAL_MSG_POLL_TX_TS_IDX   10
#define FINAL_MSG_RESP_RX_TS_IDX   14
#define FINAL_MSG_FINAL_TX_TS_IDX  18
#define RX_BUF_LEN                 20

static uint8_t tx_poll_msg[]  = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21 };
static uint8_t rx_resp_msg[]  = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0 };
static uint8_t tx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23,
                                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static uint8_t  frame_seq_nb;
static uint8_t  rx_buf[RX_BUF_LEN];
static uint64_t poll_tx_ts;
static uint64_t resp_rx_ts;
static uint64_t final_tx_ts;

/* ---- BLE result message queue ---------------------------------------------- */
#define TWR_MSG_LEN  20

struct twr_msg {
    char text[TWR_MSG_LEN];
};

K_MSGQ_DEFINE(twr_msgq, sizeof(struct twr_msg), 8, 4);

static void twr_log(const char *fmt, ...)
{
    struct twr_msg m;
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(m.text, TWR_MSG_LEN, fmt, ap);
    va_end(ap);
    /* Non-blocking: if queue is full the message is dropped. */
    k_msgq_put(&twr_msgq, &m, K_NO_WAIT);
}

/* ---- BLE sender thread ----------------------------------------------------- */
#define BLE_TX_PRIO   6
#define BLE_TX_STACK  512

K_THREAD_STACK_DEFINE(ble_tx_stack, BLE_TX_STACK);
static struct k_thread ble_tx_tid;

static void ble_tx_fn(void *p1, void *p2, void *p3)
{
    struct twr_msg m;

    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    while (1) {
        if (k_msgq_get(&twr_msgq, &m, K_FOREVER) == 0) {
            ble_log_send(m.text);
        }
    }
}

/* ---- DW3000 IRQ event ------------------------------------------------------ */
typedef enum {
    EVT_NONE  = 0,
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
 *
 * IRQ is always DISABLED on return so that the caller can safely do SPI work.
 * All SPI access in this module must happen with IRQ disabled; the only window
 * where IRQ is enabled is while the thread is blocked on this semaphore.
 */
static irq_evt_t wait_event(k_timeout_t timeout)
{
    k_sem_reset(&irq_sem);
    last_evt = EVT_NONE;

    port_EnableEXT_IRQ();

    /*
     * If an IRQ edge fired while we had it disabled, the GPIO pin is still
     * asserted. Handle it now before sleeping, so we do not miss the event.
     */
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

/* ---- Timestamp helpers ----------------------------------------------------- */
static uint64_t bytes5_to_u64(const uint8_t *b)
{
    uint64_t ts = 0;

    for (int i = 4; i >= 0; i--) {
        ts = (ts << 8) | b[i];
    }
    return ts;
}

static uint64_t get_tx_ts(void)
{
    uint8_t t[5];

    dwt_readtxtimestamp(t);
    return bytes5_to_u64(t);
}

static uint64_t get_rx_ts(void)
{
    uint8_t t[5];

    dwt_readrxtimestamp(t);
    return bytes5_to_u64(t);
}

/* Copy lower 4 bytes of a 40-bit timestamp into a frame field.
 * Upper byte is discarded — exchanges are < 67 ms so 32-bit subtraction
 * in the responder's ToF calculation remains valid (DS TWR note 12). */
static void set_ts_4b(uint8_t *dst, uint64_t ts)
{
    for (int i = 0; i < 4; i++) {
        dst[i] = (uint8_t)(ts >> (i * 8));
    }
}

/* ---- DS TWR ranging thread ------------------------------------------------- */
#define DS_TWR_PRIO   2
#define DS_TWR_STACK  2048

K_THREAD_STACK_DEFINE(ds_twr_stack, DS_TWR_STACK);
static struct k_thread ds_twr_tid;

/*
 * Interrupt masks for each phase. Using DWT_ENABLE_INT_ONLY so that only the
 * expected event class can fire — this prevents the Poll TXFRS from waking
 * the thread during the Response RX wait, and vice-versa.
 */
#define INT_RX_PHASE  (DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK | \
                       DWT_INT_RXPTO_BIT_MASK  | SYS_STATUS_ALL_RX_ERR)
#define INT_TX_PHASE  (DWT_INT_TXFRS_BIT_MASK)

static void ds_twr_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    /* One-time setup: callbacks, ISR routing, antenna delays, RX timeouts.
     * PHY config (dwt_configure / dwt_configuretxrf / dwt_setlnapamode)
     * was already applied by uwb_init() and persists in the DW3000. */
    dwt_setcallbacks(cb_txdone, cb_rxok, cb_rxto, cb_rxerr, NULL, NULL, NULL);
    port_set_dwic_isr(dwt_isr);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    twr_log("DS-TWR start\n");

    while (1) {
        /* ---- Phase 1: Poll TX → Response RX -------------------------------- */
        dwt_setinterrupt(INT_RX_PHASE, 0, DWT_ENABLE_INT_ONLY);

        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg) + FCS_LEN, 0, 1);
        /* RESPONSE_EXPECTED: DW3000 re-enables RX automatically after Poll TX. */
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
        frame_seq_nb++;

        irq_evt_t evt = wait_event(K_MSEC(5));

        if (evt == EVT_RXFCG) {
            uint16_t flen = dwt_getframelength();

            if (flen <= RX_BUF_LEN) {
                dwt_readrxdata(rx_buf, flen, 0);
            }
            rx_buf[ALL_MSG_SN_IDX] = 0; /* mask seq field for comparison */

            if (memcmp(rx_buf, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {
                /* ---- Phase 2: Final TX (delayed) --------------------------- */
                poll_tx_ts = get_tx_ts();
                resp_rx_ts = get_rx_ts();

                /* Schedule Final TX at a precise future time. */
                uint32_t final_tx_time =
                    (uint32_t)((resp_rx_ts +
                                ((uint64_t)RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);

                dwt_setdelayedtrxtime(final_tx_time);
                final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                set_ts_4b(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX],  poll_tx_ts);
                set_ts_4b(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX],  resp_rx_ts);
                set_ts_4b(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

                tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
                dwt_writetxfctrl(sizeof(tx_final_msg) + FCS_LEN, 0, 1);

                dwt_setinterrupt(INT_TX_PHASE, 0, DWT_ENABLE_INT_ONLY);

                int ret = dwt_starttx(DWT_START_TX_DELAYED);
                frame_seq_nb++;

                if (ret == DWT_SUCCESS) {
                    evt = wait_event(K_MSEC(5));
                    if (evt == EVT_TXFRS) {
                        twr_log("TWR OK s=%d\n", (int)frame_seq_nb);
                    } else {
                        twr_log("TWR TX err\n");
                    }
                } else {
                    /* Delayed TX missed its window — clear residual status. */
                    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO   |
                                         SYS_STATUS_ALL_RX_ERR  |
                                         DWT_INT_TXFRS_BIT_MASK);
                    twr_log("TWR LATE s=%d\n", (int)frame_seq_nb);
                }

            } else {
                /* Received a valid frame, but not the expected response. */
                dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | DWT_INT_TXFRS_BIT_MASK);
                twr_log("TWR FRM err\n");
            }

        } else if (evt == EVT_RXTO) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO  |
                                  SYS_STATUS_ALL_RX_ERR |
                                  DWT_INT_TXFRS_BIT_MASK);
            twr_log("TWR TO s=%d\n", (int)frame_seq_nb);

        } else {
            /* EVT_RXERR or kernel-level timeout (both treated as RX error). */
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO  |
                                  SYS_STATUS_ALL_RX_ERR |
                                  DWT_INT_TXFRS_BIT_MASK);
            twr_log("TWR ERR s=%d\n", (int)frame_seq_nb);
        }

        k_msleep(RNG_DELAY_MS);
    }
}

/* ---- Public API ------------------------------------------------------------ */

void uwb_ds_initiator_start(void)
{
    k_thread_create(&ble_tx_tid, ble_tx_stack,
                    K_THREAD_STACK_SIZEOF(ble_tx_stack),
                    ble_tx_fn, NULL, NULL, NULL,
                    BLE_TX_PRIO, 0, K_NO_WAIT);

    k_thread_create(&ds_twr_tid, ds_twr_stack,
                    K_THREAD_STACK_SIZEOF(ds_twr_stack),
                    ds_twr_fn, NULL, NULL, NULL,
                    DS_TWR_PRIO, 0, K_NO_WAIT);
}
