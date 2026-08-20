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
        DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR, CAL_RANGE_POLL_TIMEOUT_MS);

    if (got == 0) {
        return false;   /* timeout */
    }
    if (!(got & DWT_INT_RXFCG_BIT_MASK)) {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
        return false;   /* RX error */
    }
    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

    uint16_t flen = dwt_getframelength();

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
