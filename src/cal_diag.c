/*
 * Cal-image-only RX-path diagnostics: `cal listen` and `cal probe`.
 * See spec/2026-08-19-tag-calibration-image-design.md §6.
 *
 * Polls SYS_STATUS_LO with interrupts disabled, exactly like the ANCLA
 * anchor project's src/ss_initiator.c, rather than using wait_event(): that
 * keeps uwb_ss_initiator.c completely untouched (design §6.1) and sidesteps
 * the k_sem_reset()/port_DisableEXT_IRQ() pair uwb_radio_owner.h exists to
 * protect.
 */

#include "cal_diag.h"
#include "uwb_wave_frame.h"
#include "uwb_radio_owner.h"
#include "uwb_ss_initiator.h"   /* twr_log() */
#include "cal.h"                /* cal_is_valid(), cal_get_ant_dly() */
#include "cal_math.h"           /* cal_split_dly() -- TEMPORARY probe-delay-override diagnostic */
#include "phy_config.h"         /* TX_ANT_DLY / RX_ANT_DLY fallback */
#include "deca_device_api.h"

#include <zephyr/kernel.h>
#include <string.h>

/* Mirrors uwb_ss_initiator.c's own constants -- restored on release per the
 * PHY-state handover contract (uwb_radio_owner.h). No shared header exists
 * for these three; if they are retuned there, retune them here too. */
#define UUS_TO_DWT_TIME             65536UL
#define POLL_TX_TO_RESP_RX_DLY_UUS  1000U
#define RESP_RX_TIMEOUT_UUS         2000U
#define PRE_TIMEOUT                  128U

/* Mirrors cal_run.c's SPEED_OF_LIGHT -- used only by the `cal probe`
 * distance calculation below, so it can be compared directly against a
 * `cal <mm>` run's per-sample math without any of that loop's averaging,
 * outlier rejection, or iteration in the way. */
#define SPEED_OF_LIGHT   299702547.0

#define CAL_DIAG_RADIO_WAIT         K_SECONDS(2)
#define CAL_DIAG_LISTEN_DEFAULT_MS  3000U
#define CAL_DIAG_LISTEN_MAX_MS     10000U
#define CAL_DIAG_PROBE_TIMEOUT_MS     50U

#define RX_BUF_LEN  32
#define TYPE_BYTE_IDX  9   /* byte 9 of the WAVE header: message type */

enum cal_diag_kind { CAL_DIAG_LISTEN, CAL_DIAG_PROBE };

struct cal_diag_req {
    enum cal_diag_kind kind;
    uint32_t           arg;   /* listen: ms; probe: wire id, 0 = unaddressed */
};

K_MSGQ_DEFINE(cal_diag_q, sizeof(struct cal_diag_req), 2, 4);

#define CAL_DIAG_PRIO   6
#define CAL_DIAG_STACK  1024

K_THREAD_STACK_DEFINE(cal_diag_stack, CAL_DIAG_STACK);
static struct k_thread cal_diag_tid;

/* Claim the radio and drive it with interrupts fully disabled: this module
 * polls SYS_STATUS_LO by hand, and an enabled IRQ would clear the very bits
 * being polled for before the poll could see them. */
static bool claim_radio(void)
{
    if (!uwb_radio_request(CAL_DIAG_RADIO_WAIT)) {
        return false;
    }
    dwt_forcetrxoff();
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    dwt_setinterrupt(0xFFFFFFFFU, 0xFFFFFFFFU, DWT_DISABLE_INT);
    return true;
}

/* Restores the five PHY items named in uwb_radio_owner.h's handover contract
 * before releasing, so ss_twr_fn()'s next exchange runs exactly as it would
 * have with no diagnostic in between. */
static void release_radio(void)
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

/* Little-endian 4-byte timestamp field reader, matching cal_run.c's
 * cal_run_get_ts_4b() -- duplicated rather than shared for the same reason
 * cal_run.c duplicates wait_any_sysstatus_lo() from this file: both are a
 * few lines, and there is no shared header between these two cal-image-only
 * siblings for helpers this small. */
static uint32_t get_ts_4b(const uint8_t *b)
{
    uint32_t ts = 0;

    for (int i = 0; i < 4; i++) {
        ts |= (uint32_t)b[i] << (i * 8);
    }
    return ts;
}

/* Bounded wait for ANY bit in mask, polling SYS_STATUS_LO. Returns the
 * matched bits, or 0 on timeout. */
static uint32_t wait_any_sysstatus_lo(uint32_t mask, uint32_t timeout_ms)
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

static void do_listen(uint32_t ms)
{
    if (!claim_radio()) {
        twr_log("F busy\n");
        return;
    }

    dwt_setrxtimeout(0);
    dwt_setpreambledetecttimeout(0);

    uint32_t n = 0, errors = 0;
    uint32_t t_end = k_uptime_get_32() + ms;

    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    while ((int32_t)(t_end - k_uptime_get_32()) > 0) {
        uint32_t remaining = (uint32_t)(t_end - k_uptime_get_32());
        uint32_t got = wait_any_sysstatus_lo(
            DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR, remaining);

        if (got == 0) {
            break;   /* deadline reached with nothing pending */
        }

        if (got & DWT_INT_RXFCG_BIT_MASK) {
            n++;
        } else {
            errors++;
        }
        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }

    release_radio();
    twr_log("LSN n=%u e=%u\n", n, errors);
}

/* `cal probe <n>` for n in 5..CAL_MAX_TOTAL_DLY overrides the antenna delay
 * for this ONE probe (still unaddressed -- wire_id stays 0), applied via a
 * completely fresh claim_radio(). n in 1..4 keeps its existing meaning
 * (addressed probe to that anchor id).
 *
 * This is a permanent bench diagnostic, not leftover scaffolding: it is
 * what isolated the bug behind Open Work item 1 in CLAUDE.md -- a
 * freshly-claimed radio reads back correct, physically consistent
 * distances at any delay tested, while cal_run.c's old mid-claim
 * re-application of a corrected delay did not, for reasons never pinned
 * down at the register level. cal_run.c's architecture now avoids that
 * re-application entirely (see cal_run_execute()), but this command
 * remains useful for reading back the distance at a specific delay
 * without running a full calibration or touching NVS. */
static void do_probe(uint32_t param)
{
    uint8_t buf[RX_BUF_LEN];
    bool override_delay = (param > 4);
    uint32_t wire_id = override_delay ? 0 : param;

    if (!claim_radio()) {
        twr_log("F busy\n");
        return;
    }

    if (override_delay) {
        uint16_t tx, rx;

        dwt_forcetrxoff();
        cal_split_dly((uint16_t)param, &tx, &rx);
        dwt_settxantennadelay(tx);
        dwt_setrxantennadelay(rx);
    }

    if (wire_id == 0) {
        uint8_t poll[] = UWB_WAVE_POLL_INIT;

        dwt_writetxdata(sizeof(poll), poll, 0);
        dwt_writetxfctrl(sizeof(poll) + FCS_LEN, 0, 1);
    } else {
        uint8_t poll[] = UWB_WAVE_POS_POLL_INIT;

        poll[UWB_WAVE_POS_ANCHOR_ID_IDX] = (uint8_t)wire_id;
        dwt_writetxdata(sizeof(poll), poll, 0);
        dwt_writetxfctrl(sizeof(poll) + FCS_LEN, 0, 1);
    }

    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);
    dwt_setpreambledetecttimeout(0);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        release_radio();
        twr_log("F txfail\n");
        return;
    }

    uint32_t got = wait_any_sysstatus_lo(
        DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR,
        CAL_DIAG_PROBE_TIMEOUT_MS);

    if (got == 0) {
        release_radio();
        twr_log("F none\n");
        return;
    }
    if (!(got & DWT_INT_RXFCG_BIT_MASK)) {
        release_radio();
        twr_log("F err\n");
        return;
    }
    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

    uint16_t flen = dwt_getframelength();

    if (flen < ALL_MSG_COMMON_LEN + FCS_LEN) {
        release_radio();
        twr_log("F err\n");
        return;
    }
    if (flen > RX_BUF_LEN) {
        uint16_t plen = (uint16_t)(flen - FCS_LEN);

        release_radio();
        twr_log("F big %u\n", plen);
        return;
    }
    dwt_readrxdata(buf, flen, 0);

    uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
    uint32_t resp_rx_ts = dwt_readrxtimestamplo32();
    uint32_t rtt_uus     = (resp_rx_ts - poll_tx_ts) / UUS_TO_DWT_TIME;
    uint8_t  type_byte   = buf[TYPE_BYTE_IDX];
    uint16_t plen        = (uint16_t)(flen - FCS_LEN);

    /* Corrected distance, wire_id==0 (non-addressed calibration frame) only:
     * the exact same clock-offset-corrected ToF formula cal_range_poll()
     * uses in cal_run.c, applied to this single isolated exchange. Lets an
     * operator compare one clean sample directly against a `cal <mm>` run's
     * per-iteration error, with none of that loop's averaging, outlier
     * rejection, or antenna-delay correction between samples. Not computed
     * for an addressed (anchor) probe -- that is not the frame `cal <mm>`
     * uses, so it is out of scope for this comparison. */
    bool    have_dist = false;
    int32_t dist_mm   = 0;

    if (wire_id == 0 && flen >= UWB_WAVE_RESP_RESP_TX_TS_IDX + 4 + FCS_LEN) {
        uint32_t poll_rx_ts = get_ts_4b(&buf[UWB_WAVE_RESP_POLL_RX_TS_IDX]);
        uint32_t resp_tx_ts = get_ts_4b(&buf[UWB_WAVE_RESP_RESP_TX_TS_IDX]);
        double   clock_offset_ratio =
            ((double)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        int32_t rtd_init = (int32_t)(resp_rx_ts - poll_tx_ts);
        int32_t rtd_resp = (int32_t)(resp_tx_ts - poll_rx_ts);

        double tof = ((rtd_init - rtd_resp * (1 - clock_offset_ratio)) / 2.0)
                     * DWT_TIME_UNITS;
        dist_mm = (int32_t)(tof * SPEED_OF_LIGHT * 1000.0);
        have_dist = true;
    }

    release_radio();
    twr_log("F t=%u %02X %u\n", rtt_uus, type_byte, plen);
    if (have_dist) {
        /* Separate line: "F t=... %02X ..." plus a distance field would
         * exceed the 20-byte NUS limit and twr_log()'s TWR_MSG_LEN, and
         * that limit truncates silently -- see CLAUDE.md's NUS-payload
         * hard-won fact. Keeping this on its own line is what keeps both
         * readable. */
        twr_log("F d=%dmm\n", dist_mm);
    }
}

static void cal_diag_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    struct cal_diag_req req;

    while (1) {
        k_msgq_get(&cal_diag_q, &req, K_FOREVER);
        if (req.kind == CAL_DIAG_LISTEN) {
            do_listen(req.arg);
        } else {
            do_probe(req.arg);
        }
    }
}

/* Minimal unsigned-decimal parser, mirroring tag_cmd.c's parse_u32(): the
 * parse is trivial and sscanf drags in a second format engine. */
static bool parse_u32(const char *s, uint32_t *out)
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

void cal_diag_on_rx(const uint8_t *data, uint16_t len)
{
    char buf[24];
    uint16_t n = (len < sizeof(buf) - 1) ? len : (uint16_t)(sizeof(buf) - 1);

    memcpy(buf, data, n);
    buf[n] = '\0';
    while (n > 0 && (buf[n - 1] == '\r' || buf[n - 1] == '\n')) {
        buf[--n] = '\0';
    }

    struct cal_diag_req req;

    if (strncmp(buf, "cal listen", 10) == 0) {
        const char *arg = buf + 10;

        while (*arg == ' ') {
            arg++;
        }
        uint32_t ms = CAL_DIAG_LISTEN_DEFAULT_MS;

        if (*arg != '\0' && !parse_u32(arg, &ms)) {
            twr_log("CAL ERR listen ms\n");
            return;
        }
        if (ms > CAL_DIAG_LISTEN_MAX_MS) {
            ms = CAL_DIAG_LISTEN_MAX_MS;
        }
        req.kind = CAL_DIAG_LISTEN;
        req.arg  = ms;
        if (k_msgq_put(&cal_diag_q, &req, K_NO_WAIT) != 0) {
            twr_log("F busy\n");
        }
        return;
    }

    if (strncmp(buf, "cal probe", 9) == 0) {
        const char *arg = buf + 9;

        while (*arg == ' ') {
            arg++;
        }
        uint32_t id = 0;

        if (*arg != '\0') {
            /* 1..4 is an anchor id (existing meaning). >4, up to
             * CAL_MAX_TOTAL_DLY, is a TEMPORARY antenna-delay override for
             * a fresh-claim unaddressed probe -- see do_probe(). */
            if (!parse_u32(arg, &id) || id < 1 || id > CAL_MAX_TOTAL_DLY) {
                twr_log("CAL ERR probe id\n");
                return;
            }
        }
        req.kind = CAL_DIAG_PROBE;
        req.arg  = id;
        if (k_msgq_put(&cal_diag_q, &req, K_NO_WAIT) != 0) {
            twr_log("F busy\n");
        }
        return;
    }
}

void cal_diag_start(void)
{
    k_thread_create(&cal_diag_tid, cal_diag_stack,
                    K_THREAD_STACK_SIZEOF(cal_diag_stack),
                    cal_diag_fn, NULL, NULL, NULL,
                    CAL_DIAG_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&cal_diag_tid, "caldiag");
}
