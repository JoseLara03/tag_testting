/*
 * uwb_net_runner.c — DW3000 radio_ops runner driving the MAC FSM (uwb_net).
 *
 * Each superframe: RX beacon -> build uwb_net_event -> uwb_net_handle ->
 * execute returned action flags (JOIN / KEEPALIVE / DISCOVER / SWEEP / SLEEP).
 *
 * Ranging primitives (do_one_range_anchor, wait_event) are shared from
 * uwb_ss_initiator.c.  Frame builders/parsers come from uwb_frame_802_15_4z.
 */

#include "uwb_net_runner.h"
#include "uwb_radio_ops.h"
#include "uwb_net.h"
#include "uwb_frame_802_15_4z.h"
#include "uwb_ss_initiator.h"
#include "pos_solver.h"
#include "port.h"
#include "deca_device_api.h"
#include "phy_config.h"

#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <string.h>
#include <errno.h>

/* ---- v1 timing constants (protocol contract §2.1) ---- */
#define T_SUPERFRAME_MS   200u
#define T_BEACON_MS         2u   /* ~1.5 ms, round up */
#define T_GUARD_MS          1u   /* ~0.5 ms, round up */
#define T_SLOT_MS          15u
#define T_MINISLOT_MS       2u   /* ~1.5 ms, round up */
#define N_CAP               UWB_FRAME_N_CAP   /* 4 */
#define N_CFP               UWB_FRAME_N_CFP   /* 12 */

/* ---- DW3000 RX/TX timing (re-used from initiator) ---- */
#define POLL_TX_TO_RESP_RX_DLY_UUS   1000U
#define RESP_RX_TIMEOUT_UUS          2000U
#define PRE_TIMEOUT                   128U

/* ---- Runner thread parameters ---- */
#define RUNNER_PRIO    2
#define RUNNER_STACK   2048

K_THREAD_STACK_DEFINE(runner_stack, RUNNER_STACK);
static struct k_thread runner_tid;

/* ---- Tier-change request (set from motion / ISR context) ---- */
static volatile uwb_tier_t  pending_tier;
static volatile bool        tier_pending;

/* ---- EUI stored at start ---- */
static uint8_t runner_eui[UWB_FRAME_EUI_LEN];

/* ---- Anchor set (static; 4 known anchors) ---- */
static const uint8_t ANCHOR_IDS[] = { 0, 1, 2, 3 };
#define RUNNER_NUM_ANCHORS  ARRAY_SIZE(ANCHOR_IDS)

/* ---- DW3000 ISR callbacks (mirror the initiator's; same irq_sem/last_evt) ----
 * NOTE: These are NOT registered here — ss_twr_fn in uwb_ss_initiator.c already
 * registers cb_txdone / cb_rxok / cb_rxto / cb_rxerr via dwt_setcallbacks and
 * port_set_dwic_isr.  The runner reuses the same irq_sem/wait_event path. */

/* =========================================================================
 * uwb_radio_ops implementation
 * ========================================================================= */

uint32_t uwb_radio_now_ms(void)
{
    return k_uptime_get_32();
}

void uwb_radio_sleep_until(uint32_t wake_ms)
{
    int32_t rem = (int32_t)(wake_ms - k_uptime_get_32());

    if (rem > 0) {
        k_sleep(K_MSEC(rem));
    }
}

/*
 * Enable DW3000 RX and wait up to timeout_ms for a frame.
 * Returns frame length on success, -ETIMEDOUT on timeout, -EIO on RX error.
 */
int uwb_radio_rx_beacon(uint8_t *buf, size_t buf_len, uint32_t timeout_ms)
{
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    irq_evt_t evt = wait_event(K_MSEC(timeout_ms));

    if (evt != EVT_RXFCG) {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return -ETIMEDOUT;
    }

    uint16_t flen = dwt_getframelength();

    if (flen == 0 || flen > (uint16_t)buf_len) {
        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
        return -EIO;
    }

    dwt_readrxdata(buf, flen, 0);
    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
    return (int)flen;
}

/*
 * Transmit a frame in a CAP mini-slot.
 * `minislot` is the zero-based mini-slot index; caller uses this as a simple
 * Aloha backoff (each slot is T_MINISLOT_MS wide).
 */
int uwb_radio_tx_cap(const uint8_t *buf, size_t len, uint8_t minislot)
{
    /* Aloha slot delay */
    k_sleep(K_MSEC((uint32_t)minislot * T_MINISLOT_MS));

    dwt_writetxdata((uint16_t)len, (uint8_t *)(uintptr_t)buf, 0);
    dwt_writetxfctrl((uint16_t)(len + 2U), 0, 0);   /* +2 for FCS */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    irq_evt_t evt = wait_event(K_MSEC(50));

    if (evt != EVT_TXFRS) {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return -EIO;
    }

    return 0;
}

/*
 * Sweep all known anchors; fill `out` with measurements; return count.
 * Shared implementation for both discover and sweep.
 */
static int anchor_sweep(struct pos_meas *out, size_t max)
{
    size_t n = 0;

    for (size_t i = 0; i < RUNNER_NUM_ANCHORS && n < max; i++) {
        float r, ax, ay;

        if (do_one_range_anchor(ANCHOR_IDS[i], &r, &ax, &ay)) {
            out[n].x       = ax;
            out[n].y       = ay;
            out[n].range_m = r;
            n++;
        }
    }
    return (int)n;
}

int uwb_radio_discover(struct pos_meas *out, size_t max)
{
    return anchor_sweep(out, max);
}

int uwb_radio_sweep(struct pos_meas *out, size_t max)
{
    return anchor_sweep(out, max);
}

/* =========================================================================
 * Runner thread
 * ========================================================================= */

static void runner_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    struct uwb_net_ctx ctx;

    uwb_net_init(&ctx, runner_eui);

    /* DW3000 callbacks and timing — must be set before the loop.
     * The initiator thread (ss_twr_fn) also calls dwt_setcallbacks; the runner
     * starts later (called from main after uwb_ss_initiator_start), so this
     * re-application is intentional: the runner owns the radio during ranging. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    while (1) {
        /* 1. Inject any pending tier change before the beacon window. */
        if (tier_pending) {
            struct uwb_net_event mev = {
                .kind     = UWB_EV_MOTION,
                .req_tier = (uint8_t)pending_tier,
            };
            uwb_net_handle(&ctx, &mev);
            tier_pending = false;
        }

        /* 2. RX beacon for up to one full superframe. */
        uint8_t beacon_buf[UWB_FRAME_MAX_LEN];
        int beacon_len = uwb_radio_rx_beacon(beacon_buf, sizeof(beacon_buf),
                                             T_SUPERFRAME_MS);
        uint32_t t0_ms = uwb_radio_now_ms();

        /* 3. Build the event. */
        struct uwb_net_event ev = { 0 };

        if (beacon_len == UWB_FRAME_LEN_BEACON &&
            uwb_frame_is_beacon(beacon_buf, (size_t)beacon_len)) {

            uint8_t  proto_ver   = 0;
            uint32_t frame_ctr   = 0;
            uint16_t slot_map[UWB_FRAME_N_CFP];
            uint8_t  n_slots     = 0;

            if (uwb_frame_parse_beacon(beacon_buf, (size_t)beacon_len,
                                       &proto_ver, &frame_ctr,
                                       slot_map, &n_slots) == 0) {
                int slot_idx = uwb_frame_beacon_find_addr(slot_map, n_slots,
                                                          ctx.short_addr);

                ev.kind          = UWB_EV_BEACON;
                ev.proto_ver     = proto_ver;
                ev.frame_counter = frame_ctr;
                ev.in_map        = (slot_idx >= 0);
                ev.map_slot      = (slot_idx >= 0) ? (uint8_t)slot_idx : 0;
            } else {
                ev.kind = UWB_EV_BEACON_MISS;
            }
        } else {
            ev.kind = UWB_EV_BEACON_MISS;
        }

        uint32_t act = uwb_net_handle(&ctx, &ev);

        /* 4. Execute actions. */

        if (act & UWB_ACT_SEND_JOIN) {
            uint8_t join_buf[UWB_FRAME_LEN_JOIN];
            int jlen = uwb_frame_join_build(join_buf, sizeof(join_buf),
                                            runner_eui,
                                            (uint8_t)ctx.req_tier);
            if (jlen > 0) {
                /* Pick a random mini-slot (0 .. N_CAP-1) for Aloha. */
                uint8_t mslot = (uint8_t)(sys_rand32_get() % N_CAP);
                uwb_radio_tx_cap(join_buf, (size_t)jlen, mslot);

                /* Listen briefly for a GRANT addressed to our EUI. */
                uint8_t grant_buf[UWB_FRAME_LEN_GRANT];
                int glen = uwb_radio_rx_beacon(grant_buf, sizeof(grant_buf),
                                               (uint32_t)N_CAP * T_MINISLOT_MS + T_GUARD_MS);

                struct uwb_net_event gev = { 0 };
                if (glen == UWB_FRAME_LEN_GRANT &&
                    uwb_frame_is_grant(grant_buf, (size_t)glen)) {

                    uint8_t  geui[UWB_FRAME_EUI_LEN];
                    uint16_t gaddr = 0;
                    uint8_t  gslot = 0, gtier = 0;
                    uint16_t glease = 0;

                    if (uwb_frame_parse_grant(grant_buf, (size_t)glen,
                                              geui, &gaddr, &gslot,
                                              &gtier, &glease) == 0 &&
                        memcmp(geui, runner_eui, UWB_FRAME_EUI_LEN) == 0) {

                        gev.kind         = UWB_EV_GRANT;
                        gev.g_short_addr = gaddr;
                        gev.g_slot       = gslot;
                        gev.g_tier       = gtier;
                        gev.g_lease      = glease;
                    } else {
                        gev.kind = UWB_EV_GRANT_MISS;
                    }
                } else {
                    gev.kind = UWB_EV_GRANT_MISS;
                }
                uwb_net_handle(&ctx, &gev);
            }
        }

        if (act & UWB_ACT_SEND_KEEPALIVE) {
            uint8_t ka_buf[UWB_FRAME_LEN_KEEPALIVE];
            int klen = uwb_frame_keepalive_build(ka_buf, sizeof(ka_buf),
                                                 ctx.short_addr,
                                                 (uint8_t)ctx.req_tier,
                                                 ctx.slot_index);
            if (klen > 0) {
                uint8_t mslot = (uint8_t)(sys_rand32_get() % N_CAP);
                uwb_radio_tx_cap(ka_buf, (size_t)klen, mslot);
            }
        }

        if (act & UWB_ACT_RUN_DISCOVER) {
            struct pos_meas meas[POS_MAX_ANCHORS];
            int n = uwb_radio_discover(meas, POS_MAX_ANCHORS);
            struct uwb_net_event dev = {
                .kind     = UWB_EV_DISCOVERED,
                .n_anchors = (uint8_t)(n > 0 ? n : 0),
            };
            uwb_net_handle(&ctx, &dev);
        }

        if (act & UWB_ACT_RUN_SWEEP) {
            /* Sleep until our CFP slot start. */
            uint32_t slot_start = t0_ms
                + T_BEACON_MS + T_GUARD_MS
                + (uint32_t)N_CAP * T_MINISLOT_MS + T_GUARD_MS
                + (uint32_t)ctx.slot_index * (T_SLOT_MS + T_GUARD_MS);

            uwb_radio_sleep_until(slot_start);

            struct pos_meas meas[POS_MAX_ANCHORS];
            int n = uwb_radio_sweep(meas, POS_MAX_ANCHORS);

            struct pos_result pos;

            if (n >= 3 && pos_solve(meas, (size_t)n, &pos)) {
                position_publish(pos.x, pos.y);
            }

            struct uwb_net_event sev = {
                .kind      = UWB_EV_SWEPT,
                .n_anchors = (uint8_t)(n > 0 ? n : 0),
            };
            uwb_net_handle(&ctx, &sev);
        }

        if (act & UWB_ACT_SLEEP) {
            uwb_radio_sleep_until(t0_ms + T_SUPERFRAME_MS);
        }

        /* UWB_ACT_TO_SCAN: FSM already transitioned; nothing extra needed. */
    }
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void uwb_net_set_tier(uwb_tier_t t)
{
    pending_tier = t;
    tier_pending = true;
}

void uwb_net_runner_start(const uint8_t eui[8])
{
    memcpy(runner_eui, eui, UWB_FRAME_EUI_LEN);

    k_thread_create(&runner_tid, runner_stack,
                    K_THREAD_STACK_SIZEOF(runner_stack),
                    runner_fn, NULL, NULL, NULL,
                    RUNNER_PRIO, 0, K_NO_WAIT);
}
