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
#include "pos_ekf.h"
#include "pos_cfg.h"
#include "pos_dbg.h"   /* TEMPORARY -- remove with the raw-range debug log */
#include "uwb_radio_owner.h"
#include "port.h"
#include "deca_device_api.h"
#include "phy_config.h"
#include "cal.h"
#include "rx_stats.h"
#include "beacon_track_core.h"
#include "beacon_sched_core.h"
#include "scan_backoff_core.h"
#include "tag_alert.h"
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <string.h>
#include <errno.h>

/* Interrupt mask for beacon/grant RX; mirrors INT_RX_PHASE in uwb_ss_initiator.c. */
#define INT_RX_PHASE  (DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK | \
                       DWT_INT_RXPTO_BIT_MASK  | SYS_STATUS_ALL_RX_ERR)

/* ---- v1 timing constants (protocol contract §2.1) ---- */
#define T_SUPERFRAME_MS   200u
#define DW_WAKE_GUARD_MS    5u   /* wake the DW3000 this long before the next beacon
                                  * (covers >=500 us WAKEUP pulse + 2 ms settle +
                                  * checkidlerc + dwt_restoreconfig) */

/* Narrow beacon window (Spec 2) — all adjustable. GUARD ≈ 2× the measured
 * ~±2.6 ms arrival jitter; the wake margin reuses DW_WAKE_GUARD_MS. */
#define BT_GUARD_MS    5u
#define BT_WARMUP_N    8u
#define BT_EMA_SHIFT   3u
#define T_BEACON_MS         2u   /* ~1.5 ms, round up */
#define T_GUARD_MS          1u   /* ~0.5 ms, round up */
#define T_SLOT_MS          24u   /* sized to the measured worst sweep (22 ms n=4) + guard; no slot overlap */
#define T_MINISLOT_MS       2u   /* ~1.5 ms, round up */
#define N_CAP               UWB_FRAME_N_CAP   /* 4 */
#define N_CFP               UWB_FRAME_N_CFP   /* 11 */

/* ---- DW3000 RX/TX timing (re-used from initiator) ---- */
#define POLL_TX_TO_RESP_RX_DLY_UUS   1000U
#define RESP_RX_TIMEOUT_UUS          2000U
#define PRE_TIMEOUT                   128U

/* Settle time between consecutive anchor polls in a sweep. */
#define INTER_ANCHOR_DELAY_US        10U

#define DISCOVERY_WINDOW_MS      15U   /* total RX collection window; covers anchor_id=3 (12.5 ms slot) */

/* Rung of the coverage ladder at and above which discovery is suppressed
 * entirely (design §5.3): run_discovery() broadcasts and then holds a 15 ms
 * window open for anchors that are, by definition, not there. */
#define SCAN_QUIET_RUNG           2U

/* ---- Anchor-pool / CIR selection ---- */
#define ANCHOR_POOL_MAX           6
#define ANCHOR_SELECT_MAX         4
/* The minimum that makes a round "enough" is UWB_NET_MIN_ANCHORS (uwb_net.h),
 * the same symbol the FSM compares n_anchors against. A private copy here is
 * what let this file's sweep gate and the FSM disagree about whether the tag
 * was making progress. */
#define EMA_ALPHA              0.3f
#define EMA_DECAY              0.5f
#define CIR_QUALITY_WEIGHT     1.0f

typedef struct {
    uint8_t id;
    float   ema_score;
    bool    valid;
    bool    seen;   /* transient: set during discovery, cleared before each round */
} anchor_entry_t;

static anchor_entry_t anchor_pool[ANCHOR_POOL_MAX];
static uint8_t        selected[ANCHOR_SELECT_MAX];
static uint8_t        n_selected;
/* Discover-vs-sweep decision and its state. Pure and host-tested in
 * uwb_net.c/tests/uwb_net -- it used to be three file-scope variables and an
 * inline condition here, and in that shape it latched the tag out of ranging
 * for good after a single short sweep. See struct uwb_sweep_gate. */
static struct uwb_sweep_gate sweep_gate;

/* The debug log indexes by SELECTED slot, not by the compacted pos_meas array,
 * because a slot's identity does not stop existing when its anchor misses one
 * sweep -- see the caller contract on pos_dbg_sweep(). Coordinates persist per
 * slot and are cleared only when the slot is reassigned to a different anchor.
 * TEMPORARY, with the debug log. */
BUILD_ASSERT(ANCHOR_SELECT_MAX == POS_MAX_ANCHORS,
             "the per-slot debug arrays are sized by POS_MAX_ANCHORS");
static uint8_t  sweep_aid[POS_MAX_ANCHORS];
static int16_t  sweep_ax_cm[POS_MAX_ANCHORS];
static int16_t  sweep_ay_cm[POS_MAX_ANCHORS];
static int16_t  sweep_r_cm[POS_MAX_ANCHORS];
static uint8_t  sweep_mask;

/* ---- Position filter ------------------------------------------------------
 * Tightly-coupled EKF over the raw ranges. Owned by the runner thread and
 * touched from nowhere else, so it needs no lock.
 * See spec/2026-08-22-position-filtering-design.md. */
static struct pos_ekf     ekf;
static struct pos_ekf_cfg ekf_cfg;
static uint32_t           last_fix_ms;
static bool               have_last_fix;

/* ---- Runner thread parameters ---- */
#define RUNNER_PRIO    2
#define RUNNER_STACK   2048

K_THREAD_STACK_DEFINE(runner_stack, RUNNER_STACK);
static struct k_thread runner_tid;

/* ---- Tier-change request (set from motion / ISR context) ---- */
static volatile uwb_tier_t  pending_tier;
static volatile bool        tier_pending;

/* Raw motion state from the LIS2HH12 INT1 handler. Kept raw rather than
 * pre-mapped to a tier because uwb_net_tier_filter() owns the hysteresis, and
 * it needs the edge, not the conclusion. */
static volatile bool        motion_moving;
static volatile bool        motion_edge;

/* Beacon scheduler + coverage ladder. File-static rather than locals in
 * runner_fn so the `pwr sched` / `pwr scan` diagnostics can read them. Written
 * only by the runner thread and read from the BT RX thread; every field read
 * out is a single aligned 32-bit or 8-bit word and the values are advisory, so
 * no lock is taken. */
static struct beacon_sched  sched;
static struct scan_backoff  backoff;

/* ---- EUI stored at start ---- */
static uint8_t runner_eui[UWB_FRAME_EUI_LEN];

/* ---- Layer-1 power saving flag ---- */
static volatile bool dw_sleep_enabled = true;

void uwb_radio_set_sleep_enabled(bool en) { dw_sleep_enabled = en; }
bool uwb_radio_sleep_enabled(void)        { return dw_sleep_enabled; }

/* ---- Interruptible sleep / wake signal (design §4.5) ---- */
static K_SEM_DEFINE(runner_wake, 0, 1);
static volatile bool force_full_window;

void uwb_net_runner_wake(void)
{
    /* Two parts, and both are needed. The semaphore shortens a sleep that is
     * in progress right now; the sticky flag carries the request across a give
     * that landed while an exchange was in flight (where there was no sleep to
     * shorten and the drain at the top of the loop discards the give). Without
     * the flag, a HELP raised mid-sweep would still wait a whole skip for its
     * beacon. */
    force_full_window = true;
    k_sem_give(&runner_wake);
}


/* ---- DW3000 ISR callbacks (mirror the initiator's; same irq_sem/last_evt) ----
 * NOTE: These are NOT registered here — ss_twr_fn in uwb_ss_initiator.c already
 * registers cb_txdone / cb_rxok / cb_rxto / cb_rxerr via dwt_setcallbacks and
 * port_set_dwic_isr.  The runner reuses the same irq_sem/wait_event path. */

/* =========================================================================
 * Anchor pool helpers
 * ========================================================================= */

static void anchor_pool_update(uint8_t id, int32_t cir_power, uint16_t cir_quality)
{
    float score = (float)cir_power + CIR_QUALITY_WEIGHT * (float)cir_quality;

    /* Update existing entry */
    for (int i = 0; i < ANCHOR_POOL_MAX; i++) {
        if (anchor_pool[i].valid && anchor_pool[i].id == id) {
            anchor_pool[i].ema_score = EMA_ALPHA * score +
                                       (1.0f - EMA_ALPHA) * anchor_pool[i].ema_score;
            anchor_pool[i].seen = true;
            return;
        }
    }
    /* New anchor: find empty slot */
    for (int i = 0; i < ANCHOR_POOL_MAX; i++) {
        if (!anchor_pool[i].valid) {
            anchor_pool[i].id        = id;
            anchor_pool[i].ema_score = score;
            anchor_pool[i].valid     = true;
            anchor_pool[i].seen      = true;
            return;
        }
    }
    /* Pool full: evict lowest-scoring entry */
    int worst = 0;
    for (int i = 1; i < ANCHOR_POOL_MAX; i++) {
        if (anchor_pool[i].ema_score < anchor_pool[worst].ema_score) {
            worst = i;
        }
    }
    anchor_pool[worst].id        = id;
    anchor_pool[worst].ema_score = score;
    anchor_pool[worst].seen      = true;
}

static void anchor_pool_decay_missed(void)
{
    for (int i = 0; i < ANCHOR_POOL_MAX; i++) {
        if (anchor_pool[i].valid && !anchor_pool[i].seen) {
            anchor_pool[i].ema_score *= EMA_DECAY;
        }
    }
}

static void anchor_pool_rebuild_selected(void)
{
    /* Collect valid pool indices */
    uint8_t order[ANCHOR_POOL_MAX];
    uint8_t count = 0;

    for (int i = 0; i < ANCHOR_POOL_MAX; i++) {
        if (anchor_pool[i].valid) {
            order[count++] = (uint8_t)i;
        }
    }

    /* Insertion sort descending by ema_score (pool ≤ 6, O(n²) is fine) */
    for (int i = 1; i < count; i++) {
        uint8_t key = order[i];
        int j = i - 1;
        while (j >= 0 &&
               anchor_pool[order[j]].ema_score < anchor_pool[key].ema_score) {
            order[j + 1] = order[j];
            j--;
        }
        order[j + 1] = key;
    }

    n_selected = (count < ANCHOR_SELECT_MAX) ? count : ANCHOR_SELECT_MAX;
    for (int i = 0; i < n_selected; i++) {
        selected[i] = anchor_pool[order[i]].id;
    }
}

/* =========================================================================
 * uwb_radio_ops implementation
 * ========================================================================= */

uint32_t uwb_radio_now_ms(void)
{
    return k_uptime_get_32();
}

bool uwb_radio_sleep_until(uint32_t wake_ms)
{
    int32_t rem = (int32_t)(wake_ms - k_uptime_get_32());

    if (rem <= 0) {
        return false;
    }
    /* Wait on the wake signal rather than k_sleep(): once the runner skips
     * whole superframes, a plain sleep would make a motion edge or a HELP
     * press wait out the entire skip -- up to 60 s at the IDLE tier. For an
     * emergency button that is a defect, not a latency figure (design §4.5). */
    return k_sem_take(&runner_wake, K_MSEC(rem)) == 0;
}

void uwb_radio_sleep_until_strict(uint32_t wake_ms)
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
    /* Beacon window: no hardware RX/preamble timeout; the kernel timeout in
     * wait_event() is the only gate.  ss_twr_fn leaves RESP_RX_TIMEOUT_UUS
     * (2 ms) armed, which would fire before any beacon arrives. */
    dwt_setrxtimeout(0);
    dwt_setpreambledetecttimeout(0);
    dwt_setinterrupt(INT_RX_PHASE, 0, DWT_ENABLE_INT_ONLY);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    irq_evt_t evt = wait_event(K_MSEC(timeout_ms));

    if (evt != EVT_RXFCG) {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return -ETIMEDOUT;
    }

    uint16_t flen = dwt_getframelength();   /* includes 2-byte FCS */

    if (flen <= FCS_LEN || (flen - FCS_LEN) > (uint16_t)buf_len) {
        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
        return -EIO;
    }

    uint16_t dlen = (uint16_t)(flen - FCS_LEN);
    dwt_readrxdata(buf, dlen, 0);
    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
    return (int)dlen;
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
    dwt_setinterrupt(DWT_INT_TXFRS_BIT_MASK, 0, DWT_ENABLE_INT_ONLY);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    irq_evt_t evt = wait_event(K_MSEC(50));

    if (evt != EVT_TXFRS) {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return -EIO;
    }

    return 0;
}

/*
 * Broadcast a DISCOVERY frame and collect DISCOVERY_RESPONSE frames for
 * DISCOVERY_WINDOW_MS.  Updates anchor_pool EMA scores and rebuilds selected[].
 * src_addr: tag's current short address (UWB_ADDR_UNASSOC before joining).
 * Note: anchor ID is taken from the low byte of src_addr in each response.
 * FUTURE WORK: anti-collision — anchors currently respond with Aloha;
 *   planned fix is per-anchor mini-slot (id × T_MINISLOT_MS).
 */
static int run_discovery(uint16_t src_addr)
{
    /* Reset seen flags for decay tracking */
    for (int i = 0; i < ANCHOR_POOL_MAX; i++) {
        anchor_pool[i].seen = false;
    }

    /* Broadcast DISCOVERY frame */
    uint8_t disc_buf[UWB_FRAME_LEN_DISC];
    int dlen = uwb_frame_discovery_build(disc_buf, sizeof(disc_buf), src_addr, 0u);

    if (dlen > 0) {
        dwt_writetxdata((uint16_t)dlen, disc_buf, 0);
        dwt_writetxfctrl((uint16_t)(dlen + 2U), 0, 0);
        dwt_setinterrupt(DWT_INT_TXFRS_BIT_MASK, 0, DWT_ENABLE_INT_ONLY);
        dwt_starttx(DWT_START_TX_IMMEDIATE);
        irq_evt_t evt = wait_event(K_MSEC(10));
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK |
                             SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        if (evt != EVT_TXFRS) {
            goto finish;
        }
    }

    /* Collect DISCOVERY_RESPONSE frames within DISCOVERY_WINDOW_MS.
     * uwb_radio_rx_beacon() disables HW timeouts (setrxtimeout(0)) — correct
     * for this open-ended collection window; anchor_sweep() restores them.
     *
     * Each call is armed for the FULL remaining time, not a fixed slice --
     * mirrors the beacon RX loop a few hundred lines below, which re-arms
     * only after actually consuming a frame. Slicing into fixed-size chunks
     * (the previous DISCOVERY_RX_SLOT_MS=3 ms design) re-arms the DW3000
     * receiver on a fixed timer regardless of whether a frame arrived, and
     * each re-arm (dwt_setrxtimeout/setpreambledetecttimeout/setinterrupt/
     * rxenable, several SPI writes) leaves a brief gap where the receiver is
     * not listening. Anchor id 2 (short address 0x0003) replies at exactly
     * DISC_BASE_UUS + 2*DISC_SLOT_UUS = 9.0 ms -- precisely on a 3 ms slice
     * boundary -- so its response was the one structurally at risk of being
     * clipped by that gap, every single discovery round. Arming for the
     * full remaining time each iteration means the receiver only goes
     * through a re-arm after it has actually consumed a frame, so no
     * anchor's fixed response delay can alias against a polling boundary. */
    {
        uint32_t t_end = k_uptime_get_32() + DISCOVERY_WINDOW_MS;
        uint8_t  resp_buf[UWB_FRAME_LEN_RESP];

        for (;;) {
            int32_t rem = (int32_t)(t_end - k_uptime_get_32());
            if (rem <= 0) {
                break;
            }
            int rlen = uwb_radio_rx_beacon(resp_buf, sizeof(resp_buf), (uint32_t)rem);
            if (rlen > 0 && uwb_frame_is_response(resp_buf, (size_t)rlen)) {
                uint16_t src  = 0;
                int32_t  cir_p = 0;
                uint16_t cir_q = 0;
                if (uwb_frame_parse_discovery_response(resp_buf, (size_t)rlen,
                                                        &src, &cir_p, &cir_q) == 0) {
                    anchor_pool_update((uint8_t)src, cir_p, cir_q);
                }
            }
        }
    }

finish:
    anchor_pool_decay_missed();
    anchor_pool_rebuild_selected();
    return n_selected;
}

/*
 * Sweep all known anchors; fill `out` with measurements; return count.
 * Shared implementation for both discover and sweep.
 */
static int anchor_sweep(struct pos_meas *out, size_t max)
{
    /* uwb_radio_rx_beacon() disables both HW timeouts (setrxtimeout(0),
     * setpreambledetecttimeout(0)) for open-ended beacon/discovery listening.
     * Those settings persist, so restore them here before TWR. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    size_t n = 0;

    /* One dz for every anchor: all anchors are ceiling-mounted at one height,
     * so v1 carries a single tag-side pair of constants rather than per-anchor
     * z in the E1 frame (which would need anchor firmware). Read once per
     * sweep -- it can change under us from the BT RX thread via `pos z`, and a
     * fix built from two different dz values would be incoherent. */
    const float dz = pos_cfg_dz_m();

    /* Reset the per-slot debug snapshot. Coordinates survive a missed sweep
     * but not a slot reassignment. */
    for (size_t i = 0; i < POS_MAX_ANCHORS; i++) {
        if (i >= n_selected) {
            sweep_aid[i]   = POS_DBG_AID_NONE;
            sweep_ax_cm[i] = 0;
            sweep_ay_cm[i] = 0;
        } else if (sweep_aid[i] != selected[i]) {
            sweep_aid[i]   = selected[i];
            sweep_ax_cm[i] = 0;
            sweep_ay_cm[i] = 0;
        }
        sweep_r_cm[i] = 0;
    }
    sweep_mask = 0;

    for (size_t i = 0; i < n_selected && n < max; i++) {
        if (i > 0) {
            k_sleep(K_USEC(INTER_ANCHOR_DELAY_US));
        }

        float r, ax, ay;
        bool ok = do_one_range_anchor(selected[i], &r, &ax, &ay);

        if (ok) {
            out[n].x       = ax;
            out[n].y       = ay;
            out[n].dz      = dz;
            out[n].range_m = r;
            n++;

            sweep_ax_cm[i] = pos_dbg_m_to_cm(ax);
            sweep_ay_cm[i] = pos_dbg_m_to_cm(ay);
            sweep_r_cm[i]  = pos_dbg_m_to_cm(r);
            sweep_mask    |= (uint8_t)(1u << i);
        }
    }

    return (int)n;
}

int uwb_radio_discover(struct pos_meas *out, size_t max)
{
    ARG_UNUSED(out); ARG_UNUSED(max);
    return run_discovery(UWB_ADDR_UNASSOC);
}

int uwb_radio_sweep(struct pos_meas *out, size_t max)
{
    return anchor_sweep(out, max);
}

/* =========================================================================
 * DW3000 SLEEP / WAKE helpers (used by runner only; gated by dw_sleep_enabled)
 * ========================================================================= */

static void dw_enter_sleep(void)
{
    decaIrqStatus_t s = decamutexon();
    /* mode: DWT_PGFCAL re-enables the receiver on wake (per the DW3 SDK
     *       tx_sleep examples — "added to make sure receiver is re-enabled on
     *       wake").  Without it the RX is dead after wake → RESCAN miss.
     * wake: DWT_PRES_SLEEP preserves SLEEP_EN; wake on WAKEUP pin and CS;
     *       DWT_SLEEP selects SLEEP (config retained) over deep sleep. */
    dwt_configuresleep(DWT_CONFIG | DWT_PGFCAL,
                       DWT_PRES_SLEEP | DWT_WAKE_CSN | DWT_WAKE_WUP |
                       DWT_SLEEP | DWT_SLP_EN);
    /* DWT_DW_IDLE_RC: clear auto-INIT2IDLE so the chip parks in the stable
     * IDLE_RC state on wake instead of auto-transitioning RC->PLL.  This makes
     * dwt_checkidlerc() deterministic and lets dwt_restoreconfig() run on a
     * settled clock; the auto RC->PLL race was corrupting the RX config on an
     * occasional wake, killing the receiver permanently (RESCAN miss after a
     * few minutes).  The next dwt_rxenable() brings the device up to IDLE_PLL. */
    dwt_entersleep(DWT_DW_IDLE_RC);
    decamutexoff(s);
}

static void dw_wake(void)
{
    wakeup_device_with_io();   /* WAKEUP-pin pulse (>=500 us); no mutex needed */
    k_msleep(2);               /* settle: INIT_RC -> IDLE_RC (ref: Sleep(2)) */

    decaIrqStatus_t s = decamutexon();
    /* Make sure the device is in IDLE_RC before touching config. */
    for (int i = 0; i < 50 && !dwt_checkidlerc(); i++) {
        k_busy_wait(100);
    }
    /* Restore the configuration not auto-restored from AON.  In driver 6.0.7
     * this is the equivalent of the newer SDK's dwt_restore_common() +
     * dwt_restore_txrx().  Paired with DWT_PGFCAL in dwt_configuresleep, this
     * re-enables the receiver on wake.  SPI stays at the operating (fast) rate,
     * as in the DW3 SDK tx_sleep examples. */
    dwt_restoreconfig();

    /* Re-arm the external LNA. dwt_setlnapamode() configures DW3000 GPIOs as
     * EXTRXE outputs, and that GPIO mode is not part of what AON restores or
     * what dwt_restoreconfig() covers -- so without this the front end is live
     * only until the first dw_enter_sleep(), i.e. dead for the entire session
     * in the very mode we ship. Cheap to re-apply unconditionally; the
     * alternative is depending on undocumented behaviour of a precompiled
     * driver. RX-only front end: no PA on this board. Must mirror the
     * init-time call in uwb.c exactly. */
    dwt_setlnapamode(DWT_LNA_ENABLE);

    /* Re-apply the runner-owned TWR params + antenna delays. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    uint16_t tx, rx;
    cal_get_ant_dly(&tx, &rx);
    dwt_settxantennadelay(tx);
    dwt_setrxantennadelay(rx);
    decamutexoff(s);
}

/* =========================================================================
 * Runner thread
 * ========================================================================= */

static void runner_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    struct uwb_net_ctx ctx;

    uwb_net_init(&ctx, runner_eui);

    /* beacon_track and beacon_sched are complementary, not alternatives:
     * beacon_track runs the ACQUIRING->TRACKING acquisition FSM and gets the
     * tag locked on a full-window listen; beacon_sched then keeps it locked
     * across skips of tens of superframes using a long-baseline period
     * estimate. Merging them would put an acquisition FSM and an extrapolator
     * with completely different failure modes behind one set of state. */
    struct beacon_track bt;
    beacon_track_reset(&bt, T_SUPERFRAME_MS, BT_GUARD_MS, BT_WARMUP_N, BT_EMA_SHIFT);
    beacon_sched_reset(&sched, T_SUPERFRAME_MS);
    scan_backoff_reset(&backoff);
    uwb_sweep_gate_init(&sweep_gate);
    bool radio_asleep = false;   /* tracks whether the DW3000 is in deep sleep */

    /* DW3000 callbacks and timing — must be set before the loop.
     * The initiator thread (ss_twr_fn) also calls dwt_setcallbacks; the runner
     * starts later (called from main after uwb_ss_initiator_start), so this
     * re-application is intentional: the runner owns the radio during ranging. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    while (1) {
        /* Drain any wake given while the previous superframe's exchange was in
         * flight. It could not shorten anything then, and left pending it
         * would abort an unrelated skip an iteration or two later. The intent
         * is not lost: force_full_window is sticky and tier_pending latches
         * the motion edge separately. */
        k_sem_reset(&runner_wake);

        /* 0. Offer the radio to a waiting claimant. This is the only point in
         * the superframe with no exchange in flight, and the handover contract
         * says we leave the radio awake and idle. */
        if (uwb_radio_request_pending()) {
            if (radio_asleep) { dw_wake(); radio_asleep = false; }
            dwt_forcetrxoff();
            /* dwt_forcetrxoff() can leave RX-abort/error bits asserted. Clear
             * them here or the *other* thread's first wait_event() would see
             * them via port_CheckEXT_IRQ() -> process_deca_irq() and dispatch a
             * spurious EVT_RXERR against an exchange it never started. */
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

            /* Blocks until the claimant releases -- unless the claim was
             * withdrawn in between, in which case it returns false at once and
             * the radio never left us. */
            if (uwb_radio_yield()) {
                /* Seconds may have passed and the antenna delays may have
                 * changed. Re-establish everything the claimant could have
                 * disturbed. */
                dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
                dwt_setpreambledetecttimeout(PRE_TIMEOUT);

                uint16_t rtx, rrx;
                cal_get_ant_dly(&rtx, &rrx);
                dwt_settxantennadelay(rtx);
                dwt_setrxantennadelay(rrx);

                /* The arrival prediction is stale after that long off the air.
                 * A narrow window aimed at a dead instant would miss repeatedly
                 * and trip RESCAN, so go back to ACQUIRING. That costs
                 * BT_WARMUP_N superframes of full-window RX, which is why it is
                 * gated on an actual handover. */
                beacon_track_reset(&bt, T_SUPERFRAME_MS, BT_GUARD_MS,
                                   BT_WARMUP_N, BT_EMA_SHIFT);
                /* Same argument, only stronger for the long-baseline
                 * estimate: its phase reference is seconds old and its
                 * baseline now spans a gap in which the tag heard nothing. */
                beacon_sched_reset(&sched, T_SUPERFRAME_MS);
            }
        }

        /* 1. Tier. An explicit uwb_net_set_tier() still overrides directly;
         * motion goes through the §6.2 hysteresis filter, which must be
         * evaluated every superframe and not only on an edge -- the
         * SLOW -> IDLE demotion is a timeout, not an edge. */
        if (tier_pending) {
            struct uwb_net_event mev = {
                .kind     = UWB_EV_MOTION,
                .req_tier = (uint8_t)pending_tier,
            };
            uwb_net_handle(&ctx, &mev);
            tier_pending = false;
        }

        if (motion_edge) {
            motion_edge = false;
            if (motion_moving) {
                /* A tag leaving or entering a building is always in motion, so
                 * an activity edge is a far better predictor of a coverage
                 * change than elapsed time is (design §5.2). */
                scan_backoff_motion(&backoff);
            }
        }

        {
            uwb_tier_t want = uwb_net_tier_filter(&ctx, motion_moving,
                                                  uwb_radio_now_ms());
            if (want != ctx.tier) {
                struct uwb_net_event tev = {
                    .kind     = UWB_EV_MOTION,
                    .req_tier = (uint8_t)want,
                };
                uwb_net_handle(&ctx, &tev);
                /* beacon_sched is deliberately NOT reset here, in either
                 * direction. Its period estimate and phase reference are
                 * properties of the gateway's clock, not of our tier, and a
                 * longer skip is exactly when the long baseline is worth most
                 * -- throwing it away would force the widest window at the
                 * moment the narrowest is needed. A short baseline already
                 * widens its own window, so growing the skip is safe with
                 * whatever estimate exists. */
            }
        }

        /* 2. Listen for THE beacon, discarding cross-traffic.  In TRACKING the
         * beacon tracker predicts arrival and we sleep the radio until just
         * before it, then arm a short window; in ACQUIRING we listen the whole
         * superframe (as before) to (re)lock.  uwb_radio_rx_beacon() returns on
         * the FIRST frame of any type, so the re-arm loop keeps waiting through
         * cross-traffic until the real beacon or the deadline. */
        uint8_t beacon_buf[UWB_FRAME_MAX_LEN];
        int beacon_len = -ETIMEDOUT;

        bool     bt_narrow;
        uint32_t bt_arm_ms, bt_window_ms;
        uint32_t eff_skip = 1u;      /* superframes this window skips over */
        beacon_track_plan(&bt, &bt_narrow, &bt_arm_ms, &bt_window_ms);

        bool scanning = (ctx.state == UWB_ST_SCAN);

        struct uwb_tier_params tp;
        uwb_net_get_tier_params(ctx.tier, &tp);

        if (scanning) {
            /* Out of coverage there is no prediction to aim at: the probe is
             * always a full superframe and the ladder owns how often it runs. */
            bt_narrow = false;
        } else if (bt_narrow && beacon_sched_have_ref(&sched)) {
            /* Locked. Hand the prediction to the long-baseline scheduler,
             * which may skip whole superframes; beacon_track's single-
             * superframe plan is only used until that reference exists. */
            beacon_sched_plan(&sched, tp.listen_skip,
                              &bt_arm_ms, &bt_window_ms, &eff_skip);
        }

        /* A wake request (motion edge or HELP press) buys one full-window
         * listen: the prediction it interrupted is no longer the one we want
         * to aim at, and a wasted full-window re-sync is the correct price for
         * an emergency press. */
        if (force_full_window) {
            force_full_window = false;
            bt_narrow = false;
            eff_skip  = 1u;
        }

        if (scanning) {
            /* Sleep out the ladder's probe interval with the radio still in
             * deep sleep -- but only if it actually is asleep, so the first
             * probe after boot (and every probe in bench mode, where
             * dw_sleep_enabled is false and the radio never sleeps) happens
             * immediately instead of 10 s late. A wake signal here cuts the
             * wait short, which is exactly what a motion edge or a HELP press
             * should do out of coverage. */
            uint32_t probe_ms = (dw_sleep_enabled && radio_asleep)
                              ? scan_backoff_next_ms(&backoff) : 0u;

            if (probe_ms > DW_WAKE_GUARD_MS) {
                (void)uwb_radio_sleep_until(uwb_radio_now_ms()
                                            + probe_ms - DW_WAKE_GUARD_MS);
            }
        } else if (bt_narrow) {
            /* Sleep (radio stays in deep sleep) until just before the predicted
             * beacon, leaving DW_WAKE_GUARD_MS for the wake to settle. A wake
             * signal here abandons the narrow window: the instant we were
             * aiming at is no longer the one we are waiting for. */
            if ((int32_t)(bt_arm_ms - DW_WAKE_GUARD_MS - uwb_radio_now_ms()) > 0) {
                if (uwb_radio_sleep_until(bt_arm_ms - DW_WAKE_GUARD_MS)) {
                    force_full_window = false;
                    bt_narrow = false;
                    eff_skip  = 1u;
                }
            }
        }

        if (radio_asleep) { dw_wake(); radio_asleep = false; }

        uint32_t bcn_deadline = bt_narrow
            ? (bt_arm_ms + bt_window_ms)
            : (uwb_radio_now_ms() + T_SUPERFRAME_MS + T_BEACON_MS);

        uint32_t bcn_rx_ms = 0;

        rx_stats_arm();
        for (;;) {
            int32_t rem = (int32_t)(bcn_deadline - uwb_radio_now_ms());
            if (rem <= 0) {
                break;
            }
            int len = uwb_radio_rx_beacon(beacon_buf, sizeof(beacon_buf),
                                          (uint32_t)rem);
            if (len == UWB_FRAME_LEN_BEACON &&
                uwb_frame_is_beacon(beacon_buf, (size_t)len)) {
                beacon_len = len;
                bcn_rx_ms = uwb_radio_now_ms();
                break;
            }
            /* non-beacon frame or rx timeout/error: keep waiting for the beacon */
        }
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
                rx_stats_beacon();
                /* The gateway's frame counter is what makes a long skip
                 * self-correcting: it tells the tag exactly how many
                 * superframes actually elapsed, so the period estimate is
                 * validated against ground truth at every wake rather than
                 * left to drift. */
                beacon_sched_observe(&sched, bcn_rx_ms, frame_ctr);
                if (eff_skip <= 1u) {
                    beacon_track_beacon(&bt, bcn_rx_ms);
                }
                /* Across a skip the gap to the previous arrival is K x P, and
                 * beacon_track's EMA assumes *consecutive* superframes -- one
                 * skipped window would push its period estimate to thousands
                 * of milliseconds. It is not used for prediction here
                 * (beacon_sched owns that once it has a reference); its only
                 * remaining job is the ACQUIRING->TRACKING lock, which this
                 * beacon has already satisfied. So leave it untouched rather
                 * than poison it. A miss clears both, and the re-lock then
                 * happens at skip 1 where the EMA is valid again. */
            } else {
                ev.kind = UWB_EV_BEACON_MISS;
                rx_stats_miss();
                beacon_track_miss(&bt);
                beacon_sched_miss(&sched);
            }
        } else {
            ev.kind = UWB_EV_BEACON_MISS;
            rx_stats_miss();
            beacon_track_miss(&bt);
            beacon_sched_miss(&sched);
        }

        /* The alert is orthogonal to beacon sync: fill it in on every event,
         * BEACON or BEACON_MISS alike, before handing the event to the FSM.
         * uwb_net_handle()'s emission rule decides whether this superframe's
         * state actually earns UWB_ACT_SEND_ALERT. */
        ev.alert_pending = tag_alert_active();

        /* Coverage ladder. The alert pin goes on first so the failure below is
         * a no-op while a HELP stands -- an emergency is exactly when the tag
         * should be trying hardest to find a network. A beacon in *any* state
         * means we are in coverage, so the reset is not conditioned on SCAN;
         * only the climb is, since a miss inside RANGING is already handled by
         * UWB_NET_MISS_MAX. */
        scan_backoff_alert(&backoff, ev.alert_pending);
        if (ev.kind == UWB_EV_BEACON) {
            scan_backoff_reset(&backoff);
        } else if (scanning) {
            scan_backoff_fail(&backoff);
        }

        uint32_t act = uwb_net_handle(&ctx, &ev);

        /* Suppress ranging while the antenna delays are uncalibrated. Seat
         * maintenance survives, so the tag stays on the network and is ready
         * for a `cal <mm>` command. */
        act = uwb_net_gate_actions(act, cal_is_valid());

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
                /* Defensive: this re-OR happens after the first gate, so any
                 * action it introduces would bypass it. Today a GRANT only
                 * yields UWB_ACT_RUN_DISCOVER, which the gate deliberately
                 * lets through — but the gate belongs on every path that can
                 * add actions, not only on the ones that currently need it. */
                act |= uwb_net_gate_actions(uwb_net_handle(&ctx, &gev), cal_is_valid());
            }
        }

        if (act & UWB_ACT_SEND_KEEPALIVE) {
            uint8_t ka_buf[UWB_FRAME_LEN_KEEPALIVE];
            int klen = uwb_frame_keepalive_build(ka_buf, sizeof(ka_buf),
                                                 ctx.short_addr,
                                                 (uint8_t)ctx.req_tier,
                                                 /* SEAT id: a stable identity
                                                  * for the gateway to check,
                                                  * not this superframe's slot. */
                                                 ctx.seat_id);
            if (klen > 0) {
                uint8_t mslot = (uint8_t)(sys_rand32_get() % N_CAP);
                uwb_radio_tx_cap(ka_buf, (size_t)klen, mslot);
            }
        }

        /* Placed after JOIN/KEEPALIVE and before RUN_DISCOVER/RUN_SWEEP: the
         * action word is a bitmask evaluated top to bottom in one pass, so
         * position in this function *is* the priority. This keeps a pending
         * alert from ever delaying the sweep's slot-start deadline (and vice
         * versa), and ensures it runs before UWB_ACT_SLEEP -- an alert that
         * loses the race to sleep would wait a whole superframe to go out.
         * No uwb_radio_owner claim and no new wait_event() caller: the runner
         * already owns the radio here, exactly like position_publish(). */
        if (act & UWB_ACT_SEND_ALERT) {
            struct uwb_alert a;
            uint32_t now = uwb_radio_now_ms();

            if (tag_alert_frame_due(&a, ctx.short_addr, now)) {
                uint8_t abuf[UWB_FRAME_LEN_ALERT];
                int alen = uwb_frame_alert_build(abuf, sizeof(abuf), ctx.short_addr, &a);

                if (alen > 0) {
                    uint8_t mslot = (uint8_t)(sys_rand32_get() % N_CAP);
                    uwb_radio_tx_cap(abuf, (size_t)alen, mslot);
                    tag_alert_sent(now);
                }
            }
        }

        /* Discovery is suppressed while the coverage ladder has climbed:
         * run_discovery() broadcasts and then holds a 15 ms window open for
         * anchors that, at rung >= 2, have been absent for two minutes
         * (design §5.3). Belt and braces in practice -- UWB_ACT_RUN_DISCOVER
         * is only emitted from DISCOVER/RANGING, and any beacon resets the
         * ladder -- but the suppression is cheap and the alternative is a
         * broadcast into an empty room. */
        if (scan_backoff_rung(&backoff) >= SCAN_QUIET_RUNG) {
            act &= ~UWB_ACT_RUN_DISCOVER;
        }

        if (act & UWB_ACT_RUN_DISCOVER) {
            int n = run_discovery(ctx.short_addr);

            uwb_sweep_gate_discovered(&sweep_gate, uwb_radio_now_ms(),
                                      (uint8_t)(n > 0 ? n : 0));

            struct uwb_net_event dev = {
                .kind      = UWB_EV_DISCOVERED,
                .n_anchors = (uint8_t)(n > 0 ? n : 0),
            };
            uwb_net_handle(&ctx, &dev);
        }

        if (act & UWB_ACT_RUN_SWEEP) {
            if (uwb_sweep_gate_rediscover_due(&sweep_gate, uwb_radio_now_ms())) {
                /* Re-discovery replaces the sweep this cycle; no position fix.
                 * Do NOT send UWB_EV_DISCOVERED to the FSM — that event is only
                 * valid in UWB_ST_DISCOVER state.
                 *
                 * Recording the round through the gate is load-bearing, not
                 * bookkeeping. This branch used to update the timestamp only,
                 * leaving the sweep count it is itself gated on untouched, so
                 * once that count went short it stayed short: re-discovery every
                 * superframe forever, no sweep, hence no UWB_EV_SWEPT, hence no
                 * fall back to UWB_ST_DISCOVER, hence no way to refresh the
                 * count. An earlier version of this comment claimed recovery
                 * came through "the next sweep" returning n_anchors < 3 -- there
                 * was no next sweep; this branch had already taken its place. */
                int n = run_discovery(ctx.short_addr);

                uwb_sweep_gate_discovered(&sweep_gate, uwb_radio_now_ms(),
                                          (uint8_t)(n > 0 ? n : 0));
            } else {
                /* Sleep until our CFP slot start. */
                uint32_t slot_start = t0_ms
                    + T_BEACON_MS + T_GUARD_MS
                    + (uint32_t)N_CAP * T_MINISLOT_MS + T_GUARD_MS
                    /* tx_slot, NOT seat_id. This is a TDMA slot offset, and
                     * seat ids run to GW_MAX_SEATS (128) -- using one here
                     * would place the poll far outside the superframe. */
                    + (uint32_t)ctx.tx_slot * (T_SLOT_MS + T_GUARD_MS);

                /* Strict: this is the tag's TDMA slot boundary. Returning
                 * early would put the poll outside the slot and straight into
                 * a neighbour's -- the inter-tag collision T_SLOT_MS exists to
                 * prevent. A motion edge or a HELP press waits the few ms. */
                uwb_radio_sleep_until_strict(slot_start);

                struct pos_meas meas[POS_MAX_ANCHORS];
                int n = uwb_radio_sweep(meas, POS_MAX_ANCHORS);

                uwb_sweep_gate_swept(&sweep_gate, (uint8_t)(n > 0 ? n : 0));

                /* Seed the solve from the filter. Gauss-Newton converges more
                 * reliably from the previous fix than from a cold linear
                 * seed, and once the filter is running this is the normal
                 * path -- which is also why the solver's degeneracy check has
                 * to hold with a seed, not just without one. */
                float        seed[2] = { 0.0f, 0.0f };
                const float *seed_p = pos_ekf_get(&ekf, &seed[0], &seed[1],
                                                  NULL, NULL) ? seed : NULL;

                struct pos_result pos;
                bool solved = (n >= 3) &&
                              pos_solve(meas, (size_t)n, seed_p, &pos);

                /* Run the filter on every sweep that produced ranges, whether
                 * or not the snapshot converged: the gating and the ZUPT are
                 * still meaningful, and skipping the predict would leave dt
                 * wrong for the next one.
                 *
                 * dt is measured, never assumed -- it varies with the tier and
                 * with skipped superframes, which is the whole reason the
                 * filter cannot hardcode a superframe period. */
                uint32_t now  = uwb_radio_now_ms();
                float    dt_s = have_last_fix
                              ? (float)(uint32_t)(now - last_fix_ms) / 1000.0f
                              : 0.0f;

                last_fix_ms   = now;
                have_last_fix = true;

                if (!pos_ekf_get(&ekf, NULL, NULL, NULL, NULL)) {
                    if (solved) {
                        pos_ekf_seed(&ekf, pos.x, pos.y);
                    }
                } else {
                    pos_ekf_predict(&ekf, &ekf_cfg, dt_s, motion_moving);
                    if (n > 0) {
                        (void)pos_ekf_update_ranges(&ekf, &ekf_cfg, meas,
                                                    (size_t)n);
                    }
                    /* The accelerometer is a mode discriminator, not an
                     * inertial sensor: this is the whole of its contribution
                     * besides scheduling the process noise above. */
                    if (!motion_moving) {
                        pos_ekf_zupt(&ekf, &ekf_cfg);
                    }
                    /* Kidnapped tag, or carried while the accelerometer said
                     * still: every range disagreed for reset_after fixes
                     * running, so trust the snapshot over the filter. */
                    if (solved && pos_ekf_needs_reseed(&ekf, &ekf_cfg)) {
                        pos_ekf_seed(&ekf, pos.x, pos.y);
                    }
                }

                /* When a fix is published is deliberately UNCHANGED: still
                 * >= 3 anchors and a converged snapshot solve. The filter
                 * changes the value, not the criteria -- publishing a pure
                 * prediction is a separate decision that should be made
                 * against captured data rather than assumed here.
                 *
                 * residual_m stays the snapshot's. It measures range
                 * consistency, which is what makes it worth reporting; it is
                 * not a statement about the filter's confidence, and it is
                 * evaluated at the snapshot point rather than the filtered
                 * one. */
                if (solved) {
                    float fx, fy;

                    if (pos_ekf_get(&ekf, &fx, &fy, NULL, NULL)) {
                        pos.x = fx;
                        pos.y = fy;
                    }
                    position_publish(&pos, (uint8_t)n, ctx.short_addr);
                }

                /* TEMPORARY: raw-range capture for EKF tuning -- remove with
                 * the module. Quality is passed NULL: the per-range
                 * dwt_readdiagnostics() read is design step 6 and is not
                 * implemented, so `dbg q on` currently gates nothing and the
                 * quality bytes are always zero. */
                pos_dbg_sweep(now, sweep_aid, sweep_ax_cm, sweep_ay_cm,
                              sweep_r_cm, NULL, sweep_mask,
                              (uint8_t)ctx.tier, motion_moving, solved,
                              solved
                                ? (uint16_t)pos_dbg_m_to_cm(pos.residual_m)
                                : POS_DBG_RES_NONE);

                struct uwb_net_event sev = {
                    .kind      = UWB_EV_SWEPT,
                    .n_anchors = (uint8_t)(n > 0 ? n : 0),
                };
                uwb_net_handle(&ctx, &sev);
            }
        }

        if (act & UWB_ACT_SLEEP) {
            if (dw_sleep_enabled) {
                /* Deep-sleep the radio; the wake/arm timing is owned by step 2's
                 * beacon-window planner (which sleeps the MCU until just before
                 * the predicted beacon, or wakes immediately for a full listen in
                 * ACQUIRING).  No timed wake here. */
                dw_enter_sleep();
                radio_asleep = true;
            } else {
                (void)uwb_radio_sleep_until(t0_ms + T_SUPERFRAME_MS);
            }
        }

        if (act & UWB_ACT_TO_SCAN) {
            /* Lost-sync telemetry: seat = gateway reclaimed our lease (CAP
             * keepalive not getting through); miss = >= MISS_MAX consecutive
             * beacons lost. */
            if (ev.kind == UWB_EV_BEACON && !ev.in_map) {
                twr_log("RESCAN seat\n");
            } else {
                twr_log("RESCAN miss\n");
            }
        }
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

void uwb_net_set_moving(bool moving)
{
    motion_moving = moving;
    motion_edge   = true;
}

bool uwb_net_runner_sched_get(uint32_t *period_q16, uint32_t *window_ms,
                              uint32_t *eff_skip, uint32_t *misses,
                              uint32_t *ok_count, uint32_t *miss_count)
{
    struct uwb_tier_params tp;
    uint32_t win = 0, eff = 0;

    /* The tier here is the parameter set the runner would use next; reading
     * ctx.tier would need the runner's stack frame, so report the FAST row --
     * the diagnostics that matter (period, misses, counters) are tier-free and
     * the window/skip pair is read back per tier with `pwr tier`. */
    uwb_net_get_tier_params(UWB_TIER_FAST, &tp);
    beacon_sched_plan(&sched, tp.listen_skip, NULL, &win, &eff);

    if (period_q16) { *period_q16 = beacon_sched_period_q16(&sched); }
    if (window_ms)  { *window_ms  = win; }
    if (eff_skip)   { *eff_skip   = eff; }
    if (misses)     { *misses     = sched.misses; }
    if (ok_count)   { *ok_count   = sched.ok_count; }
    if (miss_count) { *miss_count = sched.miss_count; }
    return sched.ok_count != 0u || sched.miss_count != 0u;
}

void uwb_net_runner_sched_reset_stats(void)
{
    beacon_sched_stats_reset(&sched);
}

void uwb_net_runner_scan_get(uint8_t *rung, uint32_t *next_ms)
{
    if (rung)    { *rung    = scan_backoff_rung(&backoff); }
    if (next_ms) { *next_ms = scan_backoff_next_ms(&backoff); }
}

size_t uwb_net_runner_stack_unused(void)
{
    size_t unused = 0;

    if (k_thread_stack_space_get(&runner_tid, &unused) != 0) {
        return 0;   /* CONFIG_INIT_STACKS off, or thread not started */
    }
    return unused;
}

void uwb_net_runner_start(const uint8_t eui[8])
{
    memcpy(runner_eui, eui, UWB_FRAME_EUI_LEN);

    /* Before the thread starts, so the first sweep sees a defined filter.
     * r_range in these defaults is the design's ASSUMED range sigma, not a
     * measurement -- the static-soak campaign is what settles it. */
    pos_ekf_cfg_defaults(&ekf_cfg);
    pos_ekf_reset(&ekf);
    have_last_fix = false;

    k_thread_create(&runner_tid, runner_stack,
                    K_THREAD_STACK_SIZEOF(runner_stack),
                    runner_fn, NULL, NULL, NULL,
                    RUNNER_PRIO, 0, K_NO_WAIT);
    /* Named so a fatal error can say which thread died -- see `fault`. */
    k_thread_name_set(&runner_tid, "runner");
}
