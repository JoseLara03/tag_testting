#ifndef UWB_SS_INITIATOR_H_
#define UWB_SS_INITIATOR_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include "uwb_net_runner.h"   /* uwb_net_set_tier, uwb_tier_t */
#include "pos_solver.h"

/* Start the interrupt-driven SS-TWR initiator (spawns ranging + BLE threads).
 * Call after uwb_init() has configured the DW3000 PHY. */
void uwb_ss_initiator_start(void);

/* ---- IRQ event type (shared with uwb_net_runner) ---- */
typedef enum {
    EVT_NONE = 0,
    EVT_TXFRS,
    EVT_RXFCG,
    EVT_RXTO,
    EVT_RXERR,
} irq_evt_t;

/* Wait for a DW3000 IRQ event; returns EVT_RXTO on kernel timeout.
 * Enables the IRQ on entry and disables it on return.
 * NOT safe to call concurrently from multiple threads. */
irq_evt_t wait_event(k_timeout_t timeout);

/* Run a single addressed SS-TWR exchange against anchor `aid`.
 * On a valid response, writes range (m), anchor x/y (m), returns true.
 * Returns false on timeout, RX error, bad frame, or anchor-id mismatch.
 * Kept compiled as the structural sibling of the `cal` path and a
 * single-anchor bench fallback -- production sweeps use
 * uwb_multipoll_sweep() instead (network-scaling v3, Phase 2). */
bool do_one_range_anchor(uint8_t aid, float *range_m, float *ax, float *ay);

/* Name up to UWB_FRAME_MAX_ANCHORS anchors in one 0xE3 MULTI-POLL with
 * staggered response delays and collect their 0xED MPOL_RESP replies within
 * one window (design §3.B). `out_by_slot` and `ok_out` must each hold
 * `n_anchors` entries, in the SAME ORDER as `anchor_ids` -- ok_out[i] is true
 * iff anchor_ids[i] answered, and only then is out_by_slot[i] valid. A
 * missing response is a missing entry, not a failure. Returns the number of
 * anchors that answered. anchor_ids are the pool's 8-bit anchor ids,
 * zero-extended to the frame's 16-bit anchor short address.
 *
 * Per-anchor height (Phase 2 Task 10): when a response's own `z` is finite,
 * out_by_slot[i].dz becomes `z - tag_h_m` (that anchor's real height above
 * the tag); when it is NaN (anchor has no height configured), `dz_fallback`
 * is used instead (pos_cfg's single anchor/tag height pair). *real_z_count_out,
 * if non-NULL, is incremented once per anchor that reported a real z, for the
 * `pos z` diagnostic.
 *
 * The by-slot output (rather than a compacted array) is what lets the caller
 * keep its own per-slot bookkeeping (e.g. the pos_dbg debug log) aligned with
 * `selected[]`. */
int uwb_multipoll_sweep(struct pos_meas *out_by_slot, bool *ok_out,
                        const uint8_t *anchor_ids, uint8_t n_anchors,
                        uint16_t src_addr, float dz_fallback, float tag_h_m,
                        uint8_t *real_z_count_out);

/* Report a solved fix: logs the BLE "P:x,y" line and transmits a 0xEA POS frame
 * to the gateway. Call from the runner thread only, inside its own CFP slot —
 * it transmits on the DW3000 without taking a uwb_radio_owner claim, because
 * the runner already owns the radio there. */
void position_publish(const struct pos_result *pos, uint8_t n_anchors,
                      uint16_t src_addr);

/* Last position cached by position_publish(), for the alert frame's last_x/
 * last_y. Returns false (leaving the pointed-to x and y untouched) when there
 * has never been a fix -- the caller sends NaN in that case. */
bool pos_last_get(float *x, float *y);

/* Enqueue a BLE NUS log message (≤19 chars + NUL; drops if queue full). */
void twr_log(const char *fmt, ...);

/* Enqueue `len` raw bytes for BLE NUS (drops if queue full or len > 20).
 * For the binary raw-range debug records, which contain NUL bytes and so
 * cannot use twr_log(). TEMPORARY -- remove with the debug log, see the
 * checklist in spec/2026-08-22-position-filtering-design.md. */
void twr_log_raw(const uint8_t *buf, size_t len);

/* Compatibility shim: called by motion.c; forwards the raw motion state to
 * uwb_net_set_moving() and wakes the runner out of any skip. */
void uwb_set_moving(bool moving);

/* Minimum bytes ever left unused on the SS-TWR thread stack, as a high-water
 * mark. Calibration no longer runs on this thread (see
 * src/cal_run.c and docs/superpowers/specs/2026-08-20-cal-image-rewrite-design.md).
 * After that rewrite, ss_twr_fn() does one-time DW3000 setup and then blocks
 * forever on k_sleep(K_FOREVER) -- it never calls do_one_range_anchor() itself.
 * So this high-water mark covers only that one-time setup path, on a thread
 * that then sleeps forever; it does NOT cover do_one_range_anchor()'s call
 * chain, which runs on the runner thread in src/uwb_net_runner.c instead.
 * That thread currently has no equivalent `stack`-style diagnostic. Requires
 * CONFIG_INIT_STACKS; returns 0 when unavailable. Read over NUS with `stack`. */
size_t uwb_ss_stack_unused(void);

/* Same, for the BLE sender thread. Only 512 bytes and it calls straight into
 * the BLE host via bt_nus_send() -> bt_gatt_notify(), so it is a candidate for
 * an overflow in its own right. */
size_t uwb_ss_ble_stack_unused(void);

#endif /* UWB_SS_INITIATOR_H_ */
