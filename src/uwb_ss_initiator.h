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
 * Returns false on timeout, RX error, bad frame, or anchor-id mismatch. */
bool do_one_range_anchor(uint8_t aid, float *range_m, float *ax, float *ay);

/* Report a solved fix: logs the BLE "P:x,y" line and transmits a 0xEA POS frame
 * to the gateway. Call from the runner thread only, inside its own CFP slot —
 * it transmits on the DW3000 without taking a uwb_radio_owner claim, because
 * the runner already owns the radio there. */
void position_publish(const struct pos_result *pos, uint8_t n_anchors,
                      uint16_t src_addr);

/* Transmit one TDoA BLINK (0xF0) with the tag's granted short address -- the
 * same address its POS frames use, which is how the gateway resolves a stable
 * Tid from its seat table. Same call contract as position_publish(): runner
 * thread only, inside its own CFP slot, no uwb_radio_owner claim. */
void blink_publish(uint16_t src_addr, bool alert_pending);

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
