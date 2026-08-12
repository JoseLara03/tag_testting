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

/* Enqueue a BLE NUS log message (≤19 chars + NUL; drops if queue full). */
void twr_log(const char *fmt, ...);

/* Compatibility shim: called by motion.c; forwards to uwb_net_set_tier.
 * true  -> UWB_TIER_FAST, false -> UWB_TIER_IDLE. */
void uwb_set_moving(bool moving);

#endif /* UWB_SS_INITIATOR_H_ */
