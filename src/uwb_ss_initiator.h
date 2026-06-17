#ifndef UWB_SS_INITIATOR_H_
#define UWB_SS_INITIATOR_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include "uwb_net_runner.h"   /* uwb_net_set_tier, uwb_tier_t */

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

/* Publish a solved position over BLE NUS ("P:x.xx,y.yy\n"). */
void position_publish(float x, float y);

/* Compatibility shim: called by motion.c; forwards to uwb_net_set_tier.
 * true  -> UWB_TIER_FAST, false -> UWB_TIER_IDLE. */
void uwb_set_moving(bool moving);

#endif /* UWB_SS_INITIATOR_H_ */
