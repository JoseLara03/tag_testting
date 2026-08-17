#ifndef TAG_ALERT_H
#define TAG_ALERT_H

/*
 * Zephyr glue around tag_alert_core: NVS-backed epoch (storage id 3) and a
 * k_mutex, since raise/cancel run on the UI thread while frame_due/sent run
 * on the runner thread. See spec/2026-08-16-uwb-help-alert-design.md §3/§5
 * and plan/2026-08-16-uwb-help-alert.md Task 4.
 */

#include <stdint.h>
#include <stdbool.h>
#include "uwb_frame_802_15_4z.h"   /* struct uwb_alert */

/* Loads the epoch (and, if the tag reset mid-emergency, resumes the HELP)
 * from NVS id 3. Call after storage_init() and before uwb_net_runner_start(). */
void tag_alert_init(void);

/* Raise a HELP. Call from the UI thread (double-press). */
void tag_alert_raise(void);

/* Cancel an active HELP. Call from the UI thread (long-press >= 3 s). */
void tag_alert_cancel(void);

/* True while a HELP is the tag's current condition -- for the LED's standing
 * red-pulse indicator. False once cancelled, even while the bounded CANCEL
 * repeats are still going out. */
bool tag_alert_active(void);

/* Current epoch, for the `help` status command. Valid regardless of whether
 * an alert is currently active -- it is the last epoch raised (or, once
 * cancelled, the epoch that was cancelled), so it never regresses. */
uint8_t tag_alert_epoch(void);

/* Runner-side: fills *a if a TX is due now. Caller transmits, then calls
 * tag_alert_sent(). short_addr is the runner's current ctx.short_addr
 * (UWB_ADDR_UNASSOC before joining), copied into orig_addr verbatim. */
bool tag_alert_frame_due(struct uwb_alert *a, uint16_t short_addr, uint32_t now_ms);

/* Call after a successful TX of the frame tag_alert_frame_due() produced. */
void tag_alert_sent(uint32_t now_ms);

#endif /* TAG_ALERT_H */
