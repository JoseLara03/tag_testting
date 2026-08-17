#ifndef SCAN_BACKOFF_CORE_H_
#define SCAN_BACKOFF_CORE_H_

#include <stdint.h>
#include <stdbool.h>

/* Pure (Zephyr-free) coverage-probe back-off ladder for UWB_ST_SCAN.
 * See spec/2026-08-16-low-power-duty-cycle-design.md §5.
 *
 * A probe is not free: to be certain of catching a beacon the tag must listen
 * for a full superframe, which is 202 ms x 66 mA = 3.7 uAh. A fixed 5-minute
 * probe therefore spends ~44 uA -- 18% of the whole 250 uA budget -- answering
 * "am I home yet?". That is why the ladder backs off further than an interval
 * a human would pick, and why it is motion-gated instead: a tag leaving or
 * entering a building is always in motion, so an accelerometer edge is a far
 * better predictor of a coverage change than elapsed time is. */

/* 10 s, 30 s, 2 min, 5 min, 15 min, 30 min (terminal). */
#define SCAN_BACKOFF_RUNGS  6

struct scan_backoff {
    uint8_t rung;
    bool    alert_pinned;
};

/* Back to rung 0. */
void scan_backoff_reset(struct scan_backoff *b);

/* A probe listened for a full superframe and found no beacon: climb one rung,
 * saturating at the terminal one. No-op while pinned by an alert. */
void scan_backoff_fail(struct scan_backoff *b);

/* An accelerometer activity edge: back to rung 0 from wherever we were. */
void scan_backoff_motion(struct scan_backoff *b);

/* Pin at rung 0 while a HELP is active -- an emergency is exactly when the tag
 * should be trying hardest to find a network. Releasing the pin leaves the
 * ladder at the rung it is on and does not jump it. */
void scan_backoff_alert(struct scan_backoff *b, bool active);

/* Milliseconds until the next probe. */
uint32_t scan_backoff_next_ms(const struct scan_backoff *b);

uint8_t scan_backoff_rung(const struct scan_backoff *b);

#endif /* SCAN_BACKOFF_CORE_H_ */
