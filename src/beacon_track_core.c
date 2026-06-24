#include "beacon_track_core.h"

void beacon_track_reset(struct beacon_track *c, uint32_t period_seed_ms,
                        uint32_t guard_ms, uint16_t warmup_n, uint8_t ema_shift)
{
    c->period_est_ms  = period_seed_ms;
    c->last_beacon_ms = 0;
    c->have_last      = false;
    c->warmup_count   = 0;
    c->guard_ms       = guard_ms;
    c->warmup_n       = warmup_n;
    c->ema_shift      = ema_shift;
    c->tracking       = false;
}

void beacon_track_plan(const struct beacon_track *c,
                       bool *narrow, uint32_t *arm_at_ms, uint32_t *window_ms)
{
    if (c->tracking && c->have_last) {
        if (narrow)    { *narrow    = true; }
        if (arm_at_ms) { *arm_at_ms = c->last_beacon_ms + c->period_est_ms - c->guard_ms; }
        if (window_ms) { *window_ms = 2u * c->guard_ms; }
    } else {
        if (narrow)    { *narrow    = false; }
        if (arm_at_ms) { *arm_at_ms = 0; }
        if (window_ms) { *window_ms = 0; }
    }
}

void beacon_track_beacon(struct beacon_track *c, uint32_t now_ms)
{
    if (c->have_last) {
        /* int32 deltas are wrap-safe for ~ms intervals. */
        int32_t gap = (int32_t)(now_ms - c->last_beacon_ms);
        int32_t err = gap - (int32_t)c->period_est_ms;
        c->period_est_ms = (uint32_t)((int32_t)c->period_est_ms + (err >> c->ema_shift));
    }
    c->last_beacon_ms = now_ms;
    c->have_last      = true;

    if (c->warmup_count < c->warmup_n) {
        c->warmup_count++;
    }
    if (c->warmup_count >= c->warmup_n) {
        c->tracking = true;
    }
}

void beacon_track_miss(struct beacon_track *c)
{
    c->tracking     = false;
    c->warmup_count = 0;
    c->have_last    = false;   /* keep period_est; just drop the phase reference */
}

uint32_t beacon_track_period_ms(const struct beacon_track *c)
{
    return c->period_est_ms;
}
