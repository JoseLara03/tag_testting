#include "rx_stats_core.h"
#include <stddef.h>

/* Convert a signed cycle delta to microseconds. The int64 intermediate avoids
 * overflow (a delta of up to ~1 superframe of cycles times 1e6). */
static int cyc_to_us(const struct rx_stats_core *c, int32_t d)
{
    return (int)(((int64_t)d * 1000000) / (int64_t)c->cyc_per_sec);
}

void rx_stats_core_reset(struct rx_stats_core *c,
                         uint32_t nominal_sf_cyc, uint32_t cyc_per_sec)
{
    batt_window_reset(&c->on_win);
    batt_window_reset(&c->off_win);
    c->prev_beacon_cyc = 0;
    c->prev_valid      = false;
    c->miss_count      = 0;
    c->nominal_sf_cyc  = nominal_sf_cyc;
    c->cyc_per_sec     = cyc_per_sec;
}

void rx_stats_core_beacon(struct rx_stats_core *c,
                          uint32_t arm_cyc, uint32_t now_cyc)
{
    /* on-duration: how long RX was on before the beacon arrived. int32 delta
     * is wrap-safe within one superframe. */
    batt_window_add(&c->on_win, cyc_to_us(c, (int32_t)(now_cyc - arm_cyc)));

    if (c->prev_valid) {
        int32_t off = (int32_t)(now_cyc - (c->prev_beacon_cyc + c->nominal_sf_cyc));
        batt_window_add(&c->off_win, cyc_to_us(c, off));
    }

    c->prev_beacon_cyc = now_cyc;
    c->prev_valid      = true;
}

void rx_stats_core_miss(struct rx_stats_core *c)
{
    c->prev_valid = false;   /* do not measure offset across the gap */
    c->miss_count++;
}

int rx_stats_core_get(const struct rx_stats_core *c,
                      int *on_mean_ms, int *on_max_ms,
                      int *off_min_us, int *off_max_us,
                      uint32_t *count, uint32_t *misses)
{
    int on_mean_us = 0, on_max_us = 0;
    uint32_t n = 0;

    if (!batt_window_get(&c->on_win, NULL, &on_mean_us, &on_max_us, &n)) {
        return 0;
    }
    if (on_mean_ms) { *on_mean_ms = on_mean_us / 1000; }
    if (on_max_ms)  { *on_max_ms  = on_max_us / 1000; }

    /* off_win is empty until a second beacon arrives; leave 0/0 in that case. */
    int omin = 0, omax = 0;
    batt_window_get(&c->off_win, &omin, NULL, &omax, NULL);
    if (off_min_us) { *off_min_us = omin; }
    if (off_max_us) { *off_max_us = omax; }

    if (count)  { *count  = n; }
    if (misses) { *misses = c->miss_count; }
    return 1;
}
