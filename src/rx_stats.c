#include <zephyr/kernel.h>
#include "rx_stats.h"
#include "rx_stats_core.h"

/* Must match T_SUPERFRAME_MS in src/uwb_net_runner.c (private there). */
#define RX_STATS_SUPERFRAME_MS 200u

static struct rx_stats_core core;
static uint32_t             arm_cyc;

void rx_stats_reset(void)
{
    rx_stats_core_reset(&core,
                        k_ms_to_cyc_near32(RX_STATS_SUPERFRAME_MS),
                        sys_clock_hw_cycles_per_sec());
}

void rx_stats_arm(void)
{
    arm_cyc = k_cycle_get_32();
}

void rx_stats_beacon(void)
{
    rx_stats_core_beacon(&core, arm_cyc, k_cycle_get_32());
}

void rx_stats_miss(void)
{
    rx_stats_core_miss(&core);
}

int rx_stats_get(int *on_mean_ms, int *on_max_ms,
                 int *off_min_us, int *off_max_us,
                 uint32_t *count, uint32_t *misses)
{
    return rx_stats_core_get(&core, on_mean_ms, on_max_ms,
                             off_min_us, off_max_us, count, misses);
}
