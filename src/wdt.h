#ifndef WDT_H
#define WDT_H

/*
 * Boot-guard hardware watchdog.
 *
 * tag_wdt_start_boot_guard() arms the nRF52833 watchdog (via Zephyr task_wdt)
 * BEFORE the DW3000 bring-up. The bring-up runs unfed, so a hung/failed init
 * resets the SoC after WDT_TIMEOUT_MS and the boot retries from scratch.
 *
 * On a successful boot, call tag_wdt_run_feeder() to start a thread that feeds
 * the watchdog forever (the nRF52833 WDT cannot be stopped once started).
 */

/* Arm the watchdog. Returns the channel id (>= 0) or a negative errno. */
int tag_wdt_start_boot_guard(void);

/* Start the forever-feed thread. Call only after a successful boot guard. */
void tag_wdt_run_feeder(void);

#endif /* WDT_H */
