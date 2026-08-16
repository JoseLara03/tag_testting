#ifndef WDT_H
#define WDT_H

#include <stdint.h>
#include <stdbool.h>

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

/*
 * Why the SoC last reset: a snapshot of NRF_POWER->RESETREAS latched at
 * PRE_KERNEL_1 and cleared in hardware, so each boot reports its own cause.
 *
 * Bits: 0 = RESETPIN, 1 = DOG (watchdog expired), 2 = SREQ (software reset --
 * which is what a Zephyr fatal error does under CONFIG_RESET_ON_FATAL_ERROR,
 * so on this firmware SREQ means "something crashed"), 3 = LOCKUP. Zero means
 * power-on or brown-out.
 *
 * With CONFIG_SERIAL=n and CONFIG_LOG=n a fault produces no output at all --
 * the only visible symptom is that BLE drops. This is what distinguishes a
 * crash from a link that merely timed out. Read it over NUS with `rst`.
 */
uint32_t tag_reset_reason(void);

/*
 * Details of the fatal error that halted the previous boot, if any.
 *
 * This build has CONFIG_SERIAL=n, CONFIG_LOG=n and CONFIG_RESET_ON_FATAL_ERROR
 * unset, so a fault produces no output whatsoever: the system halts, the kernel
 * timer stops, and task_wdt's hardware fallback (MIN_TIMEOUT 100 ms +
 * HW_FALLBACK_DELAY 20 ms) resets the SoC about 120 ms later. From outside, the
 * only symptom is that BLE drops and the tag reappears almost immediately.
 *
 * tag_fault_hook() latches the reason, the faulting thread's name and the
 * program counter into a __noinit record, which survives the warm reset because
 * RAM is retained. Read it back over NUS with `fault`.
 *
 * `reason` is a K_ERR_* code: 0 CPU_EXCEPTION, 1 SPURIOUS_IRQ,
 * 2 STACK_CHK_FAIL, 3 KERNEL_OOPS, 4 KERNEL_PANIC.
 *
 * Returns false when no fault is recorded (a clean boot). Reading does not
 * clear the record; `fault clear` does.
 */
bool tag_fault_get(uint32_t *reason, uint32_t *pc, uint32_t *lr, uint32_t *bfar,
		   const char **thread);
void tag_fault_clear(void);

#endif /* WDT_H */
