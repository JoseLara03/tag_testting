#include "wdt.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/fatal.h>
#include <zephyr/task_wdt/task_wdt.h>
#include <hal/nrf_power.h>
#include <cmsis_core.h>   /* SCB->BFAR */
#include <errno.h>

/* ---- reset reason ----------------------------------------------------------
 * Latched before anything else runs and cleared immediately: RESETREAS is
 * sticky across resets, so leaving it set would make every later boot inherit
 * the previous boot's cause. SYS_INIT rather than a call from main() because
 * forgetting the call would silently degrade this to garbage. */
static uint32_t reset_reason;

static int reset_reason_latch(void)
{
	reset_reason = nrf_power_resetreas_get(NRF_POWER);
	nrf_power_resetreas_clear(NRF_POWER, reset_reason);
	return 0;
}

SYS_INIT(reset_reason_latch, PRE_KERNEL_1, 0);

uint32_t tag_reset_reason(void)
{
	return reset_reason;
}

/* ---- fatal error capture ---------------------------------------------------
 * __noinit: deliberately NOT cleared by the C startup, so the record written
 * just before the halt is still there after the watchdog's warm reset. A magic
 * word validates it, because uninitialised RAM after a cold boot is arbitrary.
 */
#define FAULT_MAGIC   0x464C5431u   /* "FLT1" */
#define FAULT_NAME_LEN 16

static __noinit struct {
	uint32_t magic;
	uint32_t reason;
	uint32_t pc;
	uint32_t lr;
	uint32_t bfar;
	char     thread[FAULT_NAME_LEN];
} fault_rec;

bool tag_fault_get(uint32_t *reason, uint32_t *pc, uint32_t *lr, uint32_t *bfar,
		   const char **thread)
{
	if (fault_rec.magic != FAULT_MAGIC) {
		return false;
	}
	fault_rec.thread[FAULT_NAME_LEN - 1] = '\0';   /* never trust retained RAM */
	*reason = fault_rec.reason;
	*pc     = fault_rec.pc;
	*lr     = fault_rec.lr;
	*bfar   = fault_rec.bfar;
	*thread = fault_rec.thread;
	return true;
}

void tag_fault_clear(void)
{
	fault_rec.magic = 0u;
}

/*
 * Overrides the __weak default in kernel/fatal.c. Keeps the default behaviour
 * -- halt, and let the hardware watchdog reset us -- but records what happened
 * first. Nothing here may block or allocate: we are in an unrecoverable
 * context, possibly in an ISR, possibly on a blown stack.
 */
void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	fault_rec.reason = reason;
	fault_rec.pc     = (esf != NULL) ? esf->basic.pc : 0u;
	/* On an assert-triggered panic the PC lands in assert_post_action(), which
	 * says nothing about WHICH assert fired. LR is the return address into the
	 * caller, so addr2line on it names the actual site. */
	fault_rec.lr     = (esf != NULL) ? esf->basic.lr : 0u;

	/* The address the faulting access targeted. Zephyr's bus_fault() has
	 * already consumed CFSR by now -- it clears BFARVALID and writes back the
	 * status bits before z_fatal_error() runs -- but it does NOT clear BFAR
	 * itself, so the address survives to here. Worth everything on a
	 * BUS_PRECISE_DATA_BUS: it says exactly which pointer was bad, which
	 * distinguishes a null, a freed/garbage value, and the 0xAA stack fill
	 * pattern that CONFIG_INIT_STACKS writes (reading uninitialised stack). */
	fault_rec.bfar = SCB->BFAR;

	const char *name = k_thread_name_get(k_current_get());

	if (name != NULL) {
		size_t i = 0;

		while (i < FAULT_NAME_LEN - 1 && name[i] != '\0') {
			fault_rec.thread[i] = name[i];
			i++;
		}
		fault_rec.thread[i] = '\0';
	} else {
		fault_rec.thread[0] = '\0';
	}

	/* Magic last: a partially written record must not read as valid. */
	fault_rec.magic = FAULT_MAGIC;

	k_fatal_halt(reason);
	CODE_UNREACHABLE;
}

/* ---- boot-guard watchdog ----------------------------------------------------
 *
 * Compiled out when CONFIG_TASK_WDT=n, which is what debug.conf does. A live
 * watchdog makes this firmware effectively undebuggable: task_wdt_add() is
 * called with a NULL callback, and Zephyr's task_wdt_trigger() responds to an
 * expired channel by calling sys_reboot(SYS_REBOOT_COLD). Halting at a
 * breakpoint for more than WDT_TIMEOUT_MS therefore reboots the tag the moment
 * execution resumes -- the debugger sees an endless run of "external reset
 * detected" and no breakpoint is ever reached. The ~120 ms hardware fallback
 * does pause while halted (WDT_OPT_PAUSE_HALTED_BY_DBG); this software channel
 * does not.
 */
#ifdef CONFIG_TASK_WDT

#define WDT_TIMEOUT_MS   10000
#define WDT_FEED_MS      (WDT_TIMEOUT_MS / 3)

#define WDT_FEED_STACK_SIZE 512
#define WDT_FEED_PRIORITY   K_LOWEST_APPLICATION_THREAD_PRIO

static const struct device *const wdt_dev = DEVICE_DT_GET(DT_NODELABEL(wdt0));

static int wdt_channel = -1;

static K_THREAD_STACK_DEFINE(wdt_feed_stack, WDT_FEED_STACK_SIZE);
static struct k_thread wdt_feed_thread;

int tag_wdt_start_boot_guard(void)
{
	int err;

	if (!device_is_ready(wdt_dev)) {
		return -ENODEV;
	}

	err = task_wdt_init(wdt_dev);
	if (err) {
		return err;
	}

	/* NULL callback => direct SoC reset when the channel expires. */
	wdt_channel = task_wdt_add(WDT_TIMEOUT_MS, NULL, NULL);
	return wdt_channel;
}

static void wdt_feed_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		task_wdt_feed(wdt_channel);
		k_msleep(WDT_FEED_MS);
	}
}

void tag_wdt_run_feeder(void)
{
	if (wdt_channel < 0) {
		return;
	}

	/* Feed once immediately so the window is reset before we start looping. */
	task_wdt_feed(wdt_channel);

	k_thread_create(&wdt_feed_thread, wdt_feed_stack,
			K_THREAD_STACK_SIZEOF(wdt_feed_stack),
			wdt_feed_fn, NULL, NULL, NULL,
			WDT_FEED_PRIORITY, 0, K_NO_WAIT);
}

#else /* !CONFIG_TASK_WDT -- debug build, see the comment above */

int  tag_wdt_start_boot_guard(void) { return -ENOTSUP; }
void tag_wdt_run_feeder(void)       { }

#endif /* CONFIG_TASK_WDT */
