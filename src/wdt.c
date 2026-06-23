#include "wdt.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/task_wdt/task_wdt.h>

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
