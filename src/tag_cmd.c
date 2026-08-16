#include <string.h>
#include <stdio.h>
#include "tag_cmd.h"
#include "ble_log.h"
#include "cal.h"
#include "uwb_net_runner.h"
#include "uwb_ss_initiator.h"
#include "batt.h"
#include "rx_stats.h"
#include "wdt.h"

static void tag_cmd_on_rx(const uint8_t *data, uint16_t len)
{
	char buf[24];
	uint16_t n = (len < sizeof(buf) - 1) ? len : (uint16_t)(sizeof(buf) - 1);

	memcpy(buf, data, n);
	buf[n] = '\0';
	while (n > 0 && (buf[n - 1] == '\r' || buf[n - 1] == '\n')) {
		buf[--n] = '\0';
	}

	if (strcmp(buf, "rst") == 0) {
		/* Why the last boot happened.
		 *
		 * CONFIG_RESET_ON_FATAL_ERROR is NOT set in this build, so a
		 * fatal error does not reboot: arch_system_halt() kills
		 * interrupts and spins, BLE dies at once, the wdt feeder stops,
		 * and the hardware watchdog resets ~10 s later. So on this
		 * firmware a crash reports "dog", not "sreq" -- and a crash is
		 * visible externally as a disconnect followed by the tag
		 * re-advertising about ten seconds afterwards.
		 *
		 * "pwr" means the SoC never reset at all, so a BLE drop was the
		 * link alone. "sreq" would mean a deliberate reboot; nothing in
		 * this firmware calls sys_reboot(). */
		uint32_t r = tag_reset_reason();
		const char *why = (r & (1u << 1)) ? "dog"
				: (r & (1u << 2)) ? "sreq"
				: (r & (1u << 3)) ? "lock"
				: (r & (1u << 0)) ? "pin"
						  : "pwr";
		char msg[20];

		snprintf(msg, sizeof(msg), "RST %s %08x\n", why, (unsigned)r);
		ble_log_send(msg);
		return;
	}

	if (strcmp(buf, "fault") == 0 || strcmp(buf, "fault clear") == 0) {
		/* What killed the previous boot. Survives the watchdog reset in
		 * __noinit RAM. reason: 0 CPU_EXCEPTION, 1 SPURIOUS_IRQ,
		 * 2 STACK_CHK_FAIL, 3 KERNEL_OOPS, 4 KERNEL_PANIC.
		 * Look the pc up in build/tag_testting/zephyr/zephyr.elf with
		 * arm-zephyr-eabi-addr2line to get the exact line. */
		uint32_t    reason = 0, pc = 0, lr = 0, bfar = 0;
		const char *thread = "";
		char        msg[20];

		if (strcmp(buf, "fault clear") == 0) {
			tag_fault_clear();
			ble_log_send("FLT cleared\n");
			return;
		}
		if (!tag_fault_get(&reason, &pc, &lr, &bfar, &thread)) {
			ble_log_send("FLT none\n");
			return;
		}
		snprintf(msg, sizeof(msg), "FLT r%u %s\n",
			 (unsigned)reason, thread);
		ble_log_send(msg);
		snprintf(msg, sizeof(msg), "FLT pc %08x\n", (unsigned)pc);
		ble_log_send(msg);
		/* On an assert panic the pc is assert_post_action(); lr names the
		 * actual assert site. */
		snprintf(msg, sizeof(msg), "FLT lr %08x\n", (unsigned)lr);
		ble_log_send(msg);
		/* The address the bad access targeted -- see tag_fault_get(). */
		snprintf(msg, sizeof(msg), "FLT bf %08x\n", (unsigned)bfar);
		ble_log_send(msg);
		return;
	}

	if (strcmp(buf, "stack") == 0) {
		/* Free-bytes high-water mark for the two deep threads. Run this
		 * right after a `cal` run: if the SS-TWR figure is near zero,
		 * the calibration path is overflowing and the "disconnect at
		 * the end of cal" is a stack fault, not a BLE problem. */
		char msg[20];

		snprintf(msg, sizeof(msg), "STK ss %u\n",
			 (unsigned)uwb_ss_stack_unused());
		ble_log_send(msg);
		snprintf(msg, sizeof(msg), "STK run %u\n",
			 (unsigned)uwb_net_runner_stack_unused());
		ble_log_send(msg);
		snprintf(msg, sizeof(msg), "STK ble %u\n",
			 (unsigned)uwb_ss_ble_stack_unused());
		ble_log_send(msg);
		return;
	}

	if (strncmp(buf, "pwr", 3) == 0) {
		if (strcmp(buf, "pwr sleep on") == 0) {
			uwb_radio_set_sleep_enabled(true);
			ble_log_send("PWR sleep on\n");
		} else if (strcmp(buf, "pwr sleep off") == 0) {
			uwb_radio_set_sleep_enabled(false);
			ble_log_send("PWR sleep off\n");
		} else if (strcmp(buf, "pwr idle") == 0) {
			int me = 0, mn = 0, mx = 0;
			uint32_t cnt = 0;
			if (batt_get_idle_window(&me, &mn, &mx, &cnt)) {
				char msg[24];
				snprintf(msg, sizeof(msg), "Idle:%d %d/%d n%u\n",
					 me, mn, mx, cnt);
				ble_log_send(msg);
			} else {
				ble_log_send("Idle none\n");
			}
		} else if (strcmp(buf, "pwr rx") == 0) {
			int on_mean = 0, on_max = 0, off_min = 0, off_max = 0;
			uint32_t cnt = 0, miss = 0;
			if (rx_stats_get(&on_mean, &on_max, &off_min, &off_max,
					 &cnt, &miss)) {
				char msg[24];
				uint32_t cnt_d = cnt > 99999u ? 99999u : cnt;
				uint32_t miss_d = miss > 9999u ? 9999u : miss;
				snprintf(msg, sizeof(msg), "RXon %d/%dms\n",
					 on_mean, on_max);
				ble_log_send(msg);
				snprintf(msg, sizeof(msg), "RXoffmin %dus\n", off_min);
				ble_log_send(msg);
				snprintf(msg, sizeof(msg), "RXoffmax %dus\n", off_max);
				ble_log_send(msg);
				snprintf(msg, sizeof(msg), "RXn %u m%u\n", cnt_d, miss_d);
				ble_log_send(msg);
			} else {
				ble_log_send("RX none\n");
			}
		} else if (strcmp(buf, "pwr rxrst") == 0) {
			rx_stats_reset();
			ble_log_send("RX reset\n");
		} else {
			ble_log_send(uwb_radio_sleep_enabled() ? "PWR sleep on\n"
							       : "PWR sleep off\n");
		}
		return;
	}

	/* Not a power command: forward the raw write to the cal parser. */
	cal_on_rx(data, len);
}

void tag_cmd_init(void)
{
	ble_log_set_rx_handler(tag_cmd_on_rx);
}
