#include <string.h>
#include <stdio.h>
#include "tag_cmd.h"
#include "ble_log.h"
#include "cal.h"
#include "tag_alert.h"
#include "uwb_net_runner.h"
#include "uwb_ss_initiator.h"
#include "batt.h"
#include "rx_stats.h"
#include "wdt.h"
#include "pos_cfg.h"
#include "blink_cfg.h"
#ifdef CONFIG_TAG_CAL_MODE
#include "cal_diag.h"
#include "cal_run.h"
#endif

/* Parse a decimal unsigned from *p, advancing it past the digits and any
 * following spaces. Returns false if there was no digit. Hand-rolled rather
 * than sscanf: the parse is trivial and sscanf drags in a second format engine
 * on a part where the printf side is already the expensive one. */
static bool parse_u32(const char **p, uint32_t *out)
{
	const char *s = *p;
	uint32_t    v = 0;
	bool        any = false;

	while (*s == ' ') {
		s++;
	}
	while (*s >= '0' && *s <= '9') {
		v = v * 10u + (uint32_t)(*s - '0');
		s++;
		any = true;
	}
	while (*s == ' ') {
		s++;
	}
	*p = s;
	*out = v;
	return any;
}

/* "pwr tier"                 -> the three rows, as they will actually be used
 *                               (i.e. after UWB_LISTEN_SKIP_CAP)
 * "pwr tier f|s|i <skip> <every>" -> set one row
 * "pwr tier def"             -> compiled defaults
 * Every reply is <= 19 chars + NUL, per the NUS payload limit. */
static void cmd_tier(const char *arg)
{
	static const char  row[3] = { 'I', 'S', 'F' };   /* uwb_tier_t order */
	struct uwb_tier_params p;
	char msg[20];

	if (strcmp(arg, "def") == 0) {
		uwb_net_reset_tier_params();
		ble_log_send("TIER def\n");
		return;
	}

	if (arg[0] == '\0') {
		for (int t = 0; t < UWB_TIER_COUNT; t++) {
			uwb_net_get_tier_params((uwb_tier_t)t, &p);
			snprintf(msg, sizeof(msg), "%c %u %u\n", row[t],
				 (unsigned)p.listen_skip, (unsigned)p.range_every);
			ble_log_send(msg);
		}
		return;
	}

	uwb_tier_t t;
	switch (arg[0]) {
	case 'f': t = UWB_TIER_FAST; break;
	case 's': t = UWB_TIER_SLOW; break;
	case 'i': t = UWB_TIER_IDLE; break;
	default:  ble_log_send("TIER ?\n"); return;
	}

	const char *rest = arg + 1;
	uint32_t skip = 0, every = 0;

	if (!parse_u32(&rest, &skip) || !parse_u32(&rest, &every)) {
		ble_log_send("TIER ?\n");
		return;
	}

	p.listen_skip = (uint16_t)(skip > 0xFFFFu ? 0xFFFFu : skip);
	p.range_every = (uint16_t)(every > 0xFFFFu ? 0xFFFFu : every);
	uwb_net_set_tier_params(t, &p);

	/* Echo what will actually be used, not what was typed -- the cap is the
	 * whole reason a set can silently differ from a get. */
	uwb_net_get_tier_params(t, &p);
	snprintf(msg, sizeof(msg), "%c %u %u\n", row[t],
		 (unsigned)p.listen_skip, (unsigned)p.range_every);
	ble_log_send(msg);
}

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

	if (strcmp(buf, "help on") == 0) {
		tag_alert_raise();
		char msg[20];

		snprintf(msg, sizeof(msg), "HELP on e%u\n", tag_alert_epoch());
		ble_log_send(msg);
		return;
	}

	if (strcmp(buf, "help off") == 0) {
		tag_alert_cancel();
		ble_log_send("HELP cancel\n");
		return;
	}

	if (strcmp(buf, "help") == 0) {
		char msg[20];

		snprintf(msg, sizeof(msg), "HELP %s e%u\n",
			 tag_alert_active() ? "on" : "off", tag_alert_epoch());
		ble_log_send(msg);
		return;
	}

	/* Positioning geometry: `pos z [<anchor_cm> <tag_cm>]`. The dz these set
	 * feeds the 3D range model -- without it a slant range from a ceiling
	 * anchor goes into a planar equation. See
	 * spec/2026-08-22-position-filtering-design.md. */
	if (strncmp(buf, "pos", 3) == 0 && (buf[3] == '\0' || buf[3] == ' ')) {
		if (!pos_cfg_on_cmd(buf)) {
			ble_log_send("POS ?\n");
		}
		return;
	}

	/* TDoA transmit mode: `blink [on|off]`. Chooses what the tag emits in
	 * the ranging cadence slot -- the TWR sweep (default) or a 0xF0 BLINK
	 * for the gateway to solve. Persisted; takes effect on the next slot. */
	if (strncmp(buf, "blink", 5) == 0 && (buf[5] == '\0' || buf[5] == ' ')) {
		if (!blink_cfg_on_cmd(buf)) {
			ble_log_send("BLINK ?\n");
		}
		return;
	}

	if (strncmp(buf, "pwr", 3) == 0) {
		if (strcmp(buf, "pwr sleep on") == 0) {
			uwb_radio_set_sleep_enabled(true);
			/* Advertising follows the sleep flag: re-arm the bounded
			 * window now that continuous advertising is no longer
			 * justified, instead of waiting for the next press. */
			ble_log_adv_window();
			ble_log_send("PWR sleep on\n");
		} else if (strcmp(buf, "pwr sleep off") == 0) {
			uwb_radio_set_sleep_enabled(false);
			ble_log_adv_window();   /* now continuous; cancels the stop timer */
			ble_log_send("PWR sleep off\n");
		} else if (strcmp(buf, "pwr adv on") == 0) {
			ble_log_adv_force(true);
			ble_log_send("PWR adv on\n");
		} else if (strcmp(buf, "pwr adv off") == 0) {
			ble_log_adv_force(false);
			ble_log_send("PWR adv off\n");
		} else if (strcmp(buf, "pwr adv") == 0) {
			ble_log_send(ble_log_adv_forced() ? "PWR adv on\n"
							  : "PWR adv off\n");
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
		} else if (strncmp(buf, "pwr tier", 8) == 0 &&
			   (buf[8] == '\0' || buf[8] == ' ')) {
			const char *arg = buf + 8;

			while (*arg == ' ') {
				arg++;
			}
			cmd_tier(arg);
		} else if (strcmp(buf, "pwr sched") == 0) {
			uint32_t pq = 0, win = 0, skip = 0, miss = 0, ok = 0, mc = 0;

			if (uwb_net_runner_sched_get(&pq, &win, &skip, &miss,
						     &ok, &mc)) {
				char msg[20];
				/* Q16.16 -> "<ms>.<hundredths>" without float:
				 * the fractional part scaled by 100 and shifted
				 * back down. */
				unsigned p_i = (unsigned)(pq >> 16);
				unsigned p_f = (unsigned)(((pq & 0xFFFFu) * 100u) >> 16);

				snprintf(msg, sizeof(msg), "P %u.%02u W %u\n",
					 p_i, p_f, (unsigned)win);
				ble_log_send(msg);
				snprintf(msg, sizeof(msg), "K %u M %u\n",
					 (unsigned)skip, (unsigned)miss);
				ble_log_send(msg);
				snprintf(msg, sizeof(msg), "OK %u MS %u\n",
					 (unsigned)(ok > 99999u ? 99999u : ok),
					 (unsigned)(mc > 9999u ? 9999u : mc));
				ble_log_send(msg);
			} else {
				ble_log_send("SCH none\n");
			}
		} else if (strcmp(buf, "pwr schedrst") == 0) {
			uwb_net_runner_sched_reset_stats();
			ble_log_send("SCH reset\n");
		} else if (strcmp(buf, "pwr scan") == 0) {
			uint8_t  rung = 0;
			uint32_t next_ms = 0;
			char     msg[20];

			uwb_net_runner_scan_get(&rung, &next_ms);
			snprintf(msg, sizeof(msg), "R %u T %u\n",
				 (unsigned)rung, (unsigned)(next_ms / 1000u));
			ble_log_send(msg);
		} else {
			ble_log_send(uwb_radio_sleep_enabled() ? "PWR sleep on\n"
							       : "PWR sleep off\n");
		}
		return;
	}

#ifdef CONFIG_TAG_CAL_MODE
	if (strncmp(buf, "cal listen", 10) == 0 || strncmp(buf, "cal probe", 9) == 0) {
		cal_diag_on_rx(data, len);
		return;
	}
	if (strncmp(buf, "cal ", 4) == 0 && strcmp(buf, "cal status") != 0) {
		cal_run_on_rx(data, len);
		return;
	}
#endif
	/* Not a power command: forward the raw write to the cal parser. */
	cal_on_rx(data, len);
}

void tag_cmd_init(void)
{
	ble_log_set_rx_handler(tag_cmd_on_rx);
}
