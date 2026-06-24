#include <string.h>
#include <stdio.h>
#include "tag_cmd.h"
#include "ble_log.h"
#include "cal.h"
#include "uwb_net_runner.h"
#include "batt.h"
#include "rx_stats.h"

static void tag_cmd_on_rx(const uint8_t *data, uint16_t len)
{
	char buf[24];
	uint16_t n = (len < sizeof(buf) - 1) ? len : (uint16_t)(sizeof(buf) - 1);

	memcpy(buf, data, n);
	buf[n] = '\0';
	while (n > 0 && (buf[n - 1] == '\r' || buf[n - 1] == '\n')) {
		buf[--n] = '\0';
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
