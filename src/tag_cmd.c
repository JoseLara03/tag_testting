#include <string.h>
#include "tag_cmd.h"
#include "ble_log.h"
#include "cal.h"
#include "uwb_net_runner.h"

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
