#include <zephyr/kernel.h>
#include "ble_log.h"
#include "lis2hh12_if.h"
#include "lis2hh12_reg.h"

int main(void)
{
    stmdev_ctx_t dev_ctx = {0};
    uint8_t who_am_i = 0;
    char msg[48];

    (void)ble_log_init();

    if (lis2hh12_if_init(&dev_ctx) != 0) {
        /* I2C bus not ready — nothing we can do without debug output yet */
        k_sleep(K_FOREVER);
        return 0;
    }

    /* Block until the central connects AND writes the NUS TX CCCD */
    ble_log_wait_ready();

    if (lis2hh12_dev_id_get(&dev_ctx, &who_am_i) != 0) {
        snprintf(msg, sizeof(msg), "WHO_AM_I read failed (I2C error)\r\n");
    } else if (who_am_i == LIS2HH12_ID) {
        snprintf(msg, sizeof(msg), "WHO_AM_I = 0x%02X (OK)\r\n", who_am_i);
    } else {
        snprintf(msg, sizeof(msg), "WHO_AM_I = 0x%02X (FAIL, expected 0x%02X)\r\n",
                 who_am_i, LIS2HH12_ID);
    }

    while (1) {
        ble_log_send(msg);
        k_msleep(3000);
    }

    return 0;
}
