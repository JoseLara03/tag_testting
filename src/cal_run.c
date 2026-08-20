#include "cal_run.h"
#include "cal.h"
#include "cal_math.h"
#include "phy_config.h"   /* CONFIG_OPTION */
#include "storage.h"
#include "cal_led.h"

#include <string.h>

#define CAL_LAST_LEN 20
static char cal_last[CAL_LAST_LEN] = "CAL none\n";

void cal_set_last_result(const char *s)
{
    strncpy(cal_last, s, sizeof(cal_last) - 1);
    cal_last[sizeof(cal_last) - 1] = '\0';
    cal_led_on_result(cal_last);
}

const char *cal_get_last_result(void)
{
    return cal_last;
}

int cal_clear(void)
{
    int rc = storage_delete(CAL_NVS_ID);

    if (rc == 0) {
        cal_internal_invalidate();
    }
    return rc;
}

int cal_store(uint16_t tx, uint16_t rx, uint32_t ref_mm, uint16_t residual_mm)
{
    struct cal_record r = {0};

    r.phy_option  = (uint8_t)CONFIG_OPTION;
    r.tx_ant_dly  = tx;
    r.rx_ant_dly  = rx;
    r.ref_mm      = ref_mm;
    r.residual_mm = residual_mm;
    cal_record_finalize(&r);

    int rc = storage_write(CAL_NVS_ID, &r, sizeof(r));

    if (rc < 0) {
        return rc;
    }
    cal_internal_activate(&r);
    return 0;
}
