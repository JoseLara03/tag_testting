#include "port.h"
#include <deca_device_api.h>
#include <zephyr/kernel.h>

void deca_sleep(unsigned int time_ms)
{
    Sleep(time_ms);
}

void deca_usleep(unsigned long time_us)
{
    k_busy_wait((uint32_t)time_us);
}
