#include "port.h"
#include <deca_device_api.h>

decaIrqStatus_t decamutexon(void)
{
    decaIrqStatus_t s = (decaIrqStatus_t)port_GetEXT_IRQStatus();
    if (s) {
        port_DisableEXT_IRQ();
    }
    return s;
}

void decamutexoff(decaIrqStatus_t s)
{
    if (s) {
        port_EnableEXT_IRQ();
    }
}
