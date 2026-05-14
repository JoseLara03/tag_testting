#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include "lis2hh12_if.h"

#define LIS2HH12_ADDR     0x1EU  /* SA0 = GND */
#define LIS2HH12_I2C_BUS  DT_NODELABEL(i2c0)

static const struct device *i2c_bus = DEVICE_DT_GET(LIS2HH12_I2C_BUS);

static int32_t platform_write(void *handle, uint8_t reg,
                               const uint8_t *buf, uint16_t len)
{
    return i2c_burst_write((const struct device *)handle,
                           LIS2HH12_ADDR, reg, buf, len);
}

static int32_t platform_read(void *handle, uint8_t reg,
                              uint8_t *buf, uint16_t len)
{
    return i2c_burst_read((const struct device *)handle,
                          LIS2HH12_ADDR, reg, buf, len);
}

int lis2hh12_if_init(stmdev_ctx_t *ctx)
{
    if (!device_is_ready(i2c_bus)) {
        return -ENODEV;
    }

    ctx->write_reg = platform_write;
    ctx->read_reg  = platform_read;
    ctx->handle    = (void *)i2c_bus;

    return 0;
}
