#include "storage.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/drivers/flash.h>
#include <errno.h>

#define STORAGE_NVS_NODE DT_NODELABEL(storage_partition)

static struct nvs_fs fs;
static bool          fs_ready;

int storage_init(void)
{
    const struct device *flash_dev =
        DEVICE_DT_GET(DT_MTD_FROM_FIXED_PARTITION(STORAGE_NVS_NODE));
    if (!device_is_ready(flash_dev)) {
        return -ENODEV;
    }

    off_t  offset = (off_t)DT_REG_ADDR(STORAGE_NVS_NODE);
    size_t size   = (size_t)DT_REG_SIZE(STORAGE_NVS_NODE);

    struct flash_pages_info info;
    int rc = flash_get_page_info_by_offs(flash_dev, offset, &info);
    if (rc) {
        return rc;
    }

    fs.flash_device = flash_dev;
    fs.offset       = offset;
    fs.sector_size  = info.size;
    fs.sector_count = (uint16_t)(size / info.size);

    rc = nvs_mount(&fs);
    if (rc == 0) {
        fs_ready = true;
    }
    return rc;
}

int storage_read(uint16_t id, void *buf, size_t len)
{
    if (!fs_ready) {
        return -ENODEV;
    }
    return (int)nvs_read(&fs, id, buf, len);
}

int storage_write(uint16_t id, const void *buf, size_t len)
{
    if (!fs_ready) {
        return -ENODEV;
    }
    return (int)nvs_write(&fs, id, buf, len);
}

int storage_delete(uint16_t id)
{
    if (!fs_ready) {
        return -ENODEV;
    }
    return nvs_delete(&fs, id);
}
