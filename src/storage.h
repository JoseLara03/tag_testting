#ifndef STORAGE_H
#define STORAGE_H

#include <stddef.h>
#include <stdint.h>

/* NVS ids in use (one nvs_fs instance for the whole app -- do not open a
 * second one): 1 = cal record (src/cal.c), 2 = NFC name (src/nfc_tag.c),
 * 3 = alert epoch/active (src/tag_alert.c), 4 = anchor/tag height for the 3D
 * range model (src/pos_cfg.c), 5 = TDoA blink mode (src/blink_cfg.c).
 * This list is the registry. Add to it when you claim an id. */

int storage_init(void);
int storage_read(uint16_t id, void *buf, size_t len);
int storage_write(uint16_t id, const void *buf, size_t len);
int storage_delete(uint16_t id);

#endif /* STORAGE_H */
