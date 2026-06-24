#ifndef STORAGE_H
#define STORAGE_H

#include <stddef.h>
#include <stdint.h>

int storage_init(void);
int storage_read(uint16_t id, void *buf, size_t len);
int storage_write(uint16_t id, const void *buf, size_t len);
int storage_delete(uint16_t id);

#endif /* STORAGE_H */
