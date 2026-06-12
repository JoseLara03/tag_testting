#include "cal_math.h"
#include <string.h>

/* ---- CRC-32 (IEEE 802.3, reflected) ---------------------------------------- */
uint32_t cal_crc32(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;

    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

/* Bytes covered by the CRC: everything before the crc32 field. */
#define CAL_CRC_LEN  (offsetof(struct cal_record, crc32))

void cal_record_finalize(struct cal_record *r)
{
    r->magic   = CAL_MAGIC;
    r->version = CAL_VERSION;
    r->_pad    = 0;
    r->_pad2   = 0;
    r->crc32   = cal_crc32(r, CAL_CRC_LEN);
}

bool cal_record_valid(const struct cal_record *r, uint8_t expected_phy)
{
    if (r->magic != CAL_MAGIC) {
        return false;
    }
    if (r->version != CAL_VERSION) {
        return false;
    }
    if (r->phy_option != expected_phy) {
        return false;
    }
    return r->crc32 == cal_crc32(r, CAL_CRC_LEN);
}

int cal_math_selftest(void)
{
    int fails = 0;

    /* CRC32 known-answer: "123456789" -> 0xCBF43926. */
    if (cal_crc32("123456789", 9) != 0xCBF43926u) {
        fails++;
    }

    /* Record round-trip: finalize then validate must pass for matching phy
     * and fail for a different phy. */
    struct cal_record r = {0};
    r.phy_option = 7;
    r.tx_ant_dly = 16371;
    r.rx_ant_dly = 16371;
    r.ref_mm = 2000;
    cal_record_finalize(&r);
    if (!cal_record_valid(&r, 7)) {
        fails++;
    }
    if (cal_record_valid(&r, 9)) {
        fails++;
    }
    /* Corrupt a byte -> CRC must reject. */
    r.tx_ant_dly ^= 0x01;
    if (cal_record_valid(&r, 7)) {
        fails++;
    }

    return fails;
}
