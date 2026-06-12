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

/* Round-to-nearest signed integer division (den must be > 0). */
static int32_t div_round_pos(int32_t num, int32_t den)
{
    if (num >= 0) {
        return (num + den / 2) / den;
    }
    return -(((-num) + den / 2) / den);
}

uint16_t cal_solve_step(int32_t measured_mm, int32_t ref_mm, uint16_t cur_total_dly)
{
    /* err > 0 => measuring too far => increase delay to pull distance down. */
    int32_t err_mm = measured_mm - ref_mm;
    int32_t delta_units = div_round_pos(err_mm * 1000, CAL_MM_PER_UNIT_X1000);

    int32_t new_total = (int32_t)cur_total_dly + delta_units;

    if (new_total < 0) {
        new_total = 0;
    }
    if (new_total > (int32_t)CAL_MAX_TOTAL_DLY) {
        new_total = (int32_t)CAL_MAX_TOTAL_DLY;
    }
    return (uint16_t)new_total;
}

void cal_split_dly(uint16_t total, uint16_t *tx, uint16_t *rx)
{
    *tx = total / 2u;
    *rx = total - *tx;
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

    /* Solver: 234 mm too far / 2.34 mm-per-unit = +100 units. */
    if (cal_solve_step(2234, 2000, 32742) != 32842) {
        fails++;
    }
    /* Solver: 234 mm too short = -100 units. */
    if (cal_solve_step(1766, 2000, 32742) != 32642) {
        fails++;
    }
    /* Solver clamps at zero (cannot go negative). */
    if (cal_solve_step(0, 100000, 10) != 0) {
        fails++;
    }
    /* Equal split: even and odd totals. */
    uint16_t tx, rx;
    cal_split_dly(32742, &tx, &rx);
    if (tx != 16371 || rx != 16371) {
        fails++;
    }
    cal_split_dly(33, &tx, &rx);
    if (tx != 16 || rx != 17) {
        fails++;
    }

    return fails;
}
