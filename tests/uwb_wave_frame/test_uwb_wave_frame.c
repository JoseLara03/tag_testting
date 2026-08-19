/*
 * Host test for src/uwb_wave_frame.h — a pure-macro header, no Zephyr
 * dependency. Proves the shared byte patterns and offsets are byte-for-byte
 * identical to the literals uwb_ss_initiator.c used to hand-roll twice.
 */
#include "uwb_wave_frame.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int fails;

#define CHECK(c)                                                 \
    do {                                                         \
        if (!(c)) {                                              \
            printf("FAIL line %d: %s\n", __LINE__, #c);          \
            fails++;                                             \
        }                                                        \
    } while (0)

int main(void)
{
    uint8_t poll[]     = UWB_WAVE_POLL_INIT;
    uint8_t resp[]     = UWB_WAVE_RESP_INIT;
    uint8_t pos_poll[] = UWB_WAVE_POS_POLL_INIT;
    uint8_t pos_resp[] = UWB_WAVE_POS_RESP_INIT;

    static const uint8_t poll_ref[]     = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0 };
    static const uint8_t resp_ref[]     = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1 };
    static const uint8_t pos_poll_ref[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0 };
    static const uint8_t pos_resp_ref[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1 };

    CHECK(sizeof(poll) == sizeof(poll_ref) &&
          memcmp(poll, poll_ref, sizeof(poll_ref)) == 0);
    CHECK(sizeof(resp) == sizeof(resp_ref) &&
          memcmp(resp, resp_ref, sizeof(resp_ref)) == 0);
    CHECK(sizeof(pos_poll) == sizeof(pos_poll_ref) &&
          memcmp(pos_poll, pos_poll_ref, sizeof(pos_poll_ref)) == 0);
    CHECK(sizeof(pos_resp) == sizeof(pos_resp_ref) &&
          memcmp(pos_resp, pos_resp_ref, sizeof(pos_resp_ref)) == 0);

    CHECK(ALL_MSG_COMMON_LEN == 10);
    CHECK(ALL_MSG_SN_IDX == 2);
    CHECK(UWB_WAVE_RESP_POLL_RX_TS_IDX == 10);
    CHECK(UWB_WAVE_RESP_RESP_TX_TS_IDX == 14);
    CHECK(UWB_WAVE_POS_ANCHOR_ID_IDX == 10);
    CHECK(UWB_WAVE_POS_POLL_RX_TS_IDX == 11);
    CHECK(UWB_WAVE_POS_RESP_TX_TS_IDX == 15);
    CHECK(UWB_WAVE_POS_ANCHOR_X_IDX == 19);
    CHECK(UWB_WAVE_POS_ANCHOR_Y_IDX == 23);

    printf("uwb_wave_frame: %d failure(s)\n", fails);
    return fails == 0 ? 0 : 1;
}
