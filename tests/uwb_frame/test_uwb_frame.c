#include "uwb_frame_802_15_4z.h"
#include <errno.h>
#include <stdio.h>
#include <string.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

static void test_scaffold(void)
{
    /* Constants are wired correctly. */
    CHECK(UWB_FRAME_TYPE_DISC == 0xE2);
    CHECK(UWB_FRAME_TYPE_MPOL == 0xE3);
    CHECK(UWB_FRAME_TYPE_RESP == 0xE4);
    CHECK(UWB_FRAME_LEN_DISC == 14);
    CHECK(UWB_FRAME_LEN_RESP == 20);
}

static void test_utilities(void)
{
    /* Hand-built frame: dest=0xBEEF (EF BE), src=0x1234 (34 12), seq=7. */
    uint8_t f[UWB_FRAME_HDR_LEN] = {
        0x41, 0x88, 0x07, 0xCA, 0xDE, 0xEF, 0xBE, 0x34, 0x12, UWB_FRAME_TYPE_DISC
    };
    CHECK(uwb_frame_get_seq_num(f)  == 0x07);
    CHECK(uwb_frame_get_dest_addr(f) == 0xBEEF);
    CHECK(uwb_frame_get_src_addr(f)  == 0x1234);

    uwb_frame_set_seq_num(f, 0x2A);
    CHECK(uwb_frame_get_seq_num(f) == 0x2A);
    CHECK(f[2] == 0x2A);  /* set is a plain setter, no auto-increment */
}

static void test_discovery(void)
{
    uint8_t buf[32];
    int n = uwb_frame_discovery_build(buf, sizeof(buf), 0x1234, 0xAABBCCDD);
    CHECK(n == UWB_FRAME_LEN_DISC);

    /* Exact byte layout. */
    uint8_t expect[UWB_FRAME_LEN_DISC] = {
        0x41, 0x88, 0x00, 0xCA, 0xDE,   /* FC, seq, PANID */
        0xFF, 0xFF,                     /* dest = broadcast */
        0x34, 0x12,                     /* src 0x1234 LE */
        UWB_FRAME_TYPE_DISC,            /* type 0xE2 */
        0xDD, 0xCC, 0xBB, 0xAA          /* tx_ts 0xAABBCCDD LE */
    };
    CHECK(memcmp(buf, expect, UWB_FRAME_LEN_DISC) == 0);

    /* Validators. */
    CHECK(uwb_frame_is_valid(buf, n));
    CHECK(uwb_frame_is_discovery(buf, n));
    CHECK(!uwb_frame_is_response(buf, n));   /* type byte differs */

    /* Buffer too small. */
    CHECK(uwb_frame_discovery_build(buf, 5, 0x1234, 0) == -EMSGSIZE);
    /* Null buffer. */
    CHECK(uwb_frame_discovery_build(NULL, sizeof(buf), 0x1234, 0) == -EINVAL);
}

int main(void)
{
    test_scaffold();
    test_utilities();
    test_discovery();
    if (g_fail) { printf("%d CHECK(s) FAILED\n", g_fail); return 1; }
    printf("ALL TESTS PASSED\n");
    return 0;
}
