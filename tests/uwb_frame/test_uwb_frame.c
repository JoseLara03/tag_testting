#include "uwb_frame_802_15_4z.h"
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

int main(void)
{
    test_scaffold();
    test_utilities();
    if (g_fail) { printf("%d CHECK(s) FAILED\n", g_fail); return 1; }
    printf("ALL TESTS PASSED\n");
    return 0;
}
