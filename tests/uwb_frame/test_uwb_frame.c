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

int main(void)
{
    test_scaffold();
    if (g_fail) { printf("%d CHECK(s) FAILED\n", g_fail); return 1; }
    printf("ALL TESTS PASSED\n");
    return 0;
}
