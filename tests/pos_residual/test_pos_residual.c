#include "pos_residual.h"
#include "pos_solver.h"
#include <math.h>
#include <stdio.h>

static int g_fail = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

/* Anchors on the unit square; true position is its centre, (0.5, 0.5), which
 * is sqrt(0.5) from every corner. */
#define D_CENTRE  0.70710678f

static void test_consistent_geometry_has_zero_residual(void)
{
    struct pos_meas m[4] = {
        { 0.0f, 0.0f, D_CENTRE }, { 1.0f, 0.0f, D_CENTRE },
        { 0.0f, 1.0f, D_CENTRE }, { 1.0f, 1.0f, D_CENTRE },
    };
    CHECK(fabsf(pos_residual_rms(m, 4, 0.5f, 0.5f)) < 1e-5f);
}

static void test_one_bad_range_shows_up(void)
{
    /* One range inflated by exactly 1 m. Three errors are 0, one is 1, so
     * the RMS is sqrt(1/4) = 0.5 exactly. */
    struct pos_meas m[4] = {
        { 0.0f, 0.0f, D_CENTRE }, { 1.0f, 0.0f, D_CENTRE },
        { 0.0f, 1.0f, D_CENTRE }, { 1.0f, 1.0f, D_CENTRE + 1.0f },
    };
    CHECK(fabsf(pos_residual_rms(m, 4, 0.5f, 0.5f) - 0.5f) < 1e-4f);
}

static void test_three_anchor_case(void)
{
    /* Three anchors, consistent. n == 3 is the minimum the solver accepts. */
    struct pos_meas m[3] = {
        { 0.0f, 0.0f, D_CENTRE }, { 1.0f, 0.0f, D_CENTRE },
        { 0.0f, 1.0f, D_CENTRE },
    };
    CHECK(fabsf(pos_residual_rms(m, 3, 0.5f, 0.5f)) < 1e-5f);
}

static void test_offset_position_is_penalised(void)
{
    /* Same consistent ranges, but evaluated at the wrong position: the
     * residual must be clearly non-zero. Guards against a stub that always
     * returns 0. */
    struct pos_meas m[4] = {
        { 0.0f, 0.0f, D_CENTRE }, { 1.0f, 0.0f, D_CENTRE },
        { 0.0f, 1.0f, D_CENTRE }, { 1.0f, 1.0f, D_CENTRE },
    };
    CHECK(pos_residual_rms(m, 4, 2.0f, 2.0f) > 0.5f);
}

static void test_zero_measurements(void)
{
    struct pos_meas m[1] = { { 0.0f, 0.0f, 1.0f } };
    CHECK(pos_residual_rms(m, 0, 0.0f, 0.0f) == 0.0f);
}

int main(void)
{
    test_consistent_geometry_has_zero_residual();
    test_one_bad_range_shows_up();
    test_three_anchor_case();
    test_offset_position_is_penalised();
    test_zero_measurements();

    if (g_fail) { printf("%d FAILURES\n", g_fail); return 1; }
    printf("ALL TESTS PASSED\n");
    return 0;
}
