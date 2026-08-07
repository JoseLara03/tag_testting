#include "batt_curve.h"

/* Resting open-circuit voltage of a standard lithium-polymer cell (4.2 V full,
 * 3.7 V nominal, ~3.5 V empty), ascending by voltage. Note how little the
 * voltage moves between 20% and 60% — that flat middle is the cell's real
 * behaviour, and the reason this estimate is coarse there. */
static const struct {
    short mv;
    signed char pct;
} curve[] = {
    {3500,   0}, {3610,   5}, {3690,  10}, {3710,  15}, {3730,  20},
    {3750,  25}, {3770,  30}, {3790,  35}, {3800,  40}, {3820,  45},
    {3840,  50}, {3850,  55}, {3870,  60}, {3910,  65}, {3950,  70},
    {3980,  75}, {4020,  80}, {4080,  85}, {4110,  90}, {4150,  95},
    {4200, 100},
};

#define CURVE_N ((int)(sizeof(curve) / sizeof(curve[0])))

int lipo_mv_to_pct(int mv)
{
    if (mv <= curve[0].mv) {
        return 0;
    }
    if (mv >= curve[CURVE_N - 1].mv) {
        return 100;
    }

    for (int i = 1; i < CURVE_N; i++) {
        if (mv < curve[i].mv) {
            int dv   = curve[i].mv  - curve[i - 1].mv;
            int dpct = curve[i].pct - curve[i - 1].pct;

            return curve[i - 1].pct + ((mv - curve[i - 1].mv) * dpct) / dv;
        }
    }

    return 100;   /* not reached: the top of the curve is handled above */
}
