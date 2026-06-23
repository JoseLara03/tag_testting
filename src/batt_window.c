#include "batt_window.h"

void batt_window_reset(struct batt_window *w)
{
    w->min_ma = 0;
    w->max_ma = 0;
    w->sum_ma = 0;
    w->count  = 0;
}

void batt_window_add(struct batt_window *w, int ma)
{
    if (w->count == 0) {
        w->min_ma = ma;
        w->max_ma = ma;
    } else {
        if (ma < w->min_ma) { w->min_ma = ma; }
        if (ma > w->max_ma) { w->max_ma = ma; }
    }
    w->sum_ma += ma;
    w->count++;
}

int batt_window_get(const struct batt_window *w,
                    int *min_ma, int *mean_ma, int *max_ma, uint32_t *count)
{
    if (w->count == 0) {
        return 0;
    }
    if (min_ma)  { *min_ma  = w->min_ma; }
    if (max_ma)  { *max_ma  = w->max_ma; }
    if (mean_ma) { *mean_ma = (int)(w->sum_ma / (long)w->count); }
    if (count)   { *count   = w->count; }
    return 1;
}
