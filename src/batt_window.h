#ifndef BATT_WINDOW_H_
#define BATT_WINDOW_H_

#include <stdint.h>

struct batt_window {
    int      min_ma;
    int      max_ma;
    long     sum_ma;
    uint32_t count;
};

void batt_window_reset(struct batt_window *w);
void batt_window_add(struct batt_window *w, int ma);
int  batt_window_get(const struct batt_window *w,
                     int *min_ma, int *mean_ma, int *max_ma, uint32_t *count);

#endif /* BATT_WINDOW_H_ */
