#include "cal_run_math.h"

enum cal_run_verdict cal_run_iteration_result(const int32_t *samples, size_t got,
                                               size_t samples_per_iter,
                                               int32_t ref_mm, uint16_t cur_total,
                                               int32_t *out_err, size_t *out_kept,
                                               uint16_t *out_new_total)
{
    int32_t mean;
    size_t  kept;

    if (got < samples_per_iter / 4 || !cal_filtered_mean(samples, got, &mean, &kept)) {
        return CAL_RUN_NO_RESP;
    }

    int32_t err    = mean - ref_mm;
    int32_t abserr = (err < 0) ? -err : err;

    *out_err  = err;
    *out_kept = kept;

    if (abserr <= CAL_ACCEPT_MM) {
        return CAL_RUN_CONVERGED;
    }

    *out_new_total = cal_solve_step(mean, ref_mm, cur_total);
    return CAL_RUN_CONTINUE;
}

int cal_run_math_selftest(void)
{
    int fails = 0;

    int32_t  err;
    size_t   kept;
    uint16_t new_total;

    /* Too few samples: refuse to trust the mean, whatever it looks like. */
    int32_t few[] = {2000, 2000};
    if (cal_run_iteration_result(few, 2, 100, 2000, 32742, &err, &kept, &new_total)
        != CAL_RUN_NO_RESP) {
        fails++;
    }

    /* Converged: mean within CAL_ACCEPT_MM of ref_mm. */
    int32_t close[] = {2000, 2005, 1998, 2003, 1999, 2001, 2002, 2000, 2004, 1997};
    if (cal_run_iteration_result(close, 10, 10, 2000, 32742, &err, &kept, &new_total)
        != CAL_RUN_CONVERGED) {
        fails++;
    }

    /* Continue: mean far from ref_mm -- expect the exact correction
     * cal_solve_step() itself would compute, same single source of truth. */
    int32_t far[] = {2234, 2234, 2234, 2234, 2234, 2234, 2234, 2234, 2234, 2234};
    enum cal_run_verdict v = cal_run_iteration_result(far, 10, 10, 2000, 32742,
                                                       &err, &kept, &new_total);
    if (v != CAL_RUN_CONTINUE) {
        fails++;
    } else {
        uint16_t expected = cal_solve_step(2234, 2000, 32742);

        if (new_total != expected) {
            fails++;
        }
        if (err != 234) {
            fails++;
        }
    }

    /* Boundary: abserr == CAL_ACCEPT_MM must still be CAL_RUN_CONVERGED. */
    int32_t at_bound[10];
    for (int i = 0; i < 10; i++) {
        at_bound[i] = 2000 + CAL_ACCEPT_MM;
    }
    if (cal_run_iteration_result(at_bound, 10, 10, 2000, 32742, &err, &kept, &new_total)
        != CAL_RUN_CONVERGED) {
        fails++;
    }

    /* Boundary: abserr == CAL_ACCEPT_MM + 1 must be CAL_RUN_CONTINUE. */
    int32_t past_bound[10];
    for (int i = 0; i < 10; i++) {
        past_bound[i] = 2000 + CAL_ACCEPT_MM + 1;
    }
    if (cal_run_iteration_result(past_bound, 10, 10, 2000, 32742, &err, &kept, &new_total)
        != CAL_RUN_CONTINUE) {
        fails++;
    }

    /* Boundary: got == samples_per_iter/4 must NOT be CAL_RUN_NO_RESP --
     * the check is `got < samples_per_iter/4`. */
    int32_t got_bound[25];
    for (int i = 0; i < 25; i++) {
        got_bound[i] = 2000;
    }
    if (cal_run_iteration_result(got_bound, 25, 100, 2000, 32742, &err, &kept, &new_total)
        == CAL_RUN_NO_RESP) {
        fails++;
    }

    /* Boundary: got one below samples_per_iter/4 must be CAL_RUN_NO_RESP. */
    if (cal_run_iteration_result(got_bound, 24, 100, 2000, 32742, &err, &kept, &new_total)
        != CAL_RUN_NO_RESP) {
        fails++;
    }

    return fails;
}
