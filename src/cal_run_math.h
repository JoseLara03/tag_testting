#ifndef CAL_RUN_MATH_H
#define CAL_RUN_MATH_H

#include "cal_math.h"
#include <stdint.h>
#include <stddef.h>

/* Calibration run parameters. Moved out of uwb_ss_initiator.c, which no
 * longer contains any calibration code -- see
 * docs/superpowers/specs/2026-08-20-cal-image-rewrite-design.md. */
#define CAL_SAMPLES_PER_ITER  100U   /* ranges averaged per iteration */
#define CAL_MAX_ITERS         4U     /* give up after this many corrections */
#define CAL_ACCEPT_MM         15     /* residual error considered converged */

enum cal_run_verdict {
    CAL_RUN_NO_RESP,     /* too few samples survived to trust a mean */
    CAL_RUN_CONVERGED,   /* abs(mean - ref_mm) <= CAL_ACCEPT_MM */
    CAL_RUN_CONTINUE,    /* another iteration is needed */
};

/* Decide what one calibration iteration's collected samples mean. `got` is
 * how many of `samples_per_iter` attempted exchanges produced a sample, and
 * `samples` holds those `got` values. On CAL_RUN_CONVERGED or
 * CAL_RUN_CONTINUE, *out_err and *out_kept are set; on CAL_RUN_CONTINUE,
 * *out_new_total is also set (the corrected combined antenna delay to apply
 * for the next iteration). Pure function: no radio, no NVS -- host-testable
 * exactly like cal_math.c. */
enum cal_run_verdict cal_run_iteration_result(const int32_t *samples, size_t got,
                                               size_t samples_per_iter,
                                               int32_t ref_mm, uint16_t cur_total,
                                               int32_t *out_err, size_t *out_kept,
                                               uint16_t *out_new_total);

/* Run built-in assertion vectors. Returns the number of failed checks. */
int cal_run_math_selftest(void);

#endif /* CAL_RUN_MATH_H */
