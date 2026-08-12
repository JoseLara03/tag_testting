#ifndef POS_RESIDUAL_H_
#define POS_RESIDUAL_H_

#include <stddef.h>
#include "pos_solver.h"

/* RMS of (predicted range - measured range) over the first `n` measurements,
 * in metres, for the solved position (x, y). Zero means the ranges are exactly
 * self-consistent; a large value means at least one range disagrees with the
 * fix, which is the signal a backend needs to decide whether to trust it.
 *
 * Returns 0.0f when n == 0.
 *
 * Deliberately separate from pos_solver.c: that file includes arm_math.h and
 * so cannot compile on the host. This one needs only sqrtf, so it is host
 * tested in tests/pos_residual/. */
float pos_residual_rms(const struct pos_meas *m, size_t n, float x, float y);

#endif /* POS_RESIDUAL_H_ */
