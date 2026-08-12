#ifndef POS_SOLVER_H
#define POS_SOLVER_H

#include <stddef.h>
#include <stdbool.h>

/* Maximum anchors per solve (matches the static anchor list). 2D needs >=3. */
#define POS_MAX_ANCHORS  4

/* One anchor measurement: anchor position (metres) + measured range (metres). */
struct pos_meas {
    float x;
    float y;
    float range_m;
};

/* Solved 2D position. valid == false means no solution was produced. */
struct pos_result {
    float x;
    float y;
    float residual_m;  /* RMS range residual, metres; 0.0f when !valid */
    bool  valid;
};

/* 2D linear least-squares trilateration (CMSIS-DSP).
 *
 * Linearizes the circle equations by subtracting anchor index 0 (the
 * reference), then solves the 2x2 normal equations p = (At*A)^-1 * (At*b).
 * Needs 3 <= n <= POS_MAX_ANCHORS. Sets out->valid = false and returns false
 * when n is out of range or the geometry is degenerate (collinear anchors ->
 * singular normal matrix). */
bool pos_solve(const struct pos_meas *m, size_t n, struct pos_result *out);

#endif /* POS_SOLVER_H */
