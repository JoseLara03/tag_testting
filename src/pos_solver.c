#include "pos_solver.h"
#include "pos_residual.h"
#include <arm_math.h>

/* Largest A is (POS_MAX_ANCHORS - 1) x 2. */
#define POS_MAX_ROWS  (POS_MAX_ANCHORS - 1)

bool pos_solve(const struct pos_meas *m, size_t n, struct pos_result *out)
{
    out->valid      = false;
    out->residual_m = 0.0f;
    if (n < 3 || n > POS_MAX_ANCHORS) {
        return false;
    }

    const uint16_t rows = (uint16_t)(n - 1);

    /* Fixed-size scratch (max sizes; only the first `rows` are used). */
    float32_t A_d[POS_MAX_ROWS * 2];
    float32_t b_d[POS_MAX_ROWS * 1];
    float32_t At_d[2 * POS_MAX_ROWS];
    float32_t AtA_d[2 * 2];
    float32_t Atb_d[2 * 1];
    float32_t inv_d[2 * 2];
    float32_t p_d[2 * 1];

    /* Reference anchor = index 0. For each other anchor i:
     *   A_i = [ 2(xk - xi), 2(yk - yi) ]
     *   b_i = (ri^2 - rk^2) + (xk^2 + yk^2 - xi^2 - yi^2) */
    const float32_t xk = m[0].x, yk = m[0].y, rk = m[0].range_m;
    const float32_t ck = xk * xk + yk * yk;

    for (uint16_t i = 0; i < rows; i++) {
        const struct pos_meas *mi = &m[i + 1];
        float32_t ci = mi->x * mi->x + mi->y * mi->y;

        A_d[i * 2 + 0] = 2.0f * (xk - mi->x);
        A_d[i * 2 + 1] = 2.0f * (yk - mi->y);
        b_d[i] = (mi->range_m * mi->range_m - rk * rk) + (ck - ci);
    }

    arm_matrix_instance_f32 A, b, At, AtA, Atb, inv, p;
    arm_mat_init_f32(&A,   rows, 2, A_d);
    arm_mat_init_f32(&b,   rows, 1, b_d);
    arm_mat_init_f32(&At,  2, rows, At_d);
    arm_mat_init_f32(&AtA, 2, 2, AtA_d);
    arm_mat_init_f32(&Atb, 2, 1, Atb_d);
    arm_mat_init_f32(&inv, 2, 2, inv_d);
    arm_mat_init_f32(&p,   2, 1, p_d);

    arm_mat_trans_f32(&A, &At);       /* At = A^T            */
    arm_mat_mult_f32(&At, &A, &AtA);  /* AtA = A^T A (2x2)   */
    arm_mat_mult_f32(&At, &b, &Atb);  /* Atb = A^T b (2x1)   */

    /* arm_mat_inverse_f32 returns ARM_MATH_SINGULAR on a non-invertible matrix
     * (collinear/degenerate anchor geometry). It may modify AtA in place. */
    if (arm_mat_inverse_f32(&AtA, &inv) != ARM_MATH_SUCCESS) {
        return false;
    }

    arm_mat_mult_f32(&inv, &Atb, &p); /* p = inv(AtA) * Atb  */

    out->x = p_d[0];
    out->y = p_d[1];
    out->residual_m = pos_residual_rms(m, n, out->x, out->y);
    out->valid = true;
    return true;
}
