/* Plain-C implementations of the CMSIS-DSP subset used by the vendored
 * Crazyflie Kalman core. See arm_math.h in this directory. */

#include "arm_math.h"

#include <string.h>

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc,
                             arm_matrix_instance_f32 *pDst)
{
    if (pSrc->numRows != pDst->numCols || pSrc->numCols != pDst->numRows) {
        return ARM_MATH_SIZE_MISMATCH;
    }

    const uint16_t rows = pSrc->numRows;
    const uint16_t cols = pSrc->numCols;
    for (uint16_t r = 0; r < rows; r++) {
        for (uint16_t c = 0; c < cols; c++) {
            pDst->pData[(uint32_t)c * rows + r] = pSrc->pData[(uint32_t)r * cols + c];
        }
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA,
                            const arm_matrix_instance_f32 *pSrcB,
                            arm_matrix_instance_f32 *pDst)
{
    if (pSrcA->numCols != pSrcB->numRows ||
        pDst->numRows != pSrcA->numRows ||
        pDst->numCols != pSrcB->numCols) {
        return ARM_MATH_SIZE_MISMATCH;
    }

    const uint16_t m = pSrcA->numRows;
    const uint16_t n = pSrcA->numCols;
    const uint16_t p = pSrcB->numCols;

    for (uint16_t r = 0; r < m; r++) {
        for (uint16_t c = 0; c < p; c++) {
            float32_t sum = 0.0f;
            for (uint16_t k = 0; k < n; k++) {
                sum += pSrcA->pData[(uint32_t)r * n + k] * pSrcB->pData[(uint32_t)k * p + c];
            }
            pDst->pData[(uint32_t)r * p + c] = sum;
        }
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_scale_f32(const arm_matrix_instance_f32 *pSrc,
                             float32_t scale,
                             arm_matrix_instance_f32 *pDst)
{
    if (pSrc->numRows != pDst->numRows || pSrc->numCols != pDst->numCols) {
        return ARM_MATH_SIZE_MISMATCH;
    }

    const uint32_t count = (uint32_t)pSrc->numRows * pSrc->numCols;
    for (uint32_t i = 0; i < count; i++) {
        pDst->pData[i] = pSrc->pData[i] * scale;
    }
    return ARM_MATH_SUCCESS;
}

#define ARM_MAT_INV_MAX_DIM 9U

arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *pSrc,
                               arm_matrix_instance_f32 *pDst)
{
    const uint16_t n = pSrc->numRows;
    if (n != pSrc->numCols || pDst->numRows != n || pDst->numCols != n) {
        return ARM_MATH_SIZE_MISMATCH;
    }
    if (n > ARM_MAT_INV_MAX_DIM) {
        return ARM_MATH_ARGUMENT_ERROR;
    }

    /* Gauss-Jordan with partial pivoting on a scratch copy. */
    float32_t a[ARM_MAT_INV_MAX_DIM][ARM_MAT_INV_MAX_DIM];
    float32_t inv[ARM_MAT_INV_MAX_DIM][ARM_MAT_INV_MAX_DIM];

    for (uint16_t r = 0; r < n; r++) {
        for (uint16_t c = 0; c < n; c++) {
            a[r][c] = pSrc->pData[(uint32_t)r * n + c];
            inv[r][c] = (r == c) ? 1.0f : 0.0f;
        }
    }

    for (uint16_t col = 0; col < n; col++) {
        uint16_t pivot = col;
        float32_t best = fabsf(a[col][col]);
        for (uint16_t r = col + 1; r < n; r++) {
            const float32_t v = fabsf(a[r][col]);
            if (v > best) {
                best = v;
                pivot = r;
            }
        }
        if (best <= 1e-12f) {
            return ARM_MATH_SINGULAR;
        }
        if (pivot != col) {
            for (uint16_t c = 0; c < n; c++) {
                float32_t tmp = a[col][c];
                a[col][c] = a[pivot][c];
                a[pivot][c] = tmp;
                tmp = inv[col][c];
                inv[col][c] = inv[pivot][c];
                inv[pivot][c] = tmp;
            }
        }

        const float32_t d = 1.0f / a[col][col];
        for (uint16_t c = 0; c < n; c++) {
            a[col][c] *= d;
            inv[col][c] *= d;
        }
        for (uint16_t r = 0; r < n; r++) {
            if (r == col) {
                continue;
            }
            const float32_t f = a[r][col];
            if (f == 0.0f) {
                continue;
            }
            for (uint16_t c = 0; c < n; c++) {
                a[r][c] -= f * a[col][c];
                inv[r][c] -= f * inv[col][c];
            }
        }
    }

    for (uint16_t r = 0; r < n; r++) {
        for (uint16_t c = 0; c < n; c++) {
            pDst->pData[(uint32_t)r * n + c] = inv[r][c];
        }
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_sqrt_f32(float32_t in, float32_t *pOut)
{
    if (in < 0.0f) {
        /* Tolerate tiny negatives from floating point cancellation. */
        if (in > -1e-9f) {
            *pOut = 0.0f;
            return ARM_MATH_SUCCESS;
        }
        *pOut = 0.0f;
        return ARM_MATH_ARGUMENT_ERROR;
    }
    *pOut = sqrtf(in);
    return ARM_MATH_SUCCESS;
}
