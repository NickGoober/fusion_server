/*
 * Minimal CMSIS-DSP compatibility shim for the vendored Crazyflie Kalman core.
 *
 * The Crazyflie firmware targets Cortex-M and uses ARM CMSIS-DSP for its
 * matrix operations. The ESP32-S3 is an Xtensa core, so this header provides
 * plain-C implementations of the exact subset of the CMSIS API used by
 * kalman_core.c and the measurement models (9x9 matrices at most; the
 * hardware FPU handles this comfortably at the fusion rates involved).
 */
#pragma once

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef float float32_t;

typedef enum {
    ARM_MATH_SUCCESS        = 0,
    ARM_MATH_ARGUMENT_ERROR = -1,
    ARM_MATH_LENGTH_ERROR   = -2,
    ARM_MATH_SIZE_MISMATCH  = -3,
    ARM_MATH_NANINF         = -4,
    ARM_MATH_SINGULAR       = -5,
    ARM_MATH_TEST_FAILURE   = -6,
} arm_status;

typedef struct {
    uint16_t numRows;
    uint16_t numCols;
    float32_t *pData;
} arm_matrix_instance_f32;

#ifndef PI
#define PI 3.14159265358979f
#endif

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc,
                             arm_matrix_instance_f32 *pDst);

/* Note: like CMSIS, pDst must not alias pSrcA or pSrcB. */
arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA,
                            const arm_matrix_instance_f32 *pSrcB,
                            arm_matrix_instance_f32 *pDst);

arm_status arm_mat_scale_f32(const arm_matrix_instance_f32 *pSrc,
                             float32_t scale,
                             arm_matrix_instance_f32 *pDst);

arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *pSrc,
                               arm_matrix_instance_f32 *pDst);

arm_status arm_sqrt_f32(float32_t in, float32_t *pOut);

static inline float32_t arm_cos_f32(float32_t x)
{
    return cosf(x);
}

static inline float32_t arm_sin_f32(float32_t x)
{
    return sinf(x);
}

#ifdef __cplusplus
}
#endif
