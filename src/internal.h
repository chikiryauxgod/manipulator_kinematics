#ifndef K_INTERNAL_H
#define K_INTERNAL_H
#include <manipulator/transform.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#if defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__)
#error "Fast/finite-only math violates the numerical contract"
#endif
_Static_assert(CHAR_BIT == 8 && FLT_RADIX == 2, "Binary IEEE arithmetic required");
_Static_assert(sizeof(float) == 4 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128 &&
                   FLT_MIN_EXP == -125,
               "binary32 required");
_Static_assert(sizeof(double) == 8 && DBL_MANT_DIG == 53 && DBL_MAX_EXP == 1024 &&
                   DBL_MIN_EXP == -1021,
               "binary64 required");
#if K_USE_FLOAT
#define K_MAX FLT_MAX
#define K_TOL ((k_real)2e-5)
#define K_SIN sinf
#define K_COS cosf
#define K_ABS fabsf
#else
#define K_MAX DBL_MAX
#define K_TOL ((k_real)2e-12)
#define K_SIN sin
#define K_COS cos
#define K_ABS fabs
#endif

static inline int k_checked_add(k_real a, k_real b, k_real *out)
{
    if ((b > 0 && a > K_MAX - b) || (b < 0 && a < -K_MAX - b))
    {
        return 0;
    }
    *out = a + b;

    return isfinite(*out);
}

static inline int k_checked_mul(k_real a, k_real b, k_real *out)
{
    k_real aa = K_ABS(a), ab = K_ABS(b);
    if (aa > 1 && ab > K_MAX / aa)
    {
        return 0;
    }
    *out = a * b;

    return isfinite(*out);
}

static inline int k_count_valid(size_t n)
{
    return n > 0 && n <= SIZE_MAX / sizeof(k_dh_joint) && n <= SIZE_MAX / sizeof(k_real) &&
           n <= SIZE_MAX - 2 && n + 2 <= SIZE_MAX / sizeof(k_transform);
}

k_status k_internal_compose(const k_transform *a, const k_transform *b, k_transform *out);
k_status k_internal_positions_validate(const k_model *model, const k_real *joint_positions, size_t count);
k_status k_internal_walk(const k_model *model, const k_real *joint_positions,
                         k_transform *tool, k_transform *frames);
k_status k_internal_dh(const k_dh_joint *joint, k_real q, k_transform *out);
#endif
