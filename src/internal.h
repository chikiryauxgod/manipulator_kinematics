#ifndef MANIPULATOR_INTERNAL_H
#define MANIPULATOR_INTERNAL_H
#include <manipulator/transform.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#if defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__)
#error "Fast/finite-only math violates the numerical contract"
#endif
_Static_assert(CHAR_BIT == 8 && FLT_RADIX == 2, "Binary IEEE arithmetic required");
_Static_assert(sizeof(float) == 4 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128 && FLT_MIN_EXP == -125, "binary32 required");
_Static_assert(sizeof(double) == 8 && DBL_MANT_DIG == 53 && DBL_MAX_EXP == 1024 && DBL_MIN_EXP == -1021, "binary64 required");
#if MANIPULATOR_USE_FLOAT
#define MANIPULATOR_MAX FLT_MAX
#define MANIPULATOR_TOL ((manipulator_real)2e-5)
#define MANIPULATOR_SIN sinf
#define MANIPULATOR_COS cosf
#define MANIPULATOR_ABS fabsf
#else
#define MANIPULATOR_MAX DBL_MAX
#define MANIPULATOR_TOL ((manipulator_real)2e-12)
#define MANIPULATOR_SIN sin
#define MANIPULATOR_COS cos
#define MANIPULATOR_ABS fabs
#endif
static inline int manipulator_checked_add(manipulator_real a, manipulator_real b, manipulator_real *out)
{
    if ((b > 0 && a > MANIPULATOR_MAX - b) || (b < 0 && a < -MANIPULATOR_MAX - b)) return 0;
    *out = a + b;
    return isfinite(*out);
}
static inline int manipulator_checked_mul(manipulator_real a, manipulator_real b, manipulator_real *out)
{
    manipulator_real aa = MANIPULATOR_ABS(a), ab = MANIPULATOR_ABS(b);
    if (aa > 1 && ab > MANIPULATOR_MAX / aa) return 0;
    *out = a * b;
    return isfinite(*out);
}
static inline int manipulator_count_valid(size_t n)
{
    return n > 0 && n <= SIZE_MAX / sizeof(manipulator_dh_joint)
        && n <= SIZE_MAX / sizeof(manipulator_real) && n <= SIZE_MAX - 2
        && n + 2 <= SIZE_MAX / sizeof(manipulator_transform);
}

manipulator_status manipulator_internal_compose(const manipulator_transform *a, const manipulator_transform *b, manipulator_transform *out);
manipulator_status manipulator_internal_joints_validate(const manipulator_dh_joint *joints, size_t n);
manipulator_status manipulator_internal_dh(const manipulator_dh_joint *joint, manipulator_real q, manipulator_transform *out);
#endif
