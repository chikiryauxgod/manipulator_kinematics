#ifndef MK_INTERNAL_H
#define MK_INTERNAL_H
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
#if MK_USE_FLOAT
#define MK_MAX FLT_MAX
#define MK_TOL ((mk_real)2e-5)
#define MK_SIN sinf
#define MK_COS cosf
#define MK_ABS fabsf
#else
#define MK_MAX DBL_MAX
#define MK_TOL ((mk_real)2e-12)
#define MK_SIN sin
#define MK_COS cos
#define MK_ABS fabs
#endif
static inline int mk_checked_add(mk_real a, mk_real b, mk_real *out)
{
    if ((b > 0 && a > MK_MAX - b) || (b < 0 && a < -MK_MAX - b)) return 0;
    *out = a + b;
    return isfinite(*out);
}
static inline int mk_checked_mul(mk_real a, mk_real b, mk_real *out)
{
    mk_real aa = MK_ABS(a), ab = MK_ABS(b);
    if (aa > 1 && ab > MK_MAX / aa) return 0;
    *out = a * b;
    return isfinite(*out);
}
static inline int mk_count_valid(size_t n)
{
    return n > 0 && n <= SIZE_MAX / sizeof(mk_dh_joint)
        && n <= SIZE_MAX / sizeof(mk_real) && n <= SIZE_MAX - 2
        && n + 2 <= SIZE_MAX / sizeof(mk_transform);
}
/* Internal symbols are not part of the public API. */
mk_status mk_internal_compose(const mk_transform *a, const mk_transform *b, mk_transform *out);
mk_status mk_internal_joints_validate(const mk_dh_joint *joints, size_t n);
mk_status mk_internal_dh(const mk_dh_joint *joint, mk_real q, mk_transform *out);
#endif
