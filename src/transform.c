#include "internal.h"

mk_status mk_transform_identity(mk_transform *out)
{
    mk_transform result = {{{0}}};
    if (!out) return MK_INVALID_ARGUMENT;
    for (size_t i = 0; i < 4; ++i) result.m[i][i] = 1;
    *out = result;
    return MK_OK;
}

mk_status mk_transform_validate(const mk_transform *t)
{
    if (!t) return MK_INVALID_ARGUMENT;
    for (size_t i = 0; i < 4; ++i)
        for (size_t j = 0; j < 4; ++j)
            if (!isfinite(t->m[i][j])) return MK_NONFINITE_INPUT;
    for (size_t j = 0; j < 4; ++j)
        if (t->m[3][j] != (mk_real)(j == 3)) return MK_INVALID_TRANSFORM;
    for (size_t i = 0; i < 3; ++i)
        for (size_t j = 0; j < 3; ++j)
            if (MK_ABS(t->m[i][j]) > 1 + MK_TOL) return MK_INVALID_TRANSFORM;
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            mk_real dot = 0;
            for (size_t k = 0; k < 3; ++k) dot += t->m[k][i] * t->m[k][j];
            if (MK_ABS(dot - (mk_real)(i == j)) > MK_TOL) return MK_INVALID_TRANSFORM;
        }
    }
    mk_real det = t->m[0][0] * (t->m[1][1]*t->m[2][2] - t->m[1][2]*t->m[2][1])
                - t->m[0][1] * (t->m[1][0]*t->m[2][2] - t->m[1][2]*t->m[2][0])
                + t->m[0][2] * (t->m[1][0]*t->m[2][1] - t->m[1][1]*t->m[2][0]);
    return MK_ABS(det - 1) <= MK_TOL ? MK_OK : MK_INVALID_TRANSFORM;
}

mk_status mk_internal_compose(const mk_transform *a, const mk_transform *b, mk_transform *out)
{
    mk_transform r = {{{0}}};
    r.m[3][3] = 1;
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            mk_real sum = 0;
            for (size_t k = 0; k < 3; ++k) {
                mk_real product;
                if (!mk_checked_mul(a->m[i][k], b->m[k][j], &product)
                    || !mk_checked_add(sum, product, &sum)) return MK_NUMERIC_ERROR;
            }
            if (j == 3 && !mk_checked_add(sum, a->m[i][3], &sum)) return MK_NUMERIC_ERROR;
            r.m[i][j] = sum;
        }
    }
    if (mk_transform_validate(&r) != MK_OK) return MK_NUMERIC_ERROR;
    *out = r;
    return MK_OK;
}

mk_status mk_transform_compose(const mk_transform *a, const mk_transform *b, mk_transform *out)
{
    if (!out) return MK_INVALID_ARGUMENT;
    mk_status status = mk_transform_validate(a);
    if (status != MK_OK) return status;
    status = mk_transform_validate(b);
    return status != MK_OK ? status : mk_internal_compose(a, b, out);
}
