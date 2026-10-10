#include "internal.h"

manipulator_status manipulator_transform_identity(manipulator_transform *out)
{
    manipulator_transform result = {{{0}}};
    if (!out) return MANIPULATOR_INVALID_ARGUMENT;
    for (size_t i = 0; i < 4; ++i) result.m[i][i] = 1;
    *out = result;
    return MANIPULATOR_OK;
}

manipulator_status manipulator_transform_validate(const manipulator_transform *t)
{
    if (!t) return MANIPULATOR_INVALID_ARGUMENT;
    for (size_t i = 0; i < 4; ++i)
        for (size_t j = 0; j < 4; ++j)
            if (!isfinite(t->m[i][j])) return MANIPULATOR_NONFINITE_INPUT;
    for (size_t j = 0; j < 4; ++j)
        if (t->m[3][j] != (manipulator_real)(j == 3)) return MANIPULATOR_INVALID_TRANSFORM;
    for (size_t i = 0; i < 3; ++i)
        for (size_t j = 0; j < 3; ++j)
            if (MANIPULATOR_ABS(t->m[i][j]) > 1 + MANIPULATOR_TOL) return MANIPULATOR_INVALID_TRANSFORM;
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            manipulator_real dot = 0;
            for (size_t k = 0; k < 3; ++k) dot += t->m[k][i] * t->m[k][j];
            if (MANIPULATOR_ABS(dot - (manipulator_real)(i == j)) > MANIPULATOR_TOL) return MANIPULATOR_INVALID_TRANSFORM;
        }
    }
    manipulator_real det = t->m[0][0] * (t->m[1][1]*t->m[2][2] - t->m[1][2]*t->m[2][1])
                - t->m[0][1] * (t->m[1][0]*t->m[2][2] - t->m[1][2]*t->m[2][0])
                + t->m[0][2] * (t->m[1][0]*t->m[2][1] - t->m[1][1]*t->m[2][0]);
    return MANIPULATOR_ABS(det - 1) <= MANIPULATOR_TOL ? MANIPULATOR_OK : MANIPULATOR_INVALID_TRANSFORM;
}

manipulator_status manipulator_internal_compose(const manipulator_transform *a, const manipulator_transform *b, manipulator_transform *out)
{
    manipulator_transform r = {{{0}}};
    r.m[3][3] = 1;
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            manipulator_real sum = 0;
            for (size_t k = 0; k < 3; ++k) {
                manipulator_real product;
                if (!manipulator_checked_mul(a->m[i][k], b->m[k][j], &product)
                    || !manipulator_checked_add(sum, product, &sum)) return MANIPULATOR_NUMERIC_ERROR;
            }
            if (j == 3 && !manipulator_checked_add(sum, a->m[i][3], &sum)) return MANIPULATOR_NUMERIC_ERROR;
            r.m[i][j] = sum;
        }
    }
    if (manipulator_transform_validate(&r) != MANIPULATOR_OK) return MANIPULATOR_NUMERIC_ERROR;
    *out = r;
    return MANIPULATOR_OK;
}

manipulator_status manipulator_transform_compose(const manipulator_transform *a, const manipulator_transform *b, manipulator_transform *out)
{
    if (!out) return MANIPULATOR_INVALID_ARGUMENT;
    manipulator_status status = manipulator_transform_validate(a);
    if (status != MANIPULATOR_OK) return status;
    status = manipulator_transform_validate(b);
    return status != MANIPULATOR_OK ? status : manipulator_internal_compose(a, b, out);
}
