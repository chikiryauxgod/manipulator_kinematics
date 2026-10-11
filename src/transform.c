#include "internal.h"

k_status k_transform_identity(k_transform *out)
{
    k_transform result = {{{0}}};
    if (!out)
        return K_INVALID_ARGUMENT;
    for (size_t i = 0; i < 4; ++i)
        result.m[i][i] = 1;
    *out = result;
    return K_OK;
}

k_status k_transform_validate(const k_transform *t)
{
    if (!t)
        return K_INVALID_ARGUMENT;
    for (size_t i = 0; i < 4; ++i)
        for (size_t j = 0; j < 4; ++j)
            if (!isfinite(t->m[i][j]))
                return K_NONFINITE_INPUT;
    for (size_t j = 0; j < 4; ++j)
        if (t->m[3][j] != (k_real)(j == 3))
            return K_INVALID_TRANSFORM;
    for (size_t i = 0; i < 3; ++i)
        for (size_t j = 0; j < 3; ++j)
            if (K_ABS(t->m[i][j]) > 1 + K_TOL)
                return K_INVALID_TRANSFORM;
    for (size_t i = 0; i < 3; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            k_real dot = 0;
            for (size_t k = 0; k < 3; ++k)
                dot += t->m[k][i] * t->m[k][j];
            if (K_ABS(dot - (k_real)(i == j)) > K_TOL)
                return K_INVALID_TRANSFORM;
        }
    }
    k_real det = t->m[0][0] * (t->m[1][1] * t->m[2][2] - t->m[1][2] * t->m[2][1]) -
                 t->m[0][1] * (t->m[1][0] * t->m[2][2] - t->m[1][2] * t->m[2][0]) +
                 t->m[0][2] * (t->m[1][0] * t->m[2][1] - t->m[1][1] * t->m[2][0]);
    return K_ABS(det - 1) <= K_TOL ? K_OK : K_INVALID_TRANSFORM;
}

k_status k_internal_compose(const k_transform *a, const k_transform *b, k_transform *out)
{
    k_transform r = {{{0}}};
    r.m[3][3] = 1;
    for (size_t i = 0; i < 3; ++i)
    {
        for (size_t j = 0; j < 4; ++j)
        {
            k_real sum = 0;
            for (size_t k = 0; k < 3; ++k)
            {
                k_real product;
                if (!k_checked_mul(a->m[i][k], b->m[k][j], &product) ||
                    !k_checked_add(sum, product, &sum))
                    return K_NUMERIC_ERROR;
            }
            if (j == 3 && !k_checked_add(sum, a->m[i][3], &sum))
                return K_NUMERIC_ERROR;
            r.m[i][j] = sum;
        }
    }
    if (k_transform_validate(&r) != K_OK)
    {
        return K_NUMERIC_ERROR;
    }

    *out = r;
    return K_OK;
}

k_status k_transform_compose(const k_transform *a, const k_transform *b, k_transform *out)
{
    if (!out)
    {
        return K_INVALID_ARGUMENT;
    }
    k_status status = k_transform_validate(a);

    if (status != K_OK)
    {
        return status;
    }

    status = k_transform_validate(b);
    if (status != K_OK)
    {
        return status;
    }
    return k_internal_compose(a, b, out);
}
