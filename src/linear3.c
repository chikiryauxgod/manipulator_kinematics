#include "internal.h"

k_status k_internal_linear3(k_real matrix[3][3], const k_real rhs[3], k_real result[3])
{
    k_real lower[3][3] = {{0}}, intermediate[3], solution[3];
    for (size_t i = 0; i < 3; ++i)
    {
        for (size_t j = 0; j <= i; ++j)
        {
            k_real value = matrix[i][j];
            if (!isfinite(value))
                return K_NUMERIC_ERROR;
            for (size_t k = 0; k < j; ++k)
            {
                k_real product;
                if (!k_checked_mul(lower[i][k], lower[j][k], &product) ||
                    !k_checked_add(value, -product, &value))
                    return K_NUMERIC_ERROR;
            }
            if (i == j)
            {
                if (value <= 0)
                    return K_NUMERIC_ERROR;
                lower[i][j] = K_SQRT(value);
            }
            else
                lower[i][j] = value / lower[j][j];
            if (!isfinite(lower[i][j]))
                return K_NUMERIC_ERROR;
        }
        k_real value = rhs[i];
        if (!isfinite(value))
            return K_NUMERIC_ERROR;
        for (size_t j = 0; j < i; ++j)
        {
            k_real product;
            if (!k_checked_mul(lower[i][j], intermediate[j], &product) ||
                !k_checked_add(value, -product, &value))
                return K_NUMERIC_ERROR;
        }
        intermediate[i] = value / lower[i][i];
        if (!isfinite(intermediate[i]))
            return K_NUMERIC_ERROR;
    }
    for (size_t r = 3; r > 0; --r)
    {
        size_t i = r - 1;
        k_real value = intermediate[i];
        for (size_t j = i + 1; j < 3; ++j)
        {
            k_real product;
            if (!k_checked_mul(lower[j][i], solution[j], &product) ||
                !k_checked_add(value, -product, &value))
                return K_NUMERIC_ERROR;
        }
        solution[i] = value / lower[i][i];
        if (!isfinite(solution[i]))
            return K_NUMERIC_ERROR;
    }
    for (size_t i = 0; i < 3; ++i)
        result[i] = solution[i];
    return K_OK;
}
