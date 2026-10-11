#include <manipulator/inverse.h>
#include "inverse_internal.h"

k_status k_internal_dls(k_real *columns, size_t count, const k_real error[3],
                        const k_ik_options *options, k_real *step, k_real gradient_rhs[3],
                        k_real *effective_damping)
{
    k_real scale = options->damping;
    for (size_t i = 0; i < count; ++i)
        for (size_t r = 0; r < 3; ++r)
        {
            k_real *value = columns + 3 * i + r;
            if (!k_checked_mul(*value, options->joint_scales[i], value))
                return K_NUMERIC_ERROR;
            *value /= options->length_scale;
            if (!isfinite(*value))
                return K_NUMERIC_ERROR;
            if (K_ABS(*value) > scale)
                scale = K_ABS(*value);
        }
    k_real ratio = options->damping / scale;
    k_real regularization = ratio * ratio;
    k_real floor = (k_real)64 * K_EPSILON * (k_real)count;
    if (regularization < floor)
        regularization = floor;
    if (!k_checked_mul(scale, K_SQRT(regularization), effective_damping))
        return K_NUMERIC_ERROR;
    k_real matrix[3][3] = {{0}}, rhs[3], result[3];
    for (size_t r = 0; r < 3; ++r)
    {
        rhs[r] = (error[r] / options->length_scale) / scale;
        if (!isfinite(rhs[r]))
            return K_NUMERIC_ERROR;
        gradient_rhs[r] = rhs[r];
        matrix[r][r] = regularization;
    }
    for (size_t i = 0; i < count; ++i)
    {
        k_real *column = columns + 3 * i;
        for (size_t r = 0; r < 3; ++r)
            column[r] /= scale;
        for (size_t r = 0; r < 3; ++r)
            for (size_t c = 0; c < 3; ++c)
            {
                k_real product;
                if (!k_checked_mul(column[r], column[c], &product) ||
                    !k_checked_add(matrix[r][c], product, &matrix[r][c]))
                    return K_NUMERIC_ERROR;
            }
    }
    k_status status = k_internal_linear3(matrix, rhs, result);
    if (status != K_OK)
        return status;
    return k_internal_direction(columns, count, result, options, step);
}

k_status k_internal_direction(const k_real *columns, size_t count, const k_real rhs[3],
                              const k_ik_options *options, k_real *step)
{
    k_real divisor = 1;
    for (size_t i = 0; i < count; ++i)
    {
        k_real value = 0;
        for (size_t r = 0; r < 3; ++r)
        {
            k_real product;
            if (!k_checked_mul(columns[3 * i + r], rhs[r], &product) ||
                !k_checked_add(value, product, &value))
                return K_NUMERIC_ERROR;
        }
        if (!k_checked_mul(value, options->joint_scales[i], &step[i]))
            return K_NUMERIC_ERROR;
        k_real ratio = K_ABS(step[i]) / options->max_joint_steps[i];
        if (!isfinite(ratio))
            return K_NUMERIC_ERROR;
        if (ratio > divisor)
            divisor = ratio;
    }
    for (size_t i = 0; i < count; ++i)
        step[i] /= divisor;
    return K_OK;
}
