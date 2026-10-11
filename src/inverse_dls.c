#include <manipulator/inverse.h>
#include "inverse_internal.h"

static int normalize_column(k_real value, k_real weight, k_real length, k_real *out)
{
    if (value == 0)
    {
        *out = 0;
        return 1;
    }
    k_real product;
    if (k_checked_mul(value, weight, &product) && fpclassify(product) == FP_NORMAL)
    {
        *out = product / length;
        if (isfinite(*out))
            return 1;
    }
    int value_exponent, weight_exponent, length_exponent;
    k_real value_fraction = K_FREXP(value, &value_exponent);
    k_real weight_fraction = K_FREXP(weight, &weight_exponent);
    k_real length_fraction = K_FREXP(length, &length_exponent);
    *out = K_SCALBN(value_fraction * weight_fraction / length_fraction,
                    value_exponent + weight_exponent - length_exponent);
    return isfinite(*out);
}

static int normalize_error(k_real value, k_real length, k_real scale, k_real *out)
{
    if (value == 0)
    {
        *out = 0;
        return 1;
    }
    k_real quotient = value / length;
    if (isfinite(quotient) && fpclassify(quotient) == FP_NORMAL)
    {
        *out = quotient / scale;
        if (isfinite(*out))
            return 1;
    }
    int value_exponent, length_exponent, scale_exponent;
    k_real value_fraction = K_FREXP(value, &value_exponent);
    k_real length_fraction = K_FREXP(length, &length_exponent);
    k_real scale_fraction = K_FREXP(scale, &scale_exponent);
    *out = K_SCALBN(value_fraction / length_fraction / scale_fraction,
                    value_exponent - length_exponent - scale_exponent);
    return isfinite(*out);
}

k_status k_internal_dls(k_real *columns, size_t count, const k_real error[3],
                        const k_ik_options *options, k_real *step, k_real gradient_rhs[3],
                        k_real *effective_damping)
{
    k_real scale = options->damping;
    for (size_t i = 0; i < count; ++i)
        for (size_t r = 0; r < 3; ++r)
        {
            k_real *value = columns + 3 * i + r;
            if (!normalize_column(*value, options->joint_scales[i], options->length_scale, value))
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
        if (!normalize_error(error[r], options->length_scale, scale, &rhs[r]))
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
