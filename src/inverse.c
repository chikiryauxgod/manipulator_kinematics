#include <manipulator/inverse.h>
#include <string.h>
#include "inverse_internal.h"

k_status k_ik_workspace_size(size_t count, size_t *elements)
{
    if (!elements)
        return K_INVALID_ARGUMENT;
    if (!k_count_valid(count) || count > SIZE_MAX / 6 / sizeof(k_real))
        return K_INVALID_MODEL;
    *elements = 6 * count;
    return K_OK;
}

static k_status options_valid(const k_ik_options *options, size_t count)
{
    if (!options || !options->joint_scales || !options->max_joint_steps)
        return K_INVALID_ARGUMENT;
    if (options->joint_count != count)
        return K_SIZE_MISMATCH;
    if (options->max_iterations == 0 || options->max_backtracking_trials == 0 ||
        options->max_backtracking_trials > (SIZE_MAX - 1) / 2 ||
        options->max_iterations > (SIZE_MAX - 1) / (2 * options->max_backtracking_trials))
        return K_INVALID_OPTIONS;
    const k_real values[] = {options->position_tolerance, options->length_scale, options->damping};
    for (size_t i = 0; i < 3; ++i)
        if (!isfinite(values[i]) || values[i] <= 0)
            return K_INVALID_OPTIONS;
    for (size_t i = 0; i < count; ++i)
        if (!isfinite(options->joint_scales[i]) || options->joint_scales[i] <= 0 ||
            !isfinite(options->max_joint_steps[i]) || options->max_joint_steps[i] <= 0)
            return K_INVALID_OPTIONS;
    return K_OK;
}

static k_status position_error(const k_vec3 *target, const k_transform *tool, k_real error[3],
                               k_real *residual)
{
    const k_real values[3] = {target->x, target->y, target->z};
    for (size_t r = 0; r < 3; ++r)
        if (!k_checked_add(values[r], -tool->m[r][3], &error[r]))
            return K_NUMERIC_ERROR;
    *residual = K_HYPOT(K_HYPOT(error[0], error[1]), error[2]);
    return isfinite(*residual) ? K_OK : K_NUMERIC_ERROR;
}

static k_status evaluate(const k_model *model, const k_real *positions, const k_vec3 *target,
                         k_transform *tool, k_real error[3], k_real *residual, k_ik_report *report)
{
    ++report->fk_evaluations;
    k_status status = k_internal_walk(model, positions, tool, NULL);
    return status == K_OK ? position_error(target, tool, error, residual) : status;
}

static k_status search(const k_model *model, const k_vec3 *target, const k_ik_options *options,
                       const k_real *current, const k_real *step, k_real *candidate,
                       k_transform *tool, k_real error[3], k_real *residual, k_ik_report *report,
                       int *had_valid_trial)
{
    k_real alpha = 1;
    for (size_t trial = 0; trial < options->max_backtracking_trials && alpha > 0; ++trial)
    {
        int changed = 0, valid = 1;
        for (size_t i = 0; i < model->joint_count; ++i)
        {
            k_real increment;
            if (!k_checked_mul(alpha, step[i], &increment) ||
                !k_checked_add(current[i], increment, &candidate[i]))
            {
                valid = 0;
                break;
            }
            const k_joint_limits *limits = &model->joints[i].limits;
            if (limits->mode == K_BOUNDED)
            {
                if (candidate[i] < limits->min)
                    candidate[i] = limits->min;
                if (candidate[i] > limits->max)
                    candidate[i] = limits->max;
            }
            if (candidate[i] != current[i])
                changed = 1;
        }
        if (valid && !changed)
        {
            *had_valid_trial = 1;
            return K_STALLED;
        }
        if (valid)
        {
            k_transform trial_tool;
            k_real trial_error[3], trial_residual;
            k_status status = evaluate(model, candidate, target, &trial_tool, trial_error,
                                       &trial_residual, report);
            if (status == K_OK)
            {
                *had_valid_trial = 1;
                if (trial_residual < *residual || trial_residual <= options->position_tolerance)
                {
                    *tool = trial_tool;
                    *residual = trial_residual;
                    for (size_t r = 0; r < 3; ++r)
                        error[r] = trial_error[r];
                    return K_OK;
                }
            }
        }
        alpha *= (k_real)0.5;
    }
    return K_STALLED;
}

static k_status solve(const k_model *model, const k_vec3 *target,
                      const k_real *initial_joint_positions, size_t count,
                      const k_ik_options *options, k_real *workspace, size_t capacity,
                      k_real *joint_positions, size_t output_capacity, k_ik_report *report)
{
    if (!target || !workspace || !joint_positions)
        return K_INVALID_ARGUMENT;
    k_status status = k_internal_positions_validate(model, initial_joint_positions, count);
    if (status != K_OK)
        return status;
    if (!isfinite(target->x) || !isfinite(target->y) || !isfinite(target->z))
        return K_NONFINITE_INPUT;
    status = options_valid(options, count);
    if (status != K_OK)
        return status;
    size_t needed;
    status = k_ik_workspace_size(count, &needed);
    if (status != K_OK)
        return status;
    if (capacity < needed || output_capacity < count)
        return K_BUFFER_TOO_SMALL;
    k_real *current = workspace;
    k_real *candidate = workspace + count;
    k_real *step = workspace + 2 * count;
    k_real *columns = workspace + 3 * count;
    memcpy(current, initial_joint_positions, count * sizeof(*current));
    k_transform tool;
    k_real error[3], residual;
    status = evaluate(model, current, target, &tool, error, &residual, report);
    if (status != K_OK)
        return status;
    report->residual = residual;
    report->residual_valid = 1;
    for (size_t attempt = 0; residual > options->position_tolerance; ++attempt)
    {
        if (attempt == options->max_iterations)
            return K_NO_CONVERGENCE;
        ++report->iteration_attempts;
        ++report->jacobian_evaluations;
        status = k_internal_jacobian(model, current, &tool, columns);
        if (status != K_OK)
            return status;
        k_real gradient_rhs[3];
        status = k_internal_dls(columns, count, error, options, step, gradient_rhs,
                                &report->effective_damping);
        if (status != K_OK)
            return status;
        int had_valid_trial = 0;
        status = search(model, target, options, current, step, candidate, &tool, error, &residual,
                        report, &had_valid_trial);
        if (status != K_OK)
        {
            status = k_internal_direction(columns, count, gradient_rhs, options, step);
            if (status != K_OK)
                return status;
            status = search(model, target, options, current, step, candidate, &tool, error,
                            &residual, report, &had_valid_trial);
        }
        if (status != K_OK)
            return had_valid_trial ? K_STALLED : K_NUMERIC_ERROR;
        memcpy(current, candidate, count * sizeof(*current));
        ++report->accepted_steps;
        report->residual = residual;
    }
    memcpy(joint_positions, current, count * sizeof(*joint_positions));
    return K_OK;
}

k_status k_inverse_position(const k_model *model, const k_vec3 *target,
                            const k_real *initial_joint_positions, size_t count,
                            const k_ik_options *options, k_real *workspace, size_t capacity,
                            k_real *joint_positions, size_t output_capacity, k_ik_report *report)
{
    k_ik_report result = {0};
    k_status status = solve(model, target, initial_joint_positions, count, options, workspace,
                            capacity, joint_positions, output_capacity, &result);
    result.termination = status;
    if (report)
        *report = result;
    return status;
}
