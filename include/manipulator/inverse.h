#ifndef MANIPULATOR_INVERSE_H
#define MANIPULATOR_INVERSE_H
#include <manipulator/model.h>
#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    size_t max_iterations, max_backtracking_trials;
    k_real position_tolerance, length_scale, damping;
    const k_real *joint_scales;
    const k_real *max_joint_steps;
    size_t joint_count;
} k_ik_options;

typedef struct
{
    k_status termination;
    size_t iteration_attempts, accepted_steps, fk_evaluations, jacobian_evaluations;
    k_real residual, effective_damping;
    int residual_valid;
} k_ik_report;

k_status k_ik_workspace_size(size_t joint_count, size_t *element_count);
k_status k_inverse_position(const k_model *model, const k_vec3 *target,
                            const k_real *initial_joint_positions, size_t joint_count,
                            const k_ik_options *options, k_real *workspace,
                            size_t workspace_capacity, k_real *joint_positions,
                            size_t output_capacity, k_ik_report *report);
#ifdef __cplusplus
}
#endif
#endif
