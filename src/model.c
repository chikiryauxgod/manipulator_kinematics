#include <manipulator/model.h>
#include "internal.h"

k_status k_internal_joints_validate(const k_dh_joint *joints, size_t n)
{
    if (!joints) return K_INVALID_ARGUMENT;
    if (!k_count_valid(n)) return K_INVALID_MODEL;
    for (size_t i = 0; i < n; ++i) {
        const k_dh_joint *j = &joints[i];
        if (j->type != K_REVOLUTE && j->type != K_PRISMATIC) return K_INVALID_MODEL;
        if (!isfinite(j->a) || !isfinite(j->alpha) || !isfinite(j->d_offset)
            || !isfinite(j->theta_offset)) return K_NONFINITE_INPUT;
    }
    return K_OK;
}

k_status k_model_validate(const k_model *model)
{
    if (!model) return K_INVALID_ARGUMENT;
    k_status status = k_internal_joints_validate(model->joints, model->joint_count);
    if (status != K_OK) return status;
    status = k_transform_validate(&model->world_base);
    return status != K_OK ? status : k_transform_validate(&model->last_tool);
}

k_status k_model_init(k_model *out, const k_dh_joint *joints, size_t n)
{
    if (!out) return K_INVALID_ARGUMENT;
    k_status status = k_internal_joints_validate(joints, n);
    if (status != K_OK) return status;
    k_model result;
    result.joints = joints;
    result.joint_count = n;
    (void)k_transform_identity(&result.world_base);
    (void)k_transform_identity(&result.last_tool);
    *out = result;
    return K_OK;
}
