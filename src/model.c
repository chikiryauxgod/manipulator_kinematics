#include <manipulator/model.h>
#include "internal.h"

manipulator_status manipulator_internal_joints_validate(const manipulator_dh_joint *joints, size_t n)
{
    if (!joints) return MANIPULATOR_INVALID_ARGUMENT;
    if (!manipulator_count_valid(n)) return MANIPULATOR_INVALID_MODEL;
    for (size_t i = 0; i < n; ++i) {
        const manipulator_dh_joint *j = &joints[i];
        if (j->type != MANIPULATOR_REVOLUTE && j->type != MANIPULATOR_PRISMATIC) return MANIPULATOR_INVALID_MODEL;
        if (!isfinite(j->a) || !isfinite(j->alpha) || !isfinite(j->d_offset)
            || !isfinite(j->theta_offset)) return MANIPULATOR_NONFINITE_INPUT;
    }
    return MANIPULATOR_OK;
}

manipulator_status manipulator_model_validate(const manipulator_model *model)
{
    if (!model) return MANIPULATOR_INVALID_ARGUMENT;
    manipulator_status status = manipulator_internal_joints_validate(model->joints, model->joint_count);
    if (status != MANIPULATOR_OK) return status;
    status = manipulator_transform_validate(&model->world_base);
    return status != MANIPULATOR_OK ? status : manipulator_transform_validate(&model->last_tool);
}

manipulator_status manipulator_model_init(manipulator_model *out, const manipulator_dh_joint *joints, size_t n)
{
    if (!out) return MANIPULATOR_INVALID_ARGUMENT;
    manipulator_status status = manipulator_internal_joints_validate(joints, n);
    if (status != MANIPULATOR_OK) return status;
    manipulator_model result;
    result.joints = joints;
    result.joint_count = n;
    (void)manipulator_transform_identity(&result.world_base);
    (void)manipulator_transform_identity(&result.last_tool);
    *out = result;
    return MANIPULATOR_OK;
}
