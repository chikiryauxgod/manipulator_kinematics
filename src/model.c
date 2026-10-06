#include <manipulator/model.h>
#include "internal.h"

mk_status mk_internal_joints_validate(const mk_dh_joint *joints, size_t n)
{
    if (!joints) return MK_INVALID_ARGUMENT;
    if (!mk_count_valid(n)) return MK_INVALID_MODEL;
    for (size_t i = 0; i < n; ++i) {
        const mk_dh_joint *j = &joints[i];
        if (j->type != MK_REVOLUTE && j->type != MK_PRISMATIC) return MK_INVALID_MODEL;
        if (!isfinite(j->a) || !isfinite(j->alpha) || !isfinite(j->d_offset)
            || !isfinite(j->theta_offset)) return MK_NONFINITE_INPUT;
    }
    return MK_OK;
}

mk_status mk_model_validate(const mk_model *model)
{
    if (!model) return MK_INVALID_ARGUMENT;
    mk_status status = mk_internal_joints_validate(model->joints, model->joint_count);
    if (status != MK_OK) return status;
    status = mk_transform_validate(&model->world_base);
    return status != MK_OK ? status : mk_transform_validate(&model->last_tool);
}

mk_status mk_model_init(mk_model *out, const mk_dh_joint *joints, size_t n)
{
    if (!out) return MK_INVALID_ARGUMENT;
    mk_status status = mk_internal_joints_validate(joints, n);
    if (status != MK_OK) return status;
    mk_model result;
    result.joints = joints;
    result.joint_count = n;
    (void)mk_transform_identity(&result.world_base);
    (void)mk_transform_identity(&result.last_tool);
    *out = result;
    return MK_OK;
}
