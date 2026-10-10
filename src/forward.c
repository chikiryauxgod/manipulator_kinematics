#include <manipulator/forward.h>
#include <string.h>
#include "internal.h"

static manipulator_status validate_inputs(const manipulator_model *model, const manipulator_real *q, size_t count)
{
    if (!q) return MANIPULATOR_INVALID_ARGUMENT;
    manipulator_status status = manipulator_model_validate(model);
    if (status != MANIPULATOR_OK) return status;
    if (count != model->joint_count) return MANIPULATOR_SIZE_MISMATCH;
    for (size_t i = 0; i < count; ++i)
        if (!isfinite(q[i])) return MANIPULATOR_NONFINITE_INPUT;
    return MANIPULATOR_OK;
}

static manipulator_status walk(const manipulator_model *model, const manipulator_real *q,
                      manipulator_transform *tool, manipulator_transform *frames)
{
    manipulator_transform current = model->world_base;
    if (frames) frames[0] = current;
    for (size_t i = 0; i < model->joint_count; ++i) {
        manipulator_transform local;
        manipulator_status status = manipulator_internal_dh(&model->joints[i], q[i], &local);
        if (status != MANIPULATOR_OK) return status;
        status = manipulator_internal_compose(&current, &local, &current);
        if (status != MANIPULATOR_OK) return status;
        if (frames) frames[i+1] = current;
    }
    manipulator_status status = manipulator_internal_compose(&current, &model->last_tool, &current);
    if (status != MANIPULATOR_OK) return status;
    if (frames) frames[model->joint_count+1] = current;
    if (tool) *tool = current;
    return MANIPULATOR_OK;
}

manipulator_status manipulator_forward(const manipulator_model *model, const manipulator_real *q, size_t n, manipulator_transform *out)
{
    if (!out) return MANIPULATOR_INVALID_ARGUMENT;
    manipulator_status status = validate_inputs(model, q, n);
    return status != MANIPULATOR_OK ? status : walk(model, q, out, NULL);
}

manipulator_status manipulator_forward_frames(const manipulator_model *model, const manipulator_real *q, size_t n,
                            manipulator_transform *frames, size_t capacity,
                            manipulator_transform *workspace, size_t work_capacity)
{
    if (!frames || !workspace) return MANIPULATOR_INVALID_ARGUMENT;
    manipulator_status status = validate_inputs(model, q, n);
    if (status != MANIPULATOR_OK) return status;
    size_t needed = model->joint_count + 2;
    if (capacity < needed || work_capacity < needed) return MANIPULATOR_BUFFER_TOO_SMALL;
    status = walk(model, q, NULL, workspace);
    if (status == MANIPULATOR_OK) memcpy(frames, workspace, needed * sizeof(*frames));
    return status;
}
