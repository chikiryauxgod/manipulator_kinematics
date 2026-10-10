#include <manipulator/forward.h>
#include <string.h>
#include "internal.h"

static k_status validate_inputs(const k_model *model, const k_real *q, size_t count)
{
    if (!q) return K_INVALID_ARGUMENT;
    k_status status = k_model_validate(model);
    if (status != K_OK) return status;
    if (count != model->joint_count) return K_SIZE_MISMATCH;
    for (size_t i = 0; i < count; ++i)
        if (!isfinite(q[i])) return K_NONFINITE_INPUT;
    return K_OK;
}

static k_status walk(const k_model *model, const k_real *q,
                      k_transform *tool, k_transform *frames)
{
    k_transform current = model->world_base;
    if (frames) frames[0] = current;
    for (size_t i = 0; i < model->joint_count; ++i) {
        k_transform local;
        k_status status = k_internal_dh(&model->joints[i], q[i], &local);
        if (status != K_OK) return status;
        status = k_internal_compose(&current, &local, &current);
        if (status != K_OK) return status;
        if (frames) frames[i+1] = current;
    }
    k_status status = k_internal_compose(&current, &model->last_tool, &current);
    if (status != K_OK) return status;
    if (frames) frames[model->joint_count+1] = current;
    if (tool) *tool = current;
    return K_OK;
}

k_status k_forward(const k_model *model, const k_real *q, size_t n, k_transform *out)
{
    if (!out) return K_INVALID_ARGUMENT;
    k_status status = validate_inputs(model, q, n);
    return status != K_OK ? status : walk(model, q, out, NULL);
}

k_status k_forward_frames(const k_model *model, const k_real *q, size_t n,
                            k_transform *frames, size_t capacity,
                            k_transform *workspace, size_t work_capacity)
{
    if (!frames || !workspace) return K_INVALID_ARGUMENT;
    k_status status = validate_inputs(model, q, n);
    if (status != K_OK) return status;
    size_t needed = model->joint_count + 2;
    if (capacity < needed || work_capacity < needed) return K_BUFFER_TOO_SMALL;
    status = walk(model, q, NULL, workspace);
    if (status == K_OK) memcpy(frames, workspace, needed * sizeof(*frames));
    return status;
}
