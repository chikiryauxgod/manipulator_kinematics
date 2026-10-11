#include <manipulator/forward.h>
#include <string.h>
#include "internal.h"

k_status k_internal_walk(const k_model *model, const k_real *joint_positions,
                      k_transform *tool, k_transform *frames)
{
    k_transform current = model->world_base;
    if (frames) frames[0] = current;
    for (size_t i = 0; i < model->joint_count; ++i) {
        k_transform local;
        k_status status = k_internal_dh(&model->joints[i].geometry, joint_positions[i], &local);
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

k_status k_forward(const k_model *model, const k_real *joint_positions, size_t n, k_transform *out)
{
    if (!out) return K_INVALID_ARGUMENT;
    k_status status = k_internal_positions_validate(model, joint_positions, n);
    return status != K_OK ? status : k_internal_walk(model, joint_positions, out, NULL);
}

k_status k_forward_frames(const k_model *model, const k_real *joint_positions, size_t n,
                            k_transform *frames, size_t capacity,
                            k_transform *workspace, size_t work_capacity)
{
    if (!frames || !workspace) return K_INVALID_ARGUMENT;
    k_status status = k_internal_positions_validate(model, joint_positions, n);
    if (status != K_OK) return status;
    size_t needed = model->joint_count + 2;
    if (capacity < needed || work_capacity < needed) return K_BUFFER_TOO_SMALL;
    status = k_internal_walk(model, joint_positions, NULL, workspace);
    if (status == K_OK) memcpy(frames, workspace, needed * sizeof(*frames));
    return status;
}
