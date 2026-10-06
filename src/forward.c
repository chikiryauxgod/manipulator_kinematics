#include <manipulator/forward.h>
#include <string.h>
#include "internal.h"

static mk_status validate_inputs(const mk_model *model, const mk_real *q, size_t count)
{
    if (!q) return MK_INVALID_ARGUMENT;
    mk_status status = mk_model_validate(model);
    if (status != MK_OK) return status;
    if (count != model->joint_count) return MK_SIZE_MISMATCH;
    for (size_t i = 0; i < count; ++i)
        if (!isfinite(q[i])) return MK_NONFINITE_INPUT;
    return MK_OK;
}

static mk_status walk(const mk_model *model, const mk_real *q,
                      mk_transform *tool, mk_transform *frames)
{
    mk_transform current = model->world_base;
    if (frames) frames[0] = current;
    for (size_t i = 0; i < model->joint_count; ++i) {
        mk_transform local;
        mk_status status = mk_internal_dh(&model->joints[i], q[i], &local);
        if (status != MK_OK) return status;
        status = mk_internal_compose(&current, &local, &current);
        if (status != MK_OK) return status;
        if (frames) frames[i+1] = current;
    }
    mk_status status = mk_internal_compose(&current, &model->last_tool, &current);
    if (status != MK_OK) return status;
    if (frames) frames[model->joint_count+1] = current;
    if (tool) *tool = current;
    return MK_OK;
}

mk_status mk_forward(const mk_model *model, const mk_real *q, size_t n, mk_transform *out)
{
    if (!out) return MK_INVALID_ARGUMENT;
    mk_status status = validate_inputs(model, q, n);
    return status != MK_OK ? status : walk(model, q, out, NULL);
}

mk_status mk_forward_frames(const mk_model *model, const mk_real *q, size_t n,
                            mk_transform *frames, size_t capacity,
                            mk_transform *workspace, size_t work_capacity)
{
    if (!frames || !workspace) return MK_INVALID_ARGUMENT;
    mk_status status = validate_inputs(model, q, n);
    if (status != MK_OK) return status;
    size_t needed = model->joint_count + 2;
    if (capacity < needed || work_capacity < needed) return MK_BUFFER_TOO_SMALL;
    status = walk(model, q, NULL, workspace);
    if (status == MK_OK) memcpy(frames, workspace, needed * sizeof(*frames));
    return status;
}
