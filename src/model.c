#include <manipulator/model.h>
#include <string.h>
#include "internal.h"

static k_status structure(const k_model *model)
{
    if (!model) return K_INVALID_ARGUMENT;
    if (!k_count_valid(model->joint_count) || model->joint_count > model->storage_capacity)
        return K_INVALID_MODEL;
    if (!model->joints) return K_INVALID_ARGUMENT;
    return K_OK;
}

static k_status configuring(k_model *model)
{
    if (!model) return K_INVALID_ARGUMENT;
    if (model->state != K_CONFIGURING) return K_INVALID_STATE;
    return structure(model);
}

static k_status geometry_valid(const k_dh_joint *joint)
{
    if (!joint) return K_INVALID_ARGUMENT;
    if (joint->type != K_REVOLUTE && joint->type != K_PRISMATIC) return K_INVALID_MODEL;
    if (!isfinite(joint->a) || !isfinite(joint->alpha) || !isfinite(joint->d_offset)
        || !isfinite(joint->theta_offset)) return K_NONFINITE_INPUT;
    return K_OK;
}

static k_status limits_valid(const k_joint_limits *limits)
{
    if (!limits) return K_INVALID_ARGUMENT;
    if (limits->mode == K_UNBOUNDED) return K_OK;
    if (limits->mode != K_BOUNDED) return K_INVALID_MODEL;
    if (!isfinite(limits->min) || !isfinite(limits->max)) return K_NONFINITE_INPUT;
    return limits->min <= limits->max ? K_OK : K_INVALID_MODEL;
}

static k_status configuration(const k_model *model)
{
    k_status status = structure(model);
    if (status != K_OK) return status;
    if (model->configured != 3u) return K_INCOMPLETE_CONFIGURATION;
    for (size_t i = 0; i < model->joint_count; ++i)
    {
        const k_joint_config *joint = &model->joints[i];
        if (joint->configured != 3u) return K_INCOMPLETE_CONFIGURATION;
        status = geometry_valid(&joint->geometry);
        if (status != K_OK) return status;
        status = limits_valid(&joint->limits);
        if (status != K_OK) return status;
    }
    status = k_transform_validate(&model->world_base);
    return status != K_OK ? status : k_transform_validate(&model->last_tool);
}

k_status k_model_begin(k_model *model, k_joint_config *storage, size_t capacity, size_t count)
{
    if (!model || !storage) return K_INVALID_ARGUMENT;
    if (!k_count_valid(count) || count > SIZE_MAX / sizeof(*storage)) return K_INVALID_MODEL;
    if (capacity < count) return K_BUFFER_TOO_SMALL;
    k_model result = {0};
    result.joints = storage;
    result.joint_count = count;
    result.storage_capacity = capacity;
    result.state = K_CONFIGURING;
    memset(storage, 0, count * sizeof(*storage));
    *model = result;
    return K_OK;
}

k_status k_model_set_joint(k_model *model, size_t index, const k_dh_joint *geometry)
{
    k_status status = configuring(model);
    if (status != K_OK) return status;
    if (index >= model->joint_count) return K_SIZE_MISMATCH;
    status = geometry_valid(geometry);
    if (status != K_OK) return status;
    model->joints[index].geometry = *geometry;
    model->joints[index].configured |= 1u;
    return K_OK;
}

k_status k_model_set_limits(k_model *model, size_t index, const k_joint_limits *limits)
{
    k_status status = configuring(model);
    if (status != K_OK) return status;
    if (index >= model->joint_count) return K_SIZE_MISMATCH;
    status = limits_valid(limits);
    if (status != K_OK) return status;
    if (limits->mode == K_UNBOUNDED)
        model->joints[index].limits = (k_joint_limits){K_UNBOUNDED, 0, 0};
    else
        model->joints[index].limits = *limits;
    model->joints[index].configured |= 2u;
    return K_OK;
}

static k_status set_transform(k_model *model, const k_transform *transform, unsigned flag)
{
    k_status status = configuring(model);
    if (status != K_OK) return status;
    status = k_transform_validate(transform);
    if (status != K_OK) return status;
    if (flag == 1u) model->world_base = *transform;
    else model->last_tool = *transform;
    model->configured |= flag;
    return K_OK;
}

k_status k_model_set_base(k_model *model, const k_transform *transform)
{
    return set_transform(model, transform, 1u);
}

k_status k_model_set_tool(k_model *model, const k_transform *transform)
{
    return set_transform(model, transform, 2u);
}

k_status k_model_finalize(k_model *model)
{
    k_status status = configuring(model);
    if (status != K_OK) return status;
    status = configuration(model);
    if (status == K_OK) model->state = K_READY;
    return status;
}

k_status k_model_validate(const k_model *model)
{
    if (!model) return K_INVALID_ARGUMENT;
    if (model->state == K_EMPTY || model->state == K_CONFIGURING) return K_NOT_READY;
    if (model->state != K_READY) return K_INVALID_STATE;
    return configuration(model);
}

k_status k_internal_positions_validate(const k_model *model,
                                       const k_real *joint_positions, size_t count)
{
    if (!joint_positions) return K_INVALID_ARGUMENT;
    k_status status = k_model_validate(model);
    if (status != K_OK) return status;
    if (count != model->joint_count) return K_SIZE_MISMATCH;
    for (size_t i = 0; i < count; ++i)
    {
        if (!isfinite(joint_positions[i])) return K_NONFINITE_INPUT;
        const k_joint_limits *limits = &model->joints[i].limits;
        if (limits->mode == K_BOUNDED
            && (joint_positions[i] < limits->min || joint_positions[i] > limits->max))
            return K_JOINT_LIMIT;
    }
    return K_OK;
}
