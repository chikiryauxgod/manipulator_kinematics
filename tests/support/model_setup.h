#ifndef TEST_MODEL_SETUP_H
#define TEST_MODEL_SETUP_H
#include <manipulator/forward.h>
static inline k_status test_setup(k_model *model, k_joint_config *storage,
                                  size_t capacity, const k_dh_joint *joints, size_t count,
                                  const k_transform *base, const k_transform *tool)
{
    k_status status = k_model_begin(model, storage, capacity, count);
    const k_joint_limits limits = {K_UNBOUNDED, 0, 0};
    for (size_t i = 0; status == K_OK && i < count; ++i)
    {
        status = k_model_set_joint(model, i, &joints[i]);
        if (status == K_OK) status = k_model_set_limits(model, i, &limits);
    }
    if (status == K_OK) status = k_model_set_base(model, base);
    if (status == K_OK) status = k_model_set_tool(model, tool);
    return status == K_OK ? k_model_finalize(model) : status;
}
#endif
