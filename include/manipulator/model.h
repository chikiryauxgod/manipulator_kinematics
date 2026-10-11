#ifndef MANIPULATOR_MODEL_H
#define MANIPULATOR_MODEL_H
#include <manipulator/types.h>
#ifdef __cplusplus
extern "C"
{
#endif
k_status k_model_begin(k_model *model, k_joint_config *storage,
                       size_t storage_capacity, size_t joint_count);
k_status k_model_set_joint(k_model *model, size_t joint_index, const k_dh_joint *geometry);
k_status k_model_set_limits(k_model *model, size_t joint_index, const k_joint_limits *limits);
k_status k_model_set_base(k_model *model, const k_transform *world_base);
k_status k_model_set_tool(k_model *model, const k_transform *last_tool);
k_status k_model_finalize(k_model *model);
k_status k_model_validate(const k_model *model);
#ifdef __cplusplus
}
#endif
#endif
