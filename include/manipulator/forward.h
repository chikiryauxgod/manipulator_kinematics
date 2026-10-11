#ifndef MANIPULATOR_FORWARD_H
#define MANIPULATOR_FORWARD_H
#include <manipulator/model.h>
#ifdef __cplusplus
extern "C"
{
#endif

k_status k_forward(const k_model *model, const k_real *joint_positions, size_t joint_count, k_transform *world_tool);

k_status k_forward_frames(const k_model *model, const k_real *joint_positions, size_t joint_count,
                          k_transform *frames, size_t frame_capacity, k_transform *workspace,
                          size_t workspace_capacity);
#ifdef __cplusplus
}
#endif
#endif
