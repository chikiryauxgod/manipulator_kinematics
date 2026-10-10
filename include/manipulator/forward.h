#ifndef MANIPULATOR_FORWARD_H
#define MANIPULATOR_FORWARD_H
#include <manipulator/model.h>
#ifdef __cplusplus
extern "C" {
#endif

manipulator_status manipulator_forward(const manipulator_model *model, const manipulator_real *q, size_t q_count,
                     manipulator_transform *world_tool);

manipulator_status manipulator_forward_frames(const manipulator_model *model, const manipulator_real *q, size_t q_count,
                            manipulator_transform *frames, size_t frame_capacity,
                            manipulator_transform *workspace, size_t workspace_capacity);
#ifdef __cplusplus
}
#endif
#endif
