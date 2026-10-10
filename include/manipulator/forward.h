#ifndef MANIPULATOR_FORWARD_H
#define MANIPULATOR_FORWARD_H
#include <manipulator/model.h>
#ifdef __cplusplus
extern "C" {
#endif

mk_status mk_forward(const mk_model *model, const mk_real *q, size_t q_count,
                     mk_transform *world_tool);

mk_status mk_forward_frames(const mk_model *model, const mk_real *q, size_t q_count,
                            mk_transform *frames, size_t frame_capacity,
                            mk_transform *workspace, size_t workspace_capacity);
#ifdef __cplusplus
}
#endif
#endif
