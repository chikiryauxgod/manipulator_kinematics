#ifndef MANIPULATOR_FORWARD_H
#define MANIPULATOR_FORWARD_H
#include <manipulator/model.h>
#ifdef __cplusplus
extern "C" {
#endif
/* q: rad for R, m for P. world_tool is unchanged on error. No input/output overlap. */
mk_status mk_forward(const mk_model *model, const mk_real *q, size_t q_count,
                     mk_transform *world_tool);
/* Both buffers need n+2 elements and must not overlap each other or inputs.
 * frames: base, n link frames, tool. Output unchanged on error; workspace may
 * contain incomplete calculations after a numeric error and is not a result. */
mk_status mk_forward_frames(const mk_model *model, const mk_real *q, size_t q_count,
                            mk_transform *frames, size_t frame_capacity,
                            mk_transform *workspace, size_t workspace_capacity);
#ifdef __cplusplus
}
#endif
#endif
