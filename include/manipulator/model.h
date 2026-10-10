#ifndef MANIPULATOR_MODEL_H
#define MANIPULATOR_MODEL_H
#include <manipulator/types.h>
#ifdef __cplusplus
extern "C" {
#endif

mk_status mk_model_init(mk_model *out, const mk_dh_joint *joints, size_t joint_count);
mk_status mk_model_validate(const mk_model *model);
#ifdef __cplusplus
}
#endif
#endif
