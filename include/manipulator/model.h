#ifndef MANIPULATOR_MODEL_H
#define MANIPULATOR_MODEL_H
#include <manipulator/types.h>
#ifdef __cplusplus
extern "C"
{
#endif

k_status k_model_init(k_model *out, const k_dh_joint *joints, size_t joint_count);
k_status k_model_validate(const k_model *model);
#ifdef __cplusplus
}
#endif
#endif
