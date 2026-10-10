#ifndef MANIPULATOR_MODEL_H
#define MANIPULATOR_MODEL_H
#include <manipulator/types.h>
#ifdef __cplusplus
extern "C" {
#endif

manipulator_status manipulator_model_init(manipulator_model *out, const manipulator_dh_joint *joints, size_t joint_count);
manipulator_status manipulator_model_validate(const manipulator_model *model);
#ifdef __cplusplus
}
#endif
#endif
