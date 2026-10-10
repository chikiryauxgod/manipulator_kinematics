#ifndef MANIPULATOR_TRANSFORM_H
#define MANIPULATOR_TRANSFORM_H
#include <manipulator/types.h>
#ifdef __cplusplus
extern "C" {
#endif
manipulator_status manipulator_transform_identity(manipulator_transform *out);
manipulator_status manipulator_transform_validate(const manipulator_transform *transform);

manipulator_status manipulator_transform_compose(const manipulator_transform *parent_mid,
                               const manipulator_transform *mid_child, manipulator_transform *out);
#ifdef __cplusplus
}
#endif
#endif
