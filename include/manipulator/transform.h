#ifndef MANIPULATOR_TRANSFORM_H
#define MANIPULATOR_TRANSFORM_H
#include <manipulator/types.h>
#ifdef __cplusplus
extern "C" {
#endif
k_status k_transform_identity(k_transform *out);
k_status k_transform_validate(const k_transform *transform);

k_status k_transform_compose(const k_transform *parent_mid,
                               const k_transform *mid_child, k_transform *out);
#ifdef __cplusplus
}
#endif
#endif
