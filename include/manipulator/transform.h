#ifndef MANIPULATOR_TRANSFORM_H
#define MANIPULATOR_TRANSFORM_H
#include <manipulator/types.h>
#ifdef __cplusplus
extern "C" {
#endif
mk_status mk_transform_identity(mk_transform *out);
mk_status mk_transform_validate(const mk_transform *transform);

mk_status mk_transform_compose(const mk_transform *parent_mid,
                               const mk_transform *mid_child, mk_transform *out);
#ifdef __cplusplus
}
#endif
#endif
